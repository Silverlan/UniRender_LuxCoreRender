/* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/.
*
* Copyright (c) 2020 Florian Weischer
*/

#include "unirender/luxcorerender/renderer.hpp"
#include <mathutil/umath_geometry.hpp>
#include <util_raytracing/camera.hpp>
#include <util_raytracing/mesh.hpp>
#include <util_raytracing/light.hpp>
#include <util_raytracing/shader.hpp>
#include <util_raytracing/model_cache.hpp>
#include <util_ocio.hpp>
#include <mathutil/umath_lighting.hpp>
#include <luxcore/luxcore.h>
#include <luxrays/core/geometry/matrix4x4.h>
#include <luxrays/core/geometry/point.h>
#include <luxrays/core/geometry/triangle.h>
#include <luxrays/core/geometry/uv.h>
#include <util_image_buffer.hpp>

using namespace unirender::luxcorerender;
#pragma optimize("",off)

#include <util_raytracing/hair.hpp>
static Vector3 calc_hair_normal(const Vector3 &flowNormal,const Vector3 &faceNormal)
{
	auto hairNormal = flowNormal -uvec::project(flowNormal,faceNormal);
	uvec::normalize(&hairNormal);
	return hairNormal;
}
static std::unique_ptr<luxrays::cyHairFile> generate_hair_file(const unirender::HairConfig &hairConfig,const unirender::HairGenerator::HairData &hairData)
{
	auto numHair = hairData.hairPoints.size();
	auto numSegments = hairConfig.numSegments;
	auto numPoints = numHair *(numSegments +1);
	auto hairFile = std::make_unique<luxrays::cyHairFile>();
	hairFile->Initialize();
	hairFile->SetHairCount(numHair);
	hairFile->SetPointCount(numPoints);
	hairFile->SetArrays(CY_HAIR_FILE_POINTS_BIT | CY_HAIR_FILE_UVS_BIT | CY_HAIR_FILE_THICKNESS_BIT);
	hairFile->SetDefaultSegmentCount(numSegments);
	hairFile->SetDefaultThickness(hairConfig.defaultThickness);

	auto *lxUvs = reinterpret_cast<Vector2*>(hairFile->GetUVsArray());
	auto *thicknessData = hairFile->GetThicknessArray();
	auto *points = reinterpret_cast<Vector3*>(hairFile->GetPointsArray());
	uint32_t hairIdx = 0;
	auto fAddHairPoint = [&hairIdx,points,lxUvs,thicknessData](const Vector3 &p,const Vector2 &uv,float thickness) {
		points[hairIdx] = {p.x,p.y,p.z};
		lxUvs[hairIdx] = {uv.x,uv.y};
		thicknessData[hairIdx] = thickness;

		++hairIdx;
	};
	for(auto i=decltype(hairData.hairPoints.size()){0u};i<hairData.hairPoints.size();++i)
	{
		auto &p = hairData.hairPoints[i];
		auto &uv = hairData.hairUvs[i];
		auto &faceNormal = hairData.hairNormals[i];

		auto hairStrength = hairConfig.defaultHairStrength;
		auto length = hairConfig.defaultLength;
		auto randomHairLengthFactor = hairConfig.randomHairLengthFactor;
		auto thickness = hairConfig.defaultThickness;

		const Vector3 gravity {0.f,-1.f,0.f};
		const Vector3 flowNormal = gravity;
		auto hairNormal = calc_hair_normal(flowNormal,faceNormal);
		hairNormal = faceNormal *hairStrength +(1.f -hairStrength) *hairNormal;
		uvec::normalize(&hairNormal);

		//hairPoints.push_back(p);
		//hairPoints.push_back(p +n *length); // TODO: Take adjacent face normals into account for hair direction?
		auto hairLength = length *(1.f -randomHairLengthFactor) +length *randomHairLengthFactor *umath::random(0.f,1.f);

		fAddHairPoint(p,uv,thickness);
		for(auto j=decltype(numSegments){0u};j<numSegments;++j)
		{
			auto f = (j +1) /static_cast<float>(numSegments);
			//auto p0 = (j > 0) ? hairPoints.back() : p;
			auto p1 = p +hairNormal *f *hairLength;
			// TODO: Apply modifier to p1
			fAddHairPoint(p1,uv,(1.f -f) *thickness);
		}
	}
	return hairFile;
}

struct LuxNodeCache
{
	LuxNodeCache(uint32_t numNodes)
	{
		nodeNames.resize(numNodes);
		convertedNodes.resize(numNodes);
		propertyCache.resize(numNodes);
	}
	const std::string &GetNodeName(const unirender::NodeDesc &node) const {return nodeNames[node.GetIndex()];}
	bool WasNodeConverted(const unirender::NodeDesc &node) const {return convertedNodes[node.GetIndex()];}
	void SetNodeName(uint32_t idx,const std::string &name) {nodeNames[idx] = name;}
	void AddToCache(const unirender::NodeDesc &node,const std::optional<luxrays::Properties> &props)
	{
		auto idx = node.GetIndex();
		convertedNodes[idx] = true;
		propertyCache[idx] = props;
	}
	std::optional<luxrays::Properties> *GetCachedProperties(const unirender::NodeDesc &node)
	{
		auto idx = node.GetIndex();
		return convertedNodes[idx] ? &propertyCache[idx] : nullptr;
	}
	std::vector<std::string> nodeNames;
	std::vector<bool> convertedNodes;
	std::vector<std::optional<luxrays::Properties>> propertyCache;
};

class LuxNodeManager
{
public:
	using NodeFactory = std::function<std::optional<luxrays::Properties>(luxcore::Scene&,unirender::GroupNodeDesc&,unirender::NodeDesc&,LuxNodeCache&)>;
	static std::optional<luxrays::Property> datavalue_to_property(LuxNodeCache &nodeCache,const unirender::DataValue &dataValue,const std::string &propName);
	static std::optional<luxrays::Property> socket_to_property(LuxNodeCache &nodeCache,const unirender::Socket &socket,const std::string &propName,bool includeTexturePrefix=true);
	static std::optional<luxrays::Property> socket_to_property(LuxNodeCache &nodeCache,unirender::NodeDesc &node,const std::string &socketName,const std::string &propName);
	static unirender::Socket *find_socket_linked_to_input(unirender::GroupNodeDesc &rootNode,const unirender::Socket &outputSocket);
	static unirender::Socket *find_socket_linked_to_input(unirender::GroupNodeDesc &rootNode,unirender::NodeDesc &node,const std::string &inputSocketName);
	NodeFactory *GetFactory(const std::string &nodeName)
	{
		auto it = m_factories.find(nodeName);
		return (it != m_factories.end()) ? &it->second : nullptr;
	}
	std::optional<luxrays::Properties> ConvertNode(luxcore::Scene &scene,unirender::GroupNodeDesc &rootNode,unirender::NodeDesc &node,LuxNodeCache &nodeCache);
	void RegisterFactory(const std::string &nodeName,const NodeFactory &factory) {m_factories[nodeName] = factory;}
	void Initialize();
private:
	bool ConvertSocketLinkedToInputToProperty(
		luxrays::Properties &props,luxcore::Scene &scene,unirender::GroupNodeDesc &rootNode,LuxNodeCache &nodeCache,unirender::NodeDesc &node,
		const std::string &inputSocketName,const std::string &propName,bool includeTexturePrefix=true
	);
	std::unordered_map<std::string,NodeFactory> m_factories {};
	bool m_initialized = false;
};

static LuxNodeManager g_luxNodeManager {};
static LuxNodeManager &get_lux_node_manager()
{
	g_luxNodeManager.Initialize();
	return g_luxNodeManager;
}

unirender::Socket *LuxNodeManager::find_socket_linked_to_input(unirender::GroupNodeDesc &rootNode,const unirender::Socket &outputSocket)
{
	auto &links = rootNode.GetLinks();
	auto it = std::find_if(links.begin(),links.end(),[&outputSocket](const unirender::NodeDescLink &link) {
		return link.toSocket == outputSocket;
	});
	if(it == links.end() || it->fromSocket.IsValid() == false)
		return nullptr;
	return const_cast<unirender::Socket*>(&it->fromSocket);
}

unirender::Socket *LuxNodeManager::find_socket_linked_to_input(unirender::GroupNodeDesc &rootNode,unirender::NodeDesc &node,const std::string &inputSocketName)
{
	auto sock = node.GetInputOrProperty(inputSocketName);
	return find_socket_linked_to_input(rootNode,sock);
}

bool LuxNodeManager::ConvertSocketLinkedToInputToProperty(
	luxrays::Properties &props,luxcore::Scene &scene,unirender::GroupNodeDesc &rootNode,LuxNodeCache &nodeCache,unirender::NodeDesc &node,
	const std::string &inputSocketName,const std::string &propName,bool includeTexturePrefix
)
{
	auto *fromSocket = find_socket_linked_to_input(rootNode,node,inputSocketName);
	auto socketToProperty = [&nodeCache,&propName](unirender::NodeDesc &node,const std::string &socketName,bool output=false) -> std::optional<luxrays::Property> {
		auto *inputSocketDesc = output ? node.FindOutputSocketDesc(socketName) : node.FindInputOrPropertyDesc(socketName);
		if(inputSocketDesc == nullptr)
			return {};
		return datavalue_to_property(nodeCache,inputSocketDesc->dataValue,propName);
	};
	if(fromSocket == nullptr)
	{
		auto prop = socketToProperty(node,inputSocketName);
		if(prop.has_value() == false)
			return false;
		props<<*prop;
		return true;
	}
	if(fromSocket->IsNodeSocket())
	{
		std::string fromSocketName;
		auto *node = fromSocket->GetNode(fromSocketName);
		if(node)
		{
			if(node->IsGroupNode())
			{
				auto *groupNode = static_cast<unirender::GroupNodeDesc*>(node);
				auto &links = groupNode->GetLinks();
				auto it = std::find_if(links.begin(),links.end(),[&fromSocket](const unirender::NodeDescLink &link) {
					return link.toSocket == *fromSocket;
				});
				if(it == links.end())
				{
					auto &links = rootNode.GetLinks();
					auto it = std::find_if(links.begin(),links.end(),[&fromSocket](const unirender::NodeDescLink &link) {
						return link.toSocket == *fromSocket;
					});
					if(it == links.end())
					{
						auto prop = socketToProperty(*groupNode,fromSocketName,fromSocket->IsOutputSocket());
						if(prop.has_value() == false)
							return false;
						props<<*prop;
						return true;
					}
					else
					{
						auto &link = *it;
						auto &groupFromSocket = link.fromSocket;
						std::string fromSocketName;
						auto *fromNode = groupFromSocket.GetNode(fromSocketName);
						return fromNode ? ConvertSocketLinkedToInputToProperty(props,scene,rootNode,nodeCache,*fromNode,fromSocketName,propName,includeTexturePrefix) : false;
					}
				}
				else
				{
					auto &link = *it;
					auto &groupFromSocket = link.fromSocket;
					auto *fromNode = groupFromSocket.GetNode();
					if(fromNode)
						ConvertNode(scene,*groupNode,*fromNode,nodeCache);
					fromSocket = const_cast<unirender::Socket*>(&link.fromSocket);
				}
			}
			else
			{
				auto prop = ConvertNode(scene,rootNode,*node,nodeCache);
				if(prop.has_value() == false)
					return false;
			}
		}
	}
	auto prop = socket_to_property(nodeCache,*fromSocket,propName,includeTexturePrefix);
	if(prop.has_value() == false)
		return false;
	props<<*prop;
	return true;
}

static luxrays::Matrix4x4 to_luxcore_matrix(const Mat4 &transform)
{
	return {
		transform[0][0],transform[0][1],transform[0][2],transform[0][3],
		transform[1][0],transform[1][1],transform[1][2],transform[1][3],
		transform[2][0],transform[2][1],transform[2][2],transform[2][3],
		transform[3][0],transform[3][1],transform[3][2],transform[3][3]
	};
}

std::optional<luxrays::Property> LuxNodeManager::datavalue_to_property(LuxNodeCache &nodeCache,const unirender::DataValue &dataValue,const std::string &propName)
{
	luxrays::Property prop {propName};
	switch(dataValue.type)
	{
	case unirender::SocketType::Bool:
		prop(*dataValue.ToValue<unirender::STBool>());
		break;
	case unirender::SocketType::Float:
		prop(*dataValue.ToValue<unirender::STFloat>());
		break;
	case unirender::SocketType::Int:
		prop(*dataValue.ToValue<unirender::STInt>());
		break;
	case unirender::SocketType::UInt:
		prop(*dataValue.ToValue<unirender::STUInt>());
		break;
	case unirender::SocketType::Color:
	case unirender::SocketType::Vector:
	case unirender::SocketType::Point:
	case unirender::SocketType::Normal:
	{
		static_assert(std::is_same_v<unirender::STColor,unirender::STVector>);
		static_assert(std::is_same_v<unirender::STColor,unirender::STPoint>);
		static_assert(std::is_same_v<unirender::STColor,unirender::STNormal>);
		auto &col = *dataValue.ToValue<unirender::STColor>();
		prop(col.x,col.y,col.z);
		break;
	}
	case unirender::SocketType::Point2:
	{
		auto &col = *dataValue.ToValue<unirender::STPoint2>();
		prop(col.x,col.y);
		break;
	}
	case unirender::SocketType::Closure:
		return {}; // Unsupported
	case unirender::SocketType::String:
		prop(*dataValue.ToValue<unirender::STString>());
		break;
	case unirender::SocketType::Enum:
		prop(*dataValue.ToValue<unirender::STEnum>());
		break;
	case unirender::SocketType::Transform:
	{
		//Mat4 m = *dataValue.ToValue<unirender::STTransform>();
		//auto lcm = to_luxcore_matrix(m);
		//prop(lcm);
		//break;
		return {}; // Unsupported
	}
	case unirender::SocketType::Node:
		return {}; // Unsupported
	case unirender::SocketType::FloatArray:
	{
		auto &floatArray = *dataValue.ToValue<unirender::STFloatArray>();
		prop(floatArray);
		break;
	}
	case unirender::SocketType::ColorArray:
	{
		auto &colorArray = *dataValue.ToValue<unirender::STColorArray>();
		std::vector<float> lxArray;
		lxArray.resize(colorArray.size() *sizeof(colorArray.front()) /sizeof(float));
		memcpy(lxArray.data(),colorArray.data(),colorArray.size() *sizeof(colorArray.front()));
		prop(lxArray);
		break;
	}
	}
	static_assert(umath::to_integral(unirender::SocketType::Count) == 16);
	return prop;
}

std::optional<luxrays::Property> LuxNodeManager::socket_to_property(LuxNodeCache &nodeCache,const unirender::Socket &socket,const std::string &propName,bool includeTexturePrefix)
{
	if(socket.IsConcreteValue())
	{
		auto val = socket.GetValue();
		if(val.has_value())
			return datavalue_to_property(nodeCache,*val,propName);
		return luxrays::Property{propName};
	}
	luxrays::Property prop {propName};
	auto *socketNode = socket.GetNode();
	if(socketNode == nullptr)
		return prop;
	std::string texPropName;
	if(includeTexturePrefix)
		texPropName = "scene.textures.";
	texPropName += nodeCache.GetNodeName(*socketNode);
	prop(texPropName);
	return prop;
}

std::optional<luxrays::Property> LuxNodeManager::socket_to_property(LuxNodeCache &nodeCache,unirender::NodeDesc &node,const std::string &socketName,const std::string &propName)
{
	auto socket = node.GetInputSocket(socketName);
	if(socket.IsValid() == false)
		return {};
	return socket_to_property(nodeCache,socket,propName);
}

std::optional<luxrays::Properties> LuxNodeManager::ConvertNode(luxcore::Scene &scene,unirender::GroupNodeDesc &rootNode,unirender::NodeDesc &node,LuxNodeCache &nodeCache)
{
	auto *props = nodeCache.GetCachedProperties(node);
	if(props)
		return *props;
	auto typeName = node.GetTypeName();
	auto *factory = GetFactory(typeName);
	if(factory == nullptr)
	{
		std::cout<<"WARNING: No shader node factory found for node type '"<<typeName<<"'!"<<std::endl;
		nodeCache.AddToCache(node,{});
		return {};
	}
	auto newProps = (*factory)(scene,rootNode,node,nodeCache);
	nodeCache.AddToCache(node,newProps);
	if(newProps.has_value())
		scene.Parse(*newProps);
	return newProps;
}

void LuxNodeManager::Initialize()
{
	if(m_initialized)
		return;
	m_initialized = true;

	RegisterFactory(unirender::NODE_MATH,[this](luxcore::Scene &scene,unirender::GroupNodeDesc &rootNode,unirender::NodeDesc &node,LuxNodeCache &nodeCache) -> std::optional<luxrays::Properties> {
		auto type = node.GetPropertyValue<unirender::STEnum>(unirender::nodes::math::IN_TYPE);
		if(type.has_value() == false)
			return {};
		std::string opName;
		switch(static_cast<unirender::nodes::math::MathType>(*type))
		{
		case unirender::nodes::math::MathType::Add:
			opName = "add";
			break;
		case unirender::nodes::math::MathType::Subtract:
			opName = "subtract";
			break;
		case unirender::nodes::math::MathType::Multiply:
			opName = "scale";
			break;
		case unirender::nodes::math::MathType::Divide:
			// TODO: Custom
			break;
		case unirender::nodes::math::MathType::Sine:
			break;
		case unirender::nodes::math::MathType::Cosine:
			break;
		case unirender::nodes::math::MathType::Tangent:
			break;
		case unirender::nodes::math::MathType::ArcSine:
			break;
		case unirender::nodes::math::MathType::ArcCosine:
			break;
		case unirender::nodes::math::MathType::ArcTangent:
			break;
		case unirender::nodes::math::MathType::Power:
			break;
		case unirender::nodes::math::MathType::Logarithm:
			break;
		case unirender::nodes::math::MathType::Minimum:
			break;
		case unirender::nodes::math::MathType::Maximum:
			break;
		case unirender::nodes::math::MathType::Round:
			break;
		case unirender::nodes::math::MathType::LessThan:
			break;
		case unirender::nodes::math::MathType::GreaterThan:
			break;
		case unirender::nodes::math::MathType::Modulo:
			break;
		case unirender::nodes::math::MathType::Absolute:
			break;
		case unirender::nodes::math::MathType::ArcTan2:
			break;
		case unirender::nodes::math::MathType::Floor:
			break;
		case unirender::nodes::math::MathType::Ceil:
			break;
		case unirender::nodes::math::MathType::Fraction:
			break;
		case unirender::nodes::math::MathType::Sqrt:
			break;
		case unirender::nodes::math::MathType::InvSqrt:
			break;
		case unirender::nodes::math::MathType::Sign:
			break;
		case unirender::nodes::math::MathType::Exponent:
			break;
		case unirender::nodes::math::MathType::Radians:
			break;
		case unirender::nodes::math::MathType::Degrees:
			break;
		case unirender::nodes::math::MathType::SinH:
			break;
		case unirender::nodes::math::MathType::CosH:
			break;
		case unirender::nodes::math::MathType::TanH:
			break;
		case unirender::nodes::math::MathType::Trunc:
			break;
		case unirender::nodes::math::MathType::Snap:
			break;
		case unirender::nodes::math::MathType::Wrap:
			break;
		case unirender::nodes::math::MathType::Compare:
			break;
		case unirender::nodes::math::MathType::MultiplyAdd:
			break;
		case unirender::nodes::math::MathType::PingPong:
			break;
		case unirender::nodes::math::MathType::SmoothMin:
			break;
		case unirender::nodes::math::MathType::SmoothMax:
			break;
		}
		if(opName.empty())
		{
			std::cout<<"WARNING: Math operation '"<<*type<<"' currently not supported for LuxCoreRender!"<<std::endl;
			return {};
		}
		auto nodeName = nodeCache.GetNodeName(node);
		std::string propName = "scene.textures." +nodeName;
		luxrays::Properties props {};
		props<<luxrays::Property{propName +".type"}(opName);
		
		ConvertSocketLinkedToInputToProperty(props,scene,rootNode,nodeCache,node,unirender::nodes::math::IN_VALUE1,propName +".texture1",false);
		ConvertSocketLinkedToInputToProperty(props,scene,rootNode,nodeCache,node,unirender::nodes::math::IN_VALUE2,propName +".texture2",false);

		// TODO: value3?
		return props;
	});
	RegisterFactory(unirender::NODE_VECTOR_MATH,[this](luxcore::Scene &scene,unirender::GroupNodeDesc &rootNode,unirender::NodeDesc &node,LuxNodeCache &nodeCache) -> std::optional<luxrays::Properties> {
		auto type = node.GetPropertyValue<unirender::STEnum>(unirender::nodes::vector_math::IN_TYPE);
		if(type.has_value() == false)
			return {};
		std::string opName;
		switch(static_cast<unirender::nodes::vector_math::MathType>(*type))
		{
		case unirender::nodes::vector_math::MathType::Add:
			opName = "add";
			break;
		case unirender::nodes::vector_math::MathType::Subtract:
			opName = "subtract";
			break;
		case unirender::nodes::vector_math::MathType::Multiply:
			opName = "scale";
			break;
		case unirender::nodes::vector_math::MathType::Divide:
			break;

		case unirender::nodes::vector_math::MathType::CrossProduct:
			break;
		case unirender::nodes::vector_math::MathType::Project:
			break;
		case unirender::nodes::vector_math::MathType::Reflect:
			break;
		case unirender::nodes::vector_math::MathType::DotProduct:
			break;

		case unirender::nodes::vector_math::MathType::Distance:
			break;
		case unirender::nodes::vector_math::MathType::Length:
			break;
		case unirender::nodes::vector_math::MathType::Scale:
			break;
		case unirender::nodes::vector_math::MathType::Normalize:
			break;

		case unirender::nodes::vector_math::MathType::Snap:
			break;
		case unirender::nodes::vector_math::MathType::Floor:
			break;
		case unirender::nodes::vector_math::MathType::Ceil:
			break;
		case unirender::nodes::vector_math::MathType::Modulo:
			break;
		case unirender::nodes::vector_math::MathType::Fraction:
			break;
		case unirender::nodes::vector_math::MathType::Absolute:
			break;
		case unirender::nodes::vector_math::MathType::Minimum:
			break;
		case unirender::nodes::vector_math::MathType::Maximum:
			break;
		};
		if(opName.empty())
		{
			std::cout<<"WARNING: Math operation '"<<*type<<"' currently not supported for LuxCoreRender!"<<std::endl;
			return {};
		}
		auto nodeName = nodeCache.GetNodeName(node);
		std::string propName = "scene.textures." +nodeName;
		luxrays::Properties props {};
		props<<luxrays::Property{propName +".type"}(opName);
		
		ConvertSocketLinkedToInputToProperty(props,scene,rootNode,nodeCache,node,unirender::nodes::vector_math::IN_VECTOR1,propName +".texture1",false);
		ConvertSocketLinkedToInputToProperty(props,scene,rootNode,nodeCache,node,unirender::nodes::vector_math::IN_VECTOR2,propName +".texture2",false);
		// TODO: vector3?
		return props;
	});
	RegisterFactory(unirender::NODE_OUTPUT,[this](luxcore::Scene &scene,unirender::GroupNodeDesc &rootNode,unirender::NodeDesc &node,LuxNodeCache &nodeCache) -> std::optional<luxrays::Properties> {
		auto *fromSocket = find_socket_linked_to_input(rootNode,node,unirender::nodes::output::IN_SURFACE);
		if(fromSocket == nullptr)
			return {};
		if(fromSocket->IsConcreteValue())
		{
			// TODO: Emission shader

		}
		else
		{
			auto *inputNode = fromSocket->GetNode();
			if(inputNode == nullptr)
				return {};
			return ConvertNode(scene,rootNode,*inputNode,nodeCache);
		}
		luxrays::Properties props {};
		return props;
	});
	RegisterFactory(unirender::NODE_PRINCIPLED_BSDF,[this](luxcore::Scene &scene,unirender::GroupNodeDesc &rootNode,unirender::NodeDesc &node,LuxNodeCache &nodeCache) -> std::optional<luxrays::Properties> {
		luxrays::Properties props {};
		std::string propName = "scene.materials." +nodeCache.GetNodeName(node);
		props<<luxrays::Property(propName +".type")("disney");
		ConvertSocketLinkedToInputToProperty(props,scene,rootNode,nodeCache,node,unirender::nodes::principled_bsdf::IN_BASE_COLOR,propName +".basecolor",false);
		ConvertSocketLinkedToInputToProperty(props,scene,rootNode,nodeCache,node,unirender::nodes::principled_bsdf::IN_ROUGHNESS,propName +".roughness",false);
		ConvertSocketLinkedToInputToProperty(props,scene,rootNode,nodeCache,node,unirender::nodes::principled_bsdf::IN_METALLIC,propName +".metallic",false);
		ConvertSocketLinkedToInputToProperty(props,scene,rootNode,nodeCache,node,unirender::nodes::principled_bsdf::IN_SUBSURFACE,propName +".subsurface",false);
		ConvertSocketLinkedToInputToProperty(props,scene,rootNode,nodeCache,node,unirender::nodes::principled_bsdf::IN_SPECULAR,propName +".specular",false);
		ConvertSocketLinkedToInputToProperty(props,scene,rootNode,nodeCache,node,unirender::nodes::principled_bsdf::IN_SPECULAR_TINT,propName +".speculartint",false);
		ConvertSocketLinkedToInputToProperty(props,scene,rootNode,nodeCache,node,unirender::nodes::principled_bsdf::IN_CLEARCOAT,propName +".clearcoat",false);
		// ConvertSocketLinkedToInputToProperty(props,scene,rootNode,nodeCache,node,unirender::nodes::principled_bsdf::IN_CLEARCOAT_ROUGHNESS,propName +".clearcoatgloss",false); // TODO
		ConvertSocketLinkedToInputToProperty(props,scene,rootNode,nodeCache,node,unirender::nodes::principled_bsdf::IN_ANISOTROPIC,propName +".anisotropic",false);
		ConvertSocketLinkedToInputToProperty(props,scene,rootNode,nodeCache,node,unirender::nodes::principled_bsdf::IN_SHEEN,propName +".sheen",false);
		ConvertSocketLinkedToInputToProperty(props,scene,rootNode,nodeCache,node,unirender::nodes::principled_bsdf::IN_SHEEN_TINT,propName +".sheentint",false);

		// ConvertSocketLinkedToInputToProperty(props,scene,rootNode,nodeCache,node,unirender::nodes::principled_bsdf::IN_ALPHA,propName +".transparency",false);
		return props;
	});
	/*RegisterFactory(unirender::NODE_SEPARATE_RGB,[this](luxcore::Scene &scene,unirender::GroupNodeDesc &rootNode,unirender::NodeDesc &node,LuxNodeCache &nodeCache) -> std::optional<luxrays::Properties> {
		luxrays::Properties props {};
		auto texName = nodeCache.GetNodeName(node);
		std::string propName = "scene.textures." +texName;

		const std::array<std::string,3> suffixes = {"_r","_g","_b"};
		const std::array<std::string,3> inputNames = {unirender::nodes::combine_rgb::IN_R,unirender::nodes::combine_rgb::IN_G,unirender::nodes::combine_rgb::IN_B};
		const std::array<Vector3,3> factors = {Vector3{1.f,0.f,0.f},Vector3{0.f,1.f,0.f},Vector3{0.f,0.f,1.f}};
		for(uint8_t i=0;i<3;++i)
		{
			auto propNameC = propName +suffixes[i];
			auto &factor = factors[i];
			props<<luxrays::Property(propNameC +".type")("scale");
			ConvertSocketLinkedToInputToProperty(props,scene,rootNode,nodeCache,node,inputNames[i],propNameC +".texture1",false);
			props<<luxrays::Property(propNameC +".texture2")(factor.x,factor.y,factor.z);
		}

		const std::string rgSuffix = "_rg";
		auto propNameRg = propName +rgSuffix;
		props<<luxrays::Property(propNameRg +".type")("add");
		props<<luxrays::Property(propNameRg +".texture1")(texName +suffixes[0]);
		props<<luxrays::Property(propNameRg +".texture2")(texName +suffixes[1]);

		auto propNameRgb = propName;
		props<<luxrays::Property(propNameRgb +".type")("add");
		props<<luxrays::Property(propNameRgb +".texture1")(texName +rgSuffix);
		props<<luxrays::Property(propNameRgb +".texture2")(texName +suffixes[2]);
		return props;
	});*/
	RegisterFactory(unirender::NODE_COMBINE_RGB,[this](luxcore::Scene &scene,unirender::GroupNodeDesc &rootNode,unirender::NodeDesc &node,LuxNodeCache &nodeCache) -> std::optional<luxrays::Properties> {
		luxrays::Properties props {};
		auto texName = nodeCache.GetNodeName(node);
		std::string propName = "scene.textures." +texName;

		const std::array<std::string,3> suffixes = {"_r","_g","_b"};
		const std::array<std::string,3> inputNames = {unirender::nodes::combine_rgb::IN_R,unirender::nodes::combine_rgb::IN_G,unirender::nodes::combine_rgb::IN_B};
		const std::array<Vector3,3> factors = {Vector3{1.f,0.f,0.f},Vector3{0.f,1.f,0.f},Vector3{0.f,0.f,1.f}};
		for(uint8_t i=0;i<3;++i)
		{
			auto propNameC = propName +suffixes[i];
			auto &factor = factors[i];
			props<<luxrays::Property(propNameC +".type")("scale");
			ConvertSocketLinkedToInputToProperty(props,scene,rootNode,nodeCache,node,inputNames[i],propNameC +".texture1",false);
			props<<luxrays::Property(propNameC +".texture2")(factor.x,factor.y,factor.z);
		}

		const std::string rgSuffix = "_rg";
		auto propNameRg = propName +rgSuffix;
		props<<luxrays::Property(propNameRg +".type")("add");
		props<<luxrays::Property(propNameRg +".texture1")(texName +suffixes[0]);
		props<<luxrays::Property(propNameRg +".texture2")(texName +suffixes[1]);

		auto propNameRgb = propName;
		props<<luxrays::Property(propNameRgb +".type")("add");
		props<<luxrays::Property(propNameRgb +".texture1")(texName +rgSuffix);
		props<<luxrays::Property(propNameRgb +".texture2")(texName +suffixes[2]);
		return props;
	});
	RegisterFactory(unirender::NODE_IMAGE_TEXTURE,[this](luxcore::Scene &scene,unirender::GroupNodeDesc &rootNode,unirender::NodeDesc &node,LuxNodeCache &nodeCache) -> std::optional<luxrays::Properties> {
		luxrays::Properties props {};
		std::string propName = "scene.textures." +nodeCache.GetNodeName(node);
		props<<luxrays::Property(propName +".type")("imagemap");
		ConvertSocketLinkedToInputToProperty(props,scene,rootNode,nodeCache,node,unirender::nodes::image_texture::IN_FILENAME,propName +".file");
		return props;
	});
}

////////////

extern "C" {
	std::shared_ptr<unirender::Renderer> __declspec(dllexport) test_luxcorerender(const unirender::Scene &scene)
	{
		return Renderer::Create(scene);
	}
};

std::shared_ptr<Renderer> Renderer::Create(const unirender::Scene &scene)
{
	auto renderer = std::shared_ptr<Renderer>{new Renderer{scene}};
	if(renderer->Initialize() == false)
		return nullptr;
	return renderer;
}

Renderer::~Renderer()
{
	StopRenderSession();
	m_lxSession = nullptr;
	m_lxConfig = nullptr;
	m_lxScene = nullptr;
}
void Renderer::StopRenderSession()
{
	if(m_lxSession == nullptr || m_lxSession->IsStarted() == false)
		return;
	m_lxSession->Stop();
}
std::string Renderer::GetName(const BaseObject &obj)
{
	auto name = obj.GetName();
	name += std::to_string(obj.GetId());
	return name;
}
std::string Renderer::GetName(const Object &obj,uint32_t shaderIdx)
{
	auto name = GetName(obj);
	return name +'_' +std::to_string(shaderIdx);
}
std::string Renderer::GetName(const Mesh &mesh,uint32_t shaderIdx)
{
	auto name = GetName(mesh);
	return name +'_' +std::to_string(shaderIdx);
}
void Renderer::Wait()
{
	StopRenderSession();
}
float Renderer::GetProgress() const
{return 1.f;}
void Renderer::Reset()
{
	StopRenderSession();
}
void Renderer::Restart()
{
	StopRenderSession();
}
void Renderer::FinalizeImage(uimg::ImageBuffer &imgBuf) {imgBuf.FlipVertically();}
util::EventReply Renderer::HandleRenderStage(RenderWorker &worker,unirender::Renderer::ImageRenderStage stage,StereoEye eyeStage,unirender::Renderer::RenderStageResult *optResult)
{
	auto reply = unirender::Renderer::HandleRenderStage(worker,stage,eyeStage,optResult);
	if(reply == util::EventReply::Handled)
		return reply;
	switch(stage)
	{
	case ImageRenderStage::InitializeScene:
	{
		worker.AddThread([this,&worker]() {
			//PrepareCyclesSceneForRendering();
			//Initialize(*m_scene);
			
			auto &cam = m_scene->GetCamera();
			auto stereoscopic = cam.IsStereoscopic();
			StartNextRenderStage(worker,unirender::Renderer::ImageRenderStage::Lighting,stereoscopic ? StereoEye::Left : StereoEye::None);
			worker.Start();
			return RenderStageResult::Continue;
		});
		break;
	}
	case ImageRenderStage::Lighting:
	{
		worker.AddThread([this,&worker,stage,eyeStage]() mutable {
			auto &createInfo = m_scene->GetCreateInfo();
			const unsigned int haltTime = m_lxSession->GetRenderConfig().GetProperties().Get(luxrays::Property("batch.halttime")(4'320'000)).Get<unsigned int>();
			const unsigned int haltSpp = m_lxSession->GetRenderConfig().GetProperties().Get(luxrays::Property("batch.haltspp")(createInfo.samples.has_value() ? *createInfo.samples : 0)).Get<unsigned int>();

			char buf[512];
			const auto &stats = m_lxSession->GetStats();
			while (!m_lxSession->HasDone()) {
				std::this_thread::sleep_for(std::chrono::milliseconds(1000));

				m_lxSession->UpdateStats();
				const double elapsedTime = stats.Get("stats.renderengine.time").Get<double>();
				const unsigned int pass = stats.Get("stats.renderengine.pass").Get<unsigned int>();
				const float convergence = stats.Get("stats.renderengine.convergence").Get<unsigned int>();
		
				// Print some information about the rendering progress
				sprintf(buf, "[Elapsed time: %3d/%dsec][Samples %4d/%d][Convergence %f%%][Avg. samples/sec % 3.2fM on %.1fK tris]",
						int(elapsedTime), int(haltTime), pass, haltSpp, 100.f * convergence,
						stats.Get("stats.renderengine.total.samplesec").Get<double>() / 1000000.0,
						stats.Get("stats.dataset.trianglecount").Get<double>() / 1000.0);

				LC_LOG(buf);
			}
			auto &film = m_lxSession->GetFilm();
			auto &resultImageBuffer = GetResultImageBuffer(eyeStage);
			film.SaveOutputs();
			resultImageBuffer = uimg::ImageBuffer::Create(film.GetWidth(),film.GetHeight(),uimg::ImageBuffer::Format::RGBA_FLOAT);
			try
			{
				film.GetOutput<float>(luxcore::Film::FilmOutputType::OUTPUT_RGBA,static_cast<float*>(resultImageBuffer->GetData()));
			}
			catch(const std::exception &e)
			{
				std::cout<<"EX: "<<e.what()<<std::endl;
			}

			if(UpdateStereoEye(worker,stage,eyeStage))
			{
				worker.Start(); // Lighting stage for the left eye is triggered by the user, but we have to start it manually for the right eye
				return RenderStageResult::Continue;
			}

			if(m_scene->ShouldDenoise() == false)
				return StartNextRenderStage(worker,ImageRenderStage::FinalizeImage,eyeStage);

			auto &albedoImageBuffer = GetAlbedoImageBuffer(eyeStage);
			auto &normalImageBuffer = GetNormalImageBuffer(eyeStage);
			albedoImageBuffer = uimg::ImageBuffer::Create(film.GetWidth(),film.GetHeight(),uimg::ImageBuffer::Format::RGB_FLOAT);
			normalImageBuffer = uimg::ImageBuffer::Create(film.GetWidth(),film.GetHeight(),uimg::ImageBuffer::Format::RGB_FLOAT);
			film.GetOutput<float>(luxcore::Film::FilmOutputType::OUTPUT_ALBEDO,static_cast<float*>(albedoImageBuffer->GetData()));
			film.GetOutput<float>(luxcore::Film::FilmOutputType::OUTPUT_SHADING_NORMAL,static_cast<float*>(normalImageBuffer->GetData()));
			return StartNextRenderStage(worker,ImageRenderStage::Denoise,eyeStage);
		});
		break;
	}
	}
	if(optResult)
		*optResult = unirender::Renderer::RenderStageResult::Continue;
	return util::EventReply::Handled;
}
bool Renderer::UpdateStereoEye(unirender::RenderWorker &worker,unirender::Renderer::ImageRenderStage stage,StereoEye &eyeStage)
{
	if(eyeStage == StereoEye::Left)
	{
		// Switch to right eye
		// TODO
		// m_cclScene->camera->stereo_eye = ccl::Camera::StereoEye::STEREO_RIGHT;
		// ReloadProgressiveRender(false);
		StartNextRenderStage(worker,stage,StereoEye::Right);
		return true;
	}
	else if(eyeStage == StereoEye::Right)
	{
		// TODO
		// Switch back to left eye and continue with next stage
		// m_cclScene->camera->stereo_eye = ccl::Camera::StereoEye::STEREO_LEFT;
		eyeStage = StereoEye::Left;
	}
	return false;
}
void Renderer::CloseRenderScene() {StopRenderSession();}
util::ParallelJob<std::shared_ptr<uimg::ImageBuffer>> Renderer::StartRender()
{
	auto job = util::create_parallel_job<RenderWorker>(*this);
	auto &worker = static_cast<RenderWorker&>(job.GetWorker());
	StartNextRenderStage(worker,ImageRenderStage::InitializeScene,StereoEye::None);
	return job;
}

bool Renderer::Initialize()
{
	PrepareCyclesSceneForRendering();
	luxcore::Init([](const char *msg) {
		std::cout<<"Luxcore: "<<msg<<std::endl;
	});
	std::unique_ptr<luxcore::Scene> lcScene{luxcore::Scene::Create()};
	if(lcScene == nullptr)
		return false;
	try
	{
		auto &sceneInfo = m_scene->GetSceneInfo();
		auto &skyTex = sceneInfo.sky;

		static Vector3 gain {0.1f,0.1f,0.1f};
		auto absPath = Scene::GetAbsSkyPath(skyTex);
		if(absPath.has_value())
		{
			lcScene->Parse(
				luxrays::Property("scene.lights.sky.type")("infinite")<<
				luxrays::Property("scene.lights.sky.file")(*absPath)<<
				luxrays::Property("scene.lights.sky.gain")(gain.x,gain.y,gain.z)<<
				luxrays::Property("scene.lights.sky.gamma")(1.0)
			);
		}

	/*lcScene->Parse(
		luxrays::Property("scene.lights.skyl.type")("sky2") <<
		luxrays::Property("scene.lights.skyl.dir")(0.166974f, 0.59908f, 0.783085f) <<
		luxrays::Property("scene.lights.skyl.turbidity")(2.2f) <<
		luxrays::Property("scene.lights.skyl.gain")(0.8f, 0.8f, 0.8f) <<
		luxrays::Property("scene.lights.sunl.type")("sun") <<
		luxrays::Property("scene.lights.sunl.dir")(0.166974f, 0.59908f, 0.783085f) <<
		luxrays::Property("scene.lights.sunl.turbidity")(2.2f) <<
		luxrays::Property("scene.lights.sunl.gain")(0.8f, 0.8f, 0.8f)
	);*/
	
	m_lxScene = std::move(lcScene);
	SyncCamera(m_scene->GetCamera());
	auto &mdlCache = m_renderData.modelCache;
	for(auto &light : m_scene->GetLights())
		SyncLight(*light);
	for(auto &chunk : mdlCache->GetChunks())
	{
		for(auto &o : chunk.GetMeshes())
			SyncMesh(*o);
	}
	m_shaderNodeIdx = 0;
	for(auto &chunk : mdlCache->GetChunks())
	{
		for(auto &o : chunk.GetObjects())
			SyncObject(*o);
	}

	//RTPATHCPU
	auto &createInfo = m_scene->GetCreateInfo();
	auto resolution = m_scene->GetResolution();

	luxrays::Properties props {};

	std::string renderEngineType = "PATHCPU";
	switch(createInfo.deviceType)
	{
	case unirender::Scene::DeviceType::GPU:
		renderEngineType = "PATHOCL";
		break;
	}

	props<<luxrays::Property("renderengine.type")(renderEngineType);
	props<<luxrays::Property("sampler.type")("SOBOL");
	props<<luxrays::Property("batch.halttime")(4'320'000);
	props<<luxrays::Property("context.cuda.optix.enable")("1");
	props<<luxrays::Property("film.outputs.1.type")("RGBA");
	props<<luxrays::Property("film.outputs.1.filename")("output.exr"); // We don't actually save the image, but we still need to specify a format
	props<<luxrays::Property("film.outputs.2.type")("ALBEDO");
	props<<luxrays::Property("film.outputs.2.filename")("albedo.exr");
	props<<luxrays::Property("film.outputs.3.type")("SHADING_NORMAL");
	props<<luxrays::Property("film.outputs.3.filename")("normal.exr");
	//props<<luxrays::Property("film.outputs.1.type")("RGB_IMAGEPIPELINE");
	//props<<luxrays::Property("film.outputs.1.filename")("image.png"); // We don't actually save the image, but we still need to specify a format
	if(createInfo.samples.has_value())
	{
		props<<luxrays::Property("batch.haltspp")(*createInfo.samples);
		props<<luxrays::Property("film.spp")(*createInfo.samples);
	}
	props<<luxrays::Property("film.width")(resolution.x);
	props<<luxrays::Property("film.height")(resolution.y);
	std::unique_ptr<luxcore::RenderConfig> lcConfig {luxcore::RenderConfig::Create(
		props,
		m_lxScene.get()
	)};
	std::unique_ptr<luxcore::RenderSession> lcSession {luxcore::RenderSession::Create(lcConfig.get())};
	if(lcSession == nullptr)
		return false;
	lcSession->Start();

	m_lxConfig = std::move(lcConfig);
	m_lxSession = std::move(lcSession);

	// Stop the rendering
	
	//lcScene->
	}
	catch(const std::exception &e)
	{
		std::cout<<"Ex: "<<e.what()<<std::endl;
		return false;
	}
	return true;
}

Renderer::Renderer(const Scene &scene)
	: unirender::Renderer{scene}
{}

void Renderer::SyncCamera(const unirender::Camera &cam)
{
	SyncFilm(cam);
	auto &lcCam = m_lxScene->GetCamera();

	std::string lcCamType = "perspective";
	switch(cam.GetType())
	{
	case unirender::Camera::CameraType::Perspective:
		lcCamType = "perspective";
		break;
	case unirender::Camera::CameraType::Orthographic:
		lcCamType = "orthographic";
		break;
	case unirender::Camera::CameraType::Panorama:
		lcCamType = "environment";
		break;
	}
	
	auto &pose = cam.GetPose();
	auto &pos = pose.GetOrigin();
	auto up = uquat::up(pose.GetRotation());
	auto target = pos +uquat::forward(pose.GetRotation()) *100.f;
	std::string propName = "scene.camera";
	m_lxScene->Parse(
		luxrays::Property(propName +".type")(lcCamType)<<
		luxrays::Property(propName +".lookat.orig")(pos.x,pos.y,pos.z)<<
		luxrays::Property(propName +".lookat.target")(target.x,target.y,target.z)<<
		luxrays::Property(propName +".up")(up.x,up.y,up.z)<<
		luxrays::Property(propName +".fieldofview")(cam.GetFov())
		// luxrays::Property(propName +".screenwindow")(0.0,1.0,0.0,1.0)
		// luxrays::Property(propName +".shutteropen")(0.0)<<
		// luxrays::Property(propName +".shutterclose")(0.0)<<
		// luxrays::Property(propName +".autovolume.enable")(true)<<
		// luxrays::Property(propName +".volume")("")
	);
}

void Renderer::SyncFilm(const unirender::Camera &cam)
{
	std::string propName = "film";
	m_lxScene->Parse(
		luxrays::Property(propName +".width")(cam.GetWidth())<<
		luxrays::Property(propName +".height")(cam.GetHeight())
	);
}

static luxrays::Matrix4x4 to_luxcore_matrix(const umath::ScaledTransform &t) {return to_luxcore_matrix(t.ToMatrix());}

void Renderer::SyncObject(const unirender::Object &obj)
{
	// Unirender supports multiple shaders per mesh, but LuxCore does not,
	// so we'll have to create multiple objects (one per shader/material)

	auto &mesh = obj.GetMesh();

	// Shaders
	auto &luxNodeManager = get_lux_node_manager();
	auto &shaders = mesh.GetSubMeshShaders();
	for(auto i=decltype(shaders.size()){0u};i<shaders.size();++i)
	{
		std::string matName;
		auto desc = shaders.at(i)->GetActivePassNode();
		if(desc == nullptr)
			desc = GroupNodeDesc::Create(m_scene->GetShaderNodeManager()); // Just create a dummy node
		auto &nodes = desc->GetChildNodes();

		LuxNodeCache nodeCache {static_cast<uint32_t>(nodes.size())};
		for(auto i=decltype(nodes.size()){0u};i<nodes.size();++i)
		{
			auto &node = nodes[i];
			auto name = "node_" +node->GetName() +std::to_string(m_shaderNodeIdx++);
			nodeCache.SetNodeName(i,name);
		}
		for(auto &node : nodes)
		{
			if(node->GetOutputs().empty() == false)
				continue;
			auto &type = node->GetTypeName();
			auto *factory = luxNodeManager.GetFactory(type);
			if(factory)
			{
				auto properties = (*factory)(*m_lxScene,*desc,*node,nodeCache);
				if(properties.has_value())
				{
					auto &names = properties->GetAllNames();
					for(auto &name : names)
					{
						const auto *prefix = "scene.materials.";
						auto pos = name.find(prefix);
						if(pos == std::string::npos)
							continue;
						auto sub = name.substr(pos +strlen(prefix));
						pos = sub.find('.');
						if(pos != std::string::npos)
							sub = sub.substr(0,pos);
						matName = sub;
					}
					//["scene.materials.node_1.type"] = {name="scene.materials.node_1.type" values={[allocator]=allocator } }
				}
			}
			else
				std::cout<<"WARNING: Unsupported node type '"<<node->GetTypeName()<<"'!"<<std::endl;
		}
		/*for(auto &node : nodes)
		{
			auto &type = node->GetTypeName();
			auto *factory = luxNodeManager.GetFactory(type);
			if(factory)
			{
				auto properties = (*factory)(*node,nodeNames);
				if(properties.has_value())
					m_lxScene->Parse(*properties);
			}
			else
				std::cout<<"WARNING: Unsupported node type '"<<node->GetTypeName()<<"'!"<<std::endl;
		}*/
		/*auto cclShader = CCLShader::Create(*this,*desc);
		if(cclShader == nullptr)
			throw std::logic_error{"Mesh shader must never be NULL!"};
		if(cclShader)
			cclMesh->used_shaders[i] = **cclShader;*/
		if(matName.empty())
			continue;
		auto objName = GetName(obj,i);
		std::string propName = "scene.objects." +objName;
		auto shapeName = GetName(obj.GetMesh(),i);
		m_lxScene->Parse(
			luxrays::Property(propName +".material")(matName)<<
			luxrays::Property(propName +".shape")(shapeName)
			//luxrays::Property(propName +".transformation")(to_luxcore_matrix(obj.GetPose()))
			// luxrays::Property(propName +".id")("")<<
			// luxrays::Property(propName +".camerainvisible")(false)
		);

		auto &hairConfig = shaders.at(i)->GetHairConfig();
		if(hairConfig.has_value() == false)
			continue;
		propName = "scene.objects." +objName +"_strands";
		m_lxScene->Parse(
			luxrays::Property(propName +".material")(matName)<<
			luxrays::Property(propName +".shape")(shapeName +"_strands")
		);
	}

	//std::string propName = "scene.objects." +GetName(obj);

	//if(true)
	//	return;
	/*propName = "scene.objects." +GetName(obj) +"_strands";
	m_lxScene->Parse(
		luxrays::Property(propName +".material")("test_mat")<<
		luxrays::Property(propName +".shape")(GetName(obj.GetMesh()) +"_strands")
		//luxrays::Property(propName +".transformation")(to_luxcore_matrix(obj.GetPose()))
		// luxrays::Property(propName +".id")("")<<
		// luxrays::Property(propName +".camerainvisible")(false)
	);*/
}

void Renderer::SyncLight(const unirender::Light &light)
{
	std::string propName = "scene.lights." +GetName(light);
	auto color = light.GetColor();

	auto lightType = (light.GetType() == unirender::Light::Type::Spot) ? util::pragma::LightType::Spot : (light.GetType() == unirender::Light::Type::Directional) ? util::pragma::LightType::Directional : util::pragma::LightType::Point;
	auto watt = (lightType == util::pragma::LightType::Spot) ? ulighting::cycles::lumen_to_watt_spot(light.GetIntensity(),light.GetColor(),light.GetOuterConeAngle()) :
		(lightType == util::pragma::LightType::Point) ? ulighting::cycles::lumen_to_watt_point(light.GetIntensity(),light.GetColor()) :
		ulighting::cycles::lumen_to_watt_area(light.GetIntensity(),light.GetColor());

	//static float lightIntensityFactor = 10.f;
	//watt *= lightIntensityFactor;

	watt *= m_scene->GetLightIntensityFactor();
	watt *= ulighting::MAX_LIGHT_EFFICIENCY_EFFICACY;


	luxrays::Properties props {};
	props<<luxrays::Property{propName +".gain"}(color.r,color.g,color.b);
	switch(light.GetType())
	{
	case unirender::Light::Type::Spot:
	{
		auto &pos = light.GetPose().GetOrigin();
		auto target = pos +uquat::forward(light.GetPose().GetRotation()) *100.f;
		auto outerConeAngle = light.GetOuterConeAngle() /2.f;
		auto innerConeAngle = umath::min(light.GetInnerConeAngle() /2.f,outerConeAngle);
		props<<luxrays::Property(propName +".type")("spot");
		props<<luxrays::Property(propName +".position")(pos.x,pos.y,pos.z);
		props<<luxrays::Property(propName +".target")(target.x,target.y,target.z);
		props<<luxrays::Property(propName +".power")(watt);
		props<<luxrays::Property(propName +".efficency")(ulighting::MAX_LIGHT_EFFICIENCY_EFFICACY);
		props<<luxrays::Property(propName +".coneangle")(light.GetOuterConeAngle());
		props<<luxrays::Property(propName +".conedeltaangle")(outerConeAngle -innerConeAngle);
		break;
	}
	case unirender::Light::Type::Point:
	{
		auto &pos = light.GetPose().GetOrigin();
		auto dir = uquat::forward(light.GetPose().GetRotation());
		props<<luxrays::Property(propName +".type")("sphere");
		props<<luxrays::Property(propName +".position")(pos.x,pos.y,pos.z);
		props<<luxrays::Property(propName +".turbidity")(2.2);
		props<<luxrays::Property(propName +".relsize")(1.0);
		props<<luxrays::Property(propName +".dir")(dir.x,dir.y,dir.z);
		props<<luxrays::Property(propName +".power")(watt);
		props<<luxrays::Property(propName +".efficency")(ulighting::MAX_LIGHT_EFFICIENCY_EFFICACY);
		break;
	}
	case unirender::Light::Type::Directional:
	{
		auto &pos = light.GetPose().GetOrigin();
		auto target = pos +uquat::forward(light.GetPose().GetRotation()) *100.f;
		auto outerConeAngle = light.GetOuterConeAngle();
		auto innerConeAngle = umath::min(light.GetInnerConeAngle(),outerConeAngle);
		props<<luxrays::Property(propName +".type")("sun");
		props<<luxrays::Property(propName +".position")(pos.x,pos.y,pos.z);
		props<<luxrays::Property(propName +".power")(0.0);
		props<<luxrays::Property(propName +".efficency")(0.0);
		break;
	}
	}
	m_lxScene->Parse(props);
}

class LuxShaderNode
{
public:
	uint32_t nodeIndex = 0u;
private:
};

class LuxShader
{
public:
private:
	uint32_t m_curNode = 0;
};

void Renderer::SyncMesh(const unirender::Mesh &mesh)
{
	auto &shaders = mesh.GetSubMeshShaders();
	struct MeshInstanceData
	{
		void Reserve(uint32_t numTriangles)
		{
			triangles.reserve(numTriangles);
		}
		std::vector<uint32_t> triangles;
	};
	std::vector<MeshInstanceData> meshInstancePerShader;
	meshInstancePerShader.resize(shaders.size());
	auto &triShaders = mesh.GetShaders();
	auto numTris = triShaders.size();
	for(auto &list : meshInstancePerShader)
		list.Reserve(numTris);
	for(auto i=decltype(triShaders.size()){0u};i<triShaders.size();++i)
	{
		auto shaderIdx = triShaders[i];
		assert(shaderIdx < triangleIndicesPerShader.size());
		if(shaderIdx >= meshInstancePerShader.size())
			continue;
		meshInstancePerShader[shaderIdx].triangles.push_back(i);
	}

	auto &tris = mesh.GetTriangles();
	auto &verts = mesh.GetVertices();
	auto &uvs = mesh.GetPerVertexUvs();
	auto &normals = mesh.GetVertexNormals();
	for(auto iShader=decltype(meshInstancePerShader.size()){0u};iShader!=meshInstancePerShader.size();++iShader)
	{
		auto &meshInstance = meshInstancePerShader[iShader];
		std::vector<uint32_t> usedVerts {};
		usedVerts.resize(verts.size(),std::numeric_limits<uint32_t>::max());
		
		auto numMeshTris = meshInstance.triangles.size();
		auto *lxTris = reinterpret_cast<luxrays::Triangle*>(m_lxScene->AllocTrianglesBuffer(numMeshTris));
		auto &instanceTris = meshInstance.triangles;
		std::vector<uint32_t> meshVertIndices;
		meshVertIndices.reserve(verts.size());
		uint32_t meshTriIdx = 0;
		for(auto triIdx : instanceTris)
		{
			auto &meshTri = lxTris[meshTriIdx++];
			uint32_t i = 0;
			for(auto idx : {tris[triIdx *3],tris[triIdx *3 +1],tris[triIdx *3 +2]})
			{
				if(usedVerts[idx] == std::numeric_limits<uint32_t>::max())
				{
					meshVertIndices.push_back(idx);
					usedVerts[idx] = meshVertIndices.size() -1;
				}
				meshTri.v[i++] = usedVerts[idx];
			}
		}

		auto numMeshVerts = meshVertIndices.size();
		auto *points = reinterpret_cast<luxrays::Point*>(m_lxScene->AllocVerticesBuffer(numMeshVerts)); // Note: Will be freed by luxcore
		auto *lxUvs = new luxrays::UV[numMeshVerts]; // Note: Will be freed by luxcore
		auto *lxNormals = new luxrays::Normal[numMeshVerts]; // Note: Will be freed by luxcore
		for(auto i=decltype(numMeshVerts){0u};i<numMeshVerts;++i)
		{
			auto idx = meshVertIndices[i];

			auto &v = verts[idx];
			static_assert(sizeof(v) == sizeof(luxrays::Point));
			points[i] = *reinterpret_cast<const luxrays::Point*>(&v);

			auto &uv = uvs[idx];
			static_assert(sizeof(uv) == sizeof(luxrays::UV));
			lxUvs[i] = *reinterpret_cast<const luxrays::UV*>(&uv);

			auto &n = normals[idx];
			static_assert(sizeof(n) == sizeof(luxrays::Normal));
			lxNormals[i] = *reinterpret_cast<const luxrays::Normal*>(&n);
		}

		auto name = GetName(mesh,iShader);
		m_lxScene->DefineMesh(name,numMeshVerts,numMeshTris,reinterpret_cast<float*>(points),reinterpret_cast<unsigned int*>(lxTris),reinterpret_cast<float*>(lxNormals),reinterpret_cast<float*>(lxUvs),nullptr,nullptr);

		auto &shader = mesh.GetSubMeshShaders()[iShader];
		auto &hairConfig = shader->GetHairConfig();
		if(hairConfig.has_value() == false)
			continue;
		unirender::HairGenerator hairGenerator {};
		struct MeshInterface
			: public unirender::HairGenerator::MeshInterface
		{
			virtual uint32_t GetTriangleCount() const override {return getTriangleCount();}
			virtual uint32_t GetVertexCount() const override {return getVertexCount();}
			virtual std::array<uint32_t,3> GetTriangle(uint32_t triIdx) const override {return getTriangle(triIdx);}
			virtual const Vector3 &GetVertexPosition(uint32_t vertIdx) const override {return getVertexPosition(vertIdx);}
			virtual const Vector3 &GetVertexNormal(uint32_t vertIdx) const override {return getVertexNormal(vertIdx);}
			virtual const Vector2 &GetVertexUv(uint32_t vertIdx) const override {return getVertexUv(vertIdx);}

			std::function<uint32_t()> getTriangleCount = nullptr;
			std::function<uint32_t()> getVertexCount = nullptr;
			std::function<std::array<uint32_t,3>(uint32_t)> getTriangle = nullptr;
			std::function<const Vector3&(uint32_t)> getVertexPosition = nullptr;
			std::function<const Vector3&(uint32_t)> getVertexNormal = nullptr;
			std::function<const Vector2&(uint32_t)> getVertexUv = nullptr;
		};
		auto meshInterface = std::make_unique<MeshInterface>();
		meshInterface->getTriangleCount = [numMeshTris]() -> uint32_t {return numMeshTris;};
		meshInterface->getVertexCount = [numMeshVerts]() -> uint32_t {return numMeshVerts;};
		meshInterface->getTriangle = [lxTris](uint32_t triIdx) -> std::array<uint32_t,3> {
			return std::array<uint32_t,3>{lxTris[triIdx].v[0],lxTris[triIdx].v[1],lxTris[triIdx].v[2]};
		};
		meshInterface->getVertexPosition = [points](uint32_t vertIdx) -> const Vector3& {
			return reinterpret_cast<Vector3*>(points)[vertIdx];
		};
		meshInterface->getVertexNormal = [lxNormals](uint32_t vertIdx) -> const Vector3& {
			return reinterpret_cast<Vector3*>(lxNormals)[vertIdx];
		};
		meshInterface->getVertexUv = [lxUvs](uint32_t vertIdx) -> const Vector2& {
			return reinterpret_cast<Vector2*>(lxUvs)[vertIdx];
		};
		hairGenerator.SetMeshDataInterface(std::move(meshInterface));
		
		auto hairData = hairGenerator.Generate(hairConfig->hairPerSquareMeter);
		auto hairFile = generate_hair_file(*hairConfig,hairData);
		m_lxScene->DefineStrands(name +"_strands",*hairFile,luxcore::Scene::StrandsTessellationType::TESSEL_RIBBON_ADAPTIVE,12,0.0075,12,false /* bottomCap */,false /* topCap */,true /* useCameraPosition */);
	}

#if 0
	// Hair
	std::vector<double> triAreas;
	triAreas.reserve(numTris);
	double totalArea = 0.0;
	for(auto i=decltype(numTris){0u};i<numTris;++i)
	{
		auto idx0 = tris[i *3];
		auto idx1 = tris[i *3 +1];
		auto idx2 = tris[i *3 +2];
		auto &v0 = verts[idx0];
		auto &v1 = verts[idx1];
		auto &v2 = verts[idx2];
		auto area = uvec::calc_area_of_triangle(v0,v1,v2);
		triAreas.push_back(area);
		totalArea += area;
	}

	double prevArea = 0.0;
	for(auto i=decltype(triAreas.size()){0u};i<triAreas.size();++i)
	{
		auto &area = triAreas[i];
		area = prevArea +(area /totalArea);
		prevArea = area;
	}

	static uint32_t numHair = 1'000'000;
	uint32_t numSegments = 1;//4;
	static auto thickness = 0.1f;
	static auto length = 0.8f;
	static auto hairStrength = 0.2f;
	static auto randomHairLengthFactor = 0.5f;
	auto numPoints = numHair *(numSegments +1);
	static auto useVertUv = true;

	// TODO: UV coordinates for hair should be interpolated on the triangle, but that doesn't work properly (see useVertUv)
	// TODO: Implement random brownian force
	// TODO: Implement curvature
	// TODO: Implement clumping
	// TODO: Prevent strand collisions / duplicates (minimum distance between strands?)
	
	luxrays::cyHairFile hairFile {};
	hairFile.Initialize();
	hairFile.SetHairCount(numHair);
	hairFile.SetPointCount(numPoints);
	hairFile.SetArrays(CY_HAIR_FILE_POINTS_BIT | CY_HAIR_FILE_UVS_BIT | CY_HAIR_FILE_THICKNESS_BIT);
	hairFile.SetDefaultSegmentCount(numSegments);
	hairFile.SetDefaultThickness(thickness);
	
	lxUvs = reinterpret_cast<luxrays::UV*>(hairFile.GetUVsArray());
	auto *thicknessData = hairFile.GetThicknessArray();
	std::vector<Vector3> hairPoints {};
	hairPoints.reserve(numPoints);
	auto fAddHairPoint = [&hairPoints,lxUvs,thicknessData](const Vector3 &p,const Vector2 &uv,float thickness) {
		auto idx = hairPoints.size();
		hairPoints.push_back(p);
		lxUvs[idx] = {uv.x,uv.y};
		thicknessData[idx] = thickness;
	};
	for(auto i=decltype(numHair){0u};i<numHair;++i)
	{
		auto r = umath::random(0.f,1.f);
		auto idx = umath::floor(r *(triAreas.size() -1));
		int32_t sign = (r < triAreas[idx]) ? -1 : 1;
		while((r < triAreas[idx] || r > triAreas[idx +1]) && (idx +sign) >= 0 && (idx +sign) < triAreas.size())
			idx += sign;
	
		auto &v0 = verts[tris[idx *3]];
		auto &v1 = verts[tris[idx *3 +1]];
		auto &v2 = verts[tris[idx *3 +2]];
		auto p = uvec::calc_point_on_triangle(v0,v1,v2,umath::random(0.f,1.f),umath::random(0.f,1.f));
		auto faceNormal = uvec::calc_face_normal(v0,v1,v2);
		float u = 0.f;
		float v = 0.f;
		umath::geometry::calc_barycentric_coordinates(v0,v1,v2,p,u,v);
		if(useVertUv) // TODO
		{
			auto &uv = uvs[tris[idx *3]];
			u = uv.x;
			v = uv.y;
		}

		const Vector3 gravity {0.f,-1.f,0.f};
		const Vector3 flowNormal = gravity;
		auto hairNormal = calc_hair_normal(flowNormal,faceNormal);
		hairNormal = faceNormal *hairStrength +(1.f -hairStrength) *hairNormal;
		uvec::normalize(&hairNormal);

		//hairPoints.push_back(p);
		//hairPoints.push_back(p +n *length); // TODO: Take adjacent face normals into account for hair direction?
		auto hairLength = length *(1.f -randomHairLengthFactor) +length *randomHairLengthFactor *umath::random(0.f,1.f);

		Vector2 uv {u,v};
		fAddHairPoint(p,uv,thickness);
		for(auto j=decltype(numSegments){0u};j<numSegments;++j)
		{
			auto f = (j +1) /static_cast<float>(numSegments);
			//auto p0 = (j > 0) ? hairPoints.back() : p;
			auto p1 = p +hairNormal *f *hairLength;
			// TODO: Apply modifier to p1
			fAddHairPoint(p1,uv,(1.f -f) *thickness);
		}
	}

	points = reinterpret_cast<luxrays::Point*>(hairFile.GetPointsArray());
	memcpy(points,hairPoints.data(),hairPoints.size() *sizeof(hairPoints[0]));

	m_lxScene->DefineStrands(name +"_strands",hairFile,luxcore::Scene::StrandsTessellationType::TESSEL_RIBBON_ADAPTIVE,12,0.0075,12,false /* bottomCap */,false /* topCap */,true /* useCameraPosition */);
#endif

#if 0
	std::string propName = "scene.shapes." +mesh.GetName();
	m_scene->Parse(
		luxrays::Property(propName +".type")("mesh")<<
		luxrays::Property(propName +".shape")("")<<
		luxrays::Property(propName +".transformation")(m)<<
		// luxrays::Property(propName +".id")("")<<
		luxrays::Property(propName +".camerainvisible")(false)
	);
	//scene.shapes.<shape name>.type


	auto *cclMesh = new ccl::Mesh{};
	m_cclScene->meshes.push_back(cclMesh);
	m_meshToCcclMesh[&mesh] = cclMesh;

	cclMesh->name = mesh.GetName();
	cclMesh->reserve_mesh(mesh.GetVertexCount(),mesh.GetTriangleCount());
	for(auto &v : mesh.GetVertices())
		cclMesh->add_vertex(Scene::ToCyclesPosition(v));
	auto &tris = mesh.GetTriangles();
	auto &shaderIds = mesh.GetShaders();
	auto &smooth = mesh.GetSmooth();
	auto ntris = tris.size();
	for(auto i=decltype(ntris){0u};i<ntris;i+=3)
		cclMesh->add_triangle(tris[i],tris[i +1],tris[i +2],shaderIds[i /3],smooth[i /3]);

	auto fToFloat4 = [](const ccl::float3 &v) -> ccl::float4 {return ccl::float4{v.x,v.y,v.z,0.f};};
	initialize_attribute<Vector3,ccl::float4>(*cclMesh,ccl::ATTR_STD_VERTEX_NORMAL,mesh.GetVertexNormals(),[&fToFloat4](const Vector3 &v) -> ccl::float4 {return fToFloat4(Scene::ToCyclesNormal(v));});
	initialize_attribute<Vector2,ccl::float2>(*cclMesh,ccl::ATTR_STD_UV,mesh.GetUvs(),[](const Vector2 &v) -> ccl::float2 {return Scene::ToCyclesUV(v);});
	initialize_attribute<Vector3,ccl::float3>(*cclMesh,ccl::ATTR_STD_UV_TANGENT,mesh.GetUvTangents(),[](const Vector3 &v) -> ccl::float3 {return Scene::ToCyclesNormal(v);});
	initialize_attribute<float,float>(*cclMesh,ccl::ATTR_STD_UV_TANGENT_SIGN,mesh.GetUvTangentSigns(),[](const float &v) -> float {return v;});

	auto *attrT = cclMesh->attributes.add(ccl::ATTR_STD_UV_TANGENT);
	if(attrT)
		attrT->name = "orco" +Mesh::TANGENT_POSTFIX;

	auto *attrTS = cclMesh->attributes.add(ccl::ATTR_STD_UV_TANGENT_SIGN);
	if(attrTS)
		attrTS->name = "orco" +Mesh::TANGENT_SIGN_POSTIFX;

	if(mesh.HasAlphas())
	{
		auto &alphas = mesh.GetAlphas();
		cclMesh->attributes.add(ALPHA_ATTRIBUTE_TYPE);
		initialize_attribute<float,float>(*cclMesh,ALPHA_ATTRIBUTE_TYPE,*alphas,[](const float &v) -> float {return v;});
	}

	auto &shaders = mesh.GetSubMeshShaders();
	cclMesh->used_shaders.resize(shaders.size());
	for(auto i=decltype(shaders.size()){0u};i<shaders.size();++i)
	{
		auto desc = shaders.at(i)->GetActivePassNode();
		if(desc == nullptr)
			desc = GroupNodeDesc::Create(m_scene->GetShaderNodeManager()); // Just create a dummy node
		auto cclShader = CCLShader::Create(*this,*desc);
		if(cclShader == nullptr)
			throw std::logic_error{"Mesh shader must never be NULL!"};
		if(cclShader)
			cclMesh->used_shaders[i] = **cclShader;
	}

	// TODO: We should be using the tangent values from m_tangents / m_tangentSigns
	// but their coordinate system needs to be converted for Cycles.
	// For now we'll just re-compute the tangents here.
	compute_tangents(cclMesh,true,true);
#endif
}

void Renderer::SetCancelled(const std::string &msg)
{}
void Renderer::PrepareCyclesSceneForRendering()
{
	unirender::Renderer::PrepareCyclesSceneForRendering();
}
#pragma optimize("",on)
