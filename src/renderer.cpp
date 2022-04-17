/* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/.
*
* Copyright (c) 2021 Silverlan
*/

#include "unirender/luxcorerender/renderer.hpp"
#include <mathutil/umath_geometry.hpp>
#include <util_raytracing/camera.hpp>
#include <util_raytracing/mesh.hpp>
#include <util_raytracing/light.hpp>
#include <util_raytracing/shader.hpp>
#include <util_raytracing/model_cache.hpp>
#include <util_raytracing/color_management.hpp>
#include <util_raytracing/denoise.hpp>
#include <util_raytracing/subdivision.hpp>
#include <util_ocio.hpp>
#include <mathutil/umath_lighting.hpp>
#include <sharedutils/util_string.h>
#include <luxcore/luxcore.h>
#include <luxrays/core/geometry/matrix4x4.h>
#include <luxrays/core/geometry/point.h>
#include <luxrays/core/geometry/triangle.h>
#include <luxrays/core/geometry/uv.h>
#include <util_image_buffer.hpp>
#include <util_image.hpp>
#include <util_texture_info.hpp>
#include <sharedutils/util_path.hpp>
#include <sharedutils/scope_guard.h>
#include <fsys/filesystem.h>
#include <fsys/ifile.hpp>
#define TINYEXR_IMPLEMENTATION
#include "tinyexr.h"

using namespace unirender::luxcorerender;
#pragma optimize("",off)
#include <sharedutils/util_hair.hpp>
static Vector3 calc_hair_normal(const Vector3 &flowNormal,const Vector3 &faceNormal)
{
	auto hairNormal = flowNormal -uvec::project(flowNormal,faceNormal);
	auto l = uvec::length(hairNormal);
	if(l > 0.001f)
		hairNormal /= l;
	else
		hairNormal = faceNormal;
	return hairNormal;
}

static Vector3 apply_curvature(const Vector3 &baseHairDir,const Vector3 &surfaceTargetNormal,float curvature,float factor)
{
	if(uvec::distance(baseHairDir,surfaceTargetNormal) < 0.001f)
		return baseHairDir;
	auto f = factor *curvature;
	auto n = glm::slerp(baseHairDir,surfaceTargetNormal,f);
	uvec::normalize(&n); // Probably not needed
	return n;
}

static std::unique_ptr<luxrays::cyHairFile> generate_hair_file(const util::HairConfig &hairConfig,const util::HairData &hairData)
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

		Vector3 gravity {0.f,-1.f,0.f};
		auto lxGravity = Renderer::ToLuxPosition(gravity);
		gravity = {lxGravity.x,lxGravity.y,lxGravity.z};
		const Vector3 flowNormal = gravity;
		auto baseHairNormal = calc_hair_normal(flowNormal,faceNormal);
		auto hairNormal = faceNormal *hairStrength +(1.f -hairStrength) *baseHairNormal;
		uvec::normalize(&hairNormal);

		//hairPoints.push_back(p);
		//hairPoints.push_back(p +n *length); // TODO: Take adjacent face normals into account for hair direction?
		auto hairLength = length *(1.f -randomHairLengthFactor) +length *randomHairLengthFactor *umath::random(0.f,1.f);

		fAddHairPoint(p,uv,thickness);
		auto lenPerSegment = hairLength /static_cast<float>(numSegments);
		auto p0 = p;
		for(auto j=decltype(numSegments){0u};j<numSegments;++j)
		{
			auto f = (j +1) /static_cast<float>(numSegments);

			auto n = apply_curvature(hairNormal,baseHairNormal,hairConfig.curvature,f);
			//auto p0 = (j > 0) ? hairPoints.back() : p;
			auto p1 = p0 +n *lenPerSegment;
			fAddHairPoint(p1,uv,(1.f -f) *thickness);
			p0 = p1;
		}
	}
	return hairFile;
}

struct LuxNodeCache
{
	LuxNodeCache(uint32_t numNodes,uint32_t &shaderNodeIdx)
		: m_shaderNodeIdx{shaderNodeIdx}
	{
		nodeNames.reserve(numNodes);
		convertedNodes.reserve(numNodes);
		propertyCache.reserve(numNodes);
	}
	const std::string &GetNodeName(const unirender::NodeDesc &node) const
	{
		auto it = nodeNames.find(&node);
		if(it == nodeNames.end())
		{
			auto name = "node_" +node.GetName() +std::to_string(m_shaderNodeIdx++);
			it = nodeNames.insert(std::make_pair(&node,name)).first;
		}
		return it->second;		
	}
	bool WasLinkConverted(const unirender::Socket &inputSocket) const {return convertedNodes.find(inputSocket) != convertedNodes.end();}
	const std::string &GetLinkedSocketName(const unirender::Socket &inputSocket) const
	{
		auto it = convertedNodes.find(inputSocket);
		if(it != convertedNodes.end())
			return propertyCache[it->second].propName;
		static std::string empty_string {};
		return empty_string;
	}
	std::string ToLinkedSocketName(const unirender::Socket &inputSocket) const
	{
		std::string socketName;
		auto *node = inputSocket.GetNode(socketName);
		assert(node);
		return GetNodeName(*node) +'_' +socketName;
	}
	void AddToCache(const unirender::Socket &inputSocket,const std::optional<luxrays::Properties> &props,const std::string &propName)
	{
		if(WasLinkConverted(inputSocket))
			return;
		auto idx = propertyCache.size();
		convertedNodes.insert(std::make_pair(inputSocket,idx));
		if(propertyCache.size() == propertyCache.capacity())
			propertyCache.reserve(propertyCache.size() *1.1f +100);
		propertyCache.push_back(PropertyCache{props,propName});
	}
	std::optional<luxrays::Properties> *GetCachedProperties(const unirender::Socket &inputSocket)
	{
		auto it = convertedNodes.find(inputSocket);
		return (it != convertedNodes.end()) ? &propertyCache[it->second].props : nullptr;
	}
	struct PropertyCache
	{
		std::optional<luxrays::Properties> props {};
		std::string propName;
	};
	mutable std::unordered_map<const unirender::NodeDesc*,std::string> nodeNames;
	std::unordered_map<unirender::Socket,size_t> convertedNodes;
	std::vector<PropertyCache> propertyCache;
private:
	uint32_t &m_shaderNodeIdx;
};

class LuxNodeManager
{
public:
	using NodeFactory = std::function<std::optional<luxrays::Properties>(unirender::luxcorerender::Renderer&,unirender::GroupNodeDesc&,unirender::NodeDesc&,const unirender::Socket&,LuxNodeCache&)>;
	static std::optional<luxrays::Property> datavalue_to_property(LuxNodeCache &nodeCache,const unirender::DataValue &dataValue,const std::string &propName);
	static std::optional<luxrays::Property> socket_to_property(LuxNodeCache &nodeCache,const unirender::Socket &socket,const std::string &propName,bool includeTexturePrefix=true);
	static std::optional<luxrays::Property> socket_to_property(LuxNodeCache &nodeCache,unirender::NodeDesc &node,const std::string &socketName,const std::string &propName);
	static unirender::Socket *find_socket_linked_to_input(unirender::GroupNodeDesc &rootNode,const unirender::Socket &outputSocket);
	static unirender::Socket *find_socket_linked_to_input(unirender::GroupNodeDesc &rootNode,unirender::NodeDesc &node,const std::string &inputSocketName,bool output=false);
	NodeFactory *GetFactory(const std::string &nodeName)
	{
		auto it = m_factories.find(nodeName);
		return (it != m_factories.end()) ? &it->second : nullptr;
	}
	std::optional<luxrays::Properties> ConvertLinkedNode(unirender::luxcorerender::Renderer &renderer,unirender::GroupNodeDesc &rootNode,const unirender::Socket &inputSocket,LuxNodeCache &nodeCache);
	void RegisterFactory(const std::string &nodeName,const NodeFactory &factory) {m_factories[nodeName] = factory;}
	void Initialize();
private:
	bool ConvertSocketLinkedToInputToProperty(
		luxrays::Properties &props,unirender::luxcorerender::Renderer &renderer,unirender::GroupNodeDesc &rootNode,LuxNodeCache &nodeCache,unirender::NodeDesc &node,
		const std::string &inputSocketName,const std::string &propName,bool includeTexturePrefix=true,bool output=false,bool ignoreConcreteValueIfNoInputLinked=false
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

unirender::Socket *LuxNodeManager::find_socket_linked_to_input(unirender::GroupNodeDesc &rootNode,unirender::NodeDesc &node,const std::string &inputSocketName,bool output)
{
	auto sock = output ? node.GetOutputSocket(inputSocketName) : node.GetInputOrProperty(inputSocketName);
	return find_socket_linked_to_input(rootNode,sock);
}

bool LuxNodeManager::ConvertSocketLinkedToInputToProperty(
	luxrays::Properties &props,unirender::luxcorerender::Renderer &renderer,unirender::GroupNodeDesc &rootNode,LuxNodeCache &nodeCache,unirender::NodeDesc &node,
	const std::string &inputSocketName,const std::string &propName,bool includeTexturePrefix,bool output,bool ignoreConcreteValueIfNoInputLinked
)
{
	auto *fromSocket = find_socket_linked_to_input(rootNode,node,inputSocketName,output);
	auto socketToProperty = [&nodeCache,&propName](unirender::NodeDesc &node,const std::string &socketName,bool output=false) -> std::optional<luxrays::Property> {
		auto *inputSocketDesc = output ? node.FindOutputSocketDesc(socketName) : node.FindInputOrPropertyDesc(socketName);
		if(inputSocketDesc == nullptr)
			return {};
		return datavalue_to_property(nodeCache,inputSocketDesc->dataValue,propName);
	};
	if(fromSocket == nullptr)
	{
		if(ignoreConcreteValueIfNoInputLinked)
			return true;
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
			assert(!node->IsGroupNode());
			if(node->IsGroupNode())
				throw std::logic_error{"Found unresolved group node!"};
			auto prop = ConvertLinkedNode(renderer,rootNode,*fromSocket,nodeCache);
			if(prop.has_value() == false)
				return false;
		}
	}
	auto prop = socket_to_property(nodeCache,*fromSocket,propName,includeTexturePrefix);
	if(prop.has_value() == false)
		return false;
	props<<*prop;
	return true;
}

static luxrays::Property to_luxcore_matrix(const std::string &propName,const Mat4 &transform)
{
	luxrays::Property prop {propName};
	for(uint8_t i=0;i<4;++i)
	{
		for(uint8_t j=0;j<4;++j)
			prop.Add<float>(transform[i][j]);
	}
	return prop;
}
static luxrays::Property to_luxcore_matrix(const std::string &propName,const luxrays::Matrix4x4 &m)
{
	luxrays::Property prop {propName};
	for(uint8_t i=0;i<4;++i)
	{
		for(uint8_t j=0;j<4;++j)
			prop.Add<float>(m.m[i][j]);
	}
	return prop;
}
static luxrays::Property to_luxcore_matrix(const std::string &propName,const umath::ScaledTransform &t)
{
	auto tlux = unirender::luxcorerender::Renderer::ToLuxTransform(t,true);
	return to_luxcore_matrix(propName,tlux.ToMatrix());
}
static const luxrays::Vector &to_luxcore_vector(const luxrays::Normal &n)
{
	static_assert(sizeof(luxrays::Normal) == sizeof(luxrays::Vector));
	return reinterpret_cast<const luxrays::Vector&>(n);
}
static const luxrays::Vector &to_luxcore_vector(const luxrays::Point &p)
{
	static_assert(sizeof(luxrays::Normal) == sizeof(luxrays::Vector));
	return reinterpret_cast<const luxrays::Vector&>(p);
}

static luxrays::Property to_luxcore_list(const std::string &propName,const std::vector<std::string> &list)
{
	luxrays::Property prop {propName};
	for(auto &item : list)
		prop.Add<std::string>(item);
	return prop;
}

std::optional<luxrays::Property> LuxNodeManager::datavalue_to_property(LuxNodeCache &nodeCache,const unirender::DataValue &dataValue,const std::string &propName)
{
	if(dataValue.value == nullptr)
		return {};
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
		auto col = *dataValue.ToValue<unirender::STColor>();
		prop(col.x,col.y,col.z);
		break;
	}
	case unirender::SocketType::Point2:
	{
		auto col = *dataValue.ToValue<unirender::STPoint2>();
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
		auto floatArray = *dataValue.ToValue<unirender::STFloatArray>();
		prop(floatArray);
		break;
	}
	case unirender::SocketType::ColorArray:
	{
		auto colorArray = *dataValue.ToValue<unirender::STColorArray>();
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
	texPropName += nodeCache.GetLinkedSocketName(socket);
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

std::optional<luxrays::Properties> LuxNodeManager::ConvertLinkedNode(unirender::luxcorerender::Renderer &renderer,unirender::GroupNodeDesc &rootNode,const unirender::Socket &inputSocket,LuxNodeCache &nodeCache)
{
	auto *inputNode = inputSocket.GetNode();
	if(inputNode == nullptr)
		return {};
	auto *props = nodeCache.GetCachedProperties(inputSocket);
	if(props)
		return *props;
	auto typeName = inputNode->GetTypeName();
	auto *factory = GetFactory(typeName);
	if(factory == nullptr)
	{
		std::cout<<"WARNING: No shader node factory found for node type '"<<typeName<<"'!"<<std::endl;
		nodeCache.AddToCache(inputSocket,{},"");
		return {};
	}
	auto newProps = (*factory)(renderer,rootNode,*inputNode,inputSocket,nodeCache);
	nodeCache.AddToCache(inputSocket,newProps,nodeCache.ToLinkedSocketName(inputSocket));
	if(newProps.has_value())
		renderer.GetLuxScene().Parse(*newProps);
	return newProps;
}

static std::string math_type_to_luxcore_op(unirender::nodes::math::MathType mathType)
{
	std::string opName;
	switch(mathType)
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
		opName = "divide";
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
		opName = "power";
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
		opName = "lessthan";
		break;
	case unirender::nodes::math::MathType::GreaterThan:
		opName = "greaterthan";
		break;
	case unirender::nodes::math::MathType::Modulo:
		break;
	case unirender::nodes::math::MathType::Absolute:
		opName = "abs";
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
	return opName;
}

void LuxNodeManager::Initialize()
{
	if(m_initialized)
		return;
	m_initialized = true;

	RegisterFactory(unirender::NODE_MATH,[this](unirender::luxcorerender::Renderer &renderer,unirender::GroupNodeDesc &rootNode,unirender::NodeDesc &node,const unirender::Socket &outputSocket,LuxNodeCache &nodeCache) -> std::optional<luxrays::Properties> {
		auto type = node.GetPropertyValue<unirender::STEnum>(unirender::nodes::math::IN_TYPE);
		if(type.has_value() == false)
			return {};
		auto opName = math_type_to_luxcore_op(static_cast<unirender::nodes::math::MathType>(*type));
		if(opName.empty())
		{
			std::cout<<"WARNING: Math operation '"<<*type<<"' currently not supported for LuxCoreRender!"<<std::endl;
			return {};
		}
		auto nodeName = nodeCache.ToLinkedSocketName(outputSocket);
		auto propName = "scene.textures." +nodeName;
		luxrays::Properties props {};
		props<<luxrays::Property{propName +".type"}(opName);
		
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::math::IN_VALUE1,propName +".texture1",false);
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::math::IN_VALUE2,propName +".texture2",false);

		// TODO: value3?
		return props;
	});
	RegisterFactory(unirender::NODE_VECTOR_MATH,[this](unirender::luxcorerender::Renderer &renderer,unirender::GroupNodeDesc &rootNode,unirender::NodeDesc &node,const unirender::Socket &outputSocket,LuxNodeCache &nodeCache) -> std::optional<luxrays::Properties> {
		auto type = node.GetPropertyValue<unirender::STEnum>(unirender::nodes::vector_math::IN_TYPE);
		if(type.has_value() == false)
			return {};
		auto opName = math_type_to_luxcore_op(static_cast<unirender::nodes::math::MathType>(*type));
		if(opName.empty())
		{
			std::cout<<"WARNING: Math operation '"<<*type<<"' currently not supported for LuxCoreRender!"<<std::endl;
			return {};
		}
		auto nodeName = nodeCache.ToLinkedSocketName(outputSocket);
		auto propName = "scene.textures." +nodeName;
		luxrays::Properties props {};
		props<<luxrays::Property{propName +".type"}(opName);
		
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::vector_math::IN_VECTOR1,propName +".texture1",false);
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::vector_math::IN_VECTOR2,propName +".texture2",false);
		// TODO: vector3?
		return props;
	});
	RegisterFactory(unirender::NODE_OUTPUT,[this](unirender::luxcorerender::Renderer &renderer,unirender::GroupNodeDesc &rootNode,unirender::NodeDesc &node,const unirender::Socket &outputSocket,LuxNodeCache &nodeCache) -> std::optional<luxrays::Properties> {
		luxrays::Properties props {};
		auto hasVolume = false;
		auto *fromSocketVol = find_socket_linked_to_input(rootNode,node,unirender::nodes::output::IN_VOLUME);
		if(fromSocketVol)
		{
			auto propsVol = ConvertLinkedNode(renderer,rootNode,*fromSocketVol,nodeCache);
			if(propsVol)
			{
				luxrays::Property prop {"scene.materials." +renderer.GetCurrentShaderName() +".volume.interior"};
				prop(nodeCache.ToLinkedSocketName(*fromSocketVol));
				props<<prop;
				hasVolume = true;
			}
		}
		
		auto *fromSocket = find_socket_linked_to_input(rootNode,node,unirender::nodes::output::IN_SURFACE);
		if(fromSocket == nullptr)
		{
			if(hasVolume)
			{
				luxrays::Property prop {"scene.materials." +renderer.GetCurrentShaderName() +".type"};
				prop("null");
				props<<prop;
				renderer.GetLuxScene().Parse(props);
				return props;
			}
			return {};
		}
		if(fromSocket->IsConcreteValue())
		{
			// TODO: Emission shader
			return {};
		}
		auto *inputNode = fromSocket->GetNode();
		if(inputNode == nullptr)
			return {};

		auto tmpProps = ConvertLinkedNode(renderer,rootNode,*fromSocket,nodeCache);
		if(!tmpProps.has_value())
			return {};
		props<<*tmpProps;
		renderer.GetLuxScene().Parse(props);
		return props;
	});
	RegisterFactory(unirender::NODE_PRINCIPLED_BSDF,[this](unirender::luxcorerender::Renderer &renderer,unirender::GroupNodeDesc &rootNode,unirender::NodeDesc &node,const unirender::Socket &outputSocket,LuxNodeCache &nodeCache) -> std::optional<luxrays::Properties> {
		luxrays::Properties props {};
		auto propName = "scene.materials." +renderer.GetCurrentShaderName();

		auto opName = math_type_to_luxcore_op(unirender::nodes::math::MathType::Multiply);
		assert(!opName.empty());
		auto propNameSss = "scene.textures." +nodeCache.ToLinkedSocketName(outputSocket) +"_sss";
		props<<luxrays::Property(propNameSss +".type")(opName);
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::principled_bsdf::IN_SUBSURFACE,propNameSss +".texture1",false);
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::principled_bsdf::IN_SUBSURFACE_COLOR,propNameSss +".texture2",false);

		props<<luxrays::Property(propName +".type")("disney");
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::principled_bsdf::IN_BASE_COLOR,propName +".basecolor",false);
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::principled_bsdf::IN_ROUGHNESS,propName +".roughness",false);
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::principled_bsdf::IN_METALLIC,propName +".metallic",false);
		props<<luxrays::Property(propName +".subsurface")(nodeCache.ToLinkedSocketName(outputSocket) +"_sss");
		//props<<luxrays::Property(propName +".subsurface")("100 100 100");
		//ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::principled_bsdf::IN_SUBSURFACE,propName +".subsurface",false);
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::principled_bsdf::IN_SPECULAR,propName +".specular",false);
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::principled_bsdf::IN_SPECULAR_TINT,propName +".speculartint",false);
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::principled_bsdf::IN_CLEARCOAT,propName +".clearcoat",false);
		// ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::principled_bsdf::IN_CLEARCOAT_ROUGHNESS,propName +".clearcoatgloss",false); // TODO
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::principled_bsdf::IN_ANISOTROPIC,propName +".anisotropic",false);
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::principled_bsdf::IN_SHEEN,propName +".sheen",false);
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::principled_bsdf::IN_SHEEN_TINT,propName +".sheentint",false);
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::principled_bsdf::IN_EMISSION,propName +".emission",false);
		if(renderer.ShouldUseHairShader() == false) // Normal map causes hair to be black for some reason
			ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::principled_bsdf::IN_NORMAL,propName +".normaltex",false,false,true);
		if(renderer.ShouldUsePhotonGiCache())
			props<<luxrays::Property(propName +".photongi.enable")("1");
		// ConvertSocketLinkedToInputToProperty(props,scene,rootNode,nodeCache,node,unirender::nodes::principled_bsdf::IN_IOR,propName +".volume.interiorior",false);
		// ConvertSocketLinkedToInputToProperty(props,scene,rootNode,nodeCache,node,unirender::nodes::principled_bsdf::IN_IOR,propName +".volume.exteriorior",false);

		static float factor = 1.f;
		props<<luxrays::Property(propName +".emission.gain")(factor,factor,factor);

		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::principled_bsdf::IN_ALPHA,propName +".transparency",false);
		return props;
	});
	RegisterFactory(unirender::NODE_GLASS_BSDF,[this](unirender::luxcorerender::Renderer &renderer,unirender::GroupNodeDesc &rootNode,unirender::NodeDesc &node,const unirender::Socket &outputSocket,LuxNodeCache &nodeCache) -> std::optional<luxrays::Properties> {
		luxrays::Properties props {};
		auto propName = "scene.materials." +renderer.GetCurrentShaderName();
		props<<luxrays::Property(propName +".type")("glass");
		// 0.0031 is the dispersion for water and is only used for testing purposes (see https://wiki.luxcorerender.org/Glass_Material_IOR_and_Dispersion#Refractive_Index_and_Dispersion_Data)
		// This should be an input parameter!
		props<<luxrays::Property{propName +".cauchyc"}(0.0031);
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::glass_bsdf::IN_COLOR,propName +".kr",false);
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::glass_bsdf::IN_COLOR,propName +".kt",false);
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::glass_bsdf::IN_IOR,propName +".interiorior",false);
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::glass_bsdf::IN_IOR,propName +".exteriorior",false);
		if(renderer.ShouldUsePhotonGiCache())
			props<<luxrays::Property(propName +".photongi.enable")("1");

		// ConvertSocketLinkedToInputToProperty(props,scene,rootNode,nodeCache,node,unirender::nodes::principled_bsdf::IN_ALPHA,propName +".transparency",false);
		return props;
	});
	RegisterFactory(unirender::NODE_GLOSSY_BSDF,[this](unirender::luxcorerender::Renderer &renderer,unirender::GroupNodeDesc &rootNode,unirender::NodeDesc &node,const unirender::Socket &outputSocket,LuxNodeCache &nodeCache) -> std::optional<luxrays::Properties> {
		luxrays::Properties props {};
		auto propName = "scene.materials." +renderer.GetCurrentShaderName();
		props<<luxrays::Property(propName +".type")("glossytranslucent");
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::glossy_bsdf::IN_COLOR,propName +".kd",false);
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::glossy_bsdf::IN_ROUGHNESS,propName +".uroughness_bf",false);
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::glossy_bsdf::IN_ROUGHNESS,propName +".vroughness_bf",false);

		if(renderer.ShouldUseHairShader() == false) // Normal map causes hair to be black for some reason
			ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::glossy_bsdf::IN_NORMAL,propName +".normaltex",false,false,true);

		if(renderer.ShouldUsePhotonGiCache())
			props<<luxrays::Property(propName +".photongi.enable")("1");
		
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::glossy_bsdf::IN_ALPHA,propName +".transparency",false);
		return props;
	});
	RegisterFactory(unirender::NODE_VOLUME_CLEAR,[this](unirender::luxcorerender::Renderer &renderer,unirender::GroupNodeDesc &rootNode,unirender::NodeDesc &node,const unirender::Socket &outputSocket,LuxNodeCache &nodeCache) -> std::optional<luxrays::Properties> {
		luxrays::Properties props {};

		auto baseName = nodeCache.ToLinkedSocketName(outputSocket);
		auto volName = "scene.volumes." +baseName;
		props<<luxrays::Property(volName +".type")("clear");
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::volume_clear::IN_PRIORITY,volName +".priority",false);
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::volume_clear::IN_IOR,volName +".ior",false);
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::volume_clear::IN_ABSORPTION,volName +".absorption",false);
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::volume_clear::IN_EMISSION,volName +".emission",false);

		auto defWorldVol = node.GetPropertyValue<bool>(unirender::nodes::volume_homogeneous::IN_DEFAULT_WORLD_VOLUME);
		if(defWorldVol.has_value())
			renderer.SetDefaultWorldVolume(baseName);

		return props;
	});
	RegisterFactory(unirender::NODE_VOLUME_HOMOGENEOUS,[this](unirender::luxcorerender::Renderer &renderer,unirender::GroupNodeDesc &rootNode,unirender::NodeDesc &node,const unirender::Socket &outputSocket,LuxNodeCache &nodeCache) -> std::optional<luxrays::Properties> {
		luxrays::Properties props {};

		auto baseName = nodeCache.ToLinkedSocketName(outputSocket);
		auto volName = "scene.volumes." +baseName;
		props<<luxrays::Property(volName +".type")("homogeneous");
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::volume_homogeneous::IN_PRIORITY,volName +".priority",false);
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::volume_homogeneous::IN_IOR,volName +".ior",false);
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::volume_homogeneous::IN_ABSORPTION,volName +".absorption",false);
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::volume_homogeneous::IN_EMISSION,volName +".emission",false);

		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::volume_homogeneous::IN_SCATTERING,volName +".scattering",false);
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::volume_homogeneous::IN_ASYMMETRY,volName +".asymmetry",false);
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::volume_homogeneous::IN_MULTI_SCATTERING,volName +".multiscattering",false);
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::volume_homogeneous::IN_ABSORPTION_DEPTH,volName +".d",0.01f);

		auto defWorldVol = node.GetPropertyValue<bool>(unirender::nodes::volume_homogeneous::IN_DEFAULT_WORLD_VOLUME);
		if(defWorldVol.has_value())
			renderer.SetDefaultWorldVolume(baseName);

		return props;
	});
	RegisterFactory(unirender::NODE_VOLUME_HETEROGENEOUS,[this](unirender::luxcorerender::Renderer &renderer,unirender::GroupNodeDesc &rootNode,unirender::NodeDesc &node,const unirender::Socket &outputSocket,LuxNodeCache &nodeCache) -> std::optional<luxrays::Properties> {
		luxrays::Properties props {};

		auto baseName = nodeCache.ToLinkedSocketName(outputSocket);
		auto volName = "scene.volumes." +baseName;
		props<<luxrays::Property(volName +".type")("heterogeneous");
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::volume_heterogeneous::IN_PRIORITY,volName +".priority",false);
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::volume_heterogeneous::IN_IOR,volName +".ior",false);
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::volume_heterogeneous::IN_ABSORPTION,volName +".absorption",false);
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::volume_heterogeneous::IN_EMISSION,volName +".emission",false);

		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::volume_heterogeneous::IN_SCATTERING,volName +".scattering",false);
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::volume_heterogeneous::IN_ASYMMETRY,volName +".asymmetry",false);
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::volume_heterogeneous::IN_MULTI_SCATTERING,volName +".multiscattering",false);

		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::volume_heterogeneous::IN_STEP_SIZE,volName +".steps.size",false);
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::volume_heterogeneous::IN_STEP_MAX_COUNT,volName +".steps.maxcount",false);

		auto defWorldVol = node.GetPropertyValue<bool>(unirender::nodes::volume_homogeneous::IN_DEFAULT_WORLD_VOLUME);
		if(defWorldVol.has_value())
			renderer.SetDefaultWorldVolume(baseName);

		return props;
	});
	RegisterFactory(unirender::NODE_SEPARATE_XYZ,[this](unirender::luxcorerender::Renderer &renderer,unirender::GroupNodeDesc &rootNode,unirender::NodeDesc &node,const unirender::Socket &outputSocket,LuxNodeCache &nodeCache) -> std::optional<luxrays::Properties> {
		luxrays::Properties props {};

		uint32_t channel = 0;
		std::string outputSocketName;
		outputSocket.GetNode(outputSocketName);
		if(outputSocketName == unirender::nodes::separate_xyz::OUT_X)
			channel = 0;
		else if(outputSocketName == unirender::nodes::separate_xyz::OUT_Y)
			channel = 1;
		else if(outputSocketName == unirender::nodes::separate_xyz::OUT_Z)
			channel = 2;

		auto propName = "scene.textures." +nodeCache.ToLinkedSocketName(outputSocket);
		props<<luxrays::Property(propName +".type")("splitfloat3");
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::separate_xyz::IN_VECTOR,propName +".texture",false);

		props<<luxrays::Property(propName +".channel")(channel);
		return props;
	});
	RegisterFactory(unirender::NODE_SEPARATE_RGB,[this](unirender::luxcorerender::Renderer &renderer,unirender::GroupNodeDesc &rootNode,unirender::NodeDesc &node,const unirender::Socket &outputSocket,LuxNodeCache &nodeCache) -> std::optional<luxrays::Properties> {
		luxrays::Properties props {};
		auto propName = "scene.textures." +nodeCache.ToLinkedSocketName(outputSocket);
		props<<luxrays::Property(propName +".type")("splitfloat3");
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::separate_rgb::IN_COLOR,propName +".texture",false);

		uint32_t channel = 0;
		std::string outputSocketName;
		outputSocket.GetNode(outputSocketName);
		if(outputSocketName == unirender::nodes::separate_rgb::OUT_R)
			channel = 0;
		else if(outputSocketName == unirender::nodes::separate_rgb::OUT_G)
			channel = 1;
		else if(outputSocketName == unirender::nodes::separate_rgb::OUT_B)
			channel = 2;
		props<<luxrays::Property(propName +".channel")(channel);
		return props;

#if 0 // Obsolete
		luxrays::Properties props {};
		auto texName = nodeCache.GetNodeName(node);
		std::string propName = "scene.textures." +texName;

		const std::array<std::string,3> suffixes = {"_r","_g","_b"};
		const std::array<std::string,3> inputNames = {unirender::nodes::combine_rgb::IN_R,unirender::nodes::combine_rgb::IN_G,unirender::nodes::combine_rgb::IN_B};
		const std::array<Vector3,3> factors = {Vector3{1.f,0.f,0.f},Vector3{0.f,1.f,0.f},Vector3{0.f,0.f,1.f}};
		uint32_t outputSocketIdx = 0;
		std::string outputSocketName;
		outputSocket.GetNode(outputSocketName);
		if(outputSocketName == unirender::nodes::separate_rgb::OUT_R)
			outputSocketIdx = 0;
		else if(outputSocketName == unirender::nodes::separate_rgb::OUT_G)
			outputSocketIdx = 1;
		else if(outputSocketName == unirender::nodes::separate_rgb::OUT_B)
			outputSocketIdx = 2;

		auto propNameC = propName +suffixes[outputSocketIdx];
		props<<luxrays::Property(propNameC +".type")("scale");
		ConvertSocketLinkedToInputToProperty(props,scene,rootNode,nodeCache,node,unirender::nodes::separate_rgb::IN_COLOR,propNameC +".texture1",false);
		auto &factor = factors[outputSocketIdx];
		props<<luxrays::Property(propNameC +".texture2")(factor.x,factor.y,factor.z); // TODO: Not this! All three values should be same! How to swap?
		return props;
#endif
	});
	
	RegisterFactory(unirender::NODE_COMBINE_XYZ,[this](unirender::luxcorerender::Renderer &renderer,unirender::GroupNodeDesc &rootNode,unirender::NodeDesc &node,const unirender::Socket &outputSocket,LuxNodeCache &nodeCache) -> std::optional<luxrays::Properties> {
		luxrays::Properties props {};
		auto propName = "scene.textures." +nodeCache.ToLinkedSocketName(outputSocket);
		props<<luxrays::Property(propName +".type")("makefloat3");
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::combine_xyz::IN_X,propName +".texture1",false);
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::combine_xyz::IN_Y,propName +".texture2",false);
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::combine_xyz::IN_Z,propName +".texture3",false);
		return props;
	});
	RegisterFactory(unirender::NODE_COMBINE_RGB,[this](unirender::luxcorerender::Renderer &renderer,unirender::GroupNodeDesc &rootNode,unirender::NodeDesc &node,const unirender::Socket &outputSocket,LuxNodeCache &nodeCache) -> std::optional<luxrays::Properties> {
		luxrays::Properties props {};
		auto propName = "scene.textures." +nodeCache.ToLinkedSocketName(outputSocket);
		props<<luxrays::Property(propName +".type")("makefloat3");
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::combine_rgb::IN_R,propName +".texture1",false);
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::combine_rgb::IN_G,propName +".texture2",false);
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::combine_rgb::IN_B,propName +".texture3",false);
		return props;

#if 0 // Obsolete
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
#endif
	});
	RegisterFactory(unirender::NODE_IMAGE_TEXTURE,[this](unirender::luxcorerender::Renderer &renderer,unirender::GroupNodeDesc &rootNode,unirender::NodeDesc &node,const unirender::Socket &outputSocket,LuxNodeCache &nodeCache) -> std::optional<luxrays::Properties> {
		luxrays::Properties props {};
		auto propName = "scene.textures." +nodeCache.ToLinkedSocketName(outputSocket);
		props<<luxrays::Property(propName +".type")("imagemap");
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::image_texture::IN_FILENAME,propName +".file");

		auto sockColorSpace = node.GetInputOrProperty(unirender::nodes::image_texture::IN_COLORSPACE);
		auto val = sockColorSpace.GetValue();
		if(val.has_value())
		{
			auto colorSpace = val->ToValue<std::string>();
			if(colorSpace.has_value() && ustring::compare<std::string>(*colorSpace,unirender::nodes::image_texture::COLOR_SPACE_SRGB) == false)
				props<<luxrays::Property(propName +".gamma")(1);
		}

		std::string outputSocketName;
		outputSocket.GetNode(outputSocketName);
		if(outputSocketName == unirender::nodes::image_texture::OUT_ALPHA)
			props<<luxrays::Property(propName +".channel")("alpha");
		return props;
	});
	RegisterFactory(unirender::NODE_NORMAL_MAP,[this](unirender::luxcorerender::Renderer &renderer,unirender::GroupNodeDesc &rootNode,unirender::NodeDesc &node,const unirender::Socket &outputSocket,LuxNodeCache &nodeCache) -> std::optional<luxrays::Properties> {
		luxrays::Properties props {};
		auto texName = nodeCache.ToLinkedSocketName(outputSocket);
		auto propName = "scene.textures." +texName;
		props<<luxrays::Property(propName +".type")("scale");
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::normal_map::IN_COLOR,propName +".texture1");

		auto texNameStrength = texName +"_strength";
		auto propNameStrength = "scene.textures." +texNameStrength;
		props<<luxrays::Property(propNameStrength +".type")("makefloat3");
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::normal_map::IN_STRENGTH,propNameStrength +".texture1",false);
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::normal_map::IN_STRENGTH,propNameStrength +".texture2",false);
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::normal_map::IN_STRENGTH,propNameStrength +".texture3",false);

		props<<luxrays::Property(propName +".texture2")(texNameStrength);
		return props;
	});
	RegisterFactory(unirender::NODE_NORMAL_TEXTURE,[this](unirender::luxcorerender::Renderer &renderer,unirender::GroupNodeDesc &rootNode,unirender::NodeDesc &node,const unirender::Socket &outputSocket,LuxNodeCache &nodeCache) -> std::optional<luxrays::Properties> {
#if 0 // This doesn't seem to work?
		luxrays::Properties props {};
		auto propName = "scene.textures." +nodeCache.ToLinkedSocketName(outputSocket);
		props<<luxrays::Property(propName +".type")("normalmap");
		ConvertSocketLinkedToInputToProperty(props,scene,rootNode,nodeCache,node,unirender::nodes::normal_texture::IN_FILENAME,propName +".file");
		ConvertSocketLinkedToInputToProperty(props,scene,rootNode,nodeCache,node,unirender::nodes::normal_texture::IN_STRENGTH,propName +".scale");
#endif

		luxrays::Properties props {};
		auto propName = "scene.textures." +nodeCache.ToLinkedSocketName(outputSocket);
		props<<luxrays::Property(propName +".type")("imagemap");
		ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::normal_texture::IN_FILENAME,propName +".file");
		props<<luxrays::Property(propName +".gamma")(1);
		// TODO: Scale
		return props;
	});
}

////////////

extern "C" {
	bool __declspec(dllexport) create_renderer(const unirender::Scene &scene,unirender::Renderer::Flags flags,std::shared_ptr<unirender::Renderer> &outRenderer)
	{
		outRenderer = Renderer::Create(scene,flags);
		return outRenderer != nullptr;
	}
};

////////////

#define ENABLE_COORDINATE_SYSTEM_CONVERSION
Vector3 Renderer::ToPragmaPosition(const luxrays::Vector &pos)
{
#ifdef ENABLE_COORDINATE_SYSTEM_CONVERSION
	auto scale = util::pragma::units_to_metres(1.f);
	Vector3 prPos {pos.x,pos.z,-pos.y};
	prPos /= scale;
	return prPos;
#else
	return {pos.x,pos.y,pos.z};
#endif
}
luxrays::Vector Renderer::ToLuxVector(const Vector3 &v)
{
	return luxrays::Vector{v.x,v.y,v.z};
}
luxrays::Point Renderer::ToLuxPosition(const Vector3 &pos)
{
#ifdef ENABLE_COORDINATE_SYSTEM_CONVERSION
	auto scale = util::pragma::units_to_metres(1.f);
	luxrays::Point cpos {pos.x,-pos.z,pos.y};
	cpos *= scale;
	return cpos;
#else
	return luxrays::Point{pos.x,pos.y,pos.z};
#endif
}
luxrays::Normal Renderer::ToLuxNormal(const Vector3 &n)
{
#ifdef ENABLE_COORDINATE_SYSTEM_CONVERSION
	return luxrays::Normal{n.x,-n.z,n.y};
#else
	return {n.x,n.y,n.z};
#endif
}
luxrays::UV Renderer::ToLuxUV(const Vector2 &uv)
{
	return luxrays::UV{uv.x,uv.y};
}

umath::ScaledTransform Renderer::ToLuxTransform(const umath::ScaledTransform &t,bool applyRotOffset)
{
#ifdef ENABLE_COORDINATE_SYSTEM_CONVERSION
	//auto rot = t.GetRotation();
	//umath::swap(rot.y,rot.z);
	//umath::negate(rot.y);
	Vector3 axis;
	float angle;
	uquat::to_axis_angle(t.GetRotation(),axis,angle);
	axis = {axis.x,-axis.z,axis.y};
	auto rot = uquat::create(axis,angle);
	auto lxScale = ToLuxVector(t.GetScale());
	umath::swap(lxScale.y,lxScale.z);
	auto lxOrigin = ToLuxPosition(t.GetOrigin());
	return umath::ScaledTransform {Vector3{lxOrigin.x,lxOrigin.y,lxOrigin.z},rot,Vector3{lxScale.x,lxScale.y,lxScale.z}};
#else
	return t;
#endif
}
float Renderer::ToLuxLength(float len)
{
#ifdef ENABLE_COORDINATE_SYSTEM_CONVERSION
	auto scale = util::pragma::units_to_metres(1.f);
	return len *scale;
#else
	return len;
#endif
}

std::shared_ptr<Renderer> Renderer::Create(const unirender::Scene &scene,Flags flags)
{
	auto renderer = std::shared_ptr<Renderer>{new Renderer{scene,flags}};
	if(renderer->Initialize(flags) == false)
		return nullptr;
	return renderer;
}

std::shared_ptr<Renderer> Renderer::CreateResume()
{
	//auto path = util::Path::CreatePath(util::get_program_path()) +"frame.rsm";
	//m_lxSession->SaveResumeFile(path.GetString());
	//return true;
	return nullptr;
}

std::string Renderer::TranslateOutputTypeToLuxCoreRender(const std::string &type)
{
	if(type == OUTPUT_COLOR)
		return "RGBA";
	else if(type == OUTPUT_ALBEDO)
		return "ALBEDO";
	else if(type == OUTPUT_NORMAL)
		return "AVG_SHADING_NORMAL";
	else if(type == OUTPUT_DEPTH)
		return "DEPTH";
	else if(type == OUTPUT_ALPHA)
		return "ALPHA";
	else if(type == OUTPUT_GEOMETRY_NORMAL)
		return "GEOMETRY_NORMAL";
	else if(type == OUTPUT_SHADING_NORMAL)
		return "SHADING_NORMAL";
	else if(type == OUTPUT_DIRECT_DIFFUSE)
		return "DIRECT_DIFFUSE";
	else if(type == OUTPUT_DIRECT_DIFFUSE_REFLECT)
		return "DIRECT_DIFFUSE_REFLECT";
	else if(type == OUTPUT_DIRECT_DIFFUSE_TRANSMIT)
		return "DIRECT_DIFFUSE_TRANSMIT";
	else if(type == OUTPUT_DIRECT_GLOSSY)
		return "DIRECT_GLOSSY";
	else if(type == OUTPUT_DIRECT_GLOSSY_REFLECT)
		return "DIRECT_GLOSSY_REFLECT";
	else if(type == OUTPUT_DIRECT_GLOSSY_TRANSMIT)
		return "DIRECT_GLOSSY_TRANSMIT";
	else if(type == OUTPUT_EMISSION)
		return "EMISSION";
	else if(type == OUTPUT_INDIRECT_DIFFUSE)
		return "INDIRECT_DIFFUSE";
	else if(type == OUTPUT_INDIRECT_DIFFUSE_REFLECT)
		return "INDIRECT_DIFFUSE_REFLECT";
	else if(type == OUTPUT_INDIRECT_DIFFUSE_TRANSMIT)
		return "INDIRECT_DIFFUSE_TRANSMIT";
	else if(type == OUTPUT_INDIRECT_GLOSSY)
		return "INDIRECT_GLOSSY";
	else if(type == OUTPUT_INDIRECT_GLOSSY_REFLECT)
		return "INDIRECT_GLOSSY_REFLECT";
	else if(type == OUTPUT_INDIRECT_GLOSSY_TRANSMIT)
		return "INDIRECT_GLOSSY_TRANSMIT";
	else if(type == OUTPUT_INDIRECT_SPECULAR)
		return "INDIRECT_SPECULAR";
	else if(type == OUTPUT_INDIRECT_SPECULAR_REFLECT)
		return "INDIRECT_SPECULAR_REFLECT";
	else if(type == OUTPUT_INDIRECT_SPECULAR_TRANSMIT)
		return "INDIRECT_SPECULAR_TRANSMIT";
	else if(type == OUTPUT_UV)
		return "UV";
	else if(type == OUTPUT_IRRADIANCE)
		return "IRRADIANCE";
	else if(type == OUTPUT_NOISE)
		return "NOISE";
	else if(type == OUTPUT_CAUSTIC)
		return "CAUSTIC";
	return type.c_str();
}

uimg::Format Renderer::GetOutputFormat(Scene::RenderMode renderMode)
{
	switch(renderMode)
	{
	case Scene::RenderMode::RenderImage:
	case Scene::RenderMode::BakeDiffuseLighting:
		return uimg::Format::RGBA32;
	case Scene::RenderMode::SceneAlbedo:
	case Scene::RenderMode::SceneNormals:
	case Scene::RenderMode::GeometryNormal:
	case Scene::RenderMode::ShadingNormal:
	case Scene::RenderMode::DirectDiffuse:
	case Scene::RenderMode::DirectDiffuseReflect:
	case Scene::RenderMode::DirectDiffuseTransmit:
	case Scene::RenderMode::DirectGlossy:
	case Scene::RenderMode::DirectGlossyReflect:
	case Scene::RenderMode::DirectGlossyTransmit:
	case Scene::RenderMode::Emission:
	case Scene::RenderMode::IndirectDiffuse:
	case Scene::RenderMode::IndirectDiffuseReflect:
	case Scene::RenderMode::IndirectDiffuseTransmit:
	case Scene::RenderMode::IndirectGlossy:
	case Scene::RenderMode::IndirectGlossyReflect:
	case Scene::RenderMode::IndirectGlossyTransmit:
	case Scene::RenderMode::IndirectSpecular:
	case Scene::RenderMode::IndirectSpecularReflect:
	case Scene::RenderMode::IndirectSpecularTransmit:
	case Scene::RenderMode::Irradiance:
	case Scene::RenderMode::Caustic:
		return uimg::Format::RGB32;
	case Scene::RenderMode::Uv:
		return uimg::Format::RG32;
	case Scene::RenderMode::SceneDepth:
	case Scene::RenderMode::Noise:
	case Scene::RenderMode::Alpha:
		return uimg::Format::R32;
	}
	return uimg::Format::RGBA32;
}

luxcore::Film::FilmOutputType Renderer::GetLuxCoreFilmOutputType(Scene::RenderMode renderMode)
{
	switch(renderMode)
	{
	case Scene::RenderMode::RenderImage:
	case Scene::RenderMode::BakeDiffuseLighting:
		return luxcore::Film::FilmOutputType::OUTPUT_RGBA;
	case Scene::RenderMode::SceneAlbedo:
		return luxcore::Film::FilmOutputType::OUTPUT_ALBEDO;
	case Scene::RenderMode::SceneNormals:
		return luxcore::Film::FilmOutputType::OUTPUT_AVG_SHADING_NORMAL;
	case Scene::RenderMode::SceneDepth:
		return luxcore::Film::FilmOutputType::OUTPUT_DEPTH;
	case Scene::RenderMode::Alpha:
		return luxcore::Film::FilmOutputType::OUTPUT_ALPHA;
	case Scene::RenderMode::GeometryNormal:
		return luxcore::Film::FilmOutputType::OUTPUT_GEOMETRY_NORMAL;
	case Scene::RenderMode::ShadingNormal:
		return luxcore::Film::FilmOutputType::OUTPUT_SHADING_NORMAL;
	case Scene::RenderMode::DirectDiffuse:
		return luxcore::Film::FilmOutputType::OUTPUT_DIRECT_DIFFUSE;
	case Scene::RenderMode::DirectDiffuseReflect:
		return luxcore::Film::FilmOutputType::OUTPUT_DIRECT_DIFFUSE_REFLECT;
	case Scene::RenderMode::DirectDiffuseTransmit:
		return luxcore::Film::FilmOutputType::OUTPUT_DIRECT_DIFFUSE_TRANSMIT;
	case Scene::RenderMode::DirectGlossy:
		return luxcore::Film::FilmOutputType::OUTPUT_DIRECT_GLOSSY;
	case Scene::RenderMode::DirectGlossyReflect:
		return luxcore::Film::FilmOutputType::OUTPUT_DIRECT_GLOSSY_REFLECT;
	case Scene::RenderMode::DirectGlossyTransmit:
		return luxcore::Film::FilmOutputType::OUTPUT_DIRECT_GLOSSY_TRANSMIT;
	case Scene::RenderMode::Emission:
		return luxcore::Film::FilmOutputType::OUTPUT_EMISSION;
	case Scene::RenderMode::IndirectDiffuse:
		return luxcore::Film::FilmOutputType::OUTPUT_INDIRECT_DIFFUSE;
	case Scene::RenderMode::IndirectDiffuseReflect:
		return luxcore::Film::FilmOutputType::OUTPUT_INDIRECT_DIFFUSE_REFLECT;
	case Scene::RenderMode::IndirectDiffuseTransmit:
		return luxcore::Film::FilmOutputType::OUTPUT_INDIRECT_DIFFUSE_TRANSMIT;
	case Scene::RenderMode::IndirectGlossy:
		return luxcore::Film::FilmOutputType::OUTPUT_INDIRECT_GLOSSY;
	case Scene::RenderMode::IndirectGlossyReflect:
		return luxcore::Film::FilmOutputType::OUTPUT_INDIRECT_GLOSSY_REFLECT;
	case Scene::RenderMode::IndirectGlossyTransmit:
		return luxcore::Film::FilmOutputType::OUTPUT_INDIRECT_GLOSSY_TRANSMIT;
	case Scene::RenderMode::IndirectSpecular:
		return luxcore::Film::FilmOutputType::OUTPUT_INDIRECT_SPECULAR;
	case Scene::RenderMode::IndirectSpecularReflect:
		return luxcore::Film::FilmOutputType::OUTPUT_INDIRECT_SPECULAR_REFLECT;
	case Scene::RenderMode::IndirectSpecularTransmit:
		return luxcore::Film::FilmOutputType::OUTPUT_INDIRECT_SPECULAR_TRANSMIT;
	case Scene::RenderMode::Uv:
		return luxcore::Film::FilmOutputType::OUTPUT_UV;
	case Scene::RenderMode::Irradiance:
		return luxcore::Film::FilmOutputType::OUTPUT_IRRADIANCE;
	case Scene::RenderMode::Noise:
		return luxcore::Film::FilmOutputType::OUTPUT_NOISE;
	case Scene::RenderMode::Caustic:
		return luxcore::Film::FilmOutputType::OUTPUT_CAUSTIC;
	}
	return luxcore::Film::FilmOutputType::OUTPUT_RGBA;
}

std::string Renderer::GetOutputType(Scene::RenderMode renderMode)
{
	switch(renderMode)
	{
	case Scene::RenderMode::RenderImage:
		return OUTPUT_COLOR;
	case Scene::RenderMode::BakeDiffuseLighting:
		return OUTPUT_COLOR;
	case Scene::RenderMode::SceneAlbedo:
		return OUTPUT_ALBEDO;
	case Scene::RenderMode::SceneNormals:
		return OUTPUT_NORMAL;
	case Scene::RenderMode::SceneDepth:
		return OUTPUT_DEPTH;
	case Scene::RenderMode::Alpha:
		return OUTPUT_ALPHA;
	case Scene::RenderMode::GeometryNormal:
		return OUTPUT_GEOMETRY_NORMAL;
	case Scene::RenderMode::ShadingNormal:
		return OUTPUT_SHADING_NORMAL;
	case Scene::RenderMode::DirectDiffuse:
		return OUTPUT_DIRECT_DIFFUSE;
	case Scene::RenderMode::DirectDiffuseReflect:
		return OUTPUT_DIRECT_DIFFUSE_REFLECT;
	case Scene::RenderMode::DirectDiffuseTransmit:
		return OUTPUT_DIRECT_DIFFUSE_TRANSMIT;
	case Scene::RenderMode::DirectGlossy:
		return OUTPUT_DIRECT_GLOSSY;
	case Scene::RenderMode::DirectGlossyReflect:
		return OUTPUT_DIRECT_GLOSSY_REFLECT;
	case Scene::RenderMode::DirectGlossyTransmit:
		return OUTPUT_DIRECT_GLOSSY_TRANSMIT;
	case Scene::RenderMode::Emission:
		return OUTPUT_EMISSION;
	case Scene::RenderMode::IndirectDiffuse:
		return OUTPUT_INDIRECT_DIFFUSE;
	case Scene::RenderMode::IndirectDiffuseReflect:
		return OUTPUT_INDIRECT_DIFFUSE_REFLECT;
	case Scene::RenderMode::IndirectDiffuseTransmit:
		return OUTPUT_INDIRECT_DIFFUSE_TRANSMIT;
	case Scene::RenderMode::IndirectGlossy:
		return OUTPUT_INDIRECT_GLOSSY;
	case Scene::RenderMode::IndirectGlossyReflect:
		return OUTPUT_INDIRECT_GLOSSY_REFLECT;
	case Scene::RenderMode::IndirectGlossyTransmit:
		return OUTPUT_INDIRECT_GLOSSY_TRANSMIT;
	case Scene::RenderMode::IndirectSpecular:
		return OUTPUT_INDIRECT_SPECULAR;
	case Scene::RenderMode::IndirectSpecularReflect:
		return OUTPUT_INDIRECT_SPECULAR_REFLECT;
	case Scene::RenderMode::IndirectSpecularTransmit:
		return OUTPUT_INDIRECT_SPECULAR_TRANSMIT;
	case Scene::RenderMode::Uv:
		return OUTPUT_UV;
	case Scene::RenderMode::Irradiance:
		return OUTPUT_IRRADIANCE;
	case Scene::RenderMode::Noise:
		return OUTPUT_NOISE;
	case Scene::RenderMode::Caustic:
		return OUTPUT_CAUSTIC;
	}
	return "";
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
float Renderer::GetProgress() const {return m_progress;}
void Renderer::Reset()
{
	StopRenderSession();
}
void Renderer::Restart()
{
	StopRenderSession();
}
bool Renderer::Stop()
{
	m_lxSession->Stop();
	return true;
}
bool Renderer::Pause()
{
	m_lxSession->Pause();
	return true;
}
bool Renderer::Resume()
{
	m_lxSession->Resume();
	return true;
}
bool Renderer::Suspend()
{
	Pause();
	auto path = util::Path::CreatePath(util::get_program_path()) +"frame.rsm";
	m_lxSession->SaveResumeFile(path.GetString());
	return true;
}
void Renderer::UpdateProgressiveRender()
{
	auto renderMode = m_scene->GetRenderMode();
	if(renderMode != unirender::Scene::RenderMode::RenderImage)
		return;
	constexpr auto useFloatFormat = true;
	auto &scene = GetScene();
	auto resolution = scene.GetResolution();
	TileManager::TileData tileData {};
	tileData.x = 0;
	tileData.y = 0;
	tileData.w = resolution.x;
	tileData.h = resolution.y;
	tileData.index = 0;

	std::shared_ptr<uimg::ImageBuffer> imgBuf = nullptr;
	if(useFloatFormat)
	{
		tileData.flags |= TileManager::TileData::Flags::Initialized;
		auto format = uimg::Format::RGBA32;
		tileData.data.resize(resolution.x *resolution.y *uimg::ImageBuffer::GetPixelSize(format));
		imgBuf = uimg::ImageBuffer::Create(tileData.data.data(),resolution.x,resolution.y,format);
	}
	else
	{
		tileData.flags |= TileManager::TileData::Flags::HDRData | TileManager::TileData::Flags::Initialized;
		tileData.data.resize(resolution.x *resolution.y *uimg::ImageBuffer::GetPixelSize(uimg::Format::RGBA16));
		imgBuf = uimg::ImageBuffer::Create(resolution.x,resolution.y,uimg::Format::RGBA32);
	}
	m_lxSession->GetFilm().GetOutput<float>(luxcore::Film::FilmOutputType::OUTPUT_RGBA,reinterpret_cast<float*>(imgBuf->GetData()));
	imgBuf->FlipVertically();

	// Denoising is too slow for real-time use
	/*DenoiseInfo denoiseInfo {};
	denoiseInfo.width = imgBuf->GetWidth();
	denoiseInfo.height = imgBuf->GetHeight();
	denoiseInfo.hdr = true;
	denoise(denoiseInfo,*imgBuf);*/

	if(!useFloatFormat)
	{
		auto imgBufDst = uimg::ImageBuffer::Create(tileData.data.data(),resolution.x,resolution.y,uimg::Format::RGBA16);
		imgBuf->Convert(*imgBufDst);
	}

	const_cast<Renderer*>(this)->m_tileManager.ApplyPostProcessingForProgressiveTile(tileData);
	const_cast<Renderer*>(this)->m_tileManager.AddRenderedTile(std::move(tileData));
}
bool Renderer::BeginSceneEdit()
{
	if(!umath::is_flag_set(m_flags,Flags::EnableLiveEditing))
		return false;
	m_lxSession->BeginSceneEdit();
	return true;
}
bool Renderer::EndSceneEdit()
{
	if(!umath::is_flag_set(m_flags,Flags::EnableLiveEditing))
		return false;
	m_lxSession->EndSceneEdit();
	return true;
}
bool Renderer::SyncEditedActor(const util::Uuid &uuid)
{
	auto *actor = FindActor(uuid);
	if(!actor)
		return false;
	if(typeid(*actor) == typeid(Camera))
		SyncCamera(m_scene->GetCamera());
	else
		return false;
	return true;
}
bool Renderer::Export(const std::string &strPath)
{
	FileManager::CreatePath(strPath.c_str());
	auto path = util::Path::CreatePath(util::get_program_path()) +strPath;
	m_lxConfig->Export(path.GetString());
	return true;
}
std::optional<std::string> Renderer::SaveRenderPreview(const std::string &relPath,std::string &outErr) const
{
	auto path = util::Path::CreatePath(util::get_program_path()) +relPath;
	std::string fileName = "render_preview.exr";
	auto &film = m_lxSession->GetFilm();
	film.SaveOutput((path +fileName).GetString(),luxcore::Film::FilmOutputType::OUTPUT_RGBA,{});
	return (path +fileName).GetString();
}
void Renderer::FinalizeImage(uimg::ImageBuffer &imgBuf,StereoEye eyeStage)
{
	auto &resultAlphaBuf = GetResultImageBuffer(OUTPUT_ALPHA,eyeStage);
	if(resultAlphaBuf)
	{
		for(auto &px : *resultAlphaBuf)
			imgBuf.GetPixelView(px.GetX(),px.GetY()).SetValue(uimg::Channel::A,px.GetFloatValue(uimg::Channel::R));
	}

	imgBuf.FlipVertically();

	if(ShouldDumpRenderStageImages())
		DumpImage("luxcore",imgBuf,uimg::ImageFormat::HDR);
}
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
			const unsigned int haltNoise = m_lxSession->GetRenderConfig().GetProperties().Get(luxrays::Property("batch.haltnoisethreshold")(0.03)).Get<double>();

			char buf[512];
			const auto &stats = m_lxSession->GetStats();
			auto &film = m_lxSession->GetFilm();
			while (!m_lxSession->HasDone()) {
				std::this_thread::sleep_for(std::chrono::milliseconds(1000));
				if(m_lxSession->IsInPause())
					continue;
				UpdateProgressiveRender();
				m_lxSession->UpdateStats();
				const double elapsedTime = stats.Get("stats.renderengine.time").Get<double>();
				const unsigned int pass = stats.Get("stats.renderengine.pass").Get<unsigned int>();
				const float convergence = stats.Get("stats.renderengine.convergence").Get<unsigned int>();
				m_progress = pass /static_cast<double>(haltSpp);
				worker.UpdateProgress(m_progress);

				static auto debugExport = false;
				if(debugExport)
				{
					debugExport = false;
					Export("render/export/");
				}
		
				// Print some information about the rendering progress
				sprintf(buf, "[Elapsed time: %3d/%dsec][Samples %4d/%d][Convergence %f%%][Avg. samples/sec % 3.2fM on %.1fK tris]",
						int(elapsedTime), int(haltTime), pass, haltSpp, 100.f * convergence,
						stats.Get("stats.renderengine.total.samplesec").Get<double>() / 1000000.0,
						stats.Get("stats.dataset.trianglecount").Get<double>() / 1000.0);

				LC_LOG(buf);
			}

			auto &resultImageBuffer = GetResultImageBuffer(OUTPUT_COLOR,eyeStage);
			auto renderMode = m_scene->GetRenderMode();
			auto width = film.GetWidth();
			auto height = film.GetHeight();
			switch(renderMode)
			{
			case Scene::RenderMode::RenderImage:
			case Scene::RenderMode::BakeDiffuseLighting:
			{
				auto bakeLightmaps = (renderMode == Scene::RenderMode::BakeDiffuseLighting);
				if(bakeLightmaps)
				{
					// LuxCoreRender currently doesn't provide a way to get bake data directly, so we'll have to save it to
					// an image file, then read it and delete it once we're done.
					film.SaveOutputs();

#if 0
					auto f = FileManager::OpenFile(LIGHTMAP_ATLAS_OUTPUT_FILENAME,"rb");
					if(f == nullptr)
					{
						worker.SetStatus(util::JobStatus::Failed,"Unable to open lightmap atlas exr file!");
						return RenderStageResult::Complete;
					}
					std::vector<uint8_t> exrData;
					auto size = f->GetSize();
					exrData.resize(size);
					f->Read(exrData.data(),size);
					f = nullptr;
					FileManager::RemoveFile(LIGHTMAP_ATLAS_OUTPUT_FILENAME);

					EXRVersion exrVersion {};
					if(ParseEXRVersionFromMemory(&exrVersion,exrData.data(),exrData.size()) != TINYEXR_SUCCESS)
					{
						worker.SetStatus(util::JobStatus::Failed,"Unable to parse exr version for lightmap atlas!");
						return RenderStageResult::Complete;
					}
					const char *err = nullptr;
					EXRHeader exrHeader {};
					InitEXRHeader(&exrHeader);
					if(ParseEXRHeaderFromMemory(&exrHeader,&exrVersion,exrData.data(),exrData.size(),&err) != TINYEXR_SUCCESS)
					{
						worker.SetStatus(util::JobStatus::Failed,"Unable to parse exr header for lightmap atlas!");
						FreeEXRErrorMessage(err);
						return RenderStageResult::Complete;
					}
					util::ScopeGuard sgHeader {[&exrHeader]() {FreeEXRHeader(&exrHeader);}};
					assert(exrHeader.pixel_types[0] == TINYEXR_PIXELTYPE_FLOAT);
					if(exrHeader.pixel_types[0] != TINYEXR_PIXELTYPE_FLOAT)
					{
						worker.SetStatus(util::JobStatus::Failed,"Lightmap atlas exr has incorrect pixel format!");
						return RenderStageResult::Complete;
					}
					EXRImage exrImg {};
					InitEXRImage(&exrImg);
					if(LoadEXRImageFromMemory(&exrImg,&exrHeader,exrData.data(),exrData.size(),&err) != TINYEXR_SUCCESS)
					{
						worker.SetStatus(util::JobStatus::Failed,"Unable to load exr image data for lightmap atlas!");
						FreeEXRErrorMessage(err);
						return RenderStageResult::Complete;
					}
					util::ScopeGuard sgImg {[&exrImg]() {FreeEXRImage(&exrImg);}};
					width = exrImg.width;
					height = exrImg.height;
					assert(exrImg.num_channels == 4);
					if(exrImg.num_channels != 4)
					{
						worker.SetStatus(util::JobStatus::Failed,"Lightmap atlas exr has incorrect number of channels!");
						return RenderStageResult::Complete;
					}
					resultImageBuffer = uimg::ImageBuffer::Create(width,height,uimg::Format::RGBA_FLOAT);
					memcpy(resultImageBuffer->GetData(),exrImg.images[0],resultImageBuffer->GetSize());
#else
					// Technically deprecated, but above code doesn't work
					// TODO: Fix the above code and use it instead
					float *rgba;
					int w,h;
					const char *err;
					auto path = util::Path::CreatePath(util::get_program_path()) +LIGHTMAP_ATLAS_OUTPUT_FILENAME;
					auto result = LoadEXR(&rgba,&w,&h,path.GetString().c_str(),&err);
					// FileManager::RemoveFile(LIGHTMAP_ATLAS_OUTPUT_FILENAME);
					if(result != TINYEXR_SUCCESS)
					{
						worker.SetStatus(util::JobStatus::Failed,"Unable to load lightmap exr atlas: " +std::string{err});
						return RenderStageResult::Complete;
					}
					width = w;
					height = h;
					resultImageBuffer = uimg::ImageBuffer::CreateWithCustomDeleter(rgba,width,height,uimg::Format::RGBA_FLOAT,[](void *ptr) {free(ptr);});
					resultImageBuffer->FlipVertically();
#endif
				}
				else
				{
					resultImageBuffer = uimg::ImageBuffer::Create(width,height,GetOutputFormat(renderMode));
					try
					{
						// film.AsyncExecuteImagePipeline(0);
						film.GetOutput<float>(luxcore::Film::FilmOutputType::OUTPUT_RGBA,static_cast<float*>(resultImageBuffer->GetData()));
						
						auto apiData = GetApiData();
						std::string rawOutputFileName;
						if(apiData.GetFromPath("luxCoreRender/debug/rawOutputFileName")(rawOutputFileName))
							DumpImage("raw",*resultImageBuffer,uimg::ImageFormat::HDR,rawOutputFileName);

						if(ShouldDumpRenderStageImages())
							DumpImage("raw",*resultImageBuffer,uimg::ImageFormat::HDR);
					}
					catch(const std::exception &e)
					{
						std::cout<<"Failed to retrieve RGBA film output: "<<e.what()<<std::endl;
					}
				}
				float exposureMul = 0.02f;
				resultImageBuffer->ApplyExposure(GetScene().GetCreateInfo().exposure *exposureMul); // Factor is subjective to match Cycles behavior
				if(ShouldDumpRenderStageImages())
					DumpImage("exposure",*resultImageBuffer,uimg::ImageFormat::HDR);
				/*auto exposure = GetScene().GetCreateInfo().exposure;
				for(auto &pxView : *resultImageBuffer)
				{
					pxView.SetValue(uimg::ImageBuffer::Channel::R,pxView.GetFloatValue(uimg::ImageBuffer::Channel::R) *exposure);
					pxView.SetValue(uimg::ImageBuffer::Channel::G,pxView.GetFloatValue(uimg::ImageBuffer::Channel::G) *exposure);
					pxView.SetValue(uimg::ImageBuffer::Channel::B,pxView.GetFloatValue(uimg::ImageBuffer::Channel::B) *exposure);
				}*/

				if(bakeLightmaps == false)
				{
					auto itAlpha = m_outputs.find(OUTPUT_ALPHA);
					if(itAlpha != m_outputs.end())
					{
						auto &resultAlphaBuf = GetResultImageBuffer(OUTPUT_ALPHA,eyeStage);
						resultAlphaBuf = uimg::ImageBuffer::Create(film.GetWidth(),film.GetHeight(),uimg::Format::R32);
						film.GetOutput<float>(luxcore::Film::FilmOutputType::OUTPUT_ALPHA,static_cast<float*>(resultAlphaBuf->GetData()));
					}
				}

				if(UpdateStereoEye(worker,stage,eyeStage))
				{
					worker.Start(); // Lighting stage for the left eye is triggered by the user, but we have to start it manually for the right eye
					return RenderStageResult::Continue;
				}

				// Note: Denoising is handled by LuxCoreRender internally
				//return StartNextRenderStage(worker,ImageRenderStage::FinalizeImage,eyeStage);
				if(m_scene->ShouldDenoise() == false)
					return StartNextRenderStage(worker,ImageRenderStage::FinalizeImage,eyeStage);

				if(bakeLightmaps == false)
				{
					auto &albedoImageBuffer = GetResultImageBuffer(OUTPUT_ALBEDO,eyeStage);
					auto &normalImageBuffer = GetResultImageBuffer(OUTPUT_NORMAL,eyeStage);
					albedoImageBuffer = uimg::ImageBuffer::Create(film.GetWidth(),film.GetHeight(),uimg::Format::RGB_FLOAT);
					normalImageBuffer = uimg::ImageBuffer::Create(film.GetWidth(),film.GetHeight(),uimg::Format::RGB_FLOAT);
					film.GetOutput<float>(luxcore::Film::FilmOutputType::OUTPUT_ALBEDO,static_cast<float*>(albedoImageBuffer->GetData()));
					film.GetOutput<float>(luxcore::Film::FilmOutputType::OUTPUT_AVG_SHADING_NORMAL,static_cast<float*>(normalImageBuffer->GetData()));
					albedoImageBuffer->ClearAlpha();
					normalImageBuffer->ClearAlpha();

					static auto dbgOutputAlbedo = false;
					if(dbgOutputAlbedo)
						resultImageBuffer = albedoImageBuffer->Copy(uimg::Format::RGBA_FLOAT);
				}
				break;
			}
			default:
			{
				resultImageBuffer = uimg::ImageBuffer::Create(film.GetWidth(),film.GetHeight(),GetOutputFormat(renderMode));
				film.GetOutput<float>(GetLuxCoreFilmOutputType(renderMode),static_cast<float*>(resultImageBuffer->GetData()));
				auto numChannels = resultImageBuffer->GetChannelCount();
				resultImageBuffer->Convert(uimg::Format::RGBA32);
				if(numChannels == 1)
				{
					for(auto &pxView : *resultImageBuffer)
					{
						pxView.SetValue(uimg::Channel::G,pxView.GetFloatValue(uimg::Channel::R));
						pxView.SetValue(uimg::Channel::B,pxView.GetFloatValue(uimg::Channel::R));
					}
				}
				break;
			}
			}
			return StartNextRenderStage(worker,ImageRenderStage::Denoise,eyeStage);
		});
		break;
	}
	}
	if(optResult)
		*optResult = unirender::Renderer::RenderStageResult::Continue;
	return util::EventReply::Handled;
}
bool Renderer::FinalizeLightmap(const std::string &inputPath,const std::string &outputPath)
{
	float *rgba;
	int w,h;
	const char *err;
	auto path = util::Path::CreatePath(util::get_program_path()) +inputPath;
	auto result = LoadEXR(&rgba,&w,&h,path.GetString().c_str(),&err);
	// FileManager::RemoveFile(LIGHTMAP_ATLAS_OUTPUT_FILENAME);
	if(result != TINYEXR_SUCCESS)
		return false;
	uint32_t width = w;
	uint32_t height = h;
	auto resultImageBuffer = uimg::ImageBuffer::CreateWithCustomDeleter(rgba,width,height,uimg::Format::RGBA_FLOAT,[](void *ptr) {free(ptr);});
	resultImageBuffer->FlipVertically();

	float exposureMul = 0.02f;
	resultImageBuffer->ApplyExposure(GetScene().GetCreateInfo().exposure *exposureMul); // Factor is subjective to match Cycles behavior

	// Denoise
	denoise::Info denoiseInfo {};
	denoiseInfo.width = resultImageBuffer->GetWidth();
	denoiseInfo.height = resultImageBuffer->GetHeight();
	denoiseInfo.lightmap = true;

	denoise::denoise(denoiseInfo,*resultImageBuffer,nullptr,nullptr,[this](float progress) -> bool {
		return true;
	});

	if(m_colorTransformProcessor) // TODO: Should we really apply color transform if we're not denoising?
	{
		std::string err;
		if(m_colorTransformProcessor->Apply(*resultImageBuffer,err) == false)
			m_scene->HandleError("Unable to apply color transform: " +err);
	}
	resultImageBuffer->Convert(uimg::Format::RGBA_HDR);
	resultImageBuffer->ClearAlpha();
	FinalizeImage(*resultImageBuffer,unirender::Renderer::StereoEye::Left);

	uimg::TextureSaveInfo texSaveInfo {};
	auto &texInfo = texSaveInfo.texInfo;
	texInfo.containerFormat = uimg::TextureInfo::ContainerFormat::DDS;
	texInfo.inputFormat = uimg::TextureInfo::InputFormat::R16G16B16A16_Float;
	texInfo.outputFormat = uimg::TextureInfo::OutputFormat::BC6;
	texInfo.flags = uimg::TextureInfo::Flags::GenerateMipmaps;
//	auto f = FileManager::OpenFile<VFilePtrReal>("materials/maps/sfm_gtav_mp_apa_06/lightmap_atlas.dds","wb");
//	if(f)
	return uimg::save_texture(outputPath,*resultImageBuffer,texSaveInfo);
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
void Renderer::CloseRenderScene()
{
	/*if(m_scene->GetRenderMode() == Scene::RenderMode::BakeDiffuseLighting)
	{
		auto &resultImageBuffer = GetResultImageBuffer(OUTPUT_COLOR);
		if(resultImageBuffer)
		{
			uimg::TextureInfo texInfo {};
			texInfo.containerFormat = uimg::TextureInfo::ContainerFormat::DDS;
			texInfo.inputFormat = uimg::TextureInfo::InputFormat::R16G16B16A16_Float;
			texInfo.outputFormat = uimg::TextureInfo::OutputFormat::BC6;
			texInfo.flags = uimg::TextureInfo::Flags::GenerateMipmaps;
			uimg::save_texture("render/lightmap_atlas.dds",*resultImageBuffer,texInfo,false);
		}
	}*/
	StopRenderSession();
}
util::ParallelJob<std::shared_ptr<uimg::ImageBuffer>> Renderer::StartRender()
{
	auto job = util::create_parallel_job<RenderWorker>(*this);
	auto &worker = static_cast<RenderWorker&>(job.GetWorker());
	StartNextRenderStage(worker,ImageRenderStage::InitializeScene,StereoEye::None);
	return job;
}

bool Renderer::Initialize(Flags flags)
{
	PrepareCyclesSceneForRendering();

	m_enablePhotonGiCache = GetScene().GetCreateInfo().preCalculateLight;

	auto &logHandler = unirender::get_log_handler();
	luxcore::Init();
	if(logHandler)
	{
		luxcore::SetLogHandler([](const char *msg) {
			auto &logHandler = unirender::get_log_handler();
			if(logHandler)
				logHandler(msg);
		});
	}
	else
		luxcore::SetLogHandler();
	std::unique_ptr<luxcore::Scene> lcScene{luxcore::Scene::Create()};
	if(lcScene == nullptr)
		return false;
	try
	{
		auto &sceneInfo = m_scene->GetSceneInfo();
		auto &skyTex = sceneInfo.sky;
		if(!skyTex.empty())
		{
			auto mulFactor = 50.f; // Factor of 50 matches Cycles very closely, reason unknown
			Vector3 gain {sceneInfo.skyStrength,sceneInfo.skyStrength,sceneInfo.skyStrength};
			gain *= mulFactor;
			float gamma = 1.0;
			auto absPath = Scene::GetAbsSkyPath(skyTex);
			if(absPath.has_value())
			{
				std::string propName = "scene.lights.sky";
				umath::ScaledTransform pose {};
				pose.SetRotation(uquat::create(sceneInfo.skyAngles));
				pose.SetScale(Vector3{-1.f,1.f,1.f});
				lcScene->Parse(
					luxrays::Property(propName +".type")("infinite")<<
					luxrays::Property(propName +".file")(*absPath)<<
					luxrays::Property(propName +".gain")(gain.x,gain.y,gain.z)<<
					luxrays::Property(propName +".gamma")(gamma)<<

					// See https://forums.luxcorerender.org/viewtopic.php?f=5&t=93#p1410
					luxrays::Property(propName +".visibilitymap.enable")(1)<<
					luxrays::Property(propName +".visibilitymap.samples")(1000000)<<
					luxrays::Property(propName +".visibilitymap.width")(512)<<
					luxrays::Property(propName +".visibilitymap.height")(256)<<
					luxrays::Property(propName +".visibilitymap.maxdepth")(4)<<

					to_luxcore_matrix(propName +".transformation",pose)
				);
			}
		}
	
	m_bakeTarget = FindObject("bake_target");

	m_lxScene = std::move(lcScene);
	SyncFilm(m_scene->GetCamera());
	SyncCamera(m_scene->GetCamera());
	auto &mdlCache = m_renderData.modelCache;
	mdlCache->GenerateData();
	uint32_t objId = 0;
	uint32_t meshId = 0;
	for(auto &chunk : mdlCache->GetChunks())
	{
		for(auto &o : chunk.GetObjects())
		{
			o->SetId(objId++);
			o->Finalize(*m_scene);
		}
		for(auto &o : chunk.GetMeshes())
		{
			o->SetId(meshId++);
			o->Finalize(*m_scene);
		}
	}
	for(auto &shader : m_renderData.shaderCache->GetShaders())
		shader->Finalize();

	uint32_t lightId = 0;
	for(auto &light : m_scene->GetLights())
	{
		light->SetId(lightId++);
		SyncLight(*light);
	}
	for(auto &chunk : mdlCache->GetChunks())
	{
		for(auto &o : chunk.GetMeshes())
			SyncMesh(*o);
	}
	m_shaderNodeIdx = 0;
	m_curShaderIdx = 0;
	for(auto &chunk : mdlCache->GetChunks())
	{
		for(auto &o : chunk.GetObjects())
			SyncObject(*o);
	}
	UpdateActorMap();

	auto &defWorldVolume = GetDefaultWorldVolume();
	if(!defWorldVolume.empty())
	{
		luxrays::Properties props {};
		/*for(auto &shader : m_objectShaders)
		{
			if(shader == defWorldVolume)
				continue;
			props<<luxrays::Property("scene.materials." +shader +".volume.exterior")(defWorldVolume);
		}*/
		props<<luxrays::Property("scene.world.volume.default")(defWorldVolume);
		m_lxScene->Parse(props);
	}

	auto &createInfo = const_cast<unirender::Scene::CreateInfo&>(m_scene->GetCreateInfo());
	auto resolution = m_scene->GetResolution();

	// TODO: Move this to shared code
	if(createInfo.colorTransform.has_value())
	{
		std::string err;
		ColorTransformProcessorCreateInfo ctpCreateInfo {};
		ctpCreateInfo.config = createInfo.colorTransform->config;
		ctpCreateInfo.lookName = createInfo.colorTransform->lookName;
		ctpCreateInfo.bitDepth = ColorTransformProcessorCreateInfo::BitDepth::Float32;
		m_colorTransformProcessor = create_color_transform_processor(ctpCreateInfo,err,0.f,m_scene->GetGamma());
		if(m_colorTransformProcessor == nullptr)
			m_scene->HandleError("Unable to initialize color transform processor: " +err);
	}

	luxrays::Properties props {};

	auto renderMode = m_scene->GetRenderMode();
	std::string renderEngineType = "PATHCPU";
	m_flags = flags;
	if(umath::is_flag_set(flags,Flags::EnableLiveEditing))
		m_renderEngine = RenderEngine::RealTime;
	switch(m_renderEngine)
	{
	case RenderEngine::PathTracer:
		renderEngineType = (createInfo.deviceType == unirender::Scene::DeviceType::GPU) ? "PATHOCL" : "PATHCPU";
		break;
	case RenderEngine::TiledPathTracer:
		renderEngineType = (createInfo.deviceType == unirender::Scene::DeviceType::GPU) ? "TILEPATHOCL" : "TILEPATHCPU";
		break;
	case RenderEngine::BidirectionalPathTracer:
		renderEngineType = "BIDIRCPU";
		createInfo.deviceType = unirender::Scene::DeviceType::CPU; // Not available for GPU
		break;
	case RenderEngine::RealTime:
		renderEngineType = (createInfo.deviceType == unirender::Scene::DeviceType::GPU) ? "RTPATHOCL" : "RTPATHCPU";
		break;
	}
	// TODO: RTPATHCPU for scene editing
	// TODO: RTPATHOCL real-time?
	auto bakeLightmaps = (renderMode == unirender::Scene::RenderMode::BakeDiffuseLighting);
	if(bakeLightmaps)
	{
		renderEngineType = "BAKECPU"; // TODO: Use GPU renderer once available
		createInfo.deviceType = unirender::Scene::DeviceType::CPU; // Not available for GPU
		m_renderEngine = RenderEngine::Bake;
	}
	else
		m_bakeTarget = nullptr;

	if(createInfo.progressive)
	{
		auto tileSize = resolution;
		m_tileManager.Initialize(resolution.x,resolution.y,tileSize.x,tileSize.y,createInfo.deviceType == unirender::Scene::DeviceType::CPU,createInfo.exposure,m_scene->GetGamma(),m_colorTransformProcessor.get());
		m_tileManager.SetExposure(sceneInfo.exposure);
	}

	props<<luxrays::Property("renderengine.type")(renderEngineType);
	if(renderEngineType == "TILEPATHCPU" || renderEngineType == "RTPATHOCL" || renderEngineType == "TILEPATHOCL")
		props<<luxrays::Property("sampler.type")("TILEPATHSAMPLER");
	else if(renderEngineType == "RTPATHCPU")
		props<<luxrays::Property("sampler.type")("RTPATHCPUSAMPLER");
	else
		props<<luxrays::Property("sampler.type")("SOBOL");
	props<<luxrays::Property("batch.halttime")(4'320'000);
	props<<luxrays::Property("context.cuda.optix.enable")("1");

	props<<luxrays::Property("film.opencl.enable")("0"); // TODO: If enabled, causes a crash in ExecuteImagePipeline, reason unknown
	props<<luxrays::Property("film.opencl.platform")("-1");
	props<<luxrays::Property("film.opencl.device")("-1");

	enum class AcceleratorType : uint8_t
	{
		Auto = 0,
		BVH,
		MBVH,
		QBVH,
		MQBVH,
		Embree
	};
	auto f = filemanager::open_file("accelerator_type.txt",filemanager::FileMode::Read);
	if(f)
	{
		auto accType = f->ReadString();
		f = nullptr;
		ustring::replace(accType,"\n","");
		ustring::replace(accType,"\r","");
		ustring::replace(accType," ","");
		std::cout<<"Using accelerator type "<<accType<<std::endl;
		props<<luxrays::Property("accelerator.type")(accType);
	}
	/*auto accType = AcceleratorType::Auto;
	switch(accType)
	{
	case AcceleratorType::Auto:
		props<<luxrays::Property("accelerator.type")("AUTO");
		break;
	case AcceleratorType::BVH:
		props<<luxrays::Property("accelerator.type")("BVH");
		break;
	case AcceleratorType::MBVH:
		props<<luxrays::Property("accelerator.type")("MBVH");
		break;
	case AcceleratorType::QBVH:
		props<<luxrays::Property("accelerator.type")("QBVH");
		break;
	case AcceleratorType::MQBVH:
		props<<luxrays::Property("accelerator.type")("MQBVH");
		break;
	case AcceleratorType::Embree:
		props<<luxrays::Property("accelerator.type")("EMBREE");
		break;
	}*/

	if(m_enablePhotonGiCache)
	{
		// See https://forums.luxcorerender.org/viewtopic.php?t=840
		props<<luxrays::Property("path.photongi.sampler.type")("METROPOLIS");

		props<<luxrays::Property("path.photongi.visibility.enabled")("1");
		props<<luxrays::Property("path.photongi.visibility.targethitrate")("0.99");
		props<<luxrays::Property("path.photongi.visibility.maxsamplecount")("1048576");

		props<<luxrays::Property("path.photongi.direct.enabled")("1");
		props<<luxrays::Property("path.photongi.indirect.enabled")("1");
		props<<luxrays::Property("path.photongi.caustic.enabled")("1");
		
		props<<luxrays::Property("path.photongi.photon.maxcount")("100000");
		props<<luxrays::Property("path.photongi.photon.maxdepth")("4");
		props<<luxrays::Property("path.photongi.direct.maxsize")("100000");
		props<<luxrays::Property("path.photongi.indirect.maxsize")("100000");
		props<<luxrays::Property("path.photongi.caustic.maxsize")("100000");
		props<<luxrays::Property("path.photongi.lookup.maxcount")("48");
		props<<luxrays::Property("path.photongi.photon.maxdepth")("4");
		props<<luxrays::Property("path.photongi.lookup.radius")("0.15");
		props<<luxrays::Property("path.photongi.lookup.normalangle")("10.0");
		props<<luxrays::Property("path.photongi.debug.type")("none"); // Possible values: showdirect, showindirect, showcaustic, none
	}

	if(m_enableLightSamplingCache)
		props<<luxrays::Property("lightstrategy.type")("DLS_CACHE");

	props<<luxrays::Property("film.opencl.device")("-1");

	auto enableColorOutput = (renderMode == unirender::Scene::RenderMode::BakeDiffuseLighting) ||
		(renderMode == unirender::Scene::RenderMode::RenderImage);
	auto enableAlbedoOutput = (renderMode == unirender::Scene::RenderMode::SceneAlbedo) ||
		((enableColorOutput || renderMode == unirender::Scene::RenderMode::BakeDiffuseLighting) && m_scene->ShouldDenoise());
	auto enableNormalOutput = (renderMode == unirender::Scene::RenderMode::SceneNormals) ||
		((enableColorOutput || renderMode == unirender::Scene::RenderMode::BakeDiffuseLighting) && m_scene->ShouldDenoise());
	if(renderMode == unirender::Scene::RenderMode::BakeDiffuseLighting)
	{
		// Don't need these for denoising lightmaps
		enableAlbedoOutput = false;
		enableNormalOutput = false;
	}
	if(enableColorOutput)
	{
		auto colorOutput = AddOutput(OUTPUT_COLOR);
		std::string propName = "film.outputs." +std::to_string(colorOutput.first);
		props<<luxrays::Property(propName +".type")(TranslateOutputTypeToLuxCoreRender(GetOutputType(Scene::RenderMode::RenderImage)));
		props<<luxrays::Property(propName +".filename")("output.exr"); // We don't actually save the image, but we still need to specify a format
		props<<luxrays::Property(propName +".filesafe")("0");
		props<<luxrays::Property(propName +".index")("0");
	}

		/*props<<luxrays::Property("film.imagepipelines.0.0.type")("INTEL_OIDN");
		props<<luxrays::Property("film.imagepipelines.0.0.sharpness")("0.0");
		if(bakeLightmaps)
			props<<luxrays::Property("film.imagepipelines.0.0.filter.type")("RTLightmap");
		else
			props<<luxrays::Property("film.imagepipelines.0.0.filter.type")("RT");*/
	if(enableAlbedoOutput)
	{
		auto albedoOutput = AddOutput(OUTPUT_ALBEDO);
		std::string propName = "film.outputs." +std::to_string(albedoOutput.first);
		props<<luxrays::Property(propName +".type")(TranslateOutputTypeToLuxCoreRender(GetOutputType(Scene::RenderMode::SceneAlbedo)));
		props<<luxrays::Property(propName +".filename")("albedo.exr");
		props<<luxrays::Property(propName +".filesafe")("0");
		props<<luxrays::Property(propName +".index")("0");
	}
	if(enableNormalOutput)
	{
		auto normalOutput = AddOutput(OUTPUT_NORMAL);
		std::string propName = "film.outputs." +std::to_string(normalOutput.first);
		props<<luxrays::Property(propName +".type")(TranslateOutputTypeToLuxCoreRender(GetOutputType(Scene::RenderMode::SceneNormals)));
		props<<luxrays::Property(propName +".filename")("normal.exr");
		props<<luxrays::Property(propName +".filesafe")("0");
		props<<luxrays::Property(propName +".index")("0");
	}
	if(
		renderMode != unirender::Scene::RenderMode::RenderImage &&
		renderMode != unirender::Scene::RenderMode::BakeAmbientOcclusion &&
		renderMode != unirender::Scene::RenderMode::BakeNormals &&
		renderMode != unirender::Scene::RenderMode::BakeDiffuseLighting &&
		renderMode != unirender::Scene::RenderMode::SceneAlbedo &&
		renderMode != unirender::Scene::RenderMode::SceneNormals
	)
	{
		auto output = AddOutput(GetOutputType(renderMode));
		std::string propName = "film.outputs." +std::to_string(output.first);
		props<<luxrays::Property(propName +".type")(TranslateOutputTypeToLuxCoreRender(GetOutputType(renderMode)));
		props<<luxrays::Property(propName +".filename")(TranslateOutputTypeToLuxCoreRender(GetOutputType(renderMode)) +".exr");
		props<<luxrays::Property(propName +".filesafe")("0");
		props<<luxrays::Property(propName +".index")("0");
	}
	if(ShouldUseTransparentSky())
	{
		props<<luxrays::Property("path.forceblackbackground.enable")(true);

		auto alphaOutput = AddOutput(OUTPUT_ALPHA);
		std::string propName = "film.outputs." +std::to_string(alphaOutput.first);
		props<<luxrays::Property(propName +".type")(alphaOutput.second);
		props<<luxrays::Property(propName +".filename")("alpha.exr");
		props<<luxrays::Property(propName +".filesafe")("0");
		props<<luxrays::Property(propName +".index")("0");
	}
	if(bakeLightmaps && m_bakeObjectNames.empty() == false)
	{
		FileManager::CreatePath((util::Path::CreateFile(LIGHTMAP_ATLAS_OUTPUT_FILENAME).GetPath()).data());
		props<<luxrays::Property("film.outputs.0.type")("RGBA");
		props<<luxrays::Property("film.outputs.0.filename")("RGBA_0.exr");
		props<<luxrays::Property("film.outputs.0.index")("0");
		props<<luxrays::Property("film.imagepipelines.0.0.type")("NOP");
		props<<luxrays::Property("film.imagepipelines.0.0.type")("TONEMAP_LINEAR");
		props<<luxrays::Property("film.imagepipelines.0.0.scale")("1");
		/*props<<luxrays::Property("film.imagepipelines.0.0.type")("NOP");
		props<<luxrays::Property("film.imagepipelines.0.1.type")("TONEMAP_LINEAR");
		props<<luxrays::Property("film.imagepipelines.0.1.scale")("1");
		props<<luxrays::Property("film.imagepipelines.0.2.type")("GAMMA_CORRECTION");
		props<<luxrays::Property("film.imagepipelines.0.2.value")("2.2000000000000002");
		props<<luxrays::Property("film.imagepipelines.1.0.type")("NOP");*/

		props<<luxrays::Property("bake.minmapautosize")(256);
		props<<luxrays::Property("bake.maxmapautosize")(512);
		props<<luxrays::Property("bake.powerof2autosize.enable")(0);
		
		props<<luxrays::Property("bake.maps.0.type")("LIGHTMAP");
		props<<luxrays::Property("bake.maps.0.filename")(LIGHTMAP_ATLAS_OUTPUT_FILENAME);
		props<<luxrays::Property("bake.maps.0.imagepipelineindex")(0);
		props<<luxrays::Property("bake.maps.0.autosize.enabled")(0);
		props<<luxrays::Property("bake.maps.0.uvindex")(LIGHTMAP_UV_CHANNEL);
		props<<luxrays::Property("bake.maps.0.width")(resolution.x);
		props<<luxrays::Property("bake.maps.0.height")(resolution.y);
		//props<<luxrays::Property("bake.maps.0.objectnames")("1398588469529680");
		
		props<<to_luxcore_list("bake.maps.0.objectnames",m_bakeObjectNames);
		
		props<<luxrays::Property("film.filter.type")("BLACKMANHARRIS");
		props<<luxrays::Property("film.filter.width")(4);
		props<<luxrays::Property("batch.haltnoisethreshold")(0.03);
	}
	//props<<luxrays::Property("film.outputs.1.type")("RGB_IMAGEPIPELINE");
	//props<<luxrays::Property("film.outputs.1.filename")("image.png"); // We don't actually save the image, but we still need to specify a format
	if(createInfo.samples.has_value())
	{
		props<<luxrays::Property("batch.haltspp")(*createInfo.samples);
		props<<luxrays::Property("film.spp")(*createInfo.samples);
	}
	//if(bakeLightmaps == false)
	{
		props<<luxrays::Property("film.width")(resolution.x);
		props<<luxrays::Property("film.height")(resolution.y);
	}
	std::unique_ptr<luxcore::RenderConfig> lcConfig {luxcore::RenderConfig::Create(
		props,
		m_lxScene.get()
	)};
	std::unique_ptr<luxcore::RenderSession> lcSession {luxcore::RenderSession::Create(lcConfig.get())};
	if(lcSession == nullptr)
		return false;
	m_lxConfig = std::move(lcConfig);
	m_lxSession = std::move(lcSession);

	static auto debugExport = false;
	if(debugExport)
	{
		debugExport = false;
		Export("render/export/");
	}
	try
	{
		if(!m_lxConfig->HasCachedKernels())
			std::cout<<"Compiling LuxCoreRender kernels... This may take a while!"<<std::endl;
		m_lxSession->Start();
	}
	catch(const std::runtime_error &e)
	{
		std::cout<<"Unable to start LuxCoreRender session: "<<e.what()<<std::endl;
		return false;
	}

	static auto finalizeLightmap = false;
	if(finalizeLightmap)
	{
		auto result = FinalizeLightmap("temp/lightmap_atlas.exr","render/lightmaps/lightmap.dds");
		std::cout<<"Result: "<<result<<std::endl;
	}

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

Renderer::Renderer(const Scene &scene,Flags flags)
	: unirender::Renderer{scene,flags}
{}

void Renderer::SyncCamera(const unirender::Camera &cam)
{
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
	auto pos = ToLuxPosition(pose.GetOrigin());
	auto up = ToLuxNormal(uquat::up(pose.GetRotation()));
	auto n = ToLuxNormal(uquat::forward(pose.GetRotation()));
	auto target = pos +to_luxcore_vector(n) *100.f;
	auto fov = umath::rad_to_deg(umath::vertical_fov_to_horizontal_fov(umath::deg_to_rad(cam.GetFov()),cam.GetWidth(),cam.GetHeight()));
	std::string propName = "scene.camera";
	m_lxScene->Parse(
		luxrays::Property(propName +".type")(lcCamType)<<
		luxrays::Property(propName +".lookat.orig")(pos.x,pos.y,pos.z)<<
		luxrays::Property(propName +".lookat.target")(target.x,target.y,target.z)<<
		luxrays::Property(propName +".up")(up.x,up.y,up.z)<<
		luxrays::Property(propName +".fieldofview")(fov)
		// luxrays::Property(propName +".screenwindow")(0.0,1.0,0.0,1.0)
		// luxrays::Property(propName +".shutteropen")(0.0)<<
		// luxrays::Property(propName +".shutterclose")(0.0)<<
		// luxrays::Property(propName +".autovolume.enable")(true)<<
		// luxrays::Property(propName +".volume")("")
	);
}

void Renderer::SyncFilm(const unirender::Camera &cam)
{
	auto renderMode = m_scene->GetRenderMode();
	auto bakeLightmaps = (renderMode == unirender::Scene::RenderMode::BakeDiffuseLighting);
	if(bakeLightmaps)
		return;
	std::string propName = "film";
	m_lxScene->Parse(
		luxrays::Property(propName +".width")(cam.GetWidth())<<
		luxrays::Property(propName +".height")(cam.GetHeight())
	);
}

void Renderer::SyncObject(const unirender::Object &obj)
{
	// Unirender supports multiple shaders per mesh, but LuxCore does not,
	// so we'll have to create multiple objects (one per shader/material)

	auto &mesh = obj.GetMesh();
	if(uvec::length_sqr(obj.GetScale()) < 0.001)
		return; // Objects with a scale of 0 result in a LuxCore matrix exception: https://github.com/LuxCoreRender/LuxCore/blob/e54bcf3ffb7feac5964e023c19ca3e7c4b98f530/src/luxrays/core/geometry/matrix4x4.cpp#L137

	// Shaders
	auto &luxNodeManager = get_lux_node_manager();
	auto &shaders = mesh.GetSubMeshShaders();
	auto isLightmapMesh = (m_lightmapMeshes.find(&mesh) != m_lightmapMeshes.end());
	for(auto i=decltype(shaders.size()){0u};i<shaders.size();++i)
	{
		std::string matName;
		std::string matNameHair;
		auto desc = shaders.at(i)->GetActivePassNode();
		if(desc == nullptr)
			desc = GroupNodeDesc::Create(m_scene->GetShaderNodeManager()); // Just create a dummy node

		desc->ResolveGroupNodes();
		auto &nodes = desc->GetChildNodes();
		
		auto &hairConfig = shaders.at(i)->GetHairConfig();
		auto hasHair = hairConfig.has_value();
		LuxNodeCache nodeCache {static_cast<uint32_t>(nodes.size()),m_shaderNodeIdx};
		std::unique_ptr<LuxNodeCache> nodeCacheHair = nullptr;
		if(hasHair)
			nodeCacheHair = std::make_unique<LuxNodeCache>(static_cast<uint32_t>(nodes.size()),m_shaderNodeIdx);
		for(auto &node : nodes)
		{
			if(node->GetOutputs().empty() == false)
				continue;
			auto &type = node->GetTypeName();
			auto *factory = luxNodeManager.GetFactory(type);
			if(factory)
			{
				auto fDefineShader = [&](std::string &matName,LuxNodeCache &nodeCache) {
					m_curShaderName = "shader_" +std::to_string(m_curShaderIdx++);
					matName = m_curShaderName;
					unirender::Socket sock {*node,"in",true /* output */};
					auto properties = (*factory)(*this,*desc,*node,sock,nodeCache);
					if(!properties.has_value())
						matName = "";
					else
						m_objectShaders.push_back(matName);
					/*if(properties.has_value())
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
					}*/
				};
				fDefineShader(matName,nodeCache);
				if(hasHair)
				{
					m_useHairShader = true;
					fDefineShader(matNameHair,*nodeCacheHair);
					m_useHairShader = false;
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

		luxrays::Properties props = luxrays::Property(propName +".material")(matName)<<
			luxrays::Property(propName +".shape")(shapeName)<<
			to_luxcore_matrix(propName +".transformation",obj.GetPose());
		// luxrays::Property(propName +".id")("")<<
		// luxrays::Property(propName +".camerainvisible")(false)

		if(isLightmapMesh)
		{
			//props<<luxrays::Property(propName +".bake.lightmap.file")("E:/projects/pragma/build_winx64/install/luxcore_lightmap.png")<<
			//luxrays::Property(propName +".bake.lightmap.gamma")(2.2)<<
			//luxrays::Property(propName +".bake.lightmap.storage")("auto")<<
			//luxrays::Property(propName +".bake.lightmap.wrap")("repeat")<<
			// luxrays::Property(propName +".bake.lightmap.channel")(matName)<<
			//luxrays::Property(propName +".bake.lightmap.uvindex")(LIGHTMAP_UV_CHANNEL);
			m_bakeObjectNames.push_back(objName);
		}

		m_lxScene->Parse(props);

#if 0
	if(o == nullptr)
		return;
	auto objName = GetName(*o,0);
	std::string propName = "scene.objects." +objName;
	m_lxScene->Parse(
		luxrays::Property(propName +".bake.lightmap.file")("luxcore_lightmap.png")<<
		//luxrays::Property(propName +".bake.lightmap.gamma")(2.2)<<
		//luxrays::Property(propName +".bake.lightmap.storage")("auto")<<
		//luxrays::Property(propName +".bake.lightmap.wrap")("repeat")<<
		// luxrays::Property(propName +".bake.lightmap.channel")(matName)<<
		luxrays::Property(propName +".bake.lightmap.uvindex")(0)
	);
#endif

		if(hasHair == false)
			continue;
		propName = "scene.objects." +objName +"_strands";
		//auto propName = "scene.materials." +nodeCache.ToLinkedSocketName(outputSocket);
		//props<<luxrays::Property(propName +".type")("disney");
		//ConvertSocketLinkedToInputToProperty(props,renderer,rootNode,nodeCache,node,unirender::nodes::principled_bsdf::IN_BASE_COLOR,propName +".basecolor",false);
		static auto enableTransform = true;
		if(enableTransform)
		{
			m_lxScene->Parse(
				luxrays::Property(propName +".material")(matNameHair)<<
				luxrays::Property(propName +".shape")(shapeName +"_strands")<<
				to_luxcore_matrix(propName +".transformation",obj.GetPose())
			);
		}
		else
		{
			m_lxScene->Parse(
				luxrays::Property(propName +".material")(matNameHair)<<
				luxrays::Property(propName +".shape")(shapeName +"_strands")
			);
		}
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

	static float lightIntensityFactor = 20.f;
	watt *= lightIntensityFactor;

	watt *= m_scene->GetLightIntensityFactor();

	auto gain = watt;
	float envLightFactor = 0.01f;
	if(light.GetType() == unirender::Light::Type::Directional)
		gain *= envLightFactor;
	else if(light.GetType() == unirender::Light::Type::Spot)
		gain *= 0.07; // Conversion to match Cycles light intensity (subjective); See https://github.com/LuxCoreRender/BlendLuxCore/blob/43852c608d617afff842f9a43daaff9862777927/export/light.py
	else
	{
		static auto gainMul = 1.f;
		gain *= gainMul;
	}

	auto apiData = GetApiData();
	auto customLightIntensityMultiplier = 1.f;
	if(apiData.GetFromPath("lightIntensityMultiplier")(customLightIntensityMultiplier))
		gain = customLightIntensityMultiplier;

	luxrays::Properties props {};
	props<<luxrays::Property{propName +".color"}(color.r,color.g,color.b);
	props<<luxrays::Property{propName +".gain"}(gain,gain,gain);
	props<<luxrays::Property(propName +".power")(0.0);
	props<<luxrays::Property(propName +".efficency")(0.0);
	props<<luxrays::Property(propName +".normalizebycolor")(false);
	switch(light.GetType())
	{
	case unirender::Light::Type::Spot:
	{
		auto pos = ToLuxPosition(light.GetPose().GetOrigin());
		auto forward = ToLuxNormal(uquat::forward(light.GetPose().GetRotation()));
		auto target = pos +to_luxcore_vector(forward) *100.f;
		props<<luxrays::Property(propName +".type")("spot");
		props<<luxrays::Property(propName +".position")(pos.x,pos.y,pos.z);
		props<<luxrays::Property(propName +".target")(target.x,target.y,target.z);
		auto outerConeAngle = light.GetOuterConeAngle();
		auto blend = light.GetBlendFraction();
		auto innerConeAngle = outerConeAngle *(1.f -blend);
		props<<luxrays::Property(propName +".coneangle")(outerConeAngle);
		props<<luxrays::Property(propName +".conedeltaangle")(light.GetOuterConeAngle() -innerConeAngle);
		break;
	}
	case unirender::Light::Type::Point:
	{
		auto pos = ToLuxPosition(light.GetPose().GetOrigin());
		auto dir = ToLuxNormal(uquat::forward(light.GetPose().GetRotation()));
		props<<luxrays::Property(propName +".type")("sphere");
		props<<luxrays::Property(propName +".position")(pos.x,pos.y,pos.z);
		props<<luxrays::Property(propName +".turbidity")(2.2);
		props<<luxrays::Property(propName +".relsize")(1.0);
		props<<luxrays::Property(propName +".dir")(dir.x,dir.y,dir.z);
		break;
	}
	case unirender::Light::Type::Directional:
	{
		auto pos = ToLuxPosition(light.GetPose().GetOrigin());
		auto dir = ToLuxNormal(uquat::forward(light.GetPose().GetRotation()));
		auto target = pos +to_luxcore_vector(dir) *100.f;
		props<<luxrays::Property(propName +".type")("sun");
		props<<luxrays::Property(propName +".position")(pos.x,pos.y,pos.z);
		props<<luxrays::Property(propName +".dir")(dir.x,dir.y,dir.z);
		break;
	}
	}
	m_lxScene->Parse(props);
}

#include <mathutil/vertex.hpp>
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
	auto &lightmapUvs = mesh.GetLightmapUvs();
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
			points[i] = ToLuxPosition(v);

			auto &uv = uvs[idx];
			static_assert(sizeof(uv) == sizeof(luxrays::UV));
			lxUvs[i] = ToLuxUV(uv);

			auto &n = normals[idx];
			static_assert(sizeof(n) == sizeof(luxrays::Normal));
			lxNormals[i] = ToLuxNormal(n);
		}

		luxrays::UV *lxLightmapUvs = nullptr;
		if(lightmapUvs.empty() == false)
		{
			lxLightmapUvs = new luxrays::UV[numMeshVerts]; // Note: Will be freed by luxcore
			for(auto i=decltype(numMeshVerts){0u};i<numMeshVerts;++i)
			{
				auto idx = meshVertIndices[i];
				auto &uv = lightmapUvs[idx];
				static_assert(sizeof(uv) == sizeof(luxrays::UV));
				lxLightmapUvs[i] = *reinterpret_cast<const luxrays::UV*>(&uv);
			}
		}

		
		std::array<float*,8> lxUvChannels {
			reinterpret_cast<float*>(lxUvs)
		};
		if(lxLightmapUvs)
		{
			lxUvChannels[LIGHTMAP_UV_CHANNEL] = reinterpret_cast<float*>(lxLightmapUvs);
			m_lightmapMeshes.insert(&mesh);
		}
		auto &shader = mesh.GetSubMeshShaders()[iShader];
		auto &subdivSettings = shader->GetSubdivisionSettings();
		auto name = GetName(mesh,iShader);
		if(subdivSettings)
			name += "_base";
		m_lxScene->DefineMeshExt(name,numMeshVerts,numMeshTris,reinterpret_cast<float*>(points),reinterpret_cast<unsigned int*>(lxTris),reinterpret_cast<float*>(lxNormals),&lxUvChannels,nullptr,nullptr);
		if(subdivSettings)
		{
			/*if(true)
			{
				std::vector<umath::Vertex> tmpVerts;
				auto numVerts = verts.size();
				tmpVerts.reserve(numVerts);
				for(auto i=decltype(numVerts){0};i<numVerts;++i)
				{
					tmpVerts.push_back({});
					auto &v = tmpVerts.back();
					v.position = verts[i];
					v.uv = uvs[i];
					v.normal = normals[i];
				}
				std::vector<umath::Vertex> newVerts;
				std::vector<int> newTris;
				uint32_t subDivLevel = 2;
				unirender::subdivide_mesh(tmpVerts,tris,newVerts,newTris,subDivLevel);
				std::cout<<"";
			}*/

			luxrays::Properties props {};
			props<<luxrays::Property("scene.shapes." +GetName(mesh,iShader) +".type")("subdiv");
			props<<luxrays::Property("scene.shapes." +GetName(mesh,iShader) +".source")(name);
			props<<luxrays::Property("scene.shapes." +GetName(mesh,iShader) +".maxlevel")(subdivSettings->maxLevel);
			props<<luxrays::Property("scene.shapes." +GetName(mesh,iShader) +".maxedgescreensize")(subdivSettings->maxEdgeScreenSize);
			m_lxScene->Parse(props);
		}
		auto &hairConfig = shader->GetHairConfig();
		if(hairConfig.has_value() == false)
			continue;
		util::HairGenerator hairGenerator {};
		struct MeshInterface
			: public util::HairGenerator::MeshInterface
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
		m_lxScene->DefineStrands(name +"_strands",*hairFile,luxcore::Scene::StrandsTessellationType::TESSEL_RIBBON_ADAPTIVE,2 /* adaptiveMaxDepth */,0.0075 /* adaptiveError */,12 /*solidSideCount */,false /* bottomCap */,false /* topCap */,true /* useCameraPosition */);
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
{
	m_lxSession->Stop();
	m_tileManager.Cancel();
}
void Renderer::PrepareCyclesSceneForRendering()
{
	unirender::Renderer::PrepareCyclesSceneForRendering();
}

#pragma optimize("",on)
