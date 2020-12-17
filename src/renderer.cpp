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
#include <util_raytracing/model_cache.hpp>
#include <luxcore/luxcore.h>
#include <luxrays/core/geometry/matrix4x4.h>
#include <luxrays/core/geometry/point.h>
#include <luxrays/core/geometry/triangle.h>
#include <luxrays/core/geometry/uv.h>

using namespace unirender::luxcorerender;
#pragma optimize("",off)

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
	renderer->StartRender();
	return renderer;
}

Renderer::~Renderer()
{
	if(m_lxSession)
		m_lxSession->Stop();
	m_lxSession = nullptr;
	m_lxConfig = nullptr;
	m_lxScene = nullptr;
}
std::string Renderer::GetName(const BaseObject &obj)
{
	auto name = obj.GetName();
	name += std::to_string(obj.GetId());
	return name;
}
void Renderer::Wait()
{
	m_lxSession->Stop();
}
float Renderer::GetProgress() const
{return 1.f;}
void Renderer::Reset()
{
	m_lxSession->Stop();
}
void Renderer::Restart()
{
	m_lxSession->Stop();
}
util::ParallelJob<std::shared_ptr<uimg::ImageBuffer>> Renderer::StartRender()
{
	const unsigned int haltTime = m_lxSession->GetRenderConfig().GetProperties().Get(luxrays::Property("batch.halttime")(0)).Get<unsigned int>();
	const unsigned int haltSpp = m_lxSession->GetRenderConfig().GetProperties().Get(luxrays::Property("batch.haltspp")(0)).Get<unsigned int>();

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

	// Save the rendered image
	m_lxSession->GetFilm().SaveOutputs();
	return {};
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
		const unsigned int size = 500;
		std::vector<unsigned char> img(size * size * 3);
		unsigned char *ptr = &img[0];
		for (unsigned int y = 0; y < size; ++y) {
			for (unsigned int x = 0; x < size; ++x) {
				if ((x % 50 < 25) ^ (y % 50 < 25)) {
					*ptr++ = 255;
					*ptr++ = 0;
					*ptr++ = 0;
				} else {
					*ptr++ = 255;
					*ptr++ = 255;
					*ptr++ = 0;
				}
			}
		}

		lcScene->DefineImageMap<unsigned char>("check_texmap", &img[0], 1.f, 3, size, size, luxcore::Scene::DEFAULT);
		lcScene->Parse(
			luxrays::Property("scene.textures.map.type")("imagemap") <<
			luxrays::Property("scene.textures.map.file")("check_texmap") <<
			luxrays::Property("scene.textures.map.gamma")(1.f)
			);

	static auto useDds = false;
	if(useDds == false)
	{
		lcScene->Parse(
			luxrays::Property("scene.textures.test_tex.type")("imagemap") <<
			luxrays::Property("scene.textures.test_tex.file")("E:/projects/pragma/build_winx64/output/addons/converted/materials/models/sneakyerz/wigwoo1/hobkin/body.png") <<
			luxrays::Property("scene.textures.test_tex.gamma")(1.f)
		);
	}
	else
	{
		lcScene->Parse(
			luxrays::Property("scene.textures.test_tex.type")("imagemap") <<
			luxrays::Property("scene.textures.test_tex.file")("E:/projects/pragma/build_winx64/output/addons/converted/materials/models/sneakyerz/wigwoo1/hobkin/body.dds") <<
			luxrays::Property("scene.textures.test_tex.gamma")(1.f)
		);
	}

	lcScene->Parse(
		luxrays::Property("scene.materials.test_mat.type")("disney") <<
		luxrays::Property("scene.materials.test_mat.basecolor")("test_tex")
		//luxrays::Property("scene.materials.test_mat.metallic")("disney") <<
		//luxrays::Property("scene.materials.test_mat.specular")("disney") <<
		//luxrays::Property("scene.materials.test_mat.roughness")("disney") <<
	);

	lcScene->Parse(
		luxrays::Property("scene.materials.whitelight.type")("matte") <<
		luxrays::Property("scene.materials.whitelight.emission")(1000000.f, 1000000.f, 1000000.f) <<
		luxrays::Property("scene.materials.mat_white.type")("matte") <<
		luxrays::Property("scene.materials.mat_white.kd")("map") <<
		luxrays::Property("scene.materials.mat_red.type")("matte") <<
		luxrays::Property("scene.materials.mat_red.kd")(0.75f, 0.f, 0.f) <<
		luxrays::Property("scene.materials.mat_glass.type")("glass") <<
		luxrays::Property("scene.materials.mat_glass.kr")(0.9f, 0.9f, 0.9f) <<
		luxrays::Property("scene.materials.mat_glass.kt")(0.9f, 0.9f, 0.9f) <<
		luxrays::Property("scene.materials.mat_glass.exteriorior")(1.f) <<
		luxrays::Property("scene.materials.mat_glass.interiorior")(1.4f) <<
		luxrays::Property("scene.materials.mat_gold.type")("metal2") <<
		luxrays::Property("scene.materials.mat_gold.preset")("gold")
	);

	lcScene->Parse(
		luxrays::Property("scene.lights.skyl.type")("sky2") <<
		luxrays::Property("scene.lights.skyl.dir")(0.166974f, 0.59908f, 0.783085f) <<
		luxrays::Property("scene.lights.skyl.turbidity")(2.2f) <<
		luxrays::Property("scene.lights.skyl.gain")(0.8f, 0.8f, 0.8f) <<
		luxrays::Property("scene.lights.sunl.type")("sun") <<
		luxrays::Property("scene.lights.sunl.dir")(0.166974f, 0.59908f, 0.783085f) <<
		luxrays::Property("scene.lights.sunl.turbidity")(2.2f) <<
		luxrays::Property("scene.lights.sunl.gain")(0.8f, 0.8f, 0.8f)
	);
	
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
	for(auto &chunk : mdlCache->GetChunks())
	{
		for(auto &o : chunk.GetObjects())
			SyncObject(*o);
	}

	std::unique_ptr<luxcore::RenderConfig> lcConfig {luxcore::RenderConfig::Create(
		luxrays::Property("renderengine.type")("PATHCPU") <<
		luxrays::Property("sampler.type")("SOBOL") <<
		luxrays::Property("batch.halttime")(10.f) <<
		luxrays::Property("film.outputs.1.type")("RGB_IMAGEPIPELINE") <<
		luxrays::Property("film.outputs.1.filename")("image.png"),
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

static luxrays::Matrix4x4 to_luxcore_matrix(const umath::ScaledTransform &t)
{
	auto transform = t.ToMatrix();
	return {
		transform[0][0],transform[0][1],transform[0][2],transform[0][3],
		transform[1][0],transform[1][1],transform[1][2],transform[1][3],
		transform[2][0],transform[2][1],transform[2][2],transform[2][3],
		transform[3][0],transform[3][1],transform[3][2],transform[3][3]
	};
}

void Renderer::SyncObject(const unirender::Object &obj)
{
	std::string propName = "scene.objects." +GetName(obj);
	m_lxScene->Parse(
		luxrays::Property(propName +".material")("test_mat")<<
		luxrays::Property(propName +".shape")(GetName(obj.GetMesh()))
		//luxrays::Property(propName +".transformation")(to_luxcore_matrix(obj.GetPose()))
		// luxrays::Property(propName +".id")("")<<
		// luxrays::Property(propName +".camerainvisible")(false)
	);
	
	propName = "scene.objects." +GetName(obj) +"_strands";
	m_lxScene->Parse(
		luxrays::Property(propName +".material")("test_mat")<<
		luxrays::Property(propName +".shape")(GetName(obj.GetMesh()) +"_strands")
		//luxrays::Property(propName +".transformation")(to_luxcore_matrix(obj.GetPose()))
		// luxrays::Property(propName +".id")("")<<
		// luxrays::Property(propName +".camerainvisible")(false)
	);
}

void Renderer::SyncLight(const unirender::Light &light)
{
	std::string propName = "scene.lights." +GetName(light);
	auto color = light.GetColor() *light.GetIntensity();
	//m_scene->Parse(
	//	luxrays::Property(propName +".gain")(color.r,color.g,color.b)
		// luxrays::Property(propName +".transformation")(to_luxcore_matrix(light.GetPose()))
	//);
	switch(light.GetType())
	{
	case unirender::Light::Type::Spot:
	{
		auto &pos = light.GetPose().GetOrigin();
		auto target = pos +uquat::forward(light.GetPose().GetRotation()) *100.f;
		auto outerConeAngle = light.GetOuterConeAngle();
		auto innerConeAngle = umath::min(light.GetInnerConeAngle(),outerConeAngle);
		m_lxScene->Parse(
			luxrays::Property(propName +".type")("spot")<<
			luxrays::Property(propName +".position")(pos.x,pos.y,pos.z)<<
			luxrays::Property(propName +".target")(target.x,target.y,target.z)<<
			luxrays::Property(propName +".color")(1.0,1.0,1.0)<<
			luxrays::Property(propName +".power")(0.0)<<
			luxrays::Property(propName +".efficency")(0.0)<<
			luxrays::Property(propName +".coneangle")(light.GetOuterConeAngle())<<
			luxrays::Property(propName +".conedeltaangle")(outerConeAngle -innerConeAngle)
		);
		break;
	}
	case unirender::Light::Type::Point:
	{
		auto &pos = light.GetPose().GetOrigin();
		auto dir = uquat::forward(light.GetPose().GetRotation());
		m_lxScene->Parse(
			luxrays::Property(propName +".type")("sphere")<<
			luxrays::Property(propName +".turbidity")(2.2)<<
			luxrays::Property(propName +".relsize")(1.0)<<
			luxrays::Property(propName +".dir")(dir.x,dir.y,dir.z)
		);
		break;
	}
	case unirender::Light::Type::Directional:
	{
		auto &pos = light.GetPose().GetOrigin();
		auto target = pos +uquat::forward(light.GetPose().GetRotation()) *100.f;
		auto outerConeAngle = light.GetOuterConeAngle();
		auto innerConeAngle = umath::min(light.GetInnerConeAngle(),outerConeAngle);
		m_lxScene->Parse(
			luxrays::Property(propName +".type")("sun")<<
			luxrays::Property(propName +".position")(pos.x,pos.y,pos.z)<<
			luxrays::Property(propName +".color")(1.0,1.0,1.0)<<
			luxrays::Property(propName +".power")(0.0)<<
			luxrays::Property(propName +".efficency")(0.0)
		);
		break;
	}
	}
}

static Vector3 calc_hair_normal(const Vector3 &flowNormal,const Vector3 &faceNormal)
{
	auto hairNormal = flowNormal -uvec::project(flowNormal,faceNormal);
	uvec::normalize(&hairNormal);
	return hairNormal;
}

void Renderer::SyncMesh(const unirender::Mesh &mesh)
{
	auto &verts = mesh.GetVertices();
	auto *points = reinterpret_cast<luxrays::Point*>(m_lxScene->AllocVerticesBuffer(verts.size()));
	static_assert(sizeof(luxrays::Point) == sizeof(Vector3));
	static_assert(std::is_same_v<std::remove_const_t<std::remove_reference_t<decltype(verts[0])>>,Vector3>);
	memcpy(points,verts.data(),verts.size() *sizeof(verts[0]));
	
	auto &uvs = mesh.GetPerVertexUvs();
	auto *lxUvs = new luxrays::UV[uvs.size()]; // Note: Will be freed by luxcore
	static_assert(sizeof(uvs[0]) == sizeof(lxUvs[0]));
	memcpy(lxUvs,uvs.data(),uvs.size() *sizeof(uvs[0]));

	static auto flipX = false;
	static auto flipY = false;
	for(auto i=decltype(uvs.size()){0u};i<uvs.size();++i)
	{
		auto &uv = lxUvs[i];
		if(flipX)
			uv.u = 1.f -uv.u;
		
		if(flipY)
			uv.v = 1.f -uv.v;
	}

	auto &normals = mesh.GetVertexNormals();
	auto *lxNormals = new luxrays::Normal[normals.size()]; // Note: Will be freed by luxcore
	static_assert(sizeof(normals[0]) == sizeof(lxNormals[0]));
	memcpy(lxNormals,normals.data(),normals.size() *sizeof(normals[0]));

	auto &tris = mesh.GetTriangles();
	auto numTris = tris.size() /3;
	auto *lxTris = reinterpret_cast<luxrays::Triangle*>(m_lxScene->AllocTrianglesBuffer(numTris));
	for(auto idx=decltype(tris.size()){0u};idx<tris.size();idx+=3)
	{
		auto *tri = &tris[idx];
		auto &lxTri = lxTris[idx /3];
		for(uint8_t i=0;i<3;++i)
			lxTri.v[i] = tri[i];
	}

	auto name = GetName(mesh);
	m_lxScene->DefineMesh(name,verts.size(),numTris,reinterpret_cast<float*>(points),reinterpret_cast<unsigned int*>(lxTris),reinterpret_cast<float*>(lxNormals),reinterpret_cast<float*>(lxUvs),nullptr,nullptr);

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
