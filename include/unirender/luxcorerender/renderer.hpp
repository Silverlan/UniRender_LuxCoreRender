/* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/.
*
* Copyright (c) 2021 Silverlan
*/

#ifndef __UNIRENDER_CYCLES_SCENE_HPP__
#define __UNIRENDER_CYCLES_SCENE_HPP__

#include <util_raytracing/object.hpp>
#include <util_raytracing/renderer.hpp>
#include <util_raytracing/scene.hpp>
#include <luxrays/core/geometry/matrix4x4.h>
#include <luxrays/core/geometry/point.h>
#include <luxrays/core/geometry/triangle.h>
#include <luxrays/core/geometry/uv.h>
#include <luxrays/core/geometry/transform.h>
#include <luxcore/luxcore.h>
#include <util_image_buffer.hpp>
#include <unordered_set>
#include <cinttypes>
#include <atomic>

namespace unirender {class Scene;};
namespace luxcore {class Scene; class RenderConfig; class RenderSession;};
namespace unirender::luxcorerender
{
	class DLLRTUTIL Renderer
		: public unirender::Renderer
	{
	public:
		enum class RenderEngine : uint32_t
		{
			PathTracer = 0,
			TiledPathTracer,
			BidirectionalPathTracer,
			RealTime,

			Bake // For internal use only!
		};

		static std::shared_ptr<Renderer> Create(const unirender::Scene &scene,Flags flags);
		static std::shared_ptr<Renderer> CreateResume();
		static std::string TranslateOutputTypeToLuxCoreRender(const std::string &type);
		static constexpr uint32_t LIGHTMAP_UV_CHANNEL = 1;
		static constexpr const char *LIGHTMAP_ATLAS_OUTPUT_FILENAME = "temp/lightmap_atlas.exr";
		static constexpr const char *OUTPUT_ALPHA = "ALPHA";
		static constexpr const char *OUTPUT_GEOMETRY_NORMAL = "OUTPUT_GEOMETRY_NORMAL";
		static constexpr const char *OUTPUT_SHADING_NORMAL = "OUTPUT_SHADING_NORMAL";
		static constexpr const char *OUTPUT_DIRECT_DIFFUSE = "OUTPUT_DIRECT_DIFFUSE";
		static constexpr const char *OUTPUT_DIRECT_DIFFUSE_REFLECT = "OUTPUT_DIRECT_DIFFUSE_REFLECT";
		static constexpr const char *OUTPUT_DIRECT_DIFFUSE_TRANSMIT = "OUTPUT_DIRECT_DIFFUSE_TRANSMIT";
		static constexpr const char *OUTPUT_DIRECT_GLOSSY = "OUTPUT_DIRECT_GLOSSY";
		static constexpr const char *OUTPUT_DIRECT_GLOSSY_REFLECT = "OUTPUT_DIRECT_GLOSSY_REFLECT";
		static constexpr const char *OUTPUT_DIRECT_GLOSSY_TRANSMIT = "OUTPUT_DIRECT_GLOSSY_TRANSMIT";
		static constexpr const char *OUTPUT_EMISSION = "OUTPUT_EMISSION";
		static constexpr const char *OUTPUT_INDIRECT_DIFFUSE = "OUTPUT_INDIRECT_DIFFUSE";
		static constexpr const char *OUTPUT_INDIRECT_DIFFUSE_REFLECT = "OUTPUT_INDIRECT_DIFFUSE_REFLECT";
		static constexpr const char *OUTPUT_INDIRECT_DIFFUSE_TRANSMIT = "OUTPUT_INDIRECT_DIFFUSE_TRANSMIT";
		static constexpr const char *OUTPUT_INDIRECT_GLOSSY = "OUTPUT_INDIRECT_GLOSSY";
		static constexpr const char *OUTPUT_INDIRECT_GLOSSY_REFLECT = "OUTPUT_INDIRECT_GLOSSY_REFLECT";
		static constexpr const char *OUTPUT_INDIRECT_GLOSSY_TRANSMIT = "OUTPUT_INDIRECT_GLOSSY_TRANSMIT";
		static constexpr const char *OUTPUT_INDIRECT_SPECULAR = "OUTPUT_INDIRECT_SPECULAR";
		static constexpr const char *OUTPUT_INDIRECT_SPECULAR_REFLECT = "OUTPUT_INDIRECT_SPECULAR_REFLECT";
		static constexpr const char *OUTPUT_INDIRECT_SPECULAR_TRANSMIT = "OUTPUT_INDIRECT_SPECULAR_TRANSMIT";
		static constexpr const char *OUTPUT_UV = "OUTPUT_UV";
		static constexpr const char *OUTPUT_IRRADIANCE = "OUTPUT_IRRADIANCE";
		static constexpr const char *OUTPUT_NOISE = "OUTPUT_NOISE";
		static constexpr const char *OUTPUT_CAUSTIC = "OUTPUT_CAUSTIC";
		static std::string GetOutputType(Scene::RenderMode renderMode);
		static luxcore::Film::FilmOutputType GetLuxCoreFilmOutputType(Scene::RenderMode renderMode);
		static uimg::Format GetOutputFormat(Scene::RenderMode renderMode);

		static Vector3 ToPragmaPosition(const luxrays::Vector &pos);
		static luxrays::Vector ToLuxVector(const Vector3 &v);
		static luxrays::Point ToLuxPosition(const Vector3 &pos);
		static luxrays::Normal ToLuxNormal(const Vector3 &n);
		static luxrays::UV ToLuxUV(const Vector2 &uv);
		static umath::ScaledTransform ToLuxTransform(const umath::ScaledTransform &t,bool applyRotOffset=false);
		static float ToLuxLength(float len);

		luxcore::Scene &GetLuxScene() {return *m_lxScene;}
		bool ShouldUsePhotonGiCache() const {return m_enablePhotonGiCache;}
		bool ShouldUseHairShader() const {return m_useHairShader;}
		const std::string &GetCurrentShaderName() const {return m_curShaderName;}

		const std::string &GetDefaultWorldVolume() const {return m_defaultWorldVolume;}
		void SetDefaultWorldVolume(const std::string &vol) {m_defaultWorldVolume = vol;}

		virtual ~Renderer() override;
		virtual void Wait() override;
		virtual void Start() override {} // TODO: Remove
		virtual float GetProgress() const override;
		virtual void Reset() override;
		virtual void Restart() override;
		virtual bool Stop() override;
		virtual bool Pause() override;
		virtual bool Resume() override;
		virtual bool Suspend() override;
		virtual bool BeginSceneEdit() const override;
		virtual bool EndSceneEdit() const override;
		virtual bool Export(const std::string &path) override;
		virtual std::optional<std::string> SaveRenderPreview(const std::string &path,std::string &outErr) const override;
		virtual util::ParallelJob<std::shared_ptr<uimg::ImageBuffer>> StartRender() override;
	private:
		Renderer(const Scene &scene);
		void StopRenderSession();
		void UpdateProgressiveRender();
		static std::string GetName(const BaseObject &obj);
		static std::string GetName(const Object &obj,uint32_t shaderIdx);
		static std::string GetName(const Mesh &mesh,uint32_t shaderIdx);

		// For testing/debugging purposes only!
		bool FinalizeLightmap(const std::string &inputPath,const std::string &outputPath);

		virtual util::EventReply HandleRenderStage(RenderWorker &worker,unirender::Renderer::ImageRenderStage stage,StereoEye eyeStage,unirender::Renderer::RenderStageResult *optResult=nullptr) override;
		virtual bool UpdateStereoEye(unirender::RenderWorker &worker,unirender::Renderer::ImageRenderStage stage,StereoEye &eyeStage) override;
		virtual void CloseRenderScene() override;
		virtual void FinalizeImage(uimg::ImageBuffer &imgBuf,StereoEye eyeStage);
		
		bool Initialize(Flags flags);
		void SyncCamera(const unirender::Camera &cam);
		void SyncFilm(const unirender::Camera &cam);
		void SyncObject(const unirender::Object &obj);
		void SyncMesh(const unirender::Mesh &mesh);
		void SyncLight(const unirender::Light &light);
		virtual void SetCancelled(const std::string &msg="Cancelled by application.") override;
		virtual void PrepareCyclesSceneForRendering() override;

		Flags m_flags = Flags::None;
		std::vector<std::string> m_bakeObjectNames {};
		std::unordered_set<const unirender::Mesh*> m_lightmapMeshes {};
		unirender::Object *m_bakeTarget = nullptr;
		std::vector<std::string> m_objectShaders;
		uint32_t m_shaderNodeIdx = 0;
		uint32_t m_curShaderIdx = 0;
		std::string m_curShaderName;
		std::string m_defaultWorldVolume;
		std::atomic<float> m_progress = 0.f;
		std::unique_ptr<luxcore::Scene> m_lxScene = nullptr;
		std::unique_ptr<luxcore::RenderConfig> m_lxConfig = nullptr;
		std::unique_ptr<luxcore::RenderSession> m_lxSession = nullptr;
		bool m_useHairShader = false;
		
		RenderEngine m_renderEngine = RenderEngine::PathTracer;
		bool m_enablePhotonGiCache = false;
		bool m_enableLightSamplingCache = false;
	};
};

#endif
