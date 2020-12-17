/* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/.
*
* Copyright (c) 2020 Florian Weischer
*/

#ifndef __UNIRENDER_CYCLES_SCENE_HPP__
#define __UNIRENDER_CYCLES_SCENE_HPP__

#include <util_raytracing/object.hpp>
#include <util_raytracing/renderer.hpp>
#include <util_raytracing/scene.hpp>
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
		static std::shared_ptr<Renderer> Create(const unirender::Scene &scene);

		virtual ~Renderer() override;
		virtual void Wait() override;
		virtual void Start() override {} // TODO: Remove
		virtual float GetProgress() const override;
		virtual void Reset() override;
		virtual void Restart() override;
		virtual util::ParallelJob<std::shared_ptr<uimg::ImageBuffer>> StartRender() override;
	private:
		Renderer(const Scene &scene);
		static std::string GetName(const BaseObject &obj);
		bool Initialize();
		void SyncCamera(const unirender::Camera &cam);
		void SyncFilm(const unirender::Camera &cam);
		void SyncObject(const unirender::Object &obj);
		void SyncMesh(const unirender::Mesh &mesh);
		void SyncLight(const unirender::Light &light);
		virtual void SetCancelled(const std::string &msg="Cancelled by application.") override;
		virtual void PrepareCyclesSceneForRendering() override;

		std::unique_ptr<luxcore::Scene> m_lxScene = nullptr;
		std::unique_ptr<luxcore::RenderConfig> m_lxConfig = nullptr;
		std::unique_ptr<luxcore::RenderSession> m_lxSession = nullptr;
	};
};

#endif
