/******************************************************************************
 * guacamole - delicious VR                                                   *
 *                                                                            *
 * Copyright: (c) 2011-2013 Bauhaus-Universität Weimar                        *
 * Contact:   felix.lauer@uni-weimar.de / simon.schneegans@uni-weimar.de      *
 *                                                                            *
 * This program is free software: you can redistribute it and/or modify it    *
 * under the terms of the GNU General Public License as published by the Free *
 * Software Foundation, either version 3 of the License, or (at your option)  *
 * any later version.                                                         *
 *                                                                            *
 * This program is distributed in the hope that it will be useful, but        *
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY *
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License   *
 * for more details.                                                          *
 *                                                                            *
 * You should have received a copy of the GNU General Public License along    *
 * with this program. If not, see <http://www.gnu.org/licenses/>.             *
 *                                                                            *
 ******************************************************************************/

#ifndef GUA_RENDERER_HPP
#define GUA_RENDERER_HPP

// external headers
#include <vector>
#include <string>
#include <memory>
#include <map>

#include <gua/platform.hpp>
#include <gua/math.hpp>
#include <gua/utils/FpsCounter.hpp>
#include <gua/concurrent/Doublebuffer.hpp>
#include <gua/renderer/RenderContext.hpp>

#include <scm/gl_core/texture_objects/texture_objects_fwd.h>
#include <scm/gl_core/data_formats.h>
#include <scm/gl_core/constants.h>


namespace gua {

class SceneGraph;

namespace node {
  struct SerializedCameraNode;
  class CameraNode;
}

/**
 * Manages the rendering on multiple contexts.
 *
 * This class is used to provide a renderer frontend interface to the user.
 */
class GUA_DLL Renderer {
 public:
  using SceneGraphs = std::vector<std::unique_ptr<const SceneGraph> >;

  /**
   * Constructor.
   *
   * This constructs a new Renderer.
   *
   * \param pipelines        A vector of Pipelines to process. For each
   *                         pipeline a RenderClient is created.
   */
  Renderer();
  Renderer(Renderer const&) = delete;
  Renderer& operator=(Renderer const&) = delete;

  /**
  *
  */
  ~Renderer();

  /**
   * Request a redraw of all RenderClients.
   *
   * Takes a Scenegraph and asks all clients to draw it.
   *
   * \param scene_graphs      The SceneGraphs to be processed.
   */
  void queue_draw(std::vector<SceneGraph const*> const& scene_graphs, bool enable_warping = false);

  void draw_single_threaded(std::vector<SceneGraph const*> const& scene_graphs);

  void stop();

  inline float get_application_fps() {
    return application_fps_.fps;
  }


 private:

  void send_renderclient(std::string const& window,
                         std::shared_ptr<const Renderer::SceneGraphs> sgs,
                         node::CameraNode* cam,
                         bool enable_warping);

  struct Item {
    Item() = default;
    Item( std::shared_ptr<node::SerializedCameraNode> const& sc,
          std::shared_ptr<const SceneGraphs> const& sgs,
          bool warp = false)
          : serialized_cam(sc), scene_graphs(sgs), enable_warping(warp)
    {}

    std::shared_ptr<node::SerializedCameraNode> serialized_cam;
    std::shared_ptr<const SceneGraphs>          scene_graphs;
    bool                                        enable_warping;
  };

  // Container struct for resources (like textures) needed for the asynchronous warping technique
  // using DBTexture = std::shared_ptr<gua::concurrent::Doublebuffer<scm::gl::texture_2d_ptr>>;
  struct WarpingResources {
    WarpingResources() = default;
    /* WarpingResources( gua::RenderContext const& ctx, gua::math::vec2ui const& resolution)
        : sampler_state_desc(scm::gl::FILTER_MIN_MAG_LINEAR, scm::gl::WRAP_MIRRORED_REPEAT, scm::gl::WRAP_MIRRORED_REPEAT),
          is_left(false,false),
          renderer_ready(false),
          updated(false),
          synch("unsynched") {
            // scm::gl::sampler_state_desc sampler_state_desc();
            std::cout << "Initializing Warping Sampler State ..." << std::endl; 
            sampler_state = ctx.render_device->create_sampler_state(sampler_state_desc);
            
            std::cout << "Initializing Warping Texture Color ..." << std::endl; 

            color_buffer.first = ctx.render_device->create_texture_2d(resolution, scm::gl::FORMAT_RGB_32F, 1);
            ctx.render_context->make_resident(color_buffer.first, sampler_state);
            color_buffer.second = ctx.render_device->create_texture_2d(resolution, scm::gl::FORMAT_RGB_32F, 1);
            ctx.render_context->make_resident(color_buffer.second, sampler_state);

            std::cout << "Initializing Warping Texture Depth ..." << std::endl; 

            depth_buffer.first = ctx.render_device->create_texture_2d(resolution, scm::gl::FORMAT_D24_S8, 1);
            ctx.render_context->make_resident(depth_buffer.first, sampler_state);
            depth_buffer.second = ctx.render_device->create_texture_2d(resolution, scm::gl::FORMAT_D24_S8, 1);
            ctx.render_context->make_resident(depth_buffer.second, sampler_state);
    } */

    WarpingResources& operator=(WarpingResources const& rhs) {
      if (this != &rhs) { 
        std::lock(copy_mutex, rhs.copy_mutex);
        std::lock_guard<std::mutex> m_lhs(copy_mutex, std::adopt_lock);
        std::lock_guard<std::mutex> m_rhs(rhs.copy_mutex, std::adopt_lock);
        color_buffer = rhs.color_buffer;
        depth_buffer = rhs.depth_buffer;
        sampler_state_desc = rhs.sampler_state_desc;
        sampler_state = rhs.sampler_state;
        is_left = rhs.is_left;
        renderer_ready = rhs.renderer_ready;
        updated = rhs.updated;
        rctx = rhs.rctx;
        pixel_data = rhs.pixel_data;
        synch = rhs.synch;
      }
      return *this;
    }

    void init(gua::RenderContext* ctx, gua::math::vec2ui const& resolution) {
      rctx = *ctx;
      synch = "unsynched";
      is_left = std::make_pair<bool,bool>(false,false);
      renderer_ready = false;
      updated = false;

      sampler_state_desc = scm::gl::sampler_state_desc(scm::gl::FILTER_MIN_MAG_LINEAR, scm::gl::WRAP_MIRRORED_REPEAT, scm::gl::WRAP_MIRRORED_REPEAT);
      std::cout << "Initializing Warping Sampler State ..." << std::endl; 
      sampler_state = ctx->render_device->create_sampler_state(sampler_state_desc);
      
      std::cout << "Initializing Warping Texture Color ..." << std::endl; 

      color_buffer.first = ctx->render_device->create_texture_2d(resolution, scm::gl::FORMAT_RGB_32F, 1);
      ctx->render_context->make_resident(color_buffer.first, sampler_state);
      color_buffer.second = ctx->render_device->create_texture_2d(resolution, scm::gl::FORMAT_RGB_32F, 1);
      ctx->render_context->make_resident(color_buffer.second, sampler_state);

      std::cout << "Initializing Warping Texture Depth ..." << std::endl; 

      depth_buffer.first = ctx->render_device->create_texture_2d(resolution, scm::gl::FORMAT_D24_S8, 1);
      ctx->render_context->make_resident(depth_buffer.first, sampler_state);
      depth_buffer.second = ctx->render_device->create_texture_2d(resolution, scm::gl::FORMAT_D24_S8, 1);
      ctx->render_context->make_resident(depth_buffer.second, sampler_state);

      initialized = true;
    }

    void update(const void *in_data, math::vec2ui const& resolution) {
      // std::cout << "creating region ..." << std::endl;
      scm::gl::texture_region region(scm::math::vec3ui(0.0, 0.0, 0.0),
                                     scm::math::vec3ui(resolution.x, resolution.y, 1.0));

      // std::cout << "updating texture ..." << std::endl;
      rctx.render_context->update_sub_texture(color_buffer.second, region, 0,
                                              color_buffer.second->format(),
                                              in_data);
      // std::cout << "updating texture FINISHED" << std::endl;
      updated = true;
    }

    void swap_buffers() {
      if(updated){
        // std::cout << "swapping buffers ..." << std::endl; 
        std::lock_guard<std::mutex> lock(copy_mutex);
        std::swap(color_buffer.first, color_buffer.second);
        std::swap(depth_buffer.first, depth_buffer.second);
        std::swap(is_left.first, is_left.second);
        updated = false;
      }
    }

    std::pair<scm::gl::texture_2d_ptr, scm::gl::texture_2d_ptr> color_buffer;
    std::pair<scm::gl::texture_2d_ptr, scm::gl::texture_2d_ptr> depth_buffer;

    scm::gl::sampler_state_desc sampler_state_desc;
    scm::gl::sampler_state_ptr sampler_state;

    std::pair<bool,bool> is_left;
    bool renderer_ready;
    bool updated;
    bool initialized = false;

    mutable gua::RenderContext rctx;

    std::vector<float> pixel_data;

    std::string synch;

    mutable std::mutex copy_mutex;
  }; 

  using Mailbox = std::shared_ptr<gua::concurrent::Doublebuffer<Item> >;
  using Renderclient = std::pair<Mailbox, std::thread>;
  using Warpclient = std::pair<std::string, std::thread>;

  std::map<std::string, std::shared_ptr<WarpingResources>> warp_resources;

  // standard renderclient
  static void renderclient(Mailbox in, std::string name);
  // renderclients used for warped rendering
  static void renderclient_slow(Mailbox in, std::string name, std::map<std::string, std::shared_ptr<WarpingResources>>&);
  static void renderclient_fast(Mailbox in, std::string name, std::map<std::string, std::shared_ptr<WarpingResources>>&);


  std::map<std::string, Renderclient> render_clients_;
  std::map<std::string, std::pair<Renderclient, Warpclient>> warp_clients_;

  FpsCounter application_fps_;
};

}

#endif  // GUA_RENDERER_HPP
