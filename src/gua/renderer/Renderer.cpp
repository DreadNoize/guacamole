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

// class header
#include <gua/renderer/Renderer.hpp>

// guacamole headers
#include <memory>
#include <tuple>
#include <typeinfo>
#include <thread>

#include <gua/platform.hpp>
#include <gua/scenegraph.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/databases/WindowDatabase.hpp>
#include <gua/node/CameraNode.hpp>
#include <gua/utils.hpp>
#include <gua/concurrent/Doublebuffer.hpp>
#include <gua/concurrent/pull_items_iterator.hpp>
#include <gua/memory.hpp>
#include <gua/config.hpp>
#include <gua/renderer/GlfwWindow.hpp>

namespace {

void display_loading_screen(gua::WindowBase& window) {
  auto loading_texture(
      gua::TextureDatabase::instance()->lookup("gua_loading_texture"));
  gua::math::vec2ui loading_texture_size(loading_texture->width(),
                                         loading_texture->height());

  auto tmp_left_resolution(window.config.left_resolution());
  auto tmp_right_resolution(window.config.right_resolution());

  auto tmp_left_position(window.config.left_position());
  auto tmp_right_position(window.config.right_position());

  window.config.set_left_resolution(loading_texture_size);
  window.config.set_left_position(
      tmp_left_position + (tmp_left_resolution - loading_texture_size) / 2);

  window.config.set_right_resolution(loading_texture_size);
  window.config.set_right_position(
      tmp_right_position + (tmp_right_resolution - loading_texture_size) / 2);

  //window.display(loading_texture);
  window.finish_frame();
  ++(window.get_context()->framecount);

  window.config.set_left_position(tmp_left_position);
  window.config.set_left_resolution(tmp_left_resolution);

  window.config.set_right_position(tmp_right_position);
  window.config.set_right_resolution(tmp_right_resolution);
}

template <class T>
using DB = std::shared_ptr<gua::concurrent::Doublebuffer<T> >;

template <class T>
std::pair<DB<T>, DB<T> > spawnDoublebufferred() {
  auto db = std::make_shared<gua::concurrent::Doublebuffer<T> >();
  return {db, db};
}

}  // namespace

namespace gua {

std::shared_ptr<const Renderer::SceneGraphs> garbage_collected_copy(
    std::vector<SceneGraph const*> const& scene_graphs) {
  auto sgs = std::make_shared<Renderer::SceneGraphs>();
  for (auto graph : scene_graphs) {
    sgs->push_back(gua::make_unique<SceneGraph>(*graph));
  }
  return sgs;
}

Renderer::~Renderer() {
  stop();
}

void Renderer::renderclient(Mailbox in, std::string window_name) {

  FpsCounter fpsc(20);
  fpsc.start();

  for (auto& cmd : gua::concurrent::pull_items_range<Item, Mailbox>(in)) {
    //auto window_name(cmd.serialized_cam->config.get_output_window_name());

    if (window_name != "") {
      auto window = WindowDatabase::instance()->lookup(window_name);

      if (window && !window->get_is_open()) {
        window->open();
      }

      // update window if one is assigned
      if (window && window->get_is_open()) {

        window->set_active(true);
        window->start_frame();

        if (window->get_context()->framecount == 0) {
          display_loading_screen(*window);
        }

        // make sure pipeline was created
        std::shared_ptr<Pipeline> pipe = nullptr;
        auto pipe_iter = window->get_context()->render_pipelines.find(
            cmd.serialized_cam->uuid);

        if (pipe_iter == window->get_context()->render_pipelines.end()) {

          pipe = std::make_shared<Pipeline>(
              *window->get_context(),
              cmd.serialized_cam->config.get_resolution());

          window->get_context()->render_pipelines.insert(
              std::make_pair(cmd.serialized_cam->uuid, pipe));

        } else {
          pipe = pipe_iter->second;
        }

        window->rendering_fps = fpsc.fps;

        if (cmd.serialized_cam->config.get_enable_stereo()) {
          if (window->config.get_stereo_mode() == StereoMode::NVIDIA_3D_VISION) {
            #ifdef GUACAMOLE_ENABLE_NVIDIA_3D_VISION
            if ((window->get_context()->framecount % 2) == 0) {
              auto img(pipe->render_scene(CameraMode::LEFT,  *cmd.serialized_cam, *cmd.scene_graphs));
              if (img) window->display(img, true);
            } else {
              auto img(pipe->render_scene(CameraMode::RIGHT,  *cmd.serialized_cam, *cmd.scene_graphs));
              if (img) window->display(img, false);
            }
            #else
            Logger::LOG_WARNING << "guacamole has not been compiled with NVIDIA 3D Vision support!" << std::endl;
            #endif
          } else if (window->config.get_stereo_mode() == StereoMode::SEPARATE_WINDOWS) {
            bool is_left = cmd.serialized_cam->config.get_left_output_window() == window_name;
            //auto mode = window->config.get_is_left() ? CameraMode::LEFT : CameraMode::RIGHT;
            auto mode = is_left ? CameraMode::LEFT : CameraMode::RIGHT;
            auto img = pipe->render_scene(mode, *cmd.serialized_cam, *cmd.scene_graphs);

            if (img) {
              window->display(img, false);
            }
          } else {
            // TODO: add alternate frame rendering here? -> take clear and render methods
            auto img(pipe->render_scene(CameraMode::LEFT, *cmd.serialized_cam, *cmd.scene_graphs));
            if (img) window->display(img, true);
            img = pipe->render_scene(CameraMode::RIGHT, *cmd.serialized_cam, *cmd.scene_graphs);
            if (img) window->display(img, false);
          }
        } else {
          auto img(pipe->render_scene(cmd.serialized_cam->config.get_mono_mode(),
                  *cmd.serialized_cam, *cmd.scene_graphs));
          if (img) window->display(img, cmd.serialized_cam->config.get_mono_mode() != CameraMode::RIGHT);
        }

        pipe->clear_frame_cache();

        // swap buffers
        window->finish_frame();

        ++(window->get_context()->framecount);

        fpsc.step();

      }
    }

  }
}

void Renderer::renderclient_warp(Mailbox in, std::string window_name, std::map<std::string, std::shared_ptr<Renderer::CustomBuffer>> &warp_res) {

  std::cout << "started renderclient for " << window_name << std::endl;
  

  auto window = WindowDatabase::instance()->lookup(window_name);

  // std::cout << "RENDER: type of main window is " << typeid(main_window).name() << std::endl;

  // std::cout << "RENDER: creating new window for render thread" << std::endl;
  auto window_r = std::make_shared<gua::GlfwWindow>(window->config);


  for (auto& cmd : gua::concurrent::pull_items_range<Item, Mailbox>(in)) {
    //auto window_name(cmd.serialized_cam->config.get_output_window_name());

    if (window_name != "") {
      // WindowDatabase::instance()->add("render_window", r_window);

      // auto window = WindowDatabase::instance()->lookup("render_window");
      // window->config = main_window->config;
      // window->init_context();

      if (window && !window->get_is_open()) {
        // std::cout << "RENDER: opening window..." << std::endl;
        window->open();
      }

      // update window if one is assigned
      if (window && window->get_is_open()) {	    
        // std::cout << "RENDER: setting window active" << std::endl;
        window->set_active(true);
        // std::cout << "RENDER: starting frame..." << std::endl;
        window->start_frame();


        /*  if (window->get_context()->framecount == 0) {
            display_loading_screen(*window);
          } */

          // make sure pipeline was created
        // std::cout << "RENDER: pipeline creation..." << std::endl;
        std::shared_ptr<Pipeline> pipe = nullptr;
        auto pipe_iter = window->get_context()->render_pipelines.find(
            cmd.serialized_cam->uuid);

        if (pipe_iter == window->get_context()->render_pipelines.end()) {
          pipe = std::make_shared<Pipeline>(
              *window->get_context(),
              cmd.serialized_cam->config.get_resolution());

          window->get_context()->render_pipelines.insert(
              std::make_pair(cmd.serialized_cam->uuid, pipe));

          } else {
            pipe = pipe_iter->second;
          }


          // std::cout << "RENDER: starting rendering process..." << std::endl;
          if (cmd.serialized_cam->config.get_enable_stereo()) {
            if (window->config.get_stereo_mode() == StereoMode::NVIDIA_3D_VISION) {
              #ifdef GUACAMOLE_ENABLE_NVIDIA_3D_VISION
              if ((window->get_context()->framecount % 2) == 0) {
                auto img(pipe->render_scene(CameraMode::LEFT,  *cmd.serialized_cam, *cmd.scene_graphs));
                /* if (img) window->display(img, true); */
              } else {
                auto img(pipe->render_scene(CameraMode::RIGHT,  *cmd.serialized_cam, *cmd.scene_graphs));
                /* if (img) window->display(img, false); */
              }
              #else
              Logger::LOG_WARNING << "guacamole has not been compiled with NVIDIA 3D Vision support!" << std::endl;
              #endif
            } else if (window->config.get_stereo_mode() == StereoMode::SEPARATE_WINDOWS) {
              // std::cout << "RENDER: Rendering stereo mode: seperate windows..." << std::endl;
              bool is_left = cmd.serialized_cam->config.get_left_output_window() == window_name;
              //auto mode = window->config.get_is_left() ? CameraMode::LEFT : CameraMode::RIGHT;
              auto mode = is_left ? CameraMode::LEFT : CameraMode::RIGHT;
              auto img = pipe->render_scene(mode, *cmd.serialized_cam, *cmd.scene_graphs);

              warp_res[window_name]->color_buffer.second->push_back(img);
              warp_res[window_name]->depth_buffer.second->push_back(pipe->get_gbuffer()->get_depth_buffer());
              warp_res[window_name]->is_left = is_left;
              /* if (img) {
                window->display(img, false);
              } */
            } else {
              // std::cout << "RENDER: Rendering stereo mode: other..." << std::endl;
              // TODO: add alternate frame rendering here? -> take clear and
              // render methods
              auto img(pipe->render_scene(CameraMode::LEFT, *cmd.serialized_cam, *cmd.scene_graphs));
              /* if (img) window->display(img, true); */
              warp_res[window_name]->color_buffer.second->push_back(img);
              warp_res[window_name]->depth_buffer.second->push_back(pipe->get_gbuffer()->get_depth_buffer());
              warp_res[window_name]->is_left = true;

              img = pipe->render_scene(CameraMode::RIGHT, *cmd.serialized_cam, *cmd.scene_graphs);
              /*  if (img) window->display(img, false); */
              warp_res[window_name]->color_buffer.second->push_back(img);
              warp_res[window_name]->depth_buffer.second->push_back(pipe->get_gbuffer()->get_depth_buffer());
              warp_res[window_name]->is_left = false;
            }
          } else {
            std::cout << "RENDER: Rendering: MONO..." << std::endl;
            std::cout << "RENDER: Context id: " << window->get_context()->id << std::endl;
            auto img(pipe->render_scene(cmd.serialized_cam->config.get_mono_mode(),
                    *cmd.serialized_cam, *cmd.scene_graphs));
            /* std::cout << "RENDER: image stats - width: " << img->dimensions().x
                      << ",\n    height: " << img->dimensions().y 
                      << ",\n    format: " << scm::gl::format_string(img->format())
                      << ",\n    mip level: " << img->mip_map_layers() 
                      << ",\n    array layers: " << img->array_layers()
                      << ",\n    samples: " << img->samples() << std::endl; */
            warp_res[window_name]->color_buffer.second->push_back(img);
            //warp_res[window_name]->color_buffer.second->set_name("COLOR");
            warp_res[window_name]->depth_buffer.second->push_back(pipe->get_gbuffer()->get_depth_buffer());

            warp_res[window_name]->is_left = cmd.serialized_cam->config.get_mono_mode() != CameraMode::RIGHT;
            warp_res[window_name]->renderer_ready = true;
            warp_res[window_name]->synch = "synchronized";

            std::lock_guard<std::mutex> lock(warp_res[window_name]->tex_mutex);
            std::swap(warp_res[window_name]->color_buffer.first,
                      warp_res[window_name]->color_buffer.second);
            std::swap(warp_res[window_name]->depth_buffer.first,
                      warp_res[window_name]->depth_buffer.second);
            // std::cout << "RENDER: image ready: "  << warp_res[window_name].renderer_ready << "for window: " << window_name << std::endl;
            if (img) window->display(warp_res[window_name]->color_buffer.first->read().get(), warp_res[window_name]->is_left);
          }

          pipe->clear_frame_cache();

          // swap buffers
          window->finish_frame();

          ++(window->get_context()->framecount);

      }
    }

  }
}

void Renderer::warpclient(Mailbox in, std::string window_name, std::map<std::string, std::shared_ptr<Renderer::CustomBuffer>> &warp_res) {
  std::cout << "started warpclient for "  << window_name << std::endl;
  FpsCounter fpsc(20);
  fpsc.start();
#if 0
  for (auto& cmd : gua::concurrent::pull_items_range<Item, Mailbox>(in)) {
    if(window_name != "") {
      auto window = WindowDatabase::instance()->lookup(window_name);
      
      if (window && !window->get_is_open()) {
        // std::cout << "WARP: opening window: " << window_name << std::endl;
        window->open();
      }

        // update window if one is assigned
      if (window && window->get_is_open()) {

        // std::cout << "WARP: setting window active: " << window_name << std::endl;
        window->set_active(true);
        window->start_frame();


        if (window->get_context()->framecount == 0) {
          display_loading_screen(*window);
        }
        std::cout << "WARP: threads are " << warp_res[window_name]->synch << std::endl;

        window->rendering_fps = fpsc.fps;
		    // std::this_thread::sleep_for(std::chrono::seconds(1));

		    // std::cout << "WARP: accessing rendered image..." << std::endl;
		    // std::cout << "WARP: image ready: " << warp_res[window_name].renderer_ready<< "for window: " << window_name << std::endl;
        // std::lock_guard<std::mutex> lock(warp_res[window_name]->tex_mutex);
        // std::swap(warp_res[window_name]->color_buffer.first,
        //           warp_res[window_name]->color_buffer.second);
        // std::swap(warp_res[window_name]->depth_buffer.first,
        //           warp_res[window_name]->depth_buffer.second);
                
        auto isLeft(warp_res[window_name]->is_left);
        auto img(warp_res[window_name]->color_buffer.first->read().get());
        //std::cout << "WARP: texture name is " << img->get_name() << std::endl;
        if(img){
          std::cout << "WARP: image stats - width: " << img->dimensions().x
                  << ",\n    height: " << img->dimensions().y 
                  << ",\n    format: " << scm::gl::format_string(img->format())
                  << ",\n    mip level: " << img->mip_map_layers() 
                  << ",\n    array layers: " << img->array_layers()
                  << ",\n    samples: " << img->samples() << std::endl;
        }

        // std::cout << "WARP: Context id: " << window->get_context()->id << std::endl;
        // std::cout << "WARP: Frame: " << window->get_context()->framecount << std::endl;
        if (img && warp_res[window_name]->renderer_ready) {
          std::cout << "WARP: displaying image..." << std::endl;
          window->display(img, isLeft);
        }
        
      }
      // swap buffers
      window->finish_frame();
      ++(window->get_context()->framecount);
      fpsc.step();
    }
  }
  #endif
}

Renderer::Renderer() :
  render_clients_(),
  application_fps_(20) {

  application_fps_.start();
}

void Renderer::send_renderclient(std::string const& window_name,
                         std::shared_ptr<const Renderer::SceneGraphs> sgs,
                         node::CameraNode* cam,
                         bool enable_warping)
{
  if(!enable_warping) {
    auto rclient = render_clients_.find(window_name);
    if (rclient != render_clients_.end()) {
      rclient->second.first->push_back(
          Item(std::make_shared<node::SerializedCameraNode>(cam->serialize()),
          sgs, enable_warping));

    } else {
      if (auto win = WindowDatabase::instance()->lookup(window_name)) {
        auto p = spawnDoublebufferred<Item>();
        p.first->push_back(Item(
            std::make_shared<node::SerializedCameraNode>(cam->serialize()),
            sgs));
        render_clients_[window_name] = std::make_pair(
            p.first, std::thread(Renderer::renderclient, p.second, window_name));
      }
    }
  } else {
    auto win = WindowDatabase::instance()->lookup(window_name);
    if (warp_resources.end() == warp_resources.find(window_name)) {
      auto color = spawnDoublebufferred<scm::gl::texture_2d_ptr>();
      color.first->push_back(scm::gl::texture_2d_ptr());
      color.second->push_back(scm::gl::texture_2d_ptr());
      auto depth = spawnDoublebufferred<scm::gl::texture_2d_ptr>();
      depth.first->push_back(scm::gl::texture_2d_ptr());
      depth.second->push_back(scm::gl::texture_2d_ptr());
      // auto cb = new CustomBuffer(color, depth);
      warp_resources[window_name] = std::make_shared<Renderer::CustomBuffer>(color, depth);
    }
    auto rwclient = warp_clients_.find(window_name); 
    if(rwclient != warp_clients_.end()) { // A render-/warpclient pair already exists
      rwclient->second.first.first->push_back(
        Item(std::make_shared<node::SerializedCameraNode>(cam->serialize()),
          sgs, enable_warping));
    } else { // no pair was found, create a new one
      if (win) {
        auto p = spawnDoublebufferred<Item>();
        p.first->push_back(Item(
            std::make_shared<node::SerializedCameraNode>(cam->serialize()),
            sgs));
        warp_clients_[window_name] = std::make_pair(
                                        std::make_pair(p.first, 
                                                       std::thread(Renderer::renderclient_warp, 
                                                                   p.second, window_name, std::ref(warp_resources))),
                                        std::make_pair(window_name, 
                                                       std::thread(Renderer::warpclient, 
                                                                   p.second, window_name, std::ref(warp_resources)))
        );        
      }
    }
  }
}

void Renderer::queue_draw(std::vector<SceneGraph const*> const& scene_graphs, bool enable_warping) {
  for (auto graph : scene_graphs) {
    graph->update_cache();
  }

  auto sgs = garbage_collected_copy(scene_graphs);

  for (auto graph : scene_graphs) {
    for (auto& cam : graph->get_camera_nodes()) {
      if (cam->config.separate_windows()) {
        send_renderclient(cam->config.get_left_output_window(), sgs, cam, enable_warping);
        send_renderclient(cam->config.get_right_output_window(), sgs, cam, enable_warping);
      } else {
        send_renderclient(cam->config.get_output_window_name(), sgs, cam, enable_warping);
      }
    }
  }
  application_fps_.step();
}

void Renderer::draw_single_threaded(std::vector<SceneGraph const*> const& scene_graphs) {
  for (auto graph : scene_graphs) {
    graph->update_cache();
  }

  auto sgs = garbage_collected_copy(scene_graphs);

  for (auto graph : scene_graphs) {
    for (auto& cam : graph->get_camera_nodes()) {
      auto window_name(cam->config.get_output_window_name());
      auto serialized_cam(cam->serialize());

      if (window_name != "") {
        auto window = WindowDatabase::instance()->lookup(window_name);

        if (window && !window->get_is_open()) {
          window->open();
        }
        // update window if one is assigned
        if (window && window->get_is_open()) {
          window->set_active(true);
          window->start_frame();

          if (window->get_context()->framecount == 0) {
            display_loading_screen(*window);
          }

          // make sure pipeline was created
          std::shared_ptr<Pipeline> pipe = nullptr;
          auto pipe_iter = window->get_context()->render_pipelines.find(
              serialized_cam.uuid);

          if (pipe_iter == window->get_context()->render_pipelines.end()) {
            pipe = std::make_shared<Pipeline>(
                *window->get_context(),
                serialized_cam.config.get_resolution());
            window->get_context()->render_pipelines.insert(
                std::make_pair(serialized_cam.uuid, pipe));
          } else {
            pipe = pipe_iter->second;
          }

          window->rendering_fps = application_fps_.fps;

          if (serialized_cam.config.get_enable_stereo()) {

            if (window->config.get_stereo_mode() == StereoMode::NVIDIA_3D_VISION) {
              #ifdef GUACAMOLE_ENABLE_NVIDIA_3D_VISION
              if ((window->get_context()->framecount % 2) == 0) {
                auto img(pipe->render_scene(CameraMode::LEFT, serialized_cam, *sgs));
                if (img) window->display(img, true);
              } else {
                auto img(pipe->render_scene(CameraMode::RIGHT, serialized_cam, *sgs));
                if (img) window->display(img, false);
              }
              #else
              Logger::LOG_WARNING << "guacamole has not been compiled with NVIDIA 3D Vision support!" << std::endl;
              #endif
            } else {
              auto img(pipe->render_scene(CameraMode::LEFT, serialized_cam, *sgs));
              if (img) window->display(img, true);
              img = pipe->render_scene(CameraMode::RIGHT, serialized_cam, *sgs);
              if (img) window->display(img, false);
            }
          } else {
            auto img(pipe->render_scene(serialized_cam.config.get_mono_mode(),
                     serialized_cam, *sgs));
            if (img) window->display(img, serialized_cam.config.get_mono_mode() != CameraMode::RIGHT);
          }

          pipe->clear_frame_cache();

          // swap buffers
          window->finish_frame();
          ++(window->get_context()->framecount);

        }
      }
    }
  }
  application_fps_.step();
}

void Renderer::stop() {
  for (auto& rc : render_clients_) {
    rc.second.first->close();
  }
  for (auto& rc : render_clients_) {
    rc.second.second.join();
  }
  render_clients_.clear();
}
}
