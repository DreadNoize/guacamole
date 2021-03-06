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
#include <gua/renderer/SurfaceDetectionPass.hpp>
#include <gua/renderer/WarpGridGeneratorPass.hpp>
#include <gua/databases/WindowDatabase.hpp>
#include <gua/node/CameraNode.hpp>
#include <gua/utils.hpp>
#include <gua/concurrent/Doublebuffer.hpp>
#include <gua/concurrent/pull_items_iterator.hpp>
#include <gua/memory.hpp>
#include <gua/config.hpp>
#include <gua/renderer/GlfwWindow.hpp>
#include <gua/renderer/WarpRenderer.hpp>

#include <scm/gl_core/render_device/opengl/util/binding_guards.h>
#include <scm/gl_core/render_device/opengl/util/data_format_helper.h>

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
using namespace std::chrono_literals;

float Renderer::time_budget = 0;
float Renderer::time_warped = 0;
float Renderer::time_left = 0;

bool Renderer::stereo_ = true;
std::string Renderer::test_scene_ = "4";

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
  auto start_time = std::chrono::high_resolution_clock::now();
  float path_time;
  FpsCounter fpsc(20);
  fpsc.start();
  TextFile single_times;
  if (stereo_) {
    if (test_scene_ == "0") {
      single_times = TextFile("../data/evaluation/single_stereo_teichplatz.txt");
    } else if (test_scene_ == "1") {
      single_times = TextFile("../data/evaluation/single_stereo_teichplatz_ruine.txt");
    } else if (test_scene_ == "2") {
      single_times = TextFile("../data/evaluation/single_stereo_teichplatz_lod.txt");
    } else {
      single_times = TextFile("../data/evaluation/single_stereo_sponza.txt");
    }
  } else {
    if (test_scene_ == "0") {
      single_times = TextFile("../data/evaluation/single_mono_teichplatz.txt");
    } else if (test_scene_ == "1") {
      single_times = TextFile("../data/evaluation/single_mono_teichplatz_ruine.txt");
    } else if (test_scene_ == "2") {
      single_times = TextFile("../data/evaluation/single_mono_teichplatz_lod.txt");
    } else {
      single_times = TextFile("../data/evaluation/single_mono_sponza.txt");
    }
  }

  if (test_scene_ == "1") {
    path_time = 420000;
  } else {    
    path_time = 180000;
  }

  float accumulated_time = 0;
  int counter = 0;
  float avg_fps = 0;
  float avg_frame_time = 0;
  float avg_gpu_time = 0;
  std::stringstream stream;
  stream << "======================== SINGLE THREAD ========================" << std::endl;
  
  for (auto& cmd : gua::concurrent::pull_items_range<Item, Mailbox>(in)) {
    //auto window_name(cmd.serialized_cam->config.get_output_window_name());

    auto frame_time_start = std::chrono::high_resolution_clock::now();
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

        auto fps = ((int)window->get_rendering_fps() > 0)? (int)window->get_rendering_fps(): 100;
        if (0 == window->get_context()->framecount % 250) {
          gua::Logger::LOG_MESSAGE << "[SINGLE] fps: " << window->get_rendering_fps() << std::endl;
        }

        pipe->fetch_gpu_query_results(pipe->get_context());
        float frame_time;
        if (pipe->get_context().framecount % fps == 0) {
          auto query_results = std::make_shared<Pipeline::time_query_collection>(pipe->get_query());
          float total_time = 0;
          // std::cout << "===== Time Queries for Context: " << pipe->get_context().id
          //           << " ============================" << std::endl;
          // for (auto const& t : query_results->results) {
          //   std::cout << t.first << " : " << t.second << " ms" << std::endl;
          //   total_time += t.second;
          // }
          // std::cout << "Total" << " : " << total_time << " ms" << std::endl;
          // std::cout << std::endl;
          auto elapsed_time = (float)std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time).count()/1000;
          if (elapsed_time < path_time && (0 != window->get_rendering_fps())) {
            stream << "===============================================================" << std::endl;
            stream << "==================        FPS:  " << window->get_rendering_fps() << "       ==================" << std::endl;
            std::time_t time_stamp = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            stream << "==================        Elapsed Time: " << elapsed_time << " ms      ==================" << std::endl;
            frame_time = (float)std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - frame_time_start).count()/1000;
            stream << "==================        Frame Time: " << frame_time << " ms      ==================" << std::endl;
            stream << "================== Time now:     " << std::ctime(&time_stamp) << " ==================" << std::endl;
            stream << "===============================================================" << std::endl;
            stream << "================= Time Queries for Context: " << pipe->get_context().id << " =================" << std::endl;
            for (auto const& t : query_results->results) {
              stream << t.first << " : " << t.second << " ms" << std::endl;
              total_time += t.second;
            }
            stream << "Total" << " : " << total_time << " ms" << std::endl;
            stream << std::endl;
            avg_frame_time += frame_time;
            avg_gpu_time += total_time;
            avg_fps += window->get_rendering_fps();
            ++counter;
          } else {
            float afr;
            float aft;
            float agt;
            if (counter > 0) {
              afr = avg_fps / counter;
              aft = avg_frame_time / counter;
              agt = avg_gpu_time / counter;
            }
            stream << "\n========  Average Frame Rate: " << afr << "  ========" << std::endl;
            stream << "\n========  Average GPU Time: " << agt << "  ========" << std::endl;
            stream << "\n========  Average Frame Time: " << aft << "  ========" << std::endl;
            std::cout << "[SINGLE] DONE RECORDING RESULTS" << std::endl;
          }
          query_results->results.clear();
        }

        pipe->clear_frame_cache();

        // swap buffers
        window->finish_frame();

        ++(window->get_context()->framecount);

        fpsc.step();

      }
    }
  }
  single_times.set_content(stream.str());
  single_times.save();
}

void Renderer::renderclient_slow(Mailbox in, std::string window_name, std::map<std::string, std::shared_ptr<Renderer::WarpingResources>> &warp_res) {
  // std::cout << "started renderclient for " << window_name << std::endl;
  auto start_time = std::chrono::high_resolution_clock::now();
  float path_time;
  if (test_scene_ == "1") {
    path_time = 420000;
  } else {    
    path_time = 180000;
  }
  FpsCounter fpsc(20);
  fpsc.start();

  auto application_window = WindowDatabase::instance()->lookup(window_name);
  auto resolution = application_window->config.get_resolution();
  auto offscreen_window = std::make_shared<gua::GlfwWindow>(application_window->config);
  offscreen_window->config.set_title("SLOW CLIENT WINDOW");

  float accumulated_time = 0;
  int counter = 0;
  float avg_fps = 0;
  float avg_frame_time = 0;
  float avg_gpu_time = 0;
  std::stringstream stream;
  stream << "========================= SLOW CLIENT =========================" << std::endl;

  for (auto& cmd : gua::concurrent::pull_items_range<Item, Mailbox>(in)) {
    auto frame_time_start = std::chrono::high_resolution_clock::now();
    {
      gua::Frustum frustum = cmd.serialized_cam->get_rendering_frustum(*(cmd.scene_graphs->front()), gua::CameraMode::CENTER);
      warp_res[window_name]->warp_state.projection_view_center = frustum.get_projection() * frustum.get_view();
      warp_res[window_name]->warp_state.view_center = frustum.get_view();
      warp_res[window_name]->warp_state.projection_center = frustum.get_projection();

      frustum = cmd.serialized_cam->get_rendering_frustum(*(cmd.scene_graphs->front()), gua::CameraMode::LEFT);
      warp_res[window_name]->warp_state.projection_view_left = frustum.get_projection() * frustum.get_view();
      warp_res[window_name]->warp_state.view_left = frustum.get_view();
      warp_res[window_name]->warp_state.projection_left = frustum.get_projection();

      frustum = cmd.serialized_cam->get_rendering_frustum(*(cmd.scene_graphs->front()), gua::CameraMode::RIGHT);
      warp_res[window_name]->warp_state.projection_view_right = frustum.get_projection() * frustum.get_view();
      warp_res[window_name]->warp_state.view_right = frustum.get_view();
      warp_res[window_name]->warp_state.projection_right = frustum.get_projection();
    } 
    //auto window_name(cmd.serialized_cam->config.get_output_window_name());
    // std::cout << "[SLOW] PiplinePass count: " << cmd.serialized_cam->pipeline_description->get_passes().size() << std::endl;
    try {
      cmd.serialized_cam->pipeline_description->get_pass_by_type<gua::SurfaceDetectionPassDescription>();
    } catch (std::runtime_error& e) {
      cmd.serialized_cam->pipeline_description->add_pass(std::make_shared<gua::SurfaceDetectionPassDescription>(warp_res[window_name]));
    }

    if (window_name != "") {
      if (offscreen_window && !offscreen_window->get_is_open()) {
        while (!(warp_res[window_name]->shared)) {
          // std::cout << "[SLOW] waiting for fast window" << std::endl;
        }
        offscreen_window->open(warp_res[window_name]->shared, true);
        warp_res[window_name]->shared_initialized = true;
      }

      // update offscreen_window if one is assigned
      if (offscreen_window && offscreen_window->get_is_open()) {	    
        offscreen_window->set_active(true);
        offscreen_window->start_frame();

        // if(!warp_res[window_name]->initialized) {
        //   warp_res[window_name]->init(offscreen_window->get_context(), offscreen_window->config.get_resolution());
        // }
        while (!warp_res[window_name]->initialized) {
          // std::cout << "[SLOW] waiting on initialization" << std::endl;
        }

        //// as fbo cannot be shared, initialize fbo after warp resources where initialized
        if (warp_res[window_name]->initialized && !warp_res[window_name]->grid_initialized) {
          warp_res[window_name]->init_fbo( offscreen_window->get_context());
          // warp_res[window_name]->init_grid_resources(offscreen_window->get_context(), offscreen_window->config.get_resolution());
        }

        std::shared_ptr<Pipeline> pipe = nullptr;
        auto pipe_iter = offscreen_window->get_context()->render_pipelines.find(
            cmd.serialized_cam->uuid);

        if (pipe_iter == offscreen_window->get_context()->render_pipelines.end()) {
          pipe = std::make_shared<Pipeline>(
              *offscreen_window->get_context(),
              cmd.serialized_cam->config.get_resolution());

          offscreen_window->get_context()->render_pipelines.insert(
              std::make_pair(cmd.serialized_cam->uuid, pipe));

          } else {
            pipe = pipe_iter->second;
          }

          offscreen_window->rendering_fps = fpsc.fps;


          // std::cout << "[RENDER] starting rendering process..." << std::endl;
          if (cmd.serialized_cam->config.get_enable_stereo()) {
            if (offscreen_window->config.get_stereo_mode() == StereoMode::NVIDIA_3D_VISION) {
              #ifdef GUACAMOLE_ENABLE_NVIDIA_3D_VISION
              if ((offscreen_window->get_context()->framecount % 2) == 0) {
                auto img(pipe->render_scene(CameraMode::LEFT,  *cmd.serialized_cam, *cmd.scene_graphs));
                /* if (img) offscreen_window->display(img, true); */
              } else {
                auto img(pipe->render_scene(CameraMode::RIGHT,  *cmd.serialized_cam, *cmd.scene_graphs));
                /* if (img) offscreen_window->display(img, false); */
              }
              #else
              Logger::LOG_WARNING << "guacamole has not been compiled with NVIDIA 3D Vision support!" << std::endl;
              #endif
            } else if (offscreen_window->config.get_stereo_mode() == StereoMode::SEPARATE_WINDOWS) {
              // std::cout << "[RENDER] Rendering stereo mode: seperate windows..." << std::endl;
              // bool is_left = cmd.serialized_cam->config.get_left_output_window() == window_name;
              // //auto mode = offscreen_window->config.get_is_left() ? CameraMode::LEFT : CameraMode::RIGHT;
              // auto mode = is_left ? CameraMode::LEFT : CameraMode::RIGHT;
              // auto img = pipe->render_scene(mode, *cmd.serialized_cam, *cmd.scene_graphs);
              // auto depth = pipe->get_gbuffer()->get_depth_buffer();
              // warp_res[window_name]->camera_mode = mode;


              // void* const tex_ptr = &img;
              // scm::gl::texture_region region(scm::math::vec3ui(0.0,0.0,0.0),scm::math::vec3ui(img->dimensions(),0.0));
              
              // // offscreen_window->get_context()->render_context->update_sub_texture(warp_res[window_name]->std::get<2>(color_buffer), region, 0, img->format(), &tex_ptr);
              
              // void* const depth_ptr = &depth;
              // // offscreen_window->get_context()->render_context->update_sub_texture(warp_res[window_name]->std::get<2>(depth_buffer), region, 0, depth->format(), &depth_ptr);

              // std::get<2>(warp_res[window_name]->is_left) = is_left;
              // /* if (img) {
              //   offscreen_window->display(img, false);
              // } */
            } else {
              // std::cout << "[RENDER] Rendering stereo mode: other..." << std::endl;
              // TODO: add alternate frame rendering here? -> take clear and
              // render methods
              auto img(pipe->render_scene(CameraMode::LEFT, *cmd.serialized_cam, *cmd.scene_graphs));
              // std::get<2>(warp_res[window_name]->is_left) = true;
              warp_res[window_name]->framebuffer = pipe->get_gbuffer()->get_fbo_read();
              warp_res[window_name]->camera_mode = cmd.serialized_cam->config.get_mono_mode();
              if(warp_res[window_name]->initialized_fbo) {
                if (img) {
                  pipe->begin_gpu_query(pipe->get_context(), "GPU: Texture Blitting - LEFT");
                  warp_res[window_name]->postprocess_frame(offscreen_window->get_context(), CameraMode::LEFT);
                  pipe->end_gpu_query(pipe->get_context(), "GPU: Texture Blitting - LEFT");
                  warp_res[window_name]->swap_buffers_slow(CameraMode::LEFT);
                  // offscreen_window->display(img, true);                
                  if(!warp_res[window_name]->left_ready) warp_res[window_name]->left_ready = true;  
                }
              }
              
              img = pipe->render_scene(CameraMode::RIGHT, *cmd.serialized_cam, *cmd.scene_graphs);
              // std::get<2>(warp_res[window_name]->is_left) = false;
              warp_res[window_name]->framebuffer = pipe->get_gbuffer()->get_fbo_read();
              warp_res[window_name]->camera_mode = cmd.serialized_cam->config.get_mono_mode();
              if(warp_res[window_name]->initialized_fbo) {
                if (img) {
                  pipe->begin_gpu_query(pipe->get_context(), "GPU: Texture Blitting - RIGHT");
                  warp_res[window_name]->postprocess_frame(offscreen_window->get_context(), CameraMode::RIGHT);
                  pipe->end_gpu_query(pipe->get_context(), "GPU: Texture Blitting - RIGHT");
                  warp_res[window_name]->swap_buffers_slow(CameraMode::RIGHT);
                  // offscreen_window->display(img, false);
                  if(!warp_res[window_name]->right_ready) warp_res[window_name]->right_ready = true;  
                }
              }
            }
          }
          else {
            // std::cout << "[RENDER] Rendering: MONO..." << std::endl;

            //// Rendering and retireving color and depth buffer
            // Sleep(20);
            // std::string const gpu_query_name = "GPU: Camera uuid: " + std::to_string(pipe->current_viewstate().viewpoint_uuid) + " / SLOW CLIENT"; 
            // std::string const cpu_query_name = "CPU: Camera uuid: " + std::to_string(pipe->current_viewstate().viewpoint_uuid) + " / SLOW CLIENT"; 
            // pipe->begin_gpu_query(pipe->get_context(), gpu_query_name);
            // pipe->begin_cpu_query(cpu_query_name);

            auto img(pipe->render_scene(cmd.serialized_cam->config.get_mono_mode(),
              *cmd.serialized_cam, *cmd.scene_graphs));
            warp_res[window_name]->framebuffer = pipe->get_gbuffer()->get_fbo_read();
            warp_res[window_name]->camera_mode = cmd.serialized_cam->config.get_mono_mode();
            std::get<2>(warp_res[window_name]->is_left) = cmd.serialized_cam->config.get_mono_mode() != CameraMode::RIGHT;
            // as fast client is waiting on the first image, set flag = true

            //// If warping resources are initialized, set the according parameters
            if (warp_res[window_name]->initialized_fbo) {
              if (img) {
                // warp_res[window_name]->swap_shared_resources();
                // warp_res[window_name]->swap_buffers();
                pipe->begin_gpu_query(pipe->get_context(), "GPU: Texture Blitting");
                warp_res[window_name]->postprocess_frame(offscreen_window->get_context(), CameraMode::CENTER);
                pipe->end_gpu_query(pipe->get_context(), "GPU: Texture Blitting");
                warp_res[window_name]->swap_buffers_slow(CameraMode::CENTER);
                offscreen_window->display(img, std::get<0>(warp_res[window_name]->is_left));
                if (!warp_res[window_name]->left_ready) warp_res[window_name]->left_ready = true;
                // offscreen_window->display(warp_res[window_name]->std::get<2>(color_buffer), warp_res[window_name]->std::get<0>(is_left));

              }

            }
            // pipe->end_gpu_query(pipe->get_context(), gpu_query_name);
            // pipe->end_cpu_query(cpu_query_name);
 
        }

        auto fps = ((int)offscreen_window->get_rendering_fps() > 0)? (int)offscreen_window->get_rendering_fps(): 100;
        if (0 == offscreen_window->get_context()->framecount % 250) {
          gua::Logger::LOG_MESSAGE << "[SLOW] fps: " << offscreen_window->get_rendering_fps() << std::endl;
        }

        pipe->fetch_gpu_query_results(pipe->get_context());
        float frame_time;
        if (pipe->get_context().framecount % fps == 0) {
          auto query_results = std::make_shared<Pipeline::time_query_collection>(pipe->get_query());
          float total_time = 0;
          // std::cout << "===== Time Queries for Context: " << pipe->get_context().id
          //   << " ============================" << std::endl;
          // for (auto const& t : query_results->results) {
          //   std::cout << t.first << " : " << t.second << " ms" << std::endl;
          //   total_time += t.second;
          // }
          // std::cout << "Total" << " : " << total_time << " ms" << std::endl;
          // std::cout << std::endl;
          auto elapsed_time = (float)std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time).count()/1000;
          if (elapsed_time < path_time && (0 != offscreen_window->get_rendering_fps())) {
            stream << "===============================================================" << std::endl;
            stream << "==================        FPS: " << offscreen_window->get_rendering_fps() << "       ==================" <<  std::endl; 
            std::time_t time_stamp = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            stream << "==================        Elapsed Time: " << elapsed_time << " ms      ==================" << std::endl;
            frame_time = (float)std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - frame_time_start).count()/1000;
            stream << "==================        Frame Time: " << frame_time << " ms      ==================" << std::endl;
            stream << "================== Time now: " << std::ctime(&time_stamp) << " ==================" << std::endl;
            stream << "===============================================================" << std::endl;
            stream << "================= Time Queries for Context: " << pipe->get_context().id << " =================" << std::endl;
            for (auto const& t : query_results->results) {
              stream << t.first << " : " << t.second << " ms" << std::endl;
              total_time += t.second;
            }
            stream << "Total" << " : " << total_time << " ms" << std::endl;
            stream << std::endl;
            avg_frame_time += frame_time;
            avg_gpu_time += total_time;
            avg_fps += offscreen_window->get_rendering_fps();
            ++counter;
          } else {
            float afr;
            float aft;
            float agt;
            if (counter > 0) {
              afr = avg_fps / counter;
              aft = avg_frame_time / counter;
              agt = avg_gpu_time / counter;
            }
            stream << "\n========  Average Frame Rate: " << afr << "  ========" << std::endl;
            stream << "\n========  Average Frame Rate: " << agt << "  ========" << std::endl;
            stream << "\n========  Average Frame Time: " << aft << "  ========" << std::endl;
            std::cout << "[SLOW] DONE RECORDING RESULTS" << std::endl;
          }
          query_results->results.clear();
        }

        pipe->clear_frame_cache();

        // swap buffers
        offscreen_window->finish_frame();
        // std::cout << "[SLOW] time bygone: " << 1000/offscreen_window->get_rendering_fps() << std::endl;
        ++(offscreen_window->get_context()->framecount);
        fpsc.step();
      }
    }

    warp_res[window_name]->slow_client_times.set_content(stream.str());
    warp_res[window_name]->slow_client_times.save();
  // Sleep(50);
  }
  
}

void Renderer::renderclient_fast(Mailbox in, std::string window_name, std::map<std::string, std::shared_ptr<Renderer::WarpingResources>> &warp_res) {
  // std::cout << "started warpclient for "  << window_name << std::endl;
  auto start_time = std::chrono::high_resolution_clock::now();
  float path_time;
  if (test_scene_ == "1") {
    path_time = 420000;
  } else {    
    path_time = 180000;
  }
  FpsCounter fpsc(20);
  fpsc.start();
  float accumulated_time = 0;
  int counter = 0;
  std::stringstream stream;
  stream << "========================= FAST CLIENT =========================" << std::endl;
  float avg_fps = 0;
  float avg_frame_time = 0;
  float avg_gpu_time = 0;

  for (auto& cmd : gua::concurrent::pull_items_range<Item, Mailbox>(in)) {
    // std::cout << "[FAST] Camera id: " << cmd.serialized_cam->uuid << std::endl;
    auto frame_time_start = std::chrono::high_resolution_clock::now();
    {
      // auto warp_cam = std::make_shared<gua::node::CameraNode>("Warp_Cam", std::make_shared<PipelineDescription>(), cmd.serialized_cam->config, cmd.serialized_cam->transform);
      auto warp_cam = std::make_shared<gua::node::CameraNode>();
      // std::cout << "[FAST] scenegraphs camera vector size: " << cmd.scene_graphs->front()->get_camera_nodes().size() << std::endl;
      for (auto cam : cmd.scene_graphs->front()->get_camera_nodes()) {
        // std::cout << "[SLOW] Camera Name: " << cam->get_name() << "; Camera id: " << cam->uuid() << std::endl;
        // auto frust_left = cam->get_rendering_frustum(*(cmd.scene_graphs->front()), CameraMode::LEFT);
        // auto frust_right = cam->get_rendering_frustum(*(cmd.scene_graphs->front()), CameraMode::RIGHT);
        // std::cout << "[FAST Camera: " << cam->get_name() << std::endl;
        // gua::WarpRenderer::print_matrix(frust_left.get_view(), "VIEW LEFT"); 
        // gua::WarpRenderer::print_matrix(frust_right.get_view(), "VIEW RIGHT"); 
        // WarpRenderer::print_matrix(cam->get_world_transform(), "[FAST] Camera: " + cam->get_name());
        if (cam == nullptr) {
          warp_cam = std::make_shared<gua::node::CameraNode>("Warp_Cam", std::make_shared<PipelineDescription>(), cmd.serialized_cam->config, cmd.serialized_cam->transform);
        }
        else if ("warp_cam" == cam->get_name()) {
          // WarpRenderer::print_matrix(cam->get_world_transform(), "[FAST] Warp Cam");
          warp_cam = std::make_shared<gua::node::CameraNode>(*cam);
          // WarpRenderer::print_matrix(warp_cam->get_world_transform(), "[FAST] New Warp Cam");
        }
      }
      // auto graph = (cmd.scene_graphs->front());
      warp_cam->get_pipeline_description()->clear();
      warp_cam->get_pipeline_description()->add_pass(std::make_shared<gua::WarpGridGeneratorPassDescription>(warp_res[window_name]));
      warp_cam->get_pipeline_description()->add_pass(std::make_shared<gua::WarpPassDescription>(warp_res[window_name]));
      // std::cout << "[FAST] number of passes: " << warp_cam->get_pipeline_description()->get_passes().size() << std::endl;

      warp_res[window_name]->serialized_warp_cam = std::make_shared<node::SerializedCameraNode>(warp_cam->serialize());
      // std::cout << "[FAST] Warp Camera id: " << warp_res[window_name]->serialized_warp_cam->uuid << std::endl;
    }
	  // std::cout << "[FAST] TIMING AFTER CAM INIT: " << (float)std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - frame_time_start).count() / 1000 << std::endl;
    // if(!warp_res[window_name]->initialized) {
    //   gua::Frustum frustum = warp_res[window_name]->serialized_warp_cam->get_rendering_frustum(*(cmd.scene_graphs->front()), gua::CameraMode::CENTER);
    //   // gua::Frustum frustum = cmd.serialized_cam->get_rendering_frustum(*(cmd.scene_graphs->front()), gua::CameraMode::CENTER);
    //   warp_res[window_name]->warp_state.projection_view_center = frustum.get_projection() * frustum.get_view();

    //   frustum = warp_res[window_name]->serialized_warp_cam->get_rendering_frustum(*(cmd.scene_graphs->front()), gua::CameraMode::LEFT);
    //   // frustum = cmd.serialized_cam->get_rendering_frustum(*(cmd.scene_graphs->front()), gua::CameraMode::LEFT);
    //   warp_res[window_name]->warp_state.projection_view_left = frustum.get_projection() * frustum.get_view();

    //   frustum = warp_res[window_name]->serialized_warp_cam->get_rendering_frustum(*(cmd.scene_graphs->front()), gua::CameraMode::RIGHT);
    //   // frustum = cmd.serialized_cam->get_rendering_frustum(*(cmd.scene_graphs->front()), gua::CameraMode::RIGHT);
    //   warp_res[window_name]->warp_state.projection_view_right = frustum.get_projection() * frustum.get_view();
    // }
    // {
    //   // gua::Frustum frustum = warp_res[window_name]->serialized_warp_cam->get_rendering_frustum(*(cmd.scene_graphs->front()), gua::CameraMode::CENTER);
    //   gua::Frustum frustum = cmd.serialized_cam->get_rendering_frustum(*(cmd.scene_graphs->front()), gua::CameraMode::CENTER);
    //   warp_res[window_name]->warp_state.projection_view_center = frustum.get_projection() * frustum.get_view();
    //   warp_res[window_name]->warp_state.view_center = frustum.get_view();
    //   warp_res[window_name]->warp_state.projection_center = frustum.get_projection();

    //   // frustum = warp_res[window_name]->serialized_warp_cam->get_rendering_frustum(*(cmd.scene_graphs->front()), gua::CameraMode::LEFT);
    //   frustum = cmd.serialized_cam->get_rendering_frustum(*(cmd.scene_graphs->front()), gua::CameraMode::LEFT);
    //   warp_res[window_name]->warp_state.projection_view_left = frustum.get_projection() * frustum.get_view();
    //   warp_res[window_name]->warp_state.view_left = frustum.get_view();
    //   warp_res[window_name]->warp_state.projection_left = frustum.get_projection();

    //   // frustum = warp_res[window_name]->serialized_warp_cam->get_rendering_frustum(*(cmd.scene_graphs->front()), gua::CameraMode::RIGHT);
    //   frustum = cmd.serialized_cam->get_rendering_frustum(*(cmd.scene_graphs->front()), gua::CameraMode::RIGHT);
    //   warp_res[window_name]->warp_state.projection_view_right = frustum.get_projection() * frustum.get_view();
    //   warp_res[window_name]->warp_state.view_right = frustum.get_view();
    //   warp_res[window_name]->warp_state.projection_right = frustum.get_projection();

    // }

    if(window_name != "") {
      auto window = WindowDatabase::instance()->lookup(window_name);
      
      if (window && !window->get_is_open()) {
        window->open();
        auto casted_window = std::dynamic_pointer_cast<gua::GlfwWindow>(window);
        warp_res[window_name]->shared = casted_window->get_glfw_window();
      }

        // update window if one is assigned
      if (window && window->get_is_open()) {
        while(!warp_res[window_name]->shared_initialized) {
          // std::cout << "[FAST] waiting for offscreen window to get initialized" << std::endl;
        }
        auto frame_time_start_2 = std::chrono::high_resolution_clock::now();
        window->set_active(true);
        auto set_active_time = (float)std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - frame_time_start_2).count() / 1000;
        window->start_frame();
        // if warp resources arent initialized, do it now
        if(!warp_res[window_name]->initialized) {
          warp_res[window_name]->init(window->get_context(), window->config.get_resolution());
        }

        //// create warping pipeline
        std::shared_ptr<Pipeline> pipe = nullptr;
        auto pipe_iter = window->get_context()->render_pipelines.find(
          warp_res[window_name]->serialized_warp_cam->uuid);

        if (pipe_iter == window->get_context()->render_pipelines.end()) {
          pipe = std::make_shared<Pipeline>(
              *window->get_context(),
              warp_res[window_name]->serialized_warp_cam->config.get_resolution());

          window->get_context()->render_pipelines.insert(
              std::make_pair(warp_res[window_name]->serialized_warp_cam->uuid, pipe));

          } else {
            pipe = pipe_iter->second;
          }

        if (window->get_context()->framecount == 0) {
          display_loading_screen(*window);
        }
        // std::cout << "[WARP] threads are " << warp_res[window_name]->synch << std::endl;

        window->rendering_fps = fpsc.fps;
        auto swap_time_start = std::chrono::high_resolution_clock::now();
        float swap_time = 0;
        float render_time = 0;
        float display_time = 0;
        if(cmd.serialized_cam->config.get_enable_stereo()) {
          if(window->config.get_stereo_mode() == StereoMode::NVIDIA_3D_VISION) {
            #if GUACAMOLE_ENABLE_NVIDIA_3D_VISION
            #else
              Logger::LOG_WARNING << "guacamole has not been compiled with NVIDIA 3D Vision support!" << std::endl;
            #endif
          } else if (window->config.get_stereo_mode() == StereoMode::SEPARATE_WINDOWS) {

          } else {
            // std::string const gpu_query_name = "GPU: Camera uuid: " + std::to_string(pipe->current_viewstate().viewpoint_uuid) + " / FAST CLIENT"; 
            // pipe->begin_gpu_query(pipe->get_context(), gpu_query_name);

            //// if the slow client rendered for the first time, start display
            // if ((window->get_context()->framecount % 2) == 0) {  
              if(warp_res[window_name]->initialized && warp_res[window_name]->left_ready) {
                // std::cout << "[FAST] left eye" << std::endl;
                warp_res[window_name]->swap_surface_buffer_fast(CameraMode::LEFT);
                warp_res[window_name]->swap_buffers_fast(CameraMode::LEFT);
                auto tex = pipe->render_scene(gua::CameraMode::LEFT, *warp_res[window_name]->serialized_warp_cam, *cmd.scene_graphs);
                window->display(tex, true);
              }
            // } else {
              if(warp_res[window_name]->initialized && warp_res[window_name]->right_ready){
                // std::cout << "[FAST] right eye" << std::endl;
                warp_res[window_name]->swap_surface_buffer_fast(CameraMode::RIGHT);
                warp_res[window_name]->swap_buffers_fast(CameraMode::RIGHT);
                auto tex = pipe->render_scene(gua::CameraMode::RIGHT, *warp_res[window_name]->serialized_warp_cam, *cmd.scene_graphs);
                window->display(tex, false);
                // display
              }
            // }
                  
            // pipe->end_gpu_query(pipe->get_context(), gpu_query_name);
            /*pipe->fetch_gpu_query_results(pipe->get_context());
            auto query_results = pipe->get_query();
            time_bygone = query_results.results[gpu_query_name];*/
          }
        } else {
          // std::string const gpu_query_name = "GPU: Camera uuid: " + std::to_string(pipe->current_viewstate().viewpoint_uuid) + " / FAST CLIENT"; 
          // std::string const cpu_query_name = "CPU: Camera uuid: " + std::to_string(pipe->current_viewstate().viewpoint_uuid) + " / FAST CLIENT";
          // pipe->begin_gpu_query(pipe->get_context(), gpu_query_name);
          // pipe->begin_cpu_query(cpu_query_name);

          //// if the slow client rendered for the first time, start display
          if(warp_res[window_name]->initialized && warp_res[window_name]->left_ready) {
            warp_res[window_name]->swap_surface_buffer_fast(CameraMode::CENTER);
            warp_res[window_name]->swap_buffers_fast(CameraMode::CENTER);
            swap_time = (float)std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - swap_time_start).count()/1000;
            swap_time_start = std::chrono::high_resolution_clock::now();
            auto tex = pipe->render_scene(warp_res[window_name]->camera_mode, *warp_res[window_name]->serialized_warp_cam, *cmd.scene_graphs);
            render_time = (float)std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - swap_time_start).count()/1000;
            // display
            // std::cout << "[FAST] tex adress: " << tex->native_handle() << std::endl;
            swap_time_start = std::chrono::high_resolution_clock::now();
            window->display(tex, std::get<0>(warp_res[window_name]->is_left));
            display_time = (float)std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - swap_time_start).count()/1000;
            // window->display(std::get<0>(warp_res[window_name]->color_buffer), std::get<0>(warp_res[window_name]->is_left));
            // std::cout << "[FAST] color buffer.first adress: " << warp_res[window_name]->std::get<0>(color_buffer)->native_handle() << std::endl;
            // std::cout << "[FAST] color buffer.second adress: " << warp_res[window_name]->std::get<2>(color_buffer)->native_handle() << std::endl;
            // std::cout << "Swap Time: " << swap_time << " | Render Time: " << render_time << " | Display Time: " << display_time << std::endl;
            // window->display(warp_res[window_name]->std::get<0>(color_buffer), warp_res[window_name]->std::get<0>(is_left));
            // window->display(temp_tex, warp_res[window_name]->std::get<0>(is_left));
          }
                
          // pipe->end_gpu_query(pipe->get_context(), gpu_query_name);
          // auto query_results = pipe->get_query();
          // time_bygone = query_results.results[gpu_query_name];
        }
        auto fps = ((int)window->get_rendering_fps() > 0)? (int)window->get_rendering_fps(): 100;
        if (0 == window->get_context()->framecount % 250) {
          gua::Logger::LOG_MESSAGE << "[FAST] fps: " << window->get_rendering_fps() << std::endl;
        }

        pipe->fetch_gpu_query_results(pipe->get_context());
        float total_time = 0;
        auto query_results = std::make_shared<Pipeline::time_query_collection>(pipe->get_query());
        for (auto const& t : query_results->results) {
            total_time += t.second;
        }
        
        float frame_time;
        if (pipe->get_context().framecount % fps == 0) {
          auto end = std::chrono::high_resolution_clock::now();
          // std::cout << "[FAST] GPU Time: " << total_time << " | Actual time: " << (float)std::chrono::duration_cast<std::chrono::microseconds>(end - frame_time_start).count()/1000 << std::endl;
          // std::cout << "===== Time Queries for Context: " << pipe->get_context().id
          //           << " ============================" << std::endl;
          // for (auto const& t : query_results->results) {
          //   std::cout << t.first << " : " << t.second << " ms" << std::endl;
          // }
          // std::cout << "Total" << " : " << total_time << " ms" << std::endl;
          // std::cout << std::endl;
          auto elapsed_time = (float)std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time).count()/1000;
          if (elapsed_time < path_time && (0 != window->get_rendering_fps()) && (0 != total_time)) {
            stream << "===============================================================" << std::endl;
            stream << "==================        FPS: " << window->get_rendering_fps() << "       ==================" << std::endl;
            stream << "==================        Elapsed Time: " << elapsed_time << " ms      ==================" << std::endl;
            frame_time = (float)std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - frame_time_start).count()/1000;
            stream << "==================        Frame Time: " << frame_time << " ms      ==================" << std::endl;
            std::time_t time_stamp = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            stream << "================== Time now:     " << std::ctime(&time_stamp) << " ==================" << std::endl;
            stream << "===============================================================" << std::endl;
            stream << "================= Time Queries for Context: " << pipe->get_context().id << " =================" << std::endl;
            for (auto const& t : query_results->results) {
              stream << t.first << " : " << t.second << " ms" << std::endl;
            }
            stream << "Total" << " : " << total_time << " ms" << std::endl;
            stream << std::endl;
            avg_frame_time += frame_time;
            avg_gpu_time += total_time;
            avg_fps += window->get_rendering_fps();
            ++counter;
          } else {
            float afr;
            float aft;
            float agt;
            if (counter > 0) {
              afr = avg_fps / counter;
              aft = avg_frame_time / counter;
              agt = avg_gpu_time / counter;
            }
            stream << "\n========  Average Frame Rate: " << afr << "  ========" << std::endl;
            stream << "\n========  Average GPU Time: " << agt << "  ========" << std::endl;
            stream << "\n========  Average Frame Time: " << aft << "  ========" << std::endl;
            std::cout << "[FAST] DONE RECORDING RESULTS" << std::endl;
          }

        }
        // std::cout << "[FAST] is textfile valid: " << (warp_res[window_name]->fast_client_times.is_valid()? "Yo" : "Nope") << std::endl;
        query_results->results.clear();
        

        // warp_res[window_name]->time_left = warp_res[window_name]->time_budget - time_bygone;
        // pipe->end_cpu_query(cpu_query_name);
        // while (warp_res[window_name]->time_left > 1) {
        //   warp_res[window_name]->time_left -= 0.1;
        //   // std::cout << "[FAST] TIME LEFT: " << warp_res[window_name]->time_left << std::endl;
        //   Sleep(0.1);
        // }

        pipe->clear_frame_cache();
        window->finish_frame();
        time_warped = time_budget - frame_time;
        time_left = time_warped;
        // std::cout << "[FAST] time left: " << time_left << std::endl;
        while (time_warped >= time_left > 1.0) {
          // std::this_thread::sleep_for(0.09ms);
          time_left -= 0.1;
          // std::cout << "[FAST] time left: " << time_left << std::endl;
        }
        time_left = time_budget;
        ++(window->get_context()->framecount);
        fpsc.step();
      }
      // swap buffers
    }

    warp_res[window_name]->fast_client_times.set_content(stream.str());
    warp_res[window_name]->fast_client_times.save();
  }
}

Renderer::Renderer() :
  render_clients_(),
  application_fps_(20) {
  
  application_fps_.start();
}

void Renderer::send_renderclient(std::string const& window_name,
                         std::shared_ptr<const Renderer::SceneGraphs> sgs,
                         node::CameraNode* cam,
                         bool enable_warping,
                         int const desired_framerate)
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
      warp_resources[window_name] = std::make_shared<Renderer::WarpingResources>();
      warp_resources[window_name]->choose_scene();
      // calculate time budget in ms
      time_budget = float(1000.0/desired_framerate);
      std::cout << "Time budget: " << time_budget << std::endl;
    }
	// std::cout << "Generating clients ..." << std::endl;
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
                                                       std::thread(Renderer::renderclient_slow, 
                                                                   p.second, window_name, std::ref(warp_resources))),
                                        std::make_pair(window_name, 
                                                       std::thread(Renderer::renderclient_fast, 
                                                                   p.second, window_name, std::ref(warp_resources)))
        );        
      }
    }
  }
}

void Renderer::queue_draw(std::vector<SceneGraph const*> const& scene_graphs, bool enable_warping, int const desired_framerate) {
	// std::cout << "RENDERER: starting queue draw ..." << std::endl;
  for (auto graph : scene_graphs) {
    graph->update_cache();
  }

  auto sgs = garbage_collected_copy(scene_graphs);

  for (auto graph : scene_graphs) {
    for (auto& cam : graph->get_camera_nodes()) {
      if (cam->config.separate_windows()) {
        send_renderclient(cam->config.get_left_output_window(), sgs, cam, enable_warping, desired_framerate);
        send_renderclient(cam->config.get_right_output_window(), sgs, cam, enable_warping, desired_framerate);
      } else {
        send_renderclient(cam->config.get_output_window_name(), sgs, cam, enable_warping, desired_framerate);
      }
    }
  }
  application_fps_.step();
}

//void Renderer::draw_single_threaded(std::vector<SceneGraph const*> const& scene_graphs) {
//  for (auto graph : scene_graphs) {
//    graph->update_cache();
//  }
//
//  auto sgs = garbage_collected_copy(scene_graphs);
//
//  for (auto graph : scene_graphs) {
//    for (auto& cam : graph->get_camera_nodes()) {
//      auto window_name(cam->config.get_output_window_name());
//      auto serialized_cam(cam->serialize());
//
//      if (window_name != "") {
//        auto window = WindowDatabase::instance()->lookup(window_name);
//
//        if (window && !window->get_is_open()) {
//          window->open();
//        }
//        // update window if one is assigned
//        if (window && window->get_is_open()) {
//          window->set_active(true);
//          window->start_frame();
//
//          if (window->get_context()->framecount == 0) {
//            display_loading_screen(*window);
//          }
//
//          // make sure pipeline was created
//          std::shared_ptr<Pipeline> pipe = nullptr;
//          auto pipe_iter = window->get_context()->render_pipelines.find(
//              serialized_cam.uuid);
//
//          if (pipe_iter == window->get_context()->render_pipelines.end()) {
//            pipe = std::make_shared<Pipeline>(
//                *window->get_context(),
//                serialized_cam.config.get_resolution());
//            window->get_context()->render_pipelines.insert(
//                std::make_pair(serialized_cam.uuid, pipe));
//          } else {
//            pipe = pipe_iter->second;
//          }
//
//          window->rendering_fps = application_fps_.fps;
//
//          if (serialized_cam.config.get_enable_stereo()) {
//
//            if (window->config.get_stereo_mode() == StereoMode::NVIDIA_3D_VISION) {
//              #ifdef GUACAMOLE_ENABLE_NVIDIA_3D_VISION
//              if ((window->get_context()->framecount % 2) == 0) {
//                auto img(pipe->render_scene(CameraMode::LEFT, serialized_cam, *sgs));
//                if (img) window->display(img, true);
//              } else {
//                auto img(pipe->render_scene(CameraMode::RIGHT, serialized_cam, *sgs));
//                if (img) window->display(img, false);
//              }
//              #else
//              Logger::LOG_WARNING << "guacamole has not been compiled with NVIDIA 3D Vision support!" << std::endl;
//              #endif
//            } else {
//              auto img(pipe->render_scene(CameraMode::LEFT, serialized_cam, *sgs));
//              if (img) window->display(img, true);
//              img = pipe->render_scene(CameraMode::RIGHT, serialized_cam, *sgs);
//              if (img) window->display(img, false);
//            }
//          } else {
//            auto img(pipe->render_scene(serialized_cam.config.get_mono_mode(),
//                     serialized_cam, *sgs));
//            if (img) window->display(img, serialized_cam.config.get_mono_mode() != CameraMode::RIGHT);
//          }
//
//          pipe->clear_frame_cache();
//
//          // swap buffers
//          window->finish_frame();
//          ++(window->get_context()->framecount);
//
//        }
//      }
//    }
//  }
//  application_fps_.step();
//}

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
