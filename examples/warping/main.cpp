/******************************************************************************
 * guacamole - delicious VR                                                   *
 *                                                                            *
 * Copyright: (c) 2011-2018 Bauhaus-Universität Weimar                        *
 * Contact:   yorrick.paolo.sieler-morzuch@uni-weimar.de                      *
 *            joachim.billert@uni-weimar.de                                   *
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

#define ENABLE_LOD false
#define ENABLE_HMD true
#include <functional>

#include <gua/guacamole.hpp>
#include <gua/renderer/TriMeshLoader.hpp>
#include <gua/renderer/ToneMappingPass.hpp>
#include <gua/renderer/SSAAPass.hpp>
#include <gua/renderer/BBoxPass.hpp>
#include <gua/renderer/DebugViewPass.hpp>
#include <gua/renderer/LineStripPass.hpp>
#include <gua/renderer/LightVisibilityPass.hpp>
#include <gua/renderer/TexturedQuadPass.hpp>
#include <gua/renderer/TexturedScreenSpaceQuadPass.hpp>

#if ENABLE_LOD
#include <gua/renderer/LodLoader.hpp>
#include <gua/renderer/PLodPass.hpp>
#include <gua/renderer/MLodPass.hpp>
#include <gua/node/PLodNode.hpp>
#include <gua/node/MLodNode.hpp>
#endif

#if ENABLE_HMD
#include <gua/ViveWindow.hpp>
#endif

#include <gua/utils/Trackball.hpp>
#include <Navigator.hpp>

bool manipulation_navigator = true;
bool manipulation_camera = false;
bool warping = true;
bool stereo = true;


/* scenegraph overview for "main_scenegraph"
  /
  └ screen
    └ camera
*/
// forward mouse interaction to trackball
void mouse_button (gua::utils::Trackball& trackball, int mousebutton, int action, int mods) {
  gua::utils::Trackball::button_type button;
  gua::utils::Trackball::state_type state;

  switch (mousebutton) {
    case 0: button = gua::utils::Trackball::left; break;
    case 2: button = gua::utils::Trackball::middle; break;
    case 1: button = gua::utils::Trackball::right; break;
  };

  switch (action) {
    case 0: state = gua::utils::Trackball::released; break;
    case 1: state = gua::utils::Trackball::pressed; break;
  };

  trackball.mouse(button, state, trackball.posx(), trackball.posy());
}

int main(int argc, char** argv) {
  // initialize guacamole
  gua::init(argc, argv);

  // initialize movement and mouse interaction
  gua::utils::Trackball object_trackball(0.01, 0.002, 0.2);
  Navigator nav;
  Navigator warp_nav;
  nav.set_transform(scm::math::make_translation(0.f, 0.f, 0.f));
  warp_nav.set_transform(scm::math::make_translation(0.f, 0.f, 0.f));

  // initialize scenegraph
  gua::SceneGraph graph("main_scenegraph");

  // initialize trimeshloader for model loading
  gua::TriMeshLoader loader;

#if ENABLE_LOD
  // create simple untextured material shader
  auto lod_keep_input_desc = std::make_shared<gua::MaterialShaderDescription>("./data/materials/PLOD_use_input_color.gmd");
  auto lod_keep_color_shader(std::make_shared<gua::MaterialShader>("PLOD_pass_input_color", lod_keep_input_desc));
  gua::MaterialShaderDatabase::instance()->add(lod_keep_color_shader);

  //create material for pointcloud
  auto lod_rough = lod_keep_color_shader->make_new_material();
  lod_rough->set_uniform("metalness", 0.0f);
  lod_rough->set_uniform("roughness", 0.3f);
  lod_rough->set_uniform("emissivity", 0.0f);

  //configure lod backend
  gua::LodLoader lod_loader;
  lod_loader.set_out_of_core_budget_in_mb(4096);
  lod_loader.set_render_budget_in_mb(1024);
  lod_loader.set_upload_budget_in_mb(30);
#endif

  // create a transform node
  // which will be attached to the scenegraph
  auto transform = graph.add_node<gua::node::TransformNode>("/", "transform");

#if ENABLE_LOD
  auto mlod_transform = graph.add_node<gua::node::TransformNode>("/transform", "mlod_transform");
  auto plod_transform = graph.add_node<gua::node::TransformNode>("/transform", "plod_transform");
  auto tri_transform = graph.add_node<gua::node::TransformNode>("/transform", "tri_transform");

  auto plod_node = lod_loader.load_lod_pointcloud(
      "pointcloud",
      "/opt/3d_models/lamure/plod/pig_pr.bvh",
      lod_rough,
      gua::LodLoader::NORMALIZE_POSITION | gua::LodLoader::NORMALIZE_SCALE |
          gua::LodLoader::MAKE_PICKABLE);

  graph.add_node("/transform/plod_transform", plod_node);

  plod_transform->rotate(90.0, 0.0, 1.0, 0.0);
  // plod_transform->rotate(180.0, 0.0, 1.0, 0.0);
  plod_transform->translate(0.3, 0.08, 0.0);
#else
  // the model will be attached to the transform node
  auto geometry(loader.create_geometry_from_file(
      "geometry",  "../data/objects/old-house-2.obj",  gua::TriMeshLoader::OPTIMIZE_GEOMETRY | gua::TriMeshLoader::NORMALIZE_POSITION |
      gua::TriMeshLoader::LOAD_MATERIALS | gua::TriMeshLoader::OPTIMIZE_MATERIALS |
      gua::TriMeshLoader::NORMALIZE_SCALE));
  // geometry->translate(-0.6, 0.0, 0.0);
  //geometry->translate(0.0, -50.0,-50.0);
  //geometry->scale(0.005);
  graph.add_node("/transform", geometry);
#endif


  auto sun_light = graph.add_node<gua::node::LightNode>("/", "sun_light");
  sun_light->data.set_type(gua::node::LightNode::Type::SUN);
  sun_light->data.set_color(gua::utils::Color3f(1.5f, 1.2f, 1.f));
  sun_light->data.set_shadow_cascaded_splits({ 0.1f, 1.5, 5.f, 10.f });
  sun_light->data.set_shadow_near_clipping_in_sun_direction(100.0f);
  sun_light->data.set_shadow_far_clipping_in_sun_direction(100.0f);
  sun_light->data.set_max_shadow_dist(30.0f);
  sun_light->data.set_shadow_offset(0.0004f);
  sun_light->data.set_enable_shadows(true);
  sun_light->data.set_shadow_map_size(512);
  sun_light->rotate(-65, 1, 0, 0);
  sun_light->rotate(-100, 0, 1, 0);
  
  auto navigation = graph.add_node<gua::node::TransformNode>("/", "navigation");
  auto warp_navigation = graph.add_node<gua::node::TransformNode>("/navigation", "warp");



  // resolution to be used for camera resolution and windows size
  auto resolution = gua::math::vec2ui(1280, 720);

// set up window
#if ENABLE_HMD
  auto window = std::make_shared<gua::ViveWindow>(":0.0");
  gua::WindowDatabase::instance()->add("main_window", window);
  window->config.set_enable_vsync(false);
  window->config.set_fullscreen_mode(false);
#else
  auto window = std::make_shared<gua::GlfwWindow>();
  gua::WindowDatabase::instance()->add("main_window", window);
  window->config.set_title("FAST CLIENT WINDOW");
  window->config.set_enable_vsync(false);
  window->config.set_size(gua::math::vec2ui(resolution.x, resolution.y));
  window->config.set_resolution(resolution);
  window->on_resize.connect([&](gua::math::vec2ui const& new_size) {
    window->config.set_resolution(new_size);
    camera->config.set_resolution(new_size);
    warp_cam->config.set_resolution(new_size);
    screen->data.set_size(gua::math::vec2(0.001 * new_size.x, 0.001 * new_size.y));
    warp_screen->data.set_size(gua::math::vec2(0.001 * new_size.x, 0.001 * new_size.y));
    
  });
#endif


  // set up camera and connect to screen in scenegraph
  auto camera = graph.add_node<gua::node::CameraNode>("/navigation", "cam");
  camera->config.set_scene_graph_name("main_scenegraph");
  camera->config.set_output_window_name("main_window");
  camera->config.set_stereo_type(gua::StereoType::SPATIAL_WARP);
  camera->config.set_enable_stereo(true);
#if ENABLE_HMD
  camera->config.set_resolution(window->get_window_resolution());
  camera->config.set_left_screen_path("/navigation/cam/left_screen");
  camera->config.set_right_screen_path("/navigation/cam/right_screen");
  camera->config.set_eye_dist(window->get_IPD());
#else
  camera->translate(0, 0, 2.0);
  camera->config.set_screen_path("/navigation/screen");
  camera->config.set_resolution(resolution);
  camera->config.set_far_clip(350.f);
  camera->config.set_near_clip(0.1f);
#endif

#if ENABLE_HMD
  auto left_screen = graph.add_node<gua::node::ScreenNode>("/navigation/cam", "left_screen");
  left_screen->data.set_size(window->get_left_screen_size());
  left_screen->translate(window->get_left_screen_translation());

  auto right_screen = graph.add_node<gua::node::ScreenNode>("/navigation/cam", "right_screen");
  right_screen->data.set_size(window->get_right_screen_size());
  right_screen->translate(window->get_right_screen_translation());
#else
  auto screen = graph.add_node<gua::node::ScreenNode>("/navigation", "screen");
  screen->data.set_size(gua::math::vec2(1.28f, 0.72f));  // real world size of screen
  // screen->translate(0, 0, 1.0);
#endif

  // auto pipe_desc = camera->get_pipeline_description();
  auto pipe_desc = std::make_shared<gua::PipelineDescription>();
  pipe_desc->add_pass(std::make_shared<gua::TriMeshPassDescription>());
#if ENABLE_LOD
  auto plod_pass = std::make_shared<gua::PLodPassDescription>();
  pipe_desc->add_pass(std::make_shared<gua::MLodPassDescription>());
  pipe_desc->add_pass(plod_pass);
#endif
  pipe_desc->add_pass(std::make_shared<gua::LineStripPassDescription>());
  pipe_desc->add_pass(std::make_shared<gua::TexturedQuadPassDescription>());
  pipe_desc->add_pass(std::make_shared<gua::LightVisibilityPassDescription>());
  pipe_desc->add_pass(std::make_shared<gua::BBoxPassDescription>());
  pipe_desc->add_pass(std::make_shared<gua::ResolvePassDescription>());
  pipe_desc->add_pass(std::make_shared<gua::TexturedScreenSpaceQuadPassDescription>());

  pipe_desc->get_resolve_pass()->
    background_mode(gua::ResolvePassDescription::BackgroundMode::SKYMAP_TEXTURE).
    background_texture( "../data/textures/sphericalskymap.jpg").
    environment_lighting_texture("../data/textures/sphericalskymap.jpg").
    background_color(gua::utils::Color3f(0,0,0)).
    environment_lighting(gua::utils::Color3f(0.4, 0.4, 0.5)).
    environment_lighting_mode(gua::ResolvePassDescription::EnvironmentLightingMode::AMBIENT_COLOR).
    ssao_enable(true).
    tone_mapping_method(gua::ResolvePassDescription::ToneMappingMethod::HEJL).
    tone_mapping_exposure(1.5f).
    horizon_fade(0.2f).
    ssao_intensity(1.5f).
    ssao_radius(2.f);

  camera->set_pipeline_description(pipe_desc);

  // set up warp cam and warp screen
#if ENABLE_HMD
  auto warp_left_screen = graph.add_node<gua::node::ScreenNode>("/navigation/warp", "warp_left_screen");
  warp_left_screen->data.set_size(window->get_left_screen_size());
  warp_left_screen->translate(window->get_left_screen_translation());

  auto warp_right_screen = graph.add_node<gua::node::ScreenNode>("/navigation/warp", "warp_right_screen");
  warp_right_screen->data.set_size(window->get_right_screen_size());
  warp_right_screen->translate(window->get_right_screen_translation());
#else
  auto warp_screen = graph.add_node<gua::node::ScreenNode>("/navigation/warp", "warp_screen");
  warp_screen->data.set_size(gua::math::vec2(1.28f, 0.72f));  // real world size of screen
#endif

  auto warp_cam = graph.add_node<gua::node::CameraNode>("/navigation", "warp_cam");
  // auto warp_cam = graph.add_node<gua::node::CameraNode>("/navigation/warp", std::make_shared<gua::node::CameraNode>("warp_cam", std::make_shared < gua::PipelineDescription > (), camera->config, camera->get_transform()));
  warp_cam->config.set_scene_graph_name("main_scenegraph");
  warp_cam->config.set_stereo_type(camera->config.get_stereo_type());
  warp_cam->config.set_enable_stereo(true);
#if ENABLE_HMD
  warp_cam->config.set_resolution(window->get_window_resolution());
  warp_cam->config.set_left_screen_path("/navigation/warp/warp_left_screen");
  warp_cam->config.set_right_screen_path("/navigation/warp/warp_right_screen");
  warp_cam->config.set_eye_offset(window->get_IPD());
#else
  warp_cam->translate(0,0,2);
  warp_cam->config.set_resolution(resolution);
  warp_cam->config.set_screen_path("/navigation/warp/warp_screen");
  warp_cam->config.set_far_clip(camera->config.get_far_clip());
  warp_cam->config.set_near_clip(camera->config.get_near_clip());
#endif

  
  auto updat_view_mode([&](){
    if(stereo) {
      // camera->config.set_enable_stereo(true);
      // window->config.set_stereo_mode(gua::StereoMode::SIDE_BY_SIDE);
      // window->config.set_size(gua::math::vec2ui(2*resolution.x, resolution.y));
      // window->config.set_left_resolution(resolution);
      // window->config.set_left_position(gua::math::vec2ui(0, 0));
      // window->config.set_right_resolution(resolution);
      // window->config.set_right_position(gua::math::vec2ui(resolution.x, 0));
    } else {
      camera->config.set_enable_stereo(false);
      window->config.set_stereo_mode(gua::StereoMode::MONO);
      // window->config.set_size(gua::math::vec2ui(resolution.x, resolution.y));
      window->config.set_left_resolution(resolution);
      window->config.set_right_resolution(resolution);
      window->config.set_right_position(gua::math::vec2ui(0,0));
    }

  });

  updat_view_mode();


  window->on_button_press.connect([&](int key, int action, int mods) {
    nav.set_mouse_button(key, action);
    warp_nav.set_mouse_button(key, action);

  });

  window->on_key_press.connect([&](int key, int scancode, int action, int mods) {
    if (manipulation_navigator) {
      nav.set_key_press(key, action);
    } else if (manipulation_camera) {
      warp_nav.set_key_press(key, action);
    }
    if (action == 1) {
      switch (key) {
        // case 'M':
        //   stereo = !stereo;
        //   updat_view_mode();
        //   break;
        case 256:
          window->set_should_close();
          break;
      }
    }
  });

  window->on_move_cursor.connect([&](gua::math::vec2 const& pos) {
    if (manipulation_navigator) {
      nav.set_mouse_position(gua::math::vec2i(pos));
    } else {
      warp_nav.set_mouse_position(gua::math::vec2i(pos));
    }
  });

  window->on_button_press.connect(std::bind(mouse_button, std::ref(object_trackball), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  // window->open();

  gua::Renderer renderer;

  // application loop
  gua::events::MainLoop loop;
  
  // registers the function given in ticker.on_tick to
  // be called by the given loop in the given interval
  gua::events::Ticker ticker(loop, 1.0 / 500.0);

  // log fps and handle close events
  size_t ctr{};
  ticker.on_tick.connect([&]() {
    auto time = gua::Timer::get_now();
    // geometry->rotate(time*0.00000000002, gua::math::vec3(0.0,1.0,0.0));
    // log fps every 150th tick
    if (ctr++ % 150 == 0) {
      gua::Logger::LOG_WARNING
        << "Frame time: " << 1000.f / window->get_rendering_fps()
        << " ms, fps: " << window->get_rendering_fps() << std::endl;
    }

    nav.update();
    warp_nav.update();

    navigation->set_transform(gua::math::mat4(nav.get_transform()));
    warp_navigation->set_transform(gua::math::mat4(warp_nav.get_transform()));

    gua::math::mat4 modelmatrix = scm::math::make_translation(gua::math::float_t(object_trackball.shiftx()),
                                                              gua::math::float_t(object_trackball.shifty()),
                                                              gua::math::float_t(object_trackball.distance())) * gua::math::mat4(object_trackball.rotation());
    transform->set_transform(modelmatrix);
#if ENABLE_HMD
    camera->set_transform(window->get_hmd_sensor_orientation());
	warp_cam->set_transform(window->get_hmd_sensor_orientation());
#endif
    window->process_events();
    if (window->should_close()) {
      // stop rendering and close the window
      renderer.stop();
      window->close();
      loop.stop();
    } else {
      // draw our scenegrapgh
      // std::cout << "MAIN: starting rendering..." << std::endl;
      renderer.queue_draw({&graph}, true);
    }
  });

  loop.start();

  return 0;
}
