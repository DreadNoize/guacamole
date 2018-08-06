/******************************************************************************
 * guacamole - delicious VR                                                   *
 *                                                                            *
 * Copyright: (c) 2011-2018 Bauhaus-Universität Weimar                        *
 * Contact:   felix.lauer@uni-weimar.de / simon.schneegans@uni-weimar.de      *
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
/* This is a simplified and adjusted version of the WarpPassDescription       *
 * which was written by Simon Schneegans                                      */

#include <gua/renderer/WarpPass.hpp>

#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/WarpRenderer.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/Resources.hpp>
#include <gua/utils/Logger.hpp>

#include <boost/variant.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////
WarpPassDescription::WarpPassDescription() 
 : PipelinePassDescription()
 , depth_test_(true)
 , debug_cell_colors_(false)
 , debug_sample_count_(false)
 , debug_bounding_volumes_(false)
 , pixel_size_(0.2f)
 , hole_filling_color_(0,0,0)
 , gbuffer_warp_mode_(GBUFFER_GRID_ADVANCED_SURFACE_ESTIMATION)
 , hole_filling_mode_(HOLE_FILLING_NONE)
 , interpolation_mode_(INTERPOLATION_MODE_NEAREST)
 , pass_res_()
{
  vertex_shader_ = "";
  fragment_shader_ = "";
  name_ = "WarpPass";
  needs_color_buffer_as_input_ = true;
  writes_only_color_buffer_ = false;
  rendermode_ = RenderMode::Custom;
}

////////////////////////////////////////////////////////////////////////////////
WarpPassDescription::WarpPassDescription(std::shared_ptr<Renderer::WarpingResources> resources) 
 : PipelinePassDescription()
 , depth_test_(true)
 , debug_cell_colors_(false)
 , debug_sample_count_(false)
 , debug_bounding_volumes_(false)
 , pixel_size_(0.2f)
 , hole_filling_color_(0.0,0.1,0.3)
 , gbuffer_warp_mode_(GBUFFER_GRID_ADVANCED_SURFACE_ESTIMATION)
 , hole_filling_mode_(HOLE_FILLING_NONE)
 , interpolation_mode_(INTERPOLATION_MODE_ADAPTIVE)
 , pass_res_(resources)
{
  vertex_shader_ = "";
  fragment_shader_ = "";
  name_ = "WarpPass";
  needs_color_buffer_as_input_ = true;
  writes_only_color_buffer_ = false;
  rendermode_ = RenderMode::Custom;
}

////////////////////////////////////////////////////////////////////////////////
WarpPassDescription& WarpPassDescription::get_warp_state(std::function<WarpState()> const& f) {
  get_warp_state_ = f;
  touch();
  return *this;
}

////////////////////////////////////////////////////////////////////////////////
std::function<WarpPassDescription::WarpState()> const& WarpPassDescription::get_warp_state() const {
  return get_warp_state_;
}

////////////////////////////////////////////////////////////////////////////////
WarpPassDescription& WarpPassDescription::depth_test(bool depth_test) {
  depth_test_ = depth_test;
  touch();
  return *this;
}

////////////////////////////////////////////////////////////////////////////////
bool WarpPassDescription::depth_test() const {
  return depth_test_;
}

////////////////////////////////////////////////////////////////////////////////
WarpPassDescription& WarpPassDescription::debug_cell_colors(bool debug_cell_colors) {
  debug_cell_colors_ = debug_cell_colors;
  touch();
  return *this;
}

////////////////////////////////////////////////////////////////////////////////
bool WarpPassDescription::debug_cell_colors() const {
  return debug_cell_colors_;
}


////////////////////////////////////////////////////////////////////////////////
WarpPassDescription& WarpPassDescription::debug_sample_count(bool debug_sample_count) {
  debug_sample_count_ = debug_sample_count;
  touch();
  return *this;
}

////////////////////////////////////////////////////////////////////////////////
bool WarpPassDescription::debug_sample_count() const {
  return debug_sample_count_;
}

////////////////////////////////////////////////////////////////////////////////
WarpPassDescription& WarpPassDescription::debug_bounding_volumes(bool debug_bounding_volumes) {
  debug_bounding_volumes_ = debug_bounding_volumes;
  touch();
  return *this;
}

////////////////////////////////////////////////////////////////////////////////
bool WarpPassDescription::debug_bounding_volumes() const {
  return debug_bounding_volumes_;
}

////////////////////////////////////////////////////////////////////////////////
WarpPassDescription& WarpPassDescription::pixel_size(float pixel_size) {
  pixel_size_ = pixel_size;
  touch();
  return *this;
}

////////////////////////////////////////////////////////////////////////////////
float WarpPassDescription::pixel_size() const {
  return pixel_size_;
}

////////////////////////////////////////////////////////////////////////////////
WarpPassDescription& WarpPassDescription::hole_filling_color(math::vec3f const& hole_filling_color) {
  hole_filling_color_ = hole_filling_color;
  touch();
  return *this;
}

////////////////////////////////////////////////////////////////////////////////
math::vec3f const& WarpPassDescription::hole_filling_color() const {
  return hole_filling_color_;
}

////////////////////////////////////////////////////////////////////////////////
WarpPassDescription& WarpPassDescription::gbuffer_warp_mode(GBufferWarpMode gbuffer_warp_mode) {
  gbuffer_warp_mode_ = gbuffer_warp_mode;
  touch();
  return *this;
}

////////////////////////////////////////////////////////////////////////////////
WarpPassDescription::GBufferWarpMode WarpPassDescription::gbuffer_warp_mode() const {
  return gbuffer_warp_mode_;
}

////////////////////////////////////////////////////////////////////////////////
WarpPassDescription& WarpPassDescription::hole_filling_mode(HoleFillingMode hole_filling_mode) {
  hole_filling_mode_ = hole_filling_mode;
  touch();
  return *this;
}

////////////////////////////////////////////////////////////////////////////////
WarpPassDescription::HoleFillingMode WarpPassDescription::hole_filling_mode() const {
  return hole_filling_mode_;
}

////////////////////////////////////////////////////////////////////////////////
WarpPassDescription& WarpPassDescription::interpolation_mode(InterpolationMode interpolation_mode) {
  interpolation_mode_ = interpolation_mode;
  touch();
  return *this;
}

////////////////////////////////////////////////////////////////////////////////
WarpPassDescription::InterpolationMode WarpPassDescription::interpolation_mode() const {
  return interpolation_mode_;
}

////////////////////////////////////////////////////////////////////////////////
std::shared_ptr<Renderer::WarpingResources> WarpPassDescription::warp_resources() const {
  return pass_res_;
}

////////////////////////////////////////////////////////////////////////////////
std::shared_ptr<PipelinePassDescription> WarpPassDescription::make_copy() const {
  return std::make_shared<WarpPassDescription>(*this);
}

////////////////////////////////////////////////////////////////////////////////
PipelinePass WarpPassDescription::make_pass(RenderContext const& ctx, SubstitutionMap& substitution_map) {
  substitution_map["gua_debug_tiles"] = "0";
  substitution_map["adaptive_entry_level"] = adaptive_entry_level_ ? "1" : "0";
  substitution_map["debug_cell_colors"] = debug_cell_colors_ ? "1" : "0";
  substitution_map["debug_sample_count"] = debug_sample_count_ ? "1" : "0";
  substitution_map["debug_bounding_volumes"] = debug_bounding_volumes_ ? "1" : "0";
  substitution_map["pixel_size"] = gua::string_utils::to_string(pixel_size_);
  substitution_map["gbuffer_warp_mode"] = std::to_string(gbuffer_warp_mode_);
  substitution_map["abuffer_warp_mode"] = std::to_string(WarpPassDescription::ABufferWarpMode::ABUFFER_NONE);
  substitution_map["hole_filling_mode"] = std::to_string(hole_filling_mode_);
  substitution_map["hole_filling_color"] = "vec3(" + gua::string_utils::to_string(hole_filling_color_.x) + ", " + gua::string_utils::to_string(hole_filling_color_.y) + ", " + gua::string_utils::to_string(hole_filling_color_.z) + ")";
  substitution_map["interpolation_mode"] = std::to_string(interpolation_mode_);
  substitution_map["max_raysteps"] = std::to_string(0);
  PipelinePass pass{*this, ctx, substitution_map};

  auto renderer = std::make_shared<WarpRenderer>();
  renderer->set_global_substitution_map(substitution_map);

  pass.process_ = [renderer](
    PipelinePass& pass, PipelinePassDescription const& desc, Pipeline & pipe) {

    renderer->render(pipe, desc);
  };

  return pass;
}

} //namespace gua