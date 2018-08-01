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
/* This is a simplified and adjusted version of the GenerateWarpGridPass      *
 * which was written by Simon Schneegans                                      */

#include <gua/renderer/SurfaceDetectionPass.hpp>

#include <gua/renderer/ABuffer.hpp>
#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/SurfaceDetector.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/Resources.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/utils/Logger.hpp>

#include <boost/variant.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////
SurfaceDetectionPassDescription::SurfaceDetectionPassDescription() 
  : PipelinePassDescription()
  , cell_size_(32)
  , split_threshold_()
  , mode_(WarpPassDescription::GBUFFER_GRID_ADVANCED_SURFACE_ESTIMATION)
  , pass_res_()
  {
    vertex_shader_ = "";
    fragment_shader_ = "";
    name_ = "SurfaceDetectionPass";
    needs_color_buffer_as_input_ = false;
    writes_only_color_buffer_ = true;
    rendermode_ = RenderMode::Custom;
  }

////////////////////////////////////////////////////////////////////////////////
SurfaceDetectionPassDescription::SurfaceDetectionPassDescription(std::shared_ptr<Renderer::WarpingResources>& resources) 
  : PipelinePassDescription()
  , cell_size_(32)
  , split_threshold_(0.0001f)
  , mode_(WarpPassDescription::GBUFFER_GRID_ADVANCED_SURFACE_ESTIMATION)
  , pass_res_(resources)
  {
    vertex_shader_ = "";
    fragment_shader_ = "";
    name_ = "SurfaceDetectionPass";
    needs_color_buffer_as_input_ = false;
    writes_only_color_buffer_ = true;
    rendermode_ = RenderMode::Custom;
  }


////////////////////////////////////////////////////////////////////////////////
SurfaceDetectionPassDescription& SurfaceDetectionPassDescription::cell_size(int size) {
  cell_size_ = size;
  touch();
  return *this;
}

////////////////////////////////////////////////////////////////////////////////
int SurfaceDetectionPassDescription::cell_size() const {
  return cell_size_;
}

////////////////////////////////////////////////////////////////////////////////
SurfaceDetectionPassDescription& SurfaceDetectionPassDescription::split_threshold(float threshold) {
  split_threshold_ = threshold;
  touch();
  return *this;
}

////////////////////////////////////////////////////////////////////////////////
float SurfaceDetectionPassDescription::split_threshold() const {
  return split_threshold_;
}


////////////////////////////////////////////////////////////////////////////////
SurfaceDetectionPassDescription& SurfaceDetectionPassDescription::mode(WarpPassDescription::GBufferWarpMode mode) {
  mode_ = mode;
  touch();
  return *this;
}

////////////////////////////////////////////////////////////////////////////////
WarpPassDescription::GBufferWarpMode SurfaceDetectionPassDescription::mode() const {
  return mode_;
}

////////////////////////////////////////////////////////////////////////////////
std::shared_ptr<Renderer::WarpingResources> SurfaceDetectionPassDescription::warp_resources() const {
  return pass_res_;
}

////////////////////////////////////////////////////////////////////////////////
std::shared_ptr<PipelinePassDescription> SurfaceDetectionPassDescription::make_copy() const {
  return std::make_shared<SurfaceDetectionPassDescription>(*this);
}

////////////////////////////////////////////////////////////////////////////////
PipelinePass SurfaceDetectionPassDescription::make_pass(RenderContext const& ctx, SubstitutionMap& substitution_map) {
  PipelinePass pass{*this, ctx, substitution_map};

  substitution_map["gbuffer_warp_mode"] = std::to_string(mode_);
  substitution_map["split_threshold"] = gua::string_utils::to_string(split_threshold_);

  auto renderer = std::make_shared<SurfaceDetector>();
  renderer->set_global_substitution_map(substitution_map);

  WarpPassDescription::GBufferWarpMode mode(mode_);

  pass.process_ = [renderer, mode](
    PipelinePass& pass, PipelinePassDescription const& desc, Pipeline & pipe) {

    if (mode == WarpPassDescription::GBUFFER_GRID_ADVANCED_SURFACE_ESTIMATION) {
      renderer->render(pipe, desc);
    }
  };

  return pass;
}

////////////////////////////////////////////////////////////////////////////////
} // namespace gua