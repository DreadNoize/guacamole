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
/* This is a simplified and adjusted version of the WarpGridGenerator         *
 * which was written by Simon Schneegans                                      */

#ifndef GUA_SURFACE_DETECTION_PASS_HPP
#define GUA_SURFACE_DETECTION_PASS_HPP

#include <gua/renderer/Renderer.hpp>
#include <gua/renderer/WarpPass.hpp>

#include <memory>

namespace gua {

class Pipeline;

class GUA_DLL SurfaceDetectionPassDescription : public PipelinePassDescription {
 public:
  SurfaceDetectionPassDescription();

  SurfaceDetectionPassDescription(std::shared_ptr<Renderer::WarpingResources>& resources);

  SurfaceDetectionPassDescription& cell_size(int size);
  int cell_size() const;

  SurfaceDetectionPassDescription& split_threshold(float threshold);
  float split_threshold() const;

  SurfaceDetectionPassDescription& mode(WarpPassDescription::GBufferWarpMode mode);
  WarpPassDescription::GBufferWarpMode mode()const;

  std::shared_ptr<Renderer::WarpingResources> warp_resources() const;

  std::shared_ptr<PipelinePassDescription> make_copy() const override;

  friend class Pipeline;
 protected:
  PipelinePass make_pass(RenderContext const&, SubstitutionMap&) override;

  int cell_size_;
  float split_threshold_;
  WarpPassDescription::GBufferWarpMode mode_;

  std::shared_ptr<Renderer::WarpingResources> pass_res_;

};

}

#endif // GUA_SURFACE_DETECTION_PASS_HPP