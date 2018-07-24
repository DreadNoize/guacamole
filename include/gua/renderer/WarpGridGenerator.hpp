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

#ifndef GUA_WARP_GRID_GENERATOR_HPP
#define GUA_WARP_GRID_GENERATOR_HPP

#include <map>
//#include <unordered map>

#include <gua/platform.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/renderer/Renderer.hpp>

#include <scm/gl_core/shader_objects.h>

namespace gua {

class Pipeline;
class PipelinePassDescription;

class WarpGridGenerator {
 public:
  WarpGridGenerator();
  virtual ~WarpGridGenerator();

  void render(Pipeline& pipe, PipelinePassDescription const& desc);
  void set_global_substitution_map(SubstitutionMap const& smap) {
    global_substitution_map_ = smap;
  }

 protected:
  std::shared_ptr<Renderer::WarpingResources> res_;
  SubstitutionMap global_substitution_map_;

  scm::gl::vertex_array_ptr grid_vao_[2];
  std::vector<scm::gl::frame_buffer_ptr> surface_detection_buffer_fbos_;

  std::vector<ShaderProgramStage> grid_generation_program_stages_;
  std::shared_ptr<ShaderProgram> grid_generation_program_;

  std::vector<ShaderProgramStage> surface_detection_program_stages_;
  std::shared_ptr<ShaderProgram> surface_detection_program_;

  Pipeline* pipe_;

};

}

#endif // GUA_WARP_GRID_GENERATOR_HPP