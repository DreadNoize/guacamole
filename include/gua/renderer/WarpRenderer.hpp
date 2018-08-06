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
/* This is a simplified and adjusted version of the WarpRenderer              *
 * which was written by Simon Schneegans                                      */

#ifndef GUA_WARP_RENDERER
#define GUA_WARP_RENDERER

#include <map>
#include <unordered_map>

#include <gua/renderer/WarpPass.hpp>
#include <gua/renderer/Frustum.hpp>
#include <gua/platform.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/renderer/Renderer.hpp>

#include <scm/gl_core/shader_objects.h>


namespace gua {

class Pipeline;
class PipelinePassDescription;

class WarpRenderer {
 public:
  WarpRenderer();
  virtual  ~WarpRenderer();

  void render(Pipeline& pipe, PipelinePassDescription const& desc);
  void render_grid(Pipeline& pipe, PipelinePassDescription const& desc, math::mat4f warp_matrix);

  void set_global_substitution_map(SubstitutionMap const& smap) {
    global_substitution_map_ = smap;
  };

  void print_matrix(gua::math::mat4f const& matrix, std::string const& name) {
    std::cout << "Matrix: " << name << std::endl;
    std::cout << matrix[0] << "," << matrix[1] << "," << matrix[2] << "," << matrix[3] << std::endl;
    std::cout << matrix[4] << "," << matrix[5] << "," << matrix[6] << "," << matrix[7] << std::endl;
    std::cout << matrix[8] << "," << matrix[9] << "," << matrix[10] << "," << matrix[11] << std::endl;
    std::cout << matrix[12] << "," << matrix[13] << "," << matrix[14] << "," << matrix[15] << std::endl;
  };
 protected:
  std::shared_ptr<Renderer::WarpingResources> res_;

  WarpPassDescription::WarpState cached_warp_state_;
  Frustum last_frustum_;

  scm::gl::vertex_array_ptr empty_vao_;
  scm::gl::rasterizer_state_ptr points_;
  scm::gl::depth_stencil_state_ptr depth_stencil_state_yes_;
  scm::gl::depth_stencil_state_ptr depth_stencil_state_no_;
  scm::gl::depth_stencil_state_ptr depth_stencil_state_grid_;

  SubstitutionMap global_substitution_map_;

  std::vector<ShaderProgramStage> warp_gbuffer_program_stages_;
  std::shared_ptr<ShaderProgram> warp_gbuffer_program_;

  std::vector<ShaderProgramStage> render_grid_program_stages_;
  std::shared_ptr<ShaderProgram> render_grid_program_;
  scm::gl::blend_state_ptr blend_state_;
  scm::gl::rasterizer_state_ptr rasterizer_state_;

  std::vector<ShaderProgramStage> warp_abuffer_program_stages_;
  std::shared_ptr<ShaderProgram> warp_abuffer_program_;

  std::shared_ptr<ShaderProgram> hole_filling_texture_program_;

  scm::gl::texture_2d_ptr color_buffer_;
  scm::gl::texture_2d_ptr depth_buffer_;

  math::vec2ui sdb_handle;
  std::shared_ptr<Texture2D> hole_filling_texture_;

  scm::gl::frame_buffer_ptr fbo_;
  std::vector<scm::gl::frame_buffer_ptr> hole_filling_fbos_;

  Pipeline* pipe_;
};

}

#endif  // GUA_WARP_RENDERER