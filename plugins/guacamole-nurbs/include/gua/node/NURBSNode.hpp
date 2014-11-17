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
#if 0
#ifndef GUA_NURBS_NODE_HPP
#define GUA_NURBS_NODE_HPP

// guacamole headers
#include <gua/renderer/NURBS.hpp>

#include <gua/node/GeometryNode.hpp>

namespace gua {
namespace node {

/**
 * This class is used to represent polygonal geometry in the SceneGraph.
 *
 * \ingroup gua_scenegraph
 */
class GUA_NURBS_DLL NURBSNode : public GeometryNode
{
public : // member

  NURBSNode(std::string const& name,
            std::string const& geometry = "gua_default_geometry",
            std::string const& material = "gua_default_material",
            math::mat4  const& transform = math::mat4::identity());

  /**
  * Implements ray picking for a NURBS object
  */
  void ray_test_impl(Ray const& ray,
                     int options,
                     Mask const& mask,
                     std::set<PickResult>& hits) override;

  void update_cache() override;

  float max_pre_tesselation() const;
  void  max_pre_tesselation(float t);

  float max_final_tesselation() const;
  void  max_final_tesselation(float t);

protected:

  std::shared_ptr<Node> copy() const override;

private : // attributes e.g. special attributes for drawing

  float max_tess_level_pre_pass; 
  float max_tess_level_final_pass;

};

} // namespace node {
} // namespace gua {

#endif  // GUA_NURBS_NODE_HPP
#endif
