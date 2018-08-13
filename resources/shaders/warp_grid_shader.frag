@include "common/header.glsl"

@include "shaders/common/gua_camera_uniforms.glsl"
@include "common/gua_gbuffer_input.glsl"
@include "warp_grid_bits.glsl"

flat in uint cellsize;
in vec2 cellcoords;
in vec2 texcoords;
in float pass_depth;
flat in vec2 pos;
// output
// layout(location=0) out vec4 gua_out_color_emit;
layout(location=0) out vec4 gua_out_color_emit;

uniform uvec2 gua_warp_grid_tex;
uniform uvec2 gbuffer_color_tex;

vec3 heat(float v) {
  float value = 1.0-v;
  return (0.5+0.5*smoothstep(0.0, 0.1, value))*vec3(
    smoothstep(0.5, 0.3, value),
    value < 0.3 ? smoothstep(0.0, 0.3, value) : smoothstep(1.0, 0.6, value),
    smoothstep(0.4, 0.6, value)
  );
}

void main() {
  uint info = texelFetch(usampler2D(gua_warp_grid_tex), ivec2(ivec2(texcoords * gua_resolution)/2), 0).x;
  const bool is_surface = (info & ALL_CONTINUITY_BITS) == ALL_CONTINUITY_BITS;
  
  vec4 color;
  if (is_surface) {
      // gua_out_color_emit.rgb = gua_get_color(texcoords);
      // gua_out_color_emit = vec4(pass_depth, pass_depth, pass_depth,1);
      // gua_out_color_emit = 0.4* vec4(texcoords.x,0.0,0.0, 1.0);
      gua_out_color_emit = texture2D(sampler2D(gbuffer_color_tex),texcoords);
      // gua_out_color_emit.a = 1.0;
      gua_out_color_emit.a = gua_get_pbr(texcoords).r;
    } else {
      gua_out_color_emit = texelFetch(sampler2D(gbuffer_color_tex), ivec2(texcoords*gua_resolution), 0);
      // gua_out_color_emit = vec4(0.0, texcoords.x, 0.0,1.0);
    	// gua_out_color_emit = vec4(pass_depth, pass_depth, pass_depth,1);
      // gua_out_color_emit = vec4(1.0,0.0,0.0, 1.0);
      gua_out_color_emit.a = texelFetch(sampler2D(gua_gbuffer_pbr), ivec2(texcoords*gua_resolution), 0).r;
      // gua_out_color_emit.a = 1.0;
  }

  // if(pass_depth >= 0.99) {
  //   gua_out_color_emit = vec4(0.0,0.0,0.0, 1.0);
  //   // gua_out_color_emit = color;
  //   // gua_out_color_emit = vec4(0.05,0.05,0.1,1.0);
  // } else {
  //   // gua_out_color_emit = vec4(0.0,0.0,0.0, 1.0);
  //   gua_out_color_emit = color;
  // }
    

  // gua_out_color_emit = vec4((texcoords.x),0.0, 0.0,1.0);
  // gua_out_color_emit = vec4((pos/(5000,2500)), 0.0,1.0);

  #if @debug_cell_colors@ == 1
  float intensity = log2(cellsize) / 7.0;
  if (pass_depth == 1) {
      gua_out_color_emit = vec4(0.0,0.0,0.0,1.0);   
  } else {
    gua_out_color_emit.rgb = heat(1-intensity);

    if (any(lessThan(cellcoords, vec2(0.6/float(cellsize)))) || any(greaterThan(cellcoords, vec2(1.0-0.6/float(cellsize))))) {
      gua_out_color_emit.rgb = mix(gua_out_color_emit.rgb, vec3(0), 0.7);
    }
    gua_out_color_emit.a = 1.0;
  }
  #endif
}
