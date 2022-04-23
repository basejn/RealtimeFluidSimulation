#version 430 core

#define texture_Depth 64
#define GRID_VOLUME_SIDE 10
layout(binding = 0) uniform samplerBuffer tex_position;
layout(binding = 2) uniform isamplerBuffer tex_gridlist;

out VS_OUT {
  vec2 coord;
  int instanceID;
}
vs_out;

void main(void) {
  vec2[4] vertices = vec2[4](vec2(-1.0, -1.0), 
                             vec2( 1.0, -1.0), 
                             vec2(-1.0,  1.0),
                             vec2( 1.0,  1.0));
  vs_out.coord = vertices[gl_VertexID];
  vs_out.instanceID = gl_InstanceID;
  gl_Position = vec4(vertices[gl_VertexID], 0.0, 1.0);
}
