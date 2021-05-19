#version 420 core

layout (binding = 0) uniform samplerBuffer tex_position;
layout (binding = 2) uniform isamplerBuffer tex_gridlist;


 out VS_OUT
{
vec2 coord ;
} vs_out;
 
void main(void)
{	
    vec2[4] vertices = vec2[4](vec2(-1.0, -1.0),
                               vec2( 1.0, -1.0),
                               vec2(-1.0,  1.0),
                               vec2( 1.0,  1.0));
	 vs_out.coord = vertices[gl_VertexID];
    gl_Position  = vec4(vertices[gl_VertexID],1.0, 1.0);
}
