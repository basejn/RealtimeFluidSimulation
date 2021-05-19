#version 420 core

layout (location = 0) in vec3 vert;
layout (binding = 0) uniform samplerBuffer tex_position;
//layout (binding = 1) uniform isamplerBuffer tex_gridlist;
layout (binding = 2) uniform samplerBuffer tex_colors;
layout (binding = 3) uniform samplerBuffer tex_density;
 uniform mat4 proj_mat;
 uniform mat4 mv_mat;
 uniform vec3 light_pos;
out VS_OUT
{
 vec3 N;
 vec3 V;
 vec3 L;
 float gridInd;
 vec4 color;
 } vs_out;
   
void main(void)
{
	vs_out.N=mat3(mv_mat)*(vert);
	vec3 pos = vert+texelFetch(tex_position,gl_InstanceID).xyz;
	vec4 mv_pos = mv_mat*vec4(pos, 1.0);
	vs_out.V=-mv_pos.xyz;
	vs_out.L = mat3(mv_mat)*(light_pos)-mv_pos.xyz;
	//vs_out.gridInd =float(CellIndex(texelFetch(tex_position,gl_InstanceID).xyz));
	vs_out.color = texelFetch(tex_colors,gl_InstanceID);
	//vs_out.color = vec4(texelFetch(tex_density,gl_InstanceID).x,texelFetch(tex_density,gl_InstanceID).y,0,1);
    gl_Position = proj_mat*mv_pos;	
}
