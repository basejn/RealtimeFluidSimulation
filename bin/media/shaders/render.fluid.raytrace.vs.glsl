#version 420 core

layout (location = 0) in vec3 position;

layout (binding = 0) uniform samplerBuffer tex_position;
layout (binding = 1) uniform samplerBuffer tex_velocity;
layout (binding = 2) uniform isamplerBuffer tex_gridlist;
layout (binding = 3) uniform samplerBuffer tex_density;

 uniform mat4 proj_mat;
 uniform mat4 mv_mat;
 uniform vec3 light_pos;
 
 out VS_OUT
{
	vec3     ray_origin ;
    vec3    ray_direction;
} vs_out;
 
void main(void)
{
	mat4 invMvMatrix = inverse(mv_mat);
	//vec4 origin = invMvMatrix*vec4(0,0,2.14450692050,1);
	vs_out.ray_origin=(invMvMatrix*vec4(0,0,0,1)).xyz;//origin.xyz;
	vec4 mv_pos = invMvMatrix*vec4(position, 1.0);
	vs_out.ray_direction = normalize(mv_pos.xyz-vs_out.ray_origin);	
	//vs_out.ray_origin.z+=0*2.14450692050;
    gl_Position =vec4(position.xy,0,1);	
}
