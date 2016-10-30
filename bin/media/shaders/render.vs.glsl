#version 410 core

layout (location = 0) in vec3 position;

 uniform mat4 proj_mat;
 uniform mat4 mv_mat;
 //uniform vec3 light_pos;

void main(void)
{
	vec4 mv_pos = mv_mat*vec4(position, 1.0);
	
    gl_Position = mv_pos;
	
}
