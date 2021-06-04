#version 430 core

layout (location = 0) in vec3 position;

 uniform mat4 mvp_mat;
 //uniform vec3 light_pos;

void main(void)
{
	gl_Position= mvp_mat*vec4(position, 1.0);		
}
