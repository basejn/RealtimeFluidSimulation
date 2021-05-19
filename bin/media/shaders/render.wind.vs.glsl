#version 420 core

layout (location = 0) in vec4 position;
 uniform mat4 proj_mat;
 uniform mat4 mv_mat;

void main(void)
{
    gl_Position =vec4(mat3(mv_mat)*position.xyz,3.0)+vec4(-2.0,1.5,-5.0,0.0);
}
