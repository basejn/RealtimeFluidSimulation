#version 410 core

layout (location = 0) out vec4 color;
layout (binding = 0) uniform sampler2D main_texture;
uniform vec3 light_pos;


in GS_OUT
{
 float pointSize;
 vec3 N;
} fs_in;


void main(void)
{

    float exc = length(gl_PointCoord-vec2(0.5))*2;
	if(exc>1)discard;
	//exc = clamp(exc,0,1);
	
    color = mix(vec4(1.0,0.0,0.0,1.0),vec4(0.1,0.3,0.4,1.0),exc);
//color=vec4(1.0,1.0,1.0,1.0);
}
