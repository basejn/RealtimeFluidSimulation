#version 410 core

layout (location = 0) out vec4 color;
//layout (binding = 1) uniform sampler2D main_texture;
uniform vec3 light_pos;


in VS_OUT
{
 vec3 N;
 vec3 V;
 vec3 L;
 float gridInd;
 vec4 color;
} vs_in;

uniform vec3 diffuse_albedo = vec3(0.6, 0.2, 0.2);
uniform vec3 specular_albedo = vec3(0.7);
uniform float specular_power = 64.0;
uniform vec3 rim_color = vec3(0);//vec3(0.5, 0.5, 0.5);
uniform float rim_power = 3.0;

vec3 calculate_rim(vec3 N, vec3 V)
{
    float f = 1.0 - dot(N, V);

    f = smoothstep(0.0, 1.0, f);
    f = pow(f, rim_power);

    return f * rim_color;
}
void main(void)
{
    vec3 N = normalize(vs_in.N); 
    vec3 L = normalize(vs_in.L);
    vec3 V = normalize(vs_in.V);

    // Calculate R locally
    vec3 R = reflect(-L, N);

    // Compute the diffuse and specular components for each fragment
    vec3 diffuse = max(dot(N, L), 0.0) * diffuse_albedo;
    vec3 specular = pow(max(dot(R, V), 0.0), specular_power) * specular_albedo;
    vec3 rim = calculate_rim(N, V);

    // Write final color to the framebuffer
    color = vec4(diffuse + specular + rim, 1.0)*vs_in.color;	
   //color = vec4(1)*N.z;
   
   vec4 colors[27]=vec4[](
     vec4(0.1,0.1,0.1,1),
     vec4(0.1,0.1,0.4,1),
     vec4(0.1,0.1,0.8,1),
	 
	 vec4(0.4,0.1,0.1,1),
	 vec4(0.4,0.1,0.4,1),
	 vec4(0.4,0.1,0.8,1),
	            
	 vec4(0.8,0.1,0.1,1),
	 vec4(0.8,0.1,0.4,1),
	 vec4(0.8,0.1,0.8,1),
	 
	 vec4(0.1,0.4,0.1,1),
     vec4(0.1,0.4,0.4,1),
     vec4(0.1,0.4,0.8,1),
	            
	 vec4(0.4,0.4,0.1,1),
	 vec4(0.4,0.4,0.4,1),
	 vec4(0.4,0.4,0.8,1),
	            
	 vec4(0.8,0.4,0.1,1),
	 vec4(0.8,0.4,0.4,1),
	 vec4(0.8,0.4,0.8,1),
	 
	 vec4(0.1,0.8,0.1,1),
     vec4(0.1,0.8,0.4,1),
     vec4(0.1,0.8,0.8,1),
	            
	 vec4(0.4,0.8,0.1,1),
	 vec4(0.4,0.8,0.4,1),
	 vec4(0.4,0.8,0.8,1),
	            
	 vec4(0.8,0.8,0.1,1),
	 vec4(0.8,0.8,0.4,1),
	 vec4(0.8,0.8,0.8,1) 
    
   );
   
   //color=colors[int(vs_in.gridInd)&27];
   
}
