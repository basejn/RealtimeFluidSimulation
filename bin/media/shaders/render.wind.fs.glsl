#version 430 core

layout (location = 0) out vec4 color;
in GS_OUT
{
    vec3 N;
    vec3 L;
    vec3 V;
} gs_in;

uniform vec3 diffuse_albedo = vec3(0.3, 0.5, 0.2);
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
    vec3 N = normalize(gs_in.N); 
    vec3 L = normalize(gs_in.L);
    vec3 V = normalize(gs_in.V);

    // Calculate R locally
    vec3 R = reflect(-L, N);

    // Compute the diffuse and specular components for each fragment
    vec3 diffuse = max(dot(N, L), 0.0) * diffuse_albedo;
    vec3 specular = pow(max(dot(R, V), 0.0), specular_power) * specular_albedo;
    vec3 rim = calculate_rim(N, V);

    // Write final color to the framebuffer
    color = vec4(diffuse + specular + rim, 1.0);

	
}
