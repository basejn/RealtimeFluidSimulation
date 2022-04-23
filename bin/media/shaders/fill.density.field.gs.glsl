#version 430 core
#define texture_Depth 64
layout (invocations = 1 , triangles) in;
layout (triangle_strip, max_vertices = 3) out;

const float GRID_VOLUME_SIDE=10;
 
in VS_OUT
{
    vec2 coord ;
    int instanceID;
} gs_in[];


out GS_OUT
{
    vec3 coord ;
} gs_out;

void main(void) {
  for (int i = 0; i < gl_in.length(); i++) {
    gs_out.coord =
        vec3(gs_in[i].coord,
             (float(gs_in[i].instanceID) / float(texture_Depth / 2.0)) - 1.0) *
        GRID_VOLUME_SIDE;
    gl_Position = vec4(gs_in[i].coord, 0.0, 1.0);
    gl_Layer = gs_in[i].instanceID;
    EmitVertex();
  }
  EndPrimitive();
}
