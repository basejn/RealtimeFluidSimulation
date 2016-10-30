#version 430 core

layout (points) in;
layout (points, max_vertices = 1) out;
 uniform mat4 proj_mat;
 uniform mat4 mv_mat;


out GS_OUT
{
    float pointSize;
	vec3 N;
} gs_out;

mat3 rotationMatrix(vec3 axis, float angle)
{
    axis = normalize(axis);
    float s = sin(angle);
    float c = cos(angle);
    float oc = 1.0 - c;
    
    return mat3(oc * axis.x * axis.x + c,           oc * axis.x * axis.y - axis.z * s,  oc * axis.z * axis.x + axis.y * s,  
                oc * axis.x * axis.y + axis.z * s,  oc * axis.y * axis.y + c,           oc * axis.y * axis.z - axis.x * s,  
                oc * axis.z * axis.x - axis.y * s,  oc * axis.y * axis.z + axis.x * s,  oc * axis.z * axis.z + c   );       
                
}
void main(void)
{
 	gl_Position=proj_mat*gl_in[0].gl_Position;
	float d = gl_Position.z;	
	//gl_PointSize =gs_out.pointSize= 40 * sqrt( 1 / (0.5 + 0.02 * d + 0.09 * d * d));

	
	EmitVertex();
	
    
}
