#version 430 core
const float rzl = 6.0;
layout (lines) in;
layout (triangle_strip, max_vertices = 28) out;
 uniform mat4 proj_mat;
  uniform vec3 light_pos=vec3(10.0,10.0,40.0);
out GS_OUT
{
    vec3 N;
    vec3 L;
    vec3 V;
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
    int i;
	vec3 arowv = gl_in[0].gl_Position.xyz-gl_in[1].gl_Position.xyz;
	vec3 arownv=normalize(arowv);
    vec3 Rv = 0.2*normalize(cross(arowv,vec3(0.0,1.0,0.0)));
    
	vec4 top =gl_in[1].gl_Position;
	vec4 bottom =gl_in[0].gl_Position;
    for (i = 0; i <= int(rzl); i++)
    {		
	 mat3 radmat = rotationMatrix(arownv,(i)*(2.0*3.145/rzl)); 
	 vec3 Rv_t = radmat*Rv;
	 vec3 cr1 = normalize(cross(Rv_t,arownv));
	 vec3 nv =normalize(cross(cr1,-Rv_t-arowv));
     gl_Position=proj_mat*top;
	 gs_out.N = normalize(nv);
     gs_out.L=light_pos-top.xyz;
	 gs_out.V=-top.xyz;	 
	 EmitVertex();
	
	vec4 r_pos = vec4(gl_in[0].gl_Position.xyz+(Rv_t),1.0);
	 gl_Position=proj_mat*r_pos;
	 gs_out.N = normalize(nv);
	 gs_out.L=light_pos-r_pos.xyz;
	 gs_out.V=-r_pos.xyz;	
	 EmitVertex();	
    }
	 EndPrimitive();
	 
	 for (i = 0; i <= int(rzl); i++)
    {	     
	 mat3 radmat = rotationMatrix(arownv,(i)*(2.0*3.145/rzl)); 
	 vec3 Rv_t = radmat*Rv;
	 vec4 r_pos = vec4(gl_in[0].gl_Position.xyz+(Rv_t),1.0);
	 gl_Position=proj_mat*r_pos;
	 gs_out.N = normalize(arownv);
	 gs_out.L=light_pos-r_pos.xyz;
	 gs_out.V=-r_pos.xyz;
	 EmitVertex();
	 gl_Position=proj_mat*bottom;
	 gs_out.N = normalize(arownv);
	  gs_out.L=light_pos-bottom.xyz;
	 gs_out.V=-bottom.xyz;	
	 EmitVertex();	 
    }
	 EndPrimitive();
	
    
}
