#version 410 core
#define PI 3.1415926535897932384626433832795
#define OPTIM_STRUCT  0 //0=no 1=array 2=lists
// This input vector contains the vertex position in xyz, and the
// mass of the vertex in w
layout (location = 0) in vec3 position;
// This is the current velocity of the vertex
layout (location = 1) in vec4 velocity_mass;

layout (location = 2) in vec4 density_pressure;

//layout (location = 3) in vec4 color;
// This is our connection vector


// This is a TBO that will be bound to the same buffer as the
// position_mass input attribute
layout (binding = 0) uniform samplerBuffer tex_position;
layout (binding = 1) uniform samplerBuffer tex_velocity;
layout (binding = 2) uniform isamplerBuffer tex_gridlist;
layout (binding = 3) uniform samplerBuffer tex_density;
// The outputs of the vertex shader are the same as the inputs
out vec3 tf_position;
out vec4 tf_velocity_mass;
out vec2 tf_density_pressure;


uniform vec3 atractC = vec3(7,0.5,0);
uniform vec3 repulsC = vec3(3,2,0);
// The global spring constant


// Gravity
const float gravity = 0.001;
const float wallDamping = 0.1;
// UNIFORM PHISICS PARAMETERS
// Global damping constant
uniform float c = 0.0005;
const float wall_mass=100;	
const vec2 wall_density_preassure = vec2(1,40);
const float wall_hard_bounce=1.05;
uniform float rest_density=0.5;
uniform float eta=1.30;
uniform float dens_K=15.5;
uniform float sigma=0.15;
uniform float surfTensTresh=0.010;
uniform float deltaT=0.050;
const float h=1.5;
const float GRID_VOLUME_SIDE=10;
const int gridSide=3;
const float cellSize=2*GRID_VOLUME_SIDE/gridSide;//GRID_VOLUME_SIDE/gridSide
uniform float offset=10;



//wind 
uniform float wind_speed=1.0;
uniform vec3 wind_dir=vec3(0.0,0.0,1.0);
const float radius = 1.2;
const float minDst = 1.0;




vec3 pressureF=vec3(0);
vec3 viscosityF=vec3(0);
vec3 color_field_gradient=vec3(0);
vec3 color_field_laplacian=vec3(0);
vec2 new_density_pressure= vec2(0);

float gausKernel(float r){	
	const float h2=h*h;
	const float h3=h*h*h;
	return (exp(r*r/h2)/(pow(PI,3.0/2.0)*h3));	
	}
	
float W_poly6(float r){	
	const float h2=h*h;
	const float h3=h*h*h;
	const float h9=h3*h3*h3;
	return (pow((h2-r*r),3)*315/(64*PI*h9));	
	}
float gradient_W_spiky(float r){	
	const float h2=h*h;
	const float h3=h*h*h;
	const float h6=h3*h3;
	return ((-r)*(45/(PI*h6*r))*pow(h-r,2));	
	}
float laplacian_W_viscosity(float r){	
	const float h2=h*h;
	const float h3=h*h*h;
	const float h5=h2*h3;
	return (45/(PI*h5)*(1-r/h));	
	}
float gradient_W_poly6(float r){	
	const float h2=h*h;
	const float h3=h*h*h;
	const float h9=h3*h3*h3;
	return ((-r)*(945/(32*PI*h9))*pow(h2-r*r,2));		
	}
float laplacian_W_poly6(float r){	
	const float h2=h*h;
	const float h3=h*h*h;
	const float h9=h3*h3*h3;
	float r2=r*r;
	float h2rr=(h2-r2);
	return (945/(8*PI*h9)*h2rr*(r2-3.0/4.0*h2rr));	
	}

    int CellIndex(vec3 point){
		int cellIndex=0;
		cellIndex = max(min(int((point.z+offset)/cellSize)*gridSide*gridSide,gridSide*gridSide*(gridSide-1)),0);
		cellIndex +=max(min(int((point.y+offset)/cellSize)*gridSide,gridSide*(gridSide-1)),0);
		cellIndex +=max(min(int((point.x+offset)/cellSize),(gridSide-1)),0);
		return cellIndex;	
	}
	
	
	 void forEveryParticle(int i){
	vec3 npos = texelFetch(tex_position,i).xyz;			
			vec3 dstV =npos-position;			
			float dst = length(dstV);
			if(dst<=h){
			dstV=normalize(dstV);
			if(abs(dst)<0.001){return;	dstV=vec3(1,1,1);dst=1;}
			vec2 n_density_pressure = texelFetch(tex_density,i).xy;
			vec4 n_velocity_mass = texelFetch(tex_velocity,i);
			new_density_pressure.x+=n_velocity_mass.w*W_poly6(dst);
			pressureF +=n_velocity_mass.w*(density_pressure.y+n_density_pressure.y)/(2*n_density_pressure.x)*gradient_W_spiky(dst)*dstV;			
			viscosityF+=eta*n_velocity_mass.w*((n_velocity_mass.xyz-velocity_mass.xyz))/n_density_pressure.x*laplacian_W_viscosity(dst);
			color_field_gradient+=n_velocity_mass.w/n_density_pressure.x*gradient_W_poly6(dst)*dstV;
			color_field_laplacian+=n_velocity_mass.w/n_density_pressure.x*laplacian_W_poly6(dst)*dstV;
			
			}			
	}
	void forEveryWall(vec4 plane){	

			vec3 dstV =-plane.xyz;						
			float dst = (position.x*plane.x+position.y*plane.y+position.z*plane.z+plane.w);
			
			if(abs(dst)<=h){			
			if(abs(dst)<0.001){return;	dstV=vec3(1,1,1);dst=1;}
			vec2 n_density_pressure =wall_density_preassure;
			vec4 n_velocity_mass = vec4(0,0,0,wall_mass);
			new_density_pressure.x+=n_velocity_mass.w*W_poly6(dst);
			pressureF +=n_velocity_mass.w*(density_pressure.y+n_density_pressure.y)/(2*n_density_pressure.x)*gradient_W_spiky(dst)*dstV;			
			viscosityF+=eta*n_velocity_mass.w*((n_velocity_mass.xyz-velocity_mass.xyz))/n_density_pressure.x*laplacian_W_viscosity(dst);
			color_field_gradient+=n_velocity_mass.w/n_density_pressure.x*gradient_W_poly6(dst)*dstV;
			color_field_laplacian+=n_velocity_mass.w/n_density_pressure.x*laplacian_W_poly6(dst)*dstV;
			}
			
	}

	/* void forEveryParticleNaive(int i){
	vec3 npos = texelFetch(tex_position,i).xyz;			
			vec3 dstV = p-npos;			
			float dst = length(dstV);
			dstV=normalize(dstV);
			
			vec3 ivel = texelFetch(tex_velocity,i).xyz;						
			if(dst<minDst){		
			p+=dstV*1.0*(minDst-dst);	
			v-=dstV*(dot(v,dstV)+dot(ivel,-dstV))*0.3;
			//v*=0.99;
			}
			//v-=dstV*(dot(v,dstV)+dot(ivel,-dstV))*1.0;
			float dst2=dst*dst;
			float atractForce = 0.1/(dst2*atractC.x+dst*atractC.y+atractC.z);
			float repulsForce =0.1/(dst2*repulsC.x+dst*repulsC.y+repulsC.z);
			
			
			//v+=-(dstV*atractForce);
			//v+=(dstV*repulsForce);
	}
	*/
	
	void doForCellIndexLists(int myCell){	
	int curInd = myCell*2;
	int counts = texelFetch(tex_gridlist,curInd+1).r;	
	curInd = texelFetch(tex_gridlist,curInd).r;// mestim kym pyrwi element
	while(curInd!=0){
	int curParticleInd = texelFetch(tex_gridlist,curInd).r;
	if(curParticleInd!=gl_VertexID)forEveryParticle(curParticleInd);
	curInd = texelFetch(tex_gridlist,curInd+1).r;// mestim kym sledwashtiq element
	}
	}
	
	void doForCellIndexArrays(int myCell){	
	int curInd = myCell*2;
	int count = texelFetch(tex_gridlist,curInd+1).r;	
	curInd = texelFetch(tex_gridlist,curInd).r;// mestim kym pyrwi element
	while(count-- >0){	
	int curParticleInd = texelFetch(tex_gridlist,curInd).r;	
	if(curParticleInd!=gl_VertexID)forEveryParticle(curParticleInd);
	curInd++;
	}
	}
void main(void)
{
  //   p = position.xyz;    // p can be our position
//	 m = velocity_mass.w;     // m is the mass of our vertex
 //    v = velocity_mass.xyz;             // u is the initial velocity
	
	//v.y-=gravity;
	
	
 #if (OPTIM_STRUCT>0)
    int visitingCelsSet[gridSide*gridSide*gridSide];//=int[gridSide*gridSide*gridSide](0);
	for(int i=0;i<visitingCelsSet.length();i++)visitingCelsSet[i]=0;
			
	visitingCelsSet[CellIndex(position+vec3(1,1,0))]=1;
	visitingCelsSet[CellIndex(position+vec3(1,1,1))]=1;
	visitingCelsSet[CellIndex(position+vec3(1,1,-1))]=1;
	visitingCelsSet[CellIndex(position+vec3(1,0,0))]=1;
	visitingCelsSet[CellIndex(position+vec3(1,0,1))]=1;
	visitingCelsSet[CellIndex(position+vec3(1,0,-1))]=1;	
	visitingCelsSet[CellIndex(position+vec3(1,-1,0))]=1;
	visitingCelsSet[CellIndex(position+vec3(1,-1,1))]=1;
	visitingCelsSet[CellIndex(position+vec3(1,-1,-1))]=1;
	
	visitingCelsSet[CellIndex(position+vec3(0,1,0))]=1;
	visitingCelsSet[CellIndex(position+vec3(0,1,1))]=1;
	visitingCelsSet[CellIndex(position+vec3(0,1,-1))]=1;
	visitingCelsSet[CellIndex(position+vec3(0,0,0))]=1;
	visitingCelsSet[CellIndex(position+vec3(0,0,1))]=1;
	visitingCelsSet[CellIndex(position+vec3(0,0,-1))]=1;	
	visitingCelsSet[CellIndex(position+vec3(0,-1,0))]=1;
	visitingCelsSet[CellIndex(position+vec3(0,-1,1))]=1;
	visitingCelsSet[CellIndex(position+vec3(0,-1,-1))]=1;
	
	visitingCelsSet[CellIndex(position+vec3(-1,1,0))]=1;
	visitingCelsSet[CellIndex(position+vec3(-1,1,1))]=1;
	visitingCelsSet[CellIndex(position+vec3(-1,1,-1))]=1;
	visitingCelsSet[CellIndex(position+vec3(-1,0,0))]=1;
	visitingCelsSet[CellIndex(position+vec3(-1,0,1))]=1;
	visitingCelsSet[CellIndex(position+vec3(-1,0,-1))]=1;	
	visitingCelsSet[CellIndex(position+vec3(-1,-1,0))]=1;
	visitingCelsSet[CellIndex(position+vec3(-1,-1,1))]=1;
	visitingCelsSet[CellIndex(position+vec3(-1,-1,-1))]=1;
	
	for(int i=0;i<visitingCelsSet.length();i++){
	if(visitingCelsSet[i]){
	#if (OPTIM_STRUCT==1)
	doForCellIndexArrays(i);		
	#elif  (OPTIM_STRUCT==2) 
	doForCellIndexLists(i);	
	#endif
	}
	}

 
 #else 
	int imax = textureSize(tex_position);
	for(int i=0;i<imax;i++){
		if(i!=gl_VertexID)
		{
		forEveryParticle(i);
		}
	}	
	#endif 
	
	forEveryWall(vec4(0,1,0,GRID_VOLUME_SIDE*wall_hard_bounce));
	forEveryWall(vec4(1,0,0, GRID_VOLUME_SIDE*wall_hard_bounce));
	forEveryWall(vec4(-1,0,0,GRID_VOLUME_SIDE*wall_hard_bounce));
	forEveryWall(vec4(0,0,1, GRID_VOLUME_SIDE*wall_hard_bounce));
	forEveryWall(vec4(0,0,-1,GRID_VOLUME_SIDE*wall_hard_bounce));
	        

	vec3 surf_tensF=vec3(0);
float gradientLength = length(color_field_gradient);	
   if(gradientLength>=surfTensTresh){
   surf_tensF = -sigma*color_field_laplacian*color_field_gradient/gradientLength;
   }

 
vec3 total_force = surf_tensF+pressureF+viscosityF;
vec3 acceleration = total_force/new_density_pressure.x*deltaT + wind_dir*wind_speed*deltaT;
vec4 final_velocity_mass = vec4(velocity_mass.xyz + acceleration*deltaT,velocity_mass.w);
vec3 final_position = position + final_velocity_mass.xyz *deltaT;		
	
	
	if(final_position.y <-(GRID_VOLUME_SIDE+0.002)){
	final_position.y=-GRID_VOLUME_SIDE;
	final_velocity_mass.y=-final_velocity_mass.y*wallDamping;
	}
	
	if(final_position.x <-(GRID_VOLUME_SIDE+0.002)){
	final_position.x=-GRID_VOLUME_SIDE;
	final_velocity_mass.x=-final_velocity_mass.x*wallDamping;
	}
	if(final_position.x >GRID_VOLUME_SIDE-0.002){
	final_position.x=GRID_VOLUME_SIDE;
	final_velocity_mass.x=-final_velocity_mass.x*wallDamping;
	}
	
	if(final_position.z <-(GRID_VOLUME_SIDE+0.002)){
	final_position.z=-GRID_VOLUME_SIDE;
	final_velocity_mass.z=-final_velocity_mass.z*wallDamping;
	}
	if(final_position.z >GRID_VOLUME_SIDE-0.002){
	final_position.z=GRID_VOLUME_SIDE;
	final_velocity_mass.z=-final_velocity_mass.z*wallDamping;
	}
	
	
	
	
 
 
 tf_velocity_mass=final_velocity_mass;
 tf_position=final_position;
 new_density_pressure.y = dens_K*(density_pressure.x-rest_density);
 tf_density_pressure = new_density_pressure;   
   
	
}
