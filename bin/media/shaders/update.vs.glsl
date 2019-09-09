#version 410 core
#define PI 3.1415926535897932384626433832795

// This input vector contains the vertex position in xyz, and the
// mass of the vertex in w
layout (location = 0) in vec3 position;
// This is the current velocity of the vertex
layout (location = 1) in vec4 velocity_mass;

layout (location = 2) in vec2 density_pressure;

//layout (location = 3) in vec4 color;

// This is a TBO that will be bound to the same buffer as the
// position_mass input attribute
layout (binding = 0) uniform samplerBuffer tex_position;
layout (binding = 1) uniform samplerBuffer tex_velocity;
layout (binding = 2) uniform isamplerBuffer tex_gridlist;
layout (binding = 3) uniform samplerBuffer tex_density;
layout (binding = 4) uniform samplerBuffer tex_griddata;
// The outputs of the vertex shader are the same as the inputs
out vec3 tf_position;
out vec4 tf_velocity_mass;
out vec2 tf_density_pressure;
out uint tf_color;

uniform vec4 sphere1 = vec4(0,0,0,2);

uniform vec3 sphere1_speed = vec3(0,0,0);

uniform vec3 atractC = vec3(7,0.5,0);
uniform vec3 repulsC = vec3(3,2,0);

// UNIFORM PHISICS PARAMETERS
const float gravity = 0.001;
const float wallDamping = 0.1;
//uniform float c = 0.0005;
const float wall_mass=100;	
const vec2 wall_density_preassure = vec2(1,40);
const float wall_hard_bounce=1.03;
uniform float rest_density=0.8;
uniform float eta=1.50;//viscosity
uniform float dens_K=20.5;
uniform float sigma=1.0;
uniform float surfTensTresh=0.02;//0.20;
uniform float deltaT=0.050;
const float h=1.5;
#define OPTIM_STRUCT 4 //0=no 1=array 2=lists 3=arrayAllNeighboursInCell 4=arrayAllNeighbourDataInCell
const float GRID_VOLUME_SIDE=10;
const int gridSide=15;
const float cellSize=2*GRID_VOLUME_SIDE/gridSide;//GRID_VOLUME_SIDE/gridSide
const float offset=10;
const int gridSize=gridSide*gridSide*gridSide;



//wind 
uniform float wind_speed=1.0;
uniform vec3 wind_dir=vec3(0.0,0.0,1.0);
uniform vec3 glass_pos=vec3(0.0,0.0,0.0);
const float radius = 1.2;
const float minDst = 1.0;




vec3 pressureF=vec3(0);
vec3 viscosityF=vec3(0);
vec3 color_field_gradient=vec3(0);
//vec3 color_field_laplacian=vec3(0);
float color_field_laplacian=0;
vec2 new_density_pressure= vec2(0.1,0);

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
	float hr2=h-r;
	return ((-r)*(45/(PI*h6*r))*hr2*hr2);	
//return ((-r)*(45/(PI*h6*r))*pow(h-r,2));	
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
	float h2rr2 = h2-r*r;
	return ((-r)*(945/(32*PI*h9))*h2rr2*h2rr2);
//return ((-r)*(945/(32*PI*h9))*pow(h2-r*r,2));	
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
		point=(point - glass_pos + offset)/cellSize;
		cellIndex = max(min(int((point.z))*gridSide*gridSide,gridSide*gridSide*(gridSide-1)),0);
		cellIndex +=max(min(int((point.y))*gridSide,gridSide*(gridSide-1)),0);
		cellIndex +=max(min(int((point.x)),(gridSide-1)),0);
		return cellIndex;	
	}
	/*
	int CellIndexNeighbour(vec3 point){
		int cellIndex=0;
		point=(point - glass_pos + offset)/cellSize;
		int tmp;
		tmp = (int(floor(point.z))*gridSide*gridSide);
		cellIndex = (tmp>=0)?((tmp<=gridSide*gridSide*(gridSide-1))?(tmp):(-10000)):-10000;
		tmp = (int(floor(point.y))*gridSide);
		cellIndex += (tmp>=0)?((tmp<=gridSide*(gridSide-1))?(tmp):(-10000)):-10000;
	    tmp = (int(floor(point.x)));
		cellIndex += (tmp>=0)?((tmp<=(gridSide-1))?(tmp):(-10000)):-10000;		
		
		return cellIndex;	
	}
	*/
	
	int CellIndexNeighbour(vec3 point){
		int cellIndex=0;
		point=(point - glass_pos + offset)/cellSize;	
		cellIndex = (int(floor(point.z))*gridSide*gridSide);
		cellIndex +=(int(floor(point.y))*gridSide);
		cellIndex +=(int(floor(point.x)));			
		return cellIndex;	
	}	
	
	void forEveryWall(vec4 plane){
		vec3 dstV =-plane.xyz;						
		float dst = (dot(position.xyz,plane.xyz)+plane.w);	
		float hWall = h;
		if(abs(dst)<=hWall){			
			//if(abs(dst)<0.001){return;	dstV=vec3(1,1,1);dst=1;}
			vec2 n_density_pressure =wall_density_preassure;
			vec4 n_velocity_mass = vec4(0,0,0,wall_mass);
			new_density_pressure.x+=n_velocity_mass.w*W_poly6(dst);
			pressureF +=n_velocity_mass.w*(density_pressure.y+n_density_pressure.y)/(2*n_density_pressure.x)*gradient_W_spiky(dst)*dstV;			
			viscosityF+=0.5*eta*n_velocity_mass.w*((n_velocity_mass.xyz-velocity_mass.xyz))/n_density_pressure.x*laplacian_W_viscosity(dst);
			//color_field_gradient+=n_velocity_mass.w/n_density_pressure.x*gradient_W_poly6(dst)*dstV;
			//color_field_laplacian+=n_velocity_mass.w/n_density_pressure.x*laplacian_W_poly6(dst)*dstV;	
			
			//color_field_gradient+=-10.0*n_velocity_mass.w/n_density_pressure.x*gradient_W_poly6(dst)*dstV;
			
			//color_field_laplacian+=n_velocity_mass.w/n_density_pressure.x*laplacian_W_poly6(dst)*dstV;
		//	color_field_laplacian+=-n_velocity_mass.w/n_density_pressure.x*laplacian_W_poly6(dst);
			
		}
	}
	void forEverySphere(vec4 sphere){
		vec3 dstV =  sphere.xyz - position.xyz;						
		float dst = abs(length(dstV)-sphere.w);			
		if((dst)<=h){	
			dstV/=dst;		
			//if(abs(dst)<0.001){return;	dstV=vec3(1,1,1);dst=1;}
			vec2 n_density_pressure =wall_density_preassure;
			vec4 n_velocity_mass = vec4(sphere1_speed,wall_mass);
			new_density_pressure.x+=n_velocity_mass.w*W_poly6(dst);
			pressureF +=n_velocity_mass.w*(density_pressure.y+n_density_pressure.y)/(2*n_density_pressure.x)*gradient_W_spiky(dst)*dstV;			
			//viscosityF+=eta*n_velocity_mass.w*((n_velocity_mass.xyz-velocity_mass.xyz))/n_density_pressure.x*laplacian_W_viscosity(dst);
			
			//color_field_gradient+=n_velocity_mass.w/n_density_pressure.x*gradient_W_poly6(dst)*dstV;
			//color_field_laplacian+=n_velocity_mass.w/n_density_pressure.x*laplacian_W_poly6(dst)*dstV;	
		
			//ecolor_field_gradient+=-50.2*n_velocity_mass.w/n_density_pressure.x*gradient_W_poly6(dst)*dstV;
			
			//color_field_laplacian+=n_velocity_mass.w/n_density_pressure.x*laplacian_W_poly6(dst)*dstV;
		//	color_field_laplacian+=-n_velocity_mass.w/n_density_pressure.x*laplacian_W_poly6(dst);
			
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
	
	

	void forEveryParticle(int i){
		vec3 npos = texelFetch(tex_position,i).xyz;			
		vec3 dstV =npos-position;			
		float dst = length(dstV);
		if(dst<=h)
		{
			if((dst)<0.001){return;	dstV=vec3(1,1,1);dst=1;}
			dstV/=dst;			
			vec2 n_density_pressure = texelFetch(tex_density,i).xy;
			vec4 n_velocity_mass = texelFetch(tex_velocity,i);
			
			new_density_pressure.x+=n_velocity_mass.w*W_poly6(dst);
			pressureF +=n_velocity_mass.w*(density_pressure.y+n_density_pressure.y)/(2*n_density_pressure.x)*gradient_W_spiky(dst)*dstV;
			
			//pressureF +=density_pressure.x *n_velocity_mass.w*(density_pressure.y/(density_pressure.x*density_pressure.x)+n_density_pressure.y/(n_density_pressure.x*n_density_pressure.x))*gradient_W_spiky(dst)*dstV;
				
			viscosityF+=eta*n_velocity_mass.w*((n_velocity_mass.xyz-velocity_mass.xyz))/n_density_pressure.x*laplacian_W_viscosity(dst);
			color_field_gradient+=n_velocity_mass.w/n_density_pressure.x*gradient_W_poly6(dst)*dstV;
			//color_field_gradient+=n_velocity_mass.w/n_density_pressure.x*vec3(gradient_W_poly6(dstV.x),gradient_W_poly6(dstV.y),gradient_W_poly6(dstV.z));
			//color_field_laplacian+=n_velocity_mass.w/n_density_pressure.x*laplacian_W_poly6(dst)*dstV;
			color_field_laplacian+=n_velocity_mass.w/n_density_pressure.x*laplacian_W_poly6(dst);
		}			
	}
	void forEveryParticleGridData(int i,int j){ /// tukaaa eeee		
		// i = gridDataIndex , j = particelInd
		vec3 npos = texelFetch(tex_position,j).xyz;	
		//vec3 npos = vec3(texelFetch(tex_griddata,i).x , texelFetch(tex_griddata,i+1).x , texelFetch(tex_griddata,i+2).x);
		//npos=npos1;
		vec3 dstV =npos-position;			
		float dst = length(dstV);
		if(dst<=h)
		{
			if((dst)<0.001){return;	dstV=vec3(1,1,1);dst=1;}
			dstV/=dst;			
			
			vec4 n_velocity_mass = texelFetch(tex_velocity,j);	
			//vec4 n_velocity_mass =vec4(texelFetch(tex_griddata,i+3).x , texelFetch(tex_griddata,i+4).x , texelFetch(tex_griddata,i+5).x, texelFetch(tex_griddata,i+6).x);
			vec2 n_density_pressure = texelFetch(tex_density,j).xy;	
			//vec2 n_density_pressure =vec2(texelFetch(tex_griddata,i+7).x , texelFetch(tex_griddata,i+8).x);
			
			new_density_pressure.x+=n_velocity_mass.w*W_poly6(dst);
			pressureF +=n_velocity_mass.w*(density_pressure.y+n_density_pressure.y)/(2*n_density_pressure.x)*gradient_W_spiky(dst)*dstV;
			
			//pressureF +=density_pressure.x *n_velocity_mass.w*(density_pressure.y/(density_pressure.x*density_pressure.x)+n_density_pressure.y/(n_density_pressure.x*n_density_pressure.x))*gradient_W_spiky(dst)*dstV;
				
			viscosityF+=eta*n_velocity_mass.w*((n_velocity_mass.xyz-velocity_mass.xyz))/n_density_pressure.x*laplacian_W_viscosity(dst);
			color_field_gradient+=n_velocity_mass.w/n_density_pressure.x*gradient_W_poly6(dst)*dstV;
			//color_field_gradient+=n_velocity_mass.w/n_density_pressure.x*vec3(gradient_W_poly6(dstV.x),gradient_W_poly6(dstV.y),gradient_W_poly6(dstV.z));
			//color_field_laplacian+=n_velocity_mass.w/n_density_pressure.x*laplacian_W_poly6(dst)*dstV;
			color_field_laplacian+=n_velocity_mass.w/n_density_pressure.x*laplacian_W_poly6(dst);
		}			
	}
#if OPTIM_STRUCT <4
	void doForCellIndexArrays(int myCell){	
		int curInd = myCell*2;
		int count = texelFetch(tex_gridlist,curInd+1).r;	
		curInd = texelFetch(tex_gridlist,curInd).r;// mestim kym pyrwi element
		while(count-- >0 ){	
			int curParticleInd = texelFetch(tex_gridlist,curInd).r;	
			//if(curParticleInd!=gl_VertexID)
			forEveryParticle(curParticleInd);
			curInd++;
		}
	}	

	void doForCellIndexLists(int myCell){	
		int curInd = myCell*2;
		int counts = texelFetch(tex_gridlist,curInd+1).r;	
		curInd = texelFetch(tex_gridlist,curInd).r;// mestim kym pyrwi element
		while(curInd!=0){
			int curParticleInd = texelFetch(tex_gridlist,curInd).r;
			//if(curParticleInd!=gl_VertexID)
			forEveryParticle(curParticleInd);
			curInd = texelFetch(tex_gridlist,curInd+1).r;// mestim kym sledwashtiq element
		}
	}
#elif OPTIM_STRUCT==4	
	void doForCellIndexArrays(int myCell){	
		int curInd = myCell*3;			
		int j = texelFetch(tex_gridlist,curInd).r;// gridIndex
		int count = texelFetch(tex_gridlist,curInd+1).r; // Num elements
		int i = texelFetch(tex_gridlist,curInd+2).r;// gridDataIndex
		while(count-- >0 ){		
			//int curParticleInd = texelFetch(tex_gridlist,j).r;	
			int curParticleInd = int(texelFetch(tex_griddata,i+9).r);
			forEveryParticleGridData(i,curParticleInd);
			i+=10;
			j++;	
		}
	}
#endif
	
void main(void)
{
 #if (OPTIM_STRUCT>0)
    int centerCellIndex = CellIndex(position);  
    #if ((OPTIM_STRUCT==1)||(OPTIM_STRUCT==3)||(OPTIM_STRUCT==4))
	doForCellIndexArrays(centerCellIndex);		
	#elif  (OPTIM_STRUCT==2) 
	doForCellIndexLists(centerCellIndex);	
	#endif 	
	#if ((OPTIM_STRUCT==1)||(OPTIM_STRUCT==2))
	const vec3[26] offsets=vec3[26](vec3(cellSize,cellSize,0),  vec3(cellSize,cellSize,cellSize),   vec3(cellSize,cellSize,-cellSize),
									vec3(cellSize,0,0), 		vec3(cellSize,0,cellSize),  	    vec3(cellSize,0,-cellSize),
									vec3(cellSize,-cellSize,0), vec3(cellSize,-cellSize,cellSize),  vec3(cellSize,-cellSize,-cellSize),
									vec3(0,cellSize,0),  		vec3(0,cellSize,cellSize),  	    vec3(0,cellSize,-cellSize),
										 			vec3(0,0,cellSize),  				vec3(0,0,-cellSize),
									vec3(0,-cellSize,0), 		vec3(0,-cellSize,cellSize), 		vec3(0,-cellSize,-cellSize),
									vec3(-cellSize,cellSize,0), vec3(-cellSize,cellSize,cellSize), 	vec3(-cellSize,cellSize,-cellSize),
									vec3(-cellSize,0,0),		vec3(-cellSize,0,cellSize), 		vec3(-cellSize,0,-cellSize),
									vec3(-cellSize,-cellSize,0),vec3(-cellSize,-cellSize,cellSize),	vec3(-cellSize,-cellSize,-cellSize));
	
	for(int i=0;i<offsets.length();i++){	
	int curCellIndex = CellIndexNeighbour(position+offsets[i]);
		if((curCellIndex>=0)&&(curCellIndex<gridSize))
		{
		#if (OPTIM_STRUCT==1)
		doForCellIndexArrays(curCellIndex);		
		#elif  (OPTIM_STRUCT==2) 
		doForCellIndexLists(curCellIndex);	
		#endif
		}
	}	
	#endif
  #else 
	int imax = textureSize(tex_position);
	for(int i=0;i<imax;i++){
		if(i!=gl_VertexID)
		{
		forEveryParticle(i);
		}
	}	
  #endif 
	
	//*
	forEveryWall(vec4(0,1,0,-glass_pos.y+GRID_VOLUME_SIDE*wall_hard_bounce));
	forEveryWall(vec4(0,-1,0,glass_pos.y+GRID_VOLUME_SIDE*wall_hard_bounce));
	forEveryWall(vec4(1,0,0, -glass_pos.x+GRID_VOLUME_SIDE*wall_hard_bounce));
	forEveryWall(vec4(-1,0,0,glass_pos.x+GRID_VOLUME_SIDE*wall_hard_bounce));
	forEveryWall(vec4(0,0,1, -glass_pos.z+GRID_VOLUME_SIDE*wall_hard_bounce));
	forEveryWall(vec4(0,0,-1,glass_pos.z+GRID_VOLUME_SIDE*wall_hard_bounce));
	 //*/   
	forEverySphere(sphere1);

	vec3 surf_tensF=vec3(0);
	float gradientLength = length(color_field_gradient);	// normal vector na povyrninata
    if(gradientLength>=surfTensTresh)
	{
	    //float k = color_field_laplacian;
		//surf_tensF =0.1*sigma*k*(color_field_gradient/gradientLength);
		surf_tensF = -sigma*(color_field_gradient)*(gradientLength);
		
    }

	tf_color = int(clamp(length(color_field_gradient)*255.0,0,255));
 
vec3 total_force = surf_tensF+pressureF+viscosityF;

/*
if(texelFetch(tex_griddata,720000).x==0.6){

}else {
	if(position.x<-8)total_force += vec3(0,0,5);
	if(position.x>8)total_force += vec3(0,0,-5);

	if(position.z<-8)total_force += vec3(5,0,0);
	if(position.z>8)total_force += vec3(-5,0,0);
}
//*/

/*
if(length(position.xz + vec2(3,4))<3)
{
total_force+=cross(position , vec3(0,0.9,0));
}
if(length(position.xz + vec2(-3,4))<3)
{
total_force+=cross(position , vec3(0,-0.9,0));
}

//else{total_force+=cross(position , vec3(0,0.1,0));}
//*/

/*
if(position.y<-8)
total_force+= vec3(position.x,0,position.z )*0.8;
//*/

vec3 acceleration = total_force/new_density_pressure.x + wind_dir*wind_speed;
vec4 final_velocity_mass = vec4(velocity_mass.xyz + acceleration*deltaT*deltaT,velocity_mass.w);
vec3 final_position = position + final_velocity_mass.xyz *deltaT;		
	/*
	
	if(final_position.y <-(GRID_VOLUME_SIDE+0.002)+glass_pos.y){
	final_position.y=-GRID_VOLUME_SIDE+glass_pos.y;
	final_velocity_mass.y=-final_velocity_mass.y*wallDamping;
	}
	if(final_position.y >(GRID_VOLUME_SIDE-0.002)+glass_pos.y){
	final_position.y=GRID_VOLUME_SIDE+glass_pos.y;
	final_velocity_mass.y=-final_velocity_mass.y*wallDamping;
	}
	
	if(final_position.x <-(GRID_VOLUME_SIDE+0.002)+glass_pos.x){
	final_position.x=-GRID_VOLUME_SIDE+glass_pos.x;
	final_velocity_mass.x=-final_velocity_mass.x*wallDamping;
	}
	if(final_position.x >GRID_VOLUME_SIDE-0.002+glass_pos.x){
	final_position.x=GRID_VOLUME_SIDE+glass_pos.x;
	final_velocity_mass.x=-final_velocity_mass.x*wallDamping;
	}
	
	if(final_position.z <-(GRID_VOLUME_SIDE+0.002)+glass_pos.z){
	final_position.z=-GRID_VOLUME_SIDE+glass_pos.z;
	final_velocity_mass.z=-final_velocity_mass.z*wallDamping;
	}
	if(final_position.z >GRID_VOLUME_SIDE-0.002+glass_pos.z){
	final_position.z=GRID_VOLUME_SIDE+glass_pos.z;
	final_velocity_mass.z=-final_velocity_mass.z*wallDamping;
	}
	//*/
	
	//if(length(final_position)>20)final_position=vec3(0);
	
 //if(length(final_velocity_mass.xyz)>10)final_velocity_mass=vec4(0,0,0,final_velocity_mass.w);
 if(length(final_position)>20){final_position=vec3(0,sphere1.w+sphere1.y+2,1);final_velocity_mass=vec4(0,0,0,final_velocity_mass.w);}
 //if(length(final_position-sphere1.xyz)<sphere1.w){final_position=vec3(0,6,0);final_velocity_mass=vec4(0,0,0,final_velocity_mass.w);}
 
 tf_velocity_mass=final_velocity_mass;
 //tf_position=clamp(final_position,-10,10);
 tf_position=final_position;
 new_density_pressure.y = dens_K*(density_pressure.x-rest_density);
 tf_density_pressure = new_density_pressure;   
  //tf_color = (0); 
	

}
