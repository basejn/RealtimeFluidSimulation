#version 410 core
#define PI 3.1415926535897932384626433832795
layout (location = 0) out vec4 color;

layout (binding = 0) uniform samplerBuffer tex_position;
layout (binding = 1) uniform samplerBuffer tex_velocity;
layout (binding = 2) uniform isamplerBuffer tex_gridlist;
layout (binding = 3) uniform samplerBuffer tex_density;
layout (binding = 4) uniform samplerBuffer tex_color;
layout (binding = 5)uniform samplerCube tex_cubemap;
layout (binding = 6) uniform sampler3D tex_Density_Field;
uniform vec3 light_pos;
uniform mat4 ray_lookat;

uniform vec4 sphere1 = vec4(0,0,0,2);
uniform vec4 sphereColor = vec4(1,0,0,1);
#define SimpleSphereRendering 0 // 1 - simple 2 simpler 0 fancy
#define OPTIM_STRUCT  2//0=no 1=array 2=lists
const float GRID_VOLUME_SIDE=10;
const int gridSide=12;
const float cellSize=2*GRID_VOLUME_SIDE/gridSide;//GRID_VOLUME_SIDE/gridSide
const int gridSize=gridSide*gridSide*gridSide;
const float offset=10;
const float h=1.5;
const float isosurfDens = .9;
const float tolerance = 0.00001;
const float step = .5;
const float air_n = 1;
const float fluid_n = 1.33;
//uniform vec3 glass_pos=vec3(0.0,0.0,0.0);


const float CUBE_SIDE = 10;
const vec3 glass_pos = vec3(0);
const vec3 p1=vec3(-CUBE_SIDE,-CUBE_SIDE,CUBE_SIDE)+glass_pos;
const vec3 p2=vec3(CUBE_SIDE,-CUBE_SIDE,CUBE_SIDE)+glass_pos;
const vec3 p3=vec3(CUBE_SIDE,CUBE_SIDE,CUBE_SIDE)+glass_pos;
const vec3 p4=vec3(-CUBE_SIDE,CUBE_SIDE,CUBE_SIDE)+glass_pos;
const vec3 p5=vec3(-CUBE_SIDE,-CUBE_SIDE,-CUBE_SIDE)+glass_pos;
const vec3 p6=vec3(CUBE_SIDE,-CUBE_SIDE,-CUBE_SIDE)+glass_pos;
const vec3 p7=vec3(CUBE_SIDE,CUBE_SIDE,-CUBE_SIDE)+glass_pos;
const vec3 p8=vec3(-CUBE_SIDE,CUBE_SIDE,-CUBE_SIDE)+glass_pos;

 in VS_OUT
{  
    vec3     ray_origin ;
    vec3    ray_direction;
} fs_in;

float distanceLinePoint(vec3 p1,vec3 p2,vec3 p0){
return length(cross((p0-p1),(p0-p2)))/length(p2-p1);
}

float W_poly6(float r){	
	const float h2=h*h;
	const float h3=h*h*h;
	const float h9=h3*h3*h3;
	return (pow((h2-r*r),3)*315/(64*PI*h9));	
	}
float gradient_W_poly6(float r){	
	const float h2=h*h;
	const float h3=h*h*h;
	const float h9=h3*h3*h3;
	float h2rr2 = h2-r*r;
	return ((-r)*(945/(32*PI*h9))*h2rr2*h2rr2);
//return ((-r)*(945/(32*PI*h9))*pow(h2-r*r,2));	
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
	vec3 tmpNorm=vec3(1,0,0);
	vec4 tmpColor=vec4(0,0.1,0.2,1);
	float tmpDensity=0;
	vec3 curPos=vec3(0);
	void forEveryParticle(int i){
	vec3 npos = texelFetch(tex_position,i).xyz;			
			vec3 dstV =npos-curPos;			
			float dst = length(dstV);
			if(dst<=h){
			//dstV=normalize(dstV);			
			vec2 n_density_pressure = texelFetch(tex_density,i).xy;
			vec4 n_velocity_mass = texelFetch(tex_velocity,i);
			tmpDensity+=n_velocity_mass.w*W_poly6(dst);			
			tmpNorm-=dstV*n_velocity_mass.w*W_poly6(dst);
			//tmpNorm-=n_velocity_mass.w/n_density_pressure.x*gradient_W_poly6(dst)*dstV;// this is more correct
			
			tmpColor=(tmpColor+ W_poly6(dst)*texelFetch(tex_color,i).xyzw);
			}			
	}
	
	void doForCellIndexLists(int myCell){	
	int curInd = myCell*2;
	int counts = texelFetch(tex_gridlist,curInd+1).r;	
	curInd = texelFetch(tex_gridlist,curInd).r;// mestim kym pyrwi element
	while(curInd!=0){
	int curParticleInd = texelFetch(tex_gridlist,curInd).r;
//if(curParticleInd!=gl_VertexID){
	forEveryParticle(curParticleInd);
//	}
	curInd = texelFetch(tex_gridlist,curInd+1).r;// mestim kym sledwashtiq element
	}
	}
	
	bool anyPreasureInPoint(vec3 point){
	
	int centerCellIndex = CellIndexNeighbour(point);  
	if(!((centerCellIndex>=0)&&(centerCellIndex<gridSize)))return false;	
	if( texelFetch(tex_gridlist,centerCellIndex+1).r>0)return true;		
	//return tmpDensity;
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
	int curCellIndex = CellIndexNeighbour(point+offsets[i]);
		if((curCellIndex>=0)&&(curCellIndex<gridSize))
		{		
		if( texelFetch(tex_gridlist,curCellIndex+1).r>0)return true;				
		}
	}	
	return false;
	}	
	float preasureInPoint(vec3 point){	
	curPos = point;
	tmpDensity = 0;tmpNorm=vec3(0);tmpColor=vec4(0,0,0,1);
	int centerCellIndex = CellIndexNeighbour(curPos);  
	if(!((centerCellIndex>=0)&&(centerCellIndex<gridSize)))return 0;	
	doForCellIndexLists(centerCellIndex);		
	//return tmpDensity;
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
	int curCellIndex = CellIndexNeighbour(curPos+offsets[i]);
		if((curCellIndex>=0)&&(curCellIndex<gridSize))
		{		
		doForCellIndexLists(curCellIndex);		
		}
	}		
	tmpNorm=normalize(tmpNorm);
	//tmpColor=normalize(tmpColor);
	tmpColor/=tmpColor.w;
	//tmpColor = vec4(0);
	//tmpColor/=	
	//return tmpDensity;
	}	
	float intersect_ray_plane(vec3 Rorig,vec3 Rdir, vec4 P, out vec3 hitpos, out vec3 normal)
{
    vec3 O = Rorig;
    vec3 D = Rdir;
    vec3 N = P.xyz;
    float d = P.w;

    float denom = dot(D, N);

    if (denom == 0.0)
        return 0.0;

    float t = -(d + dot(O, N)) / denom;

    if (t < 0.0)
        return 0.0;

    hitpos = O + t * D;
    normal = N;

    return t;
}

float intersect_ray_sqware(vec3 Rorig,vec3 Rdir, vec4 P,vec3 p1,vec3 p2,vec3 p3,vec3 p4,out vec3 hitpos, out vec3 normal)
{
    vec3 O = Rorig;
    vec3 D = Rdir;
    vec3 N = P.xyz;
    float d = P.w;

    float denom = dot(D, N);

    if (denom == 0.0)
        return 90000;

    float t = -(d + dot(O, N)) / denom;

    if (t < 0.0)
        return 90000;

    hitpos = O + t * D;
    normal = N;

float side = length(p1-p2);

if((abs((distanceLinePoint(p1,p2,hitpos)+distanceLinePoint(p3,p4,hitpos))-side)>0.0001))return 90000;
if((abs((distanceLinePoint(p2,p3,hitpos)+distanceLinePoint(p4,p1,hitpos))-side)>0.0001))return 90000;

    return t;
}

float intersect_ray_sqware_simple(vec3 Rorig,vec3 Rdir, vec4 P,vec3 p1,vec3 p2,vec3 p3,vec3 p4)
{
    vec3 O = Rorig;
    vec3 D = Rdir;
    vec3 N = P.xyz;
    float d = P.w;

    float denom = dot(D, N);

    if (denom == 0.0)
        return 0;

    float t = -(d + dot(O, N)) / denom;

    if (t < 0.0)
        return 0;

   vec3 hitpos = O + t * D;
   

float side = length(p1-p2);
//side=19;
if((abs((distanceLinePoint(p1,p2,hitpos)+distanceLinePoint(p3,p4,hitpos))-side)>0.0001))return 0;
if((abs((distanceLinePoint(p2,p3,hitpos)+distanceLinePoint(p4,p1,hitpos))-side)>0.0001))return 0;

    return t;
}
	
	float intersect_ray_sphere(vec3 orig,vec3 dir,
                           vec4 sphere)
{
   float t0, t1; // solutions for t if the ray intersects

// geometric solution
	vec3 L = sphere.xyz - orig;
	float tca = dot(dir,L);
// if (tca < 0) return 0;
	float d2 = dot(L,L) - tca * tca;
	float radius2 = sphere.w*sphere.w;
	if (d2 > radius2) return 0;
	float thc = sqrt(radius2 - d2);
	t0 = tca - thc;
	t1 = tca + thc; 
	
	return 1;
	if (t0 > t1){
	if (t0 < 0) {	
	if (t1 < 0) return 0; // both t0 and t1 are negative
	else return t1;
	}
	}else{
	if (t1 < 0) {	
	if (t0 < 0) return 0; // both t0 and t1 are negative
	else return t0;
	}
	}
	
	

}

float rendHitTheCube(out vec3 hitPos, out vec3 hitNorm){
float closestT,lastT;	
	vec3 tmphitPos;vec3 tmphitNorm;	
	lastT = closestT = intersect_ray_sqware(fs_in.ray_origin,fs_in.ray_direction,vec4(0,1,0,(-glass_pos.y -CUBE_SIDE)),p3,p4,p8,p7,tmphitPos,tmphitNorm);
	hitPos=tmphitPos;hitNorm=tmphitNorm;
	
	lastT = intersect_ray_sqware(fs_in.ray_origin,fs_in.ray_direction,vec4(0,-1,0,glass_pos.y -CUBE_SIDE),p1,p2,p6,p5,tmphitPos,tmphitNorm);	
	if(lastT<closestT){
	closestT=lastT;hitPos=tmphitPos;hitNorm=tmphitNorm;
	}
	lastT = intersect_ray_sqware(fs_in.ray_origin,fs_in.ray_direction,vec4(1,0,0, -glass_pos.x-CUBE_SIDE),p2,p6,p7,p3,tmphitPos,tmphitNorm);
	if(lastT<closestT){
	closestT=lastT;hitPos=tmphitPos;hitNorm=tmphitNorm;
	}	
	lastT = intersect_ray_sqware(fs_in.ray_origin,fs_in.ray_direction,vec4(-1,0,0,glass_pos.x- CUBE_SIDE),p1,p5,p8,p4,tmphitPos,tmphitNorm);	
	if(lastT<closestT){
	closestT=lastT;hitPos=tmphitPos;hitNorm=tmphitNorm;
	}
	lastT = intersect_ray_sqware(fs_in.ray_origin,fs_in.ray_direction,vec4(0,0,1, -glass_pos.z-CUBE_SIDE),p1,p2,p3,p4,tmphitPos,tmphitNorm);	
	if(lastT<closestT){
	closestT=lastT;hitPos=tmphitPos;hitNorm=tmphitNorm;
	}
	lastT = intersect_ray_sqware(fs_in.ray_origin,fs_in.ray_direction,vec4(0,0,-1,glass_pos.z- CUBE_SIDE),p5,p6,p7,p8,tmphitPos,tmphitNorm);	
	if(lastT<closestT){
	closestT=lastT;hitPos=tmphitPos;hitNorm=tmphitNorm;
	}
return closestT;
}

bool intersectionsOfTheCube(out float tnear,out float tfar,vec3 rOrig,vec3 rDir){
tnear = 99999;
tfar = 0;		
	float lastT = intersect_ray_sqware_simple(rOrig,rDir,vec4(0,1,0,(-glass_pos.y -CUBE_SIDE)),p3,p4,p8,p7);	
	 if(lastT>0){
	tnear=lastT;
	tfar=lastT;
	}
	 lastT = intersect_ray_sqware_simple(rOrig,rDir,vec4(0,-1,0,glass_pos.y -CUBE_SIDE),p1,p2,p6,p5);
if(lastT>0){
	if(lastT<tnear)tnear=lastT;
	if(lastT>tfar)tfar=lastT;
	}
	lastT = intersect_ray_sqware_simple(rOrig,rDir,vec4(1,0,0, -glass_pos.x-CUBE_SIDE),p2,p6,p7,p3);
	if(lastT>0){
	if(lastT<tnear)tnear=lastT;	
	if(lastT>tfar)tfar=lastT;
	}
	lastT = intersect_ray_sqware_simple(rOrig,rDir,vec4(-1,0,0,glass_pos.x- CUBE_SIDE),p1,p5,p8,p4);	
	if(lastT>0){
	if(lastT<tnear)tnear=lastT;
	if(lastT>tfar)tfar=lastT;
	}
	lastT = intersect_ray_sqware_simple(rOrig,rDir,vec4(0,0,1, -glass_pos.z-CUBE_SIDE),p1,p2,p3,p4);	
	if(lastT>0){
	if(lastT<tnear)tnear=lastT;
	if(lastT>tfar)tfar=lastT;
	}
	lastT = intersect_ray_sqware_simple(rOrig,rDir,vec4(0,0,-1,glass_pos.z- CUBE_SIDE),p5,p6,p7,p8);	
	if(lastT>0){
	if(lastT<tnear)tnear=lastT;
	if(lastT>tfar)tfar=lastT;
	}
return tnear == 0||tfar==0;
}

float preasureInPoint3DTexture(vec3 point){
	return texture(tex_Density_Field,point/(GRID_VOLUME_SIDE*2.0) + 0.5).r;
	}
	
vec3 normalInPoint(vec3 point){
vec3 res = vec3(0);
vec3 mesP ;
vec3 dstV;
const float d = 0.32;
dstV = vec3(d,0,0);
mesP = point+ dstV;
res+=dstV  * preasureInPoint3DTexture(mesP);

dstV = vec3(-d,0,0);
mesP = point+ dstV;
res+=dstV  * preasureInPoint3DTexture(mesP);

dstV = vec3(0,d,0);
mesP = point+ dstV;
res+=dstV  * preasureInPoint3DTexture(mesP);

dstV = vec3(0,-d,0);
mesP = point+ dstV;
res+=dstV  * preasureInPoint3DTexture(mesP);

dstV = vec3(0,0,d);
mesP = point+ dstV;
res+=dstV  * preasureInPoint3DTexture(mesP);

dstV = vec3(0,0,-d);
mesP = point+ dstV;
res+=dstV  * preasureInPoint3DTexture(mesP);
return normalize(res);
}

vec4 calcLight(vec3 N,float t){
	vec3 hitPos = fs_in.ray_origin + fs_in.ray_direction*(t);
    vec3 V =- fs_in.ray_direction ;
	vec3 L = normalize(light_pos - hitPos);
    vec3 R = reflect(-L, N);

    vec3 diffuse = max(dot(N, L), 0.0) *tmpColor.xyz;//* vec3(0.6,0.6,0.6);
    vec3 specular = pow(max(dot(R, V), 0.0), 4) * vec3(0.7,0.7,0.8);
 
	vec3 ambient =  max(dot(N, V), 0.0) *tmpColor.xyz*0.2;// vec3(0.1,0.1,0.1);

return  vec4(diffuse + specular + ambient, 1.0);
}

float findIsosurface(float tnear,float tfar,bool incoming,vec3 rOrig,vec3 rDir,out vec3 hitPos,out vec3 hitNorm){
//if(incoming==false)return 0;
  float pr =0;
  float prNear=0,prFar;
float foundT=0;
   for(float i =tnear;i<tfar;i+=step){
   hitPos = rOrig + rDir*(i);
  prFar = pr = preasureInPoint3DTexture(hitPos);
  if((pr<=isosurfDens)!=incoming)
  {
  tfar = i;  
  tnear = i-step;
  foundT=i;
  break;
  }
  prNear = pr;
 } 
if(foundT==0)return 0;

int counter=80;
  float curt = foundT;
 while(abs(pr-isosurfDens)>tolerance&&abs(tnear-tfar)>0.00001){  
   float k=0.5;
  if(counter--<=0)break;
  //if((prFar-prNear)==0){k=0.5;return 0;}else  
  k = (prFar-isosurfDens)/(prFar-prNear);
  //k=0.5;
  curt = mix(tfar,tnear,k); 
  hitPos = rOrig + rDir*(curt);
 pr = preasureInPoint3DTexture(hitPos);
 if((pr<isosurfDens)!=incoming){tfar=curt;prFar=pr;}
 else {tnear = curt; prNear=pr;}
 }
 if(incoming)
 if(pr>isosurfDens)hitPos = rOrig + rDir*(tfar);
 else
 if(pr<isosurfDens)hitPos = rOrig + rDir*(tfar);
 //if(abs(pr-isosurfDens)<=tolerance){
 //finalColor = vec4(pr/(isosurfDens*1.5));
//finalColor = vec4(0.8,0.8,1.0,1)*dot(tmpNorm,-fs_in.ray_direction);
//finalColor = calcLight(tmpNorm ,curt );
 //preasureInPoint(hitPos);
 //tmpNorm =-normalInPoint(hitPos);
 hitNorm = -normalInPoint(hitPos);
return curt;
 //}
// else 0;
}

float fresnelTerm(vec3 L,vec3 N , float eta){
float c = -dot(L,N)*eta;
float etasqr = eta*eta;
float g = sqrt(1 + c*c -etasqr );
float gminc = g-c;float gplusc = g+c;
float gmcgpc = (gminc/gplusc);
float asd = (c*gplusc - etasqr)/(c*gminc + etasqr);
return 0.5*(gmcgpc*gmcgpc)*(1+(asd*asd));
}
float fresnelTermAprox(vec3 L,vec3 N , float eta){
float cosT = -dot(L,N);
//const float reflZero = ((air_n-fluid_n)*(air_n-fluid_n))/((air_n+fluid_n)*(air_n+fluid_n));
const float reflZero = 0.1;
return reflZero +(1-reflZero)*(pow(1-cosT,3)); 
}

float intersect_ray_sphere1(vec3 origin,vec3 direction,
                           vec4 sphere)
{
    vec3 v = origin - sphere.xyz;
    float B = 2.0 * dot(direction, v);
    float C = dot(v, v) - sphere.w * sphere.w;
    float B2 = B * B;

    float f = B2 - 4.0 * C;

    if (f < 0.0)
        return 0.0;

    f = sqrt(f);
    float t0 = -B + f;
    float t1 = -B - f;
    float t = min(max(t0, 0.0), max(t1, 0.0)) * 0.5;

    if (t == 0.0)
        return 0.0;

  
    return t;
}

float intersect_ray_sphere2(vec3 origin,vec3 direction,vec4 sphere)
{
    vec3 v = origin - sphere.xyz;
    float a = 1.0; // dot(R.direction, R.direction);
    float b = 2.0 * dot(direction, v);
    float c = dot(v, v) - (sphere.w * sphere.w);
    float num = b * b - 4.0 * a * c;	
    if (num < 0.0)return 0;

    float d = sqrt(num);
    float e = 1.0 / (2.0 * a);

    float t1 = (-b - d) * e;
    float t2 = (-b + d) * e;
    float t;

    if (t1 <= 0.0)
    {
        t = t2;
    }
    else if (t2 <= 0.0)
    {
        t = t1;
    }
    else
    {
        t = min(t1, t2);
    }

    if (t < 0.0)
        return 0;  
    return t;
	
	//hitpos = origin + t * direction;
    //normal = normalize(hitpos - sphere.xyz);
}

vec4 calcFluidColor(vec3 origin , vec3 direction,float tnear,float tfar ){
#if(SimpleSphereRendering ==2)
return vec4(-1);
#endif
vec3 firstOrig ;
vec3 firstWaterNorm;
float firstWaterT = findIsosurface( tnear, tfar,true,origin,direction,firstOrig,firstWaterNorm);
vec4 finalColor;
float curt = firstWaterT;    
if(curt<=0)return vec4(-1);
#if(SimpleSphereRendering ==1)
return vec4(0.2,0.8,0.7,1);
#endif
finalColor = calcLight(firstWaterNorm ,curt );
//if(curt>0)color = texture(tex_cubemap, cross(firstWaterNorm,-direction));else discard;return;
//color=finalColor;return;
vec3 reflDir=reflect(direction,firstWaterNorm);
float fresnelR = fresnelTermAprox(direction,firstWaterNorm,air_n/fluid_n);
//float reflSphereT = intersect_ray_sphere2(fs_in.ray_origin+direction*curt,reflDir,sphere1);

vec4 firstReflColor;
//if(reflSphereT>0) firstReflColor = sphereColor;else
 firstReflColor = texture(tex_cubemap, reflDir);
finalColor = mix(finalColor,firstReflColor,fresnelR) ;

vec3 newDir=refract(direction,firstWaterNorm,air_n/fluid_n);
vec3 secondOrig;
vec3 secondNorm;
curt =  findIsosurface( 0, 20,false,firstOrig,newDir,secondOrig,secondNorm);

//float refrSphereT = intersect_ray_sphere2(secondOrig,-secondNorm,sphere1);

vec4 firstRefrColor = tmpColor;
//if(refrSphereT>0) firstRefrColor =mix(sphereColor,firstRefrColor,0.4);

finalColor =mix(finalColor,firstRefrColor,0.34*(1-fresnelR));

vec3 thirdDir=refract(newDir,-secondNorm,fluid_n/air_n);

// float fresnelR2 = 0;//fresnelTerm(newDir,secondNorm,fluid_n/air_n);
if(thirdDir!=vec3(0)){
//float secRefrSphereT = intersect_ray_sphere2(secondOrig,thirdDir,sphere1);
vec4 refrOutRayColor;
//if(secRefrSphereT>0)refrOutRayColor = sphereColor;else 
refrOutRayColor = texture(tex_cubemap, thirdDir);

//finalColor += vec4(1)*pow(max(dot(thirdDir,normalize(light_pos-secondOrig)),0),8);
finalColor=mix(finalColor,refrOutRayColor,0.85*(1-fresnelR)) ;

}//else finalColor = vec4(0);
//else 

finalColor = mix(finalColor,texture(tex_cubemap, newDir),0.6475*(1-fresnelR)) ;
return finalColor;
}


vec4 calSphereColor(vec3 origin,vec3 direction,vec4 sphere){
#if(SimpleSphereRendering ==2)
return vec4(-1);
#endif
float firstSphereT = intersect_ray_sphere2(origin,direction,sphere);

if(firstSphereT<=0)return vec4(-1);
#if(SimpleSphereRendering ==1)
return sphereColor;
#endif
vec4 finalColor = sphereColor;
vec3 firstSphereNormal = -normalize((fs_in.ray_origin + direction*firstSphereT) - sphere.xyz);
vec3 reflDir=reflect(direction,firstSphereNormal);
float fresnelR = fresnelTermAprox(direction,-firstSphereNormal/15,air_n/fluid_n);
vec4 firstReflColor ;//=texture(tex_cubemap, reflDir);

vec4 waterReflColor = calcFluidColor((fs_in.ray_origin + direction*firstSphereT),reflDir,0,20);

 if(waterReflColor.w>=0){
//firstReflColor = vec4(0);
firstReflColor=waterReflColor;
}else {
firstReflColor =texture(tex_cubemap, reflDir);
}
finalColor = mix(finalColor,firstReflColor,fresnelR) ;
return finalColor;
}

void visualiseDensity(float tnear,float tfar){
vec3 rOrig = fs_in.ray_origin;
vec3 rDir = normalize(fs_in.ray_direction);
float finalColor = 0;
float numUpdates=0;

for(float i =tnear;i<tfar;i+=0.1)
{
	numUpdates+=1;
    vec3 hitPos = rOrig + rDir*(i);
    float pr = preasureInPoint3DTexture(hitPos);
	//if(pr>1){color=vec4(pr*0.5); return;}
	finalColor= (finalColor+pr);
}

color=vec4(vec3(finalColor/numUpdates),1);
}


void main(void)
{


vec4 finalColor = vec4(0);
vec3 rayDir = normalize(fs_in.ray_direction);
/*   Rendering of the CUBE
	vec3 hitPos; vec3 hitNorm;
	float closestT = rendHitTheCube( hitPos,hitNorm);
	if(closestT<length(fs_in.ray_origin)+30)
	finalColor = vec4(1)*(dot(-rayDir,hitNorm));	
	//*/
	
	 float tnear,tfar;
	bool exit =  intersectionsOfTheCube(tnear,tfar,fs_in.ray_origin,rayDir);
	if(tnear==tfar)tnear =0;
//*

//visualiseDensity(tnear,tfar);return;




/* 
for(float i=tnear;i<tfar;i+=0.2){
 if(length((fs_in.ray_origin+rayDir*i)-sphere1.xyz)<sphere1.w){
  color = vec4(0);
 return;
 }
}
//*/

//{color= vec4(fs_in.ray_origin*vec3(0,0,1),1);return;}


float firstSphereT = intersect_ray_sphere2(fs_in.ray_origin,rayDir,sphere1);
//if(firstSphereT>0){color= vec4(0);return;}

vec3 firstOrig ;
vec3 firstWaterNorm;
float firstWaterT = findIsosurface( tnear, tfar,true,fs_in.ray_origin,rayDir,firstOrig,firstWaterNorm);

if(firstSphereT==0&&firstWaterT==0)discard;

if(firstSphereT>0&&(firstSphereT<firstWaterT ||firstWaterT==0)){
finalColor = sphereColor;
vec3 firstSphereNormal = -normalize((fs_in.ray_origin + rayDir*firstSphereT) - sphere1.xyz);
vec3 reflDir=reflect(rayDir,firstSphereNormal);
float fresnelR = fresnelTermAprox(rayDir,-firstSphereNormal/15,air_n/fluid_n);
vec4 firstReflColor ;//=texture(tex_cubemap, reflDir);

vec4 waterReflColor = calcFluidColor((fs_in.ray_origin + rayDir*firstSphereT),reflDir,0,20);

 if(waterReflColor.w>=0){
//firstReflColor = vec4(0);
firstReflColor=waterReflColor;
}else {
firstReflColor =texture(tex_cubemap, reflDir);
}
finalColor = mix(finalColor,firstReflColor,fresnelR) ;
}else
{
  
float curt = firstWaterT;    
//if(curt>0)
finalColor = calcLight(firstWaterNorm ,curt );
//if(curt>0)color = texture(tex_cubemap, cross(firstWaterNorm,-rayDir));else discard;return;
//color=finalColor;return;
vec3 reflDir=reflect(rayDir,firstWaterNorm);
float fresnelR = fresnelTermAprox(rayDir,firstWaterNorm,air_n/fluid_n);


vec4 reflSphereColor = calSphereColor(fs_in.ray_origin+rayDir*curt,reflDir,sphere1);

vec4 firstReflColor;
if(reflSphereColor.w>=0) firstReflColor = reflSphereColor;
else firstReflColor = texture(tex_cubemap, reflDir);
finalColor = mix(finalColor,firstReflColor,fresnelR) ;

vec3 newDir=refract(rayDir,firstWaterNorm,air_n/fluid_n);
vec3 secondOrig;
vec3 secondNorm;
curt =  findIsosurface( 0, 20,false,firstOrig,newDir,secondOrig,secondNorm);

float refrSphereT = intersect_ray_sphere2(secondOrig,-secondNorm,sphere1);

vec4 firstRefrColor = tmpColor;
//if(refrSphereT>0) firstRefrColor =mix(sphereColor,firstRefrColor,0.4);

finalColor =mix(finalColor,firstRefrColor,0.34*(1-fresnelR));

vec3 thirdDir=refract(newDir,-secondNorm,fluid_n/air_n);

// float fresnelR2 = 0;//fresnelTerm(newDir,secondNorm,fluid_n/air_n);
if(thirdDir!=vec3(0)){
vec4 sphereRflColor =calSphereColor(secondOrig,thirdDir,sphere1);
vec4 refrOutRayColor;

if(sphereRflColor.w>=0){
refrOutRayColor = sphereRflColor;
}
else refrOutRayColor = texture(tex_cubemap, thirdDir);

//finalColor += vec4(1)*pow(max(dot(thirdDir,normalize(light_pos-secondOrig)),0),8);
finalColor=mix(finalColor,refrOutRayColor,0.85*(1-fresnelR)) ;

}//else finalColor = vec4(0);
//else 

finalColor = mix(finalColor,texture(tex_cubemap, newDir),0.6475*(1-fresnelR)) ;

}



  
/*
float samples=0;float i;
  for( i=tfar;i>tnear;i-=0.2){
  float pr = preasureInPoint(fs_in.ray_origin + rayDir*(i));
  if(pr>0.1)
  {
//  finalColor = vec4(pr/isosurfDens);
 // finalColor = vec4(1)*dot(tmpNorm,-rayDir);
 samples+=1;
  finalColor =(finalColor + pr*i)/1;
 //break;
  }

 } 

 finalColor/=samples*tfar*2;
   if(finalColor==vec4(0)) discard;
//*/
	

color=finalColor;

}
