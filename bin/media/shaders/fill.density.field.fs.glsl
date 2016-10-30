#version 410 core
#define PI 3.1415926535897932384626433832795
layout (location = 0) out float density;

layout (binding = 0) uniform samplerBuffer tex_position;
layout (binding = 1) uniform samplerBuffer tex_velocity;
layout (binding = 2) uniform isamplerBuffer tex_gridlist;


#define OPTIM_STRUCT  1//0=no 1=array 2=lists
const float GRID_VOLUME_SIDE=10;
const int gridSide=15;
const float cellSize=2*GRID_VOLUME_SIDE/gridSide;//GRID_VOLUME_SIDE/gridSide
const int gridSize=gridSide*gridSide*gridSide;
const float offset=10;
const float h=1.5;
const vec3 glass_pos = vec3(0);

in GS_OUT
{
vec3 coord ;
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
   int CellIndex(vec3 point){
		int cellIndex=0;
		point=(point - glass_pos + offset)/cellSize;
		cellIndex = max(min(int((point.z))*gridSide*gridSide,gridSide*gridSide*(gridSide-1)),0);
		cellIndex +=max(min(int((point.y))*gridSide,gridSide*(gridSide-1)),0);
		cellIndex +=max(min(int((point.x)),(gridSide-1)),0);
		return cellIndex;	
	}
	int CellIndexNeighbour(vec3 point){
		int cellIndex=0;
		point=(point - glass_pos + offset)/cellSize;	
		cellIndex = (int(floor(point.z))*gridSide*gridSide);
		cellIndex +=(int(floor(point.y))*gridSide);
		cellIndex +=(int(floor(point.x)));			
		return cellIndex;	
	}
//	vec3 tmpNorm=vec3(1,0,0);
//	vec4 tmpColor=vec4(0,0,0,1);
	float tmpDensity=0;
	vec3 curPos=vec3(0);
	void forEveryParticle(int i){
	vec3 npos = texelFetch(tex_position,i).xyz;			
			vec3 dstV =npos-curPos;			
			float dst = length(dstV);
			if(dst<=h){				
			float masss =1;// texelFetch(tex_velocity,i).w;
			tmpDensity+=masss*W_poly6(dst);			
			//tmpNorm-=dstV*n_velocity_mass.w*W_poly6(dst);			
			//tmpColor=(tmpColor+ W_poly6(dst)*texelFetch(tex_color,i).xyzw);
			}			
	}
	
	
	
	void doForCellIndexArrays(int myCell){	
	int curInd = myCell*2;
	int count = texelFetch(tex_gridlist,curInd+1).r;	
	curInd = texelFetch(tex_gridlist,curInd).r;// mestim kym pyrwi element
		while(count-- >0){	
		int curParticleInd = texelFetch(tex_gridlist,curInd).r;	
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
		forEveryParticle(curParticleInd);
		curInd = texelFetch(tex_gridlist,curInd+1).r;// mestim kym sledwashtiq element
		}
	}	

		
	float preasureInPoint(vec3 point){	
	curPos = point;
	tmpDensity = 0;//tmpNorm=vec3(0);tmpColor=vec4(0,0,0,1);
 #if (OPTIM_STRUCT>0)
	int centerCellIndex = CellIndexNeighbour(curPos);  
	if(!((centerCellIndex>=0)&&(centerCellIndex<gridSize)))return 0;
	 #if (OPTIM_STRUCT==1)
	doForCellIndexArrays(centerCellIndex);		
	#elif  (OPTIM_STRUCT==2) 
	doForCellIndexLists(centerCellIndex);	
	#endif 			
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
		#if (OPTIM_STRUCT==1)
		doForCellIndexArrays(curCellIndex);		
		#elif  (OPTIM_STRUCT==2) 
		doForCellIndexLists(curCellIndex);	
		#endif	
		}
	}	
	
	#else 
	int imax = textureSize(tex_position);
	for(int i=0;i<imax;i++){
		//if(i!=gl_VertexID)
		{
		forEveryParticle(i);
		}
	}	
	#endif 
	
	//tmpNorm=normalize(tmpNorm);
	//tmpColor=normalize(tmpColor);
	//tmpColor/=tmpColor.w;
	//tmpColor = vec4(0);
	//tmpColor/=
	return tmpDensity;
	}
	
void main(void)
{
density=preasureInPoint(fs_in.coord);
}
