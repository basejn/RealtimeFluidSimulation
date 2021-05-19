#version 420 core
#define PI 3.1415926535897932384626433832795

layout (location = 0) in vec3 position;
layout (location = 1) in vec4 velocity_mass;
layout (location = 2) in vec2 density_pressure;
// Входни масиви. Това са данните за всяка частица за която се изпълнява програмата паралелно на GPU.

layout (binding = 0) uniform samplerBuffer tex_position;
layout (binding = 1) uniform samplerBuffer tex_velocity;
layout (binding = 2) uniform isamplerBuffer tex_gridlist;
layout (binding = 3) uniform samplerBuffer tex_density;
// Входни масиви с данни за всички частици. За OpenGL това са текстури , но данните в тях могат да бъдат всякакви


out vec3 tf_position;
out vec4 tf_velocity_mass;
out vec2 tf_density_pressure;
out uint tf_color;
// Изходните данни за частицата за която се изпълнява кода .

uniform vec4 sphere1 = vec4(0,0,0,2);
uniform vec3 sphere1_speed = vec3(0,0,0);
// Входни параметри на сферата.Общи за всички частици.

// Параметри
const float gravity = 0.001;
const float wallDamping = 0.1;
const float wall_mass=100;	
const vec2 wall_density_preassure = vec2(1,40);
const float wall_hard_bounce=1.03;
uniform float rest_density=0.8;
uniform float eta=1.50;//viscosity
uniform float dens_K=20.5;
uniform float sigma=1.0;
uniform float surfTensTresh=0.02;
uniform float deltaT=0.050;
const float h=1.5;
const float GRID_VOLUME_SIDE=10;
const int gridSide=15;
const float cellSize=2*GRID_VOLUME_SIDE/gridSide;
const float offset=10;
const int gridSize=gridSide*gridSide*gridSide;

uniform float wind_speed=1.0;
uniform vec3 wind_dir=vec3(0.0,0.0,1.0);
//Гравитацията
uniform vec3 glass_pos=vec3(0.0,0.0,0.0);
//позицията на съда
const float radius = 1.2;
const float minDst = 1.0;

vec3 pressureF=vec3(0);
vec3 viscosityF=vec3(0);
vec3 color_field_gradient=vec3(0);
float color_field_laplacian=0;
vec2 new_density_pressure= vec2(0.1,0);
//Векторите в които се акумулира крайния резултат

//Гаусова функция
float gausKernel(float r){	
	const float h2=h*h;
	const float h3=h*h*h;
	return (exp(r*r/h2)/(pow(PI,3.0/2.0)*h3));	
}
//Варианти на гусовата функция подходящи при интегрирането на определени величини.
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
}
float laplacian_W_poly6(float r){	
	const float h2=h*h;
	const float h3=h*h*h;
	const float h9=h3*h3*h3;
	float r2=r*r;
	float h2rr=(h2-r2);
	return (945/(8*PI*h9)*h2rr*(r2-3.0/4.0*h2rr));	
}
//Хеш функцията определяща индекса на клетката според точка в пространството.
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
//Интегриране на силите с които действа всяка близка частица 
void forEveryParticle(int i){
	vec3 npos = texelFetch(tex_position,i).xyz;			
	vec3 dstV =npos-position;			
	float dst = length(dstV);
	if(dst<=h){
		dstV/=dst;
		if((dst)<0.001){return;	dstV=vec3(1,1,1);dst=1;}
		vec2 n_density_pressure = texelFetch(tex_density,i).xy;
		vec4 n_velocity_mass = texelFetch(tex_velocity,i);
		new_density_pressure.x+=n_velocity_mass.w*W_poly6(dst);
		pressureF +=n_velocity_mass.w*(density_pressure.y+n_density_pressure.y)/(2*n_density_pressure.x)*gradient_W_spiky(dst)*dstV;
		viscosityF+=eta*n_velocity_mass.w*((n_velocity_mass.xyz-velocity_mass.xyz))/n_density_pressure.x*laplacian_W_viscosity(dst);
		color_field_gradient+=n_velocity_mass.w/n_density_pressure.x*gradient_W_poly6(dst)*dstV;			
		color_field_laplacian+=n_velocity_mass.w/n_density_pressure.x*laplacian_W_poly6(dst);
	}			
}
//Интегриране на силите които действат всички стени
void forEveryWall(vec4 plane){
	vec3 dstV =-plane.xyz;						
	float dst = (dot(position.xyz,plane.xyz)+plane.w);	
	float hWall = h;
	if(abs(dst)<=hWall){			
	
		vec2 n_density_pressure =wall_density_preassure;
		vec4 n_velocity_mass = vec4(0,0,0,wall_mass);
		new_density_pressure.x+=n_velocity_mass.w*W_poly6(dst);
		pressureF +=n_velocity_mass.w*(density_pressure.y+n_density_pressure.y)/(2*n_density_pressure.x)*gradient_W_spiky(dst)*dstV;			
		viscosityF+=0.5*eta*n_velocity_mass.w*((n_velocity_mass.xyz-velocity_mass.xyz))/n_density_pressure.x*laplacian_W_viscosity(dst);
	}
}
//Интегриране на силите с които действа сферата.
void forEverySphere(vec4 sphere){
	vec3 dstV =  sphere.xyz - position.xyz;						
	float dst = abs(length(dstV)-sphere.w);			
	if((dst)<=h){	
		dstV/=dst;	
		vec2 n_density_pressure =wall_density_preassure;
		vec4 n_velocity_mass = vec4(sphere1_speed,wall_mass);
		new_density_pressure.x+=n_velocity_mass.w*W_poly6(dst);
		pressureF +=n_velocity_mass.w*(density_pressure.y+n_density_pressure.y)/(2*n_density_pressure.x)*gradient_W_spiky(dst)*dstV;			
	}
}	
//Обхождане на всяка от частиците с клетката.
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
	// Индекса на клетката в която се намира текущата частица, за която се изпълнява програмата.
    int centerCellIndex = CellIndex(position); 
	// Интегриране на силите с които действат всички частици с кетката на текущата
	doForCellIndexArrays(centerCellIndex);	
	// Маси в с отместванията на всички съседни клетки
	const vec3[26] offsets=vec3[26](vec3(cellSize,cellSize,0),vec3(cellSize,cellSize,cellSize),vec3(cellSize,cellSize,-cellSize),
									vec3(cellSize,0,0),vec3(cellSize,0,cellSize),vec3(cellSize,0,-cellSize),
									vec3(cellSize,-cellSize,0),vec3(cellSize,-cellSize,cellSize),vec3(cellSize,-cellSize,-cellSize),
									vec3(0,cellSize,0),vec3(0,cellSize,cellSize),vec3(0,cellSize,-cellSize),
									vec3(0,0,cellSize),vec3(0,0,-cellSize),
									vec3(0,-cellSize,0),vec3(0,-cellSize,cellSize),vec3(0,-cellSize,-cellSize),
									vec3(-cellSize,cellSize,0),vec3(-cellSize,cellSize,cellSize),vec3(-cellSize,cellSize,-cellSize),
									vec3(-cellSize,0,0),vec3(-cellSize,0,cellSize),vec3(-cellSize,0,-cellSize),
									vec3(-cellSize,-cellSize,0),vec3(-cellSize,-cellSize,cellSize),vec3(-cellSize,-cellSize,-cellSize));
	// За всяка съседна клетка се обхождат частиците и се интегрират силите.
	for(int i=0;i<offsets.length();i++){
	int curCellIndex = CellIndexNeighbour(position+offsets[i]);
		if((curCellIndex>=0)&&(curCellIndex<gridSize))
		{
		doForCellIndexArrays(curCellIndex);
		}
	}
	
	forEveryWall(vec4(0,1,0,-glass_pos.y+GRID_VOLUME_SIDE*wall_hard_bounce));
	forEveryWall(vec4(0,-1,0,glass_pos.y+GRID_VOLUME_SIDE*wall_hard_bounce));
	forEveryWall(vec4(1,0,0, -glass_pos.x+GRID_VOLUME_SIDE*wall_hard_bounce));
	forEveryWall(vec4(-1,0,0,glass_pos.x+GRID_VOLUME_SIDE*wall_hard_bounce));
	forEveryWall(vec4(0,0,1, -glass_pos.z+GRID_VOLUME_SIDE*wall_hard_bounce));
	forEveryWall(vec4(0,0,-1,glass_pos.z+GRID_VOLUME_SIDE*wall_hard_bounce));  
	forEverySphere(sphere1);
	
	// Повърхностното напрежение
	vec3 surf_tensF=vec3(0);
	float gradientLength = length(color_field_gradient);	// normal vector na povyrninata
    if(gradientLength>=surfTensTresh)
		surf_tensF = -sigma*(color_field_gradient)*(gradientLength);		
	//Изход за цвят които ще се визуализара като цявт на частицата и отговаря на определна величина, с цел дебъгване.
	tf_color = int(clamp(length(color_field_gradient)*255.0,0,255));
 
	// Изчисляване на крайната сила и новите позиции.
	vec3 total_force = surf_tensF+pressureF+viscosityF;	
	vec3 acceleration = total_force/new_density_pressure.x + wind_dir*wind_speed;
	vec4 final_velocity_mass = vec4(velocity_mass.xyz + acceleration*deltaT*deltaT,velocity_mass.w);
	vec3 final_position = position + final_velocity_mass.xyz *deltaT;		
	
	if(length(final_position)>20){final_position=vec3(0,sphere1.w+sphere1.y+2,1);final_velocity_mass=vec4(0,0,0,final_velocity_mass.w);}
	//Връщане на резултат.
	tf_velocity_mass=final_velocity_mass;
	tf_position=final_position;
	new_density_pressure.y = dens_K*(density_pressure.x-rest_density);
	tf_density_pressure = new_density_pressure;   
}
