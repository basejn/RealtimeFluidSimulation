/*
 * Copyright © 2012-2015 Graham Sellers
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <sb7.h>
#include <sb7ktx.h>
#include <vmath.h>
//#include <cmath>
#include <math.h>  
#include <chrono>
#include <shader.h>
#include <sb7textoverlay.h>
#include<forward_list>
#include<minwindef.h>
#include<algorithm>
#include<vector>
#include<atomic>
#include <thread>
#include<string>
#include<fstream>
 #define DBOUT( s )            \
{                             \
   std::wostringstream os_;    \
   os_ << s;                   \
   OutputDebugStringW( os_.str().c_str() );  \
}
#ifdef DBOUT 
#include <Windows.h>
#include <iostream>
#include <sstream>
#endif

enum BUFFER_TYPE_t
{
    POSITION_A,
    POSITION_B,
    VELOCITY_A,
    VELOCITY_B,
	GRIDLIST_A,
    GRIDLIST_B,
	DENSITY_A,
    DENSITY_B,
	COLORS,
    CONNECTION
};



enum
{
	
    POINTS_X            = 15,
    POINTS_Y            = 15,
	POINTS_Z            = 15,
    POINTS_TOTAL        = (POINTS_X * POINTS_Y * POINTS_Z),
    CONNECTIONS_TOTAL   = (POINTS_X - 1) * POINTS_Y + (POINTS_Y - 1) * POINTS_X
};

#define OPTIM_STRUCT  4 //0=no 1=array 2=lists 3=listsParral 4=listsParralThreadPool

const int    GRID_SIDE =12;
const float  GRID_VOLUME_SIDE = 20.0f;  //+-10
const float  GRID_OFFSET = 10.0f ; //+-10
const float  CELL_SIZE = GRID_VOLUME_SIDE/GRID_SIDE;
#if (OPTIM_STRUCT ==1)
const int GRIDLIST_SIZE = (GRID_SIDE*GRID_SIDE*GRID_SIDE*2+POINTS_TOTAL) * sizeof(int);
#else
const int GRIDLIST_SIZE = (GRID_SIDE*GRID_SIDE*GRID_SIDE*2+POINTS_TOTAL*2) * sizeof(int);
#endif
const int POSITIONS_SIZE = POINTS_TOTAL * sizeof(vmath::vec3);

class  GridOptimiser{
public :virtual void fillList(vmath::vec3 * pointsBuffer,int * gridBuffer,vmath::vec3 offset_vec)=0;

};
class GridListsOfListsOptimiser:public GridOptimiser{
	int pointsCount;int gridSide ; float cellSize;float offset;	int gridSize;
public:	
	GridListsOfListsOptimiser(int pointsCount,int gridSide , float cellSize,float offset){
		this->pointsCount=pointsCount;
		this->gridSide=gridSide;
		this->cellSize=cellSize;
		this->offset=offset;
		gridSize = gridSide*gridSide*gridSide;
		lastUsedInd=new std::atomic<int>(0);
	}
 void fillList(vmath::vec3 * pointsBuffer,int * gridBuffer,vmath::vec3 offset_vec){	
		this->gridBuffer=gridBuffer;
		this->offset_vec=offset_vec;
		std::vector<int> cellsSizes =std::vector<int>(gridSize,0);
		std::vector<std::forward_list<int>> cells = std::vector<std::forward_list<int>>(gridSize);		
		for(int i=0;i<pointsCount;i++){
			int cellIndex = CellIndex(&(pointsBuffer[i]));					
			cells[cellIndex].push_front(i);
			cellsSizes[cellIndex]++;
		}
		 *lastUsedInd=((gridSize)-1);
		 std::vector<std::thread> v;
		for(int i=0;i<gridSize;i++){
			//v.emplace_back(std::thread(&GridListsOfListsOptimiser::GridListKernel,this,i,&cells[i],cellsSizes[i]));
			GridListKernel(i,&cells[i],cellsSizes[i]);
		}
	//	for (auto& t : v)t.join();
	}
private:
	vmath::vec3 offset_vec;
	int * gridBuffer;
	std::atomic<int>* lastUsedInd;
	void GridListKernel(int index ,std::forward_list<int>* cell,int cellSize){
		if((*cell).empty()){
			gridBuffer[index*2]=0;
			gridBuffer[index*2+1]=0;//cellSize;	
		}else{
			int lastLocalUsedInd = gridBuffer[index*2]=(++(*lastUsedInd))*2;
			gridBuffer[index*2+1]=cellSize;				
			for(std::forward_list<int>::iterator it = ++(*cell).begin();it!=(*cell).end();it++){
				gridBuffer[lastLocalUsedInd]=*it;//value
				gridBuffer[lastLocalUsedInd+1]=(++(*lastUsedInd))*2;//new alocated index pointing to the next element
				lastLocalUsedInd = gridBuffer[lastLocalUsedInd+1];//pointer to the next
			}
			gridBuffer[lastLocalUsedInd]=*((*cell).begin());//value
			gridBuffer[lastLocalUsedInd+1]=0;//terminator
		}
	}
	inline int CellIndex(vmath::vec3 * point){
	int cellIndex=0;
	cellIndex =std::max( std::min(((int)(((*point)[2]-offset_vec[2]+offset)/cellSize))*gridSide*gridSide,gridSide*gridSide*(gridSide-1)),0);
	cellIndex +=std::max( std::min(((int)(((*point)[1]-offset_vec[1]+offset)/cellSize))*gridSide,gridSide*(gridSide-1)),0);
	cellIndex +=std::max( std::min(((int)(((*point)[0]-offset_vec[0]+offset)/cellSize)),(gridSide-1)),0);
	return cellIndex;	
	}
};
class GridListsOfListsParralOptimiser:public GridOptimiser{
	
public:	
	GridListsOfListsParralOptimiser(int pointsCount,int gridSide , float cellSize,float offset,int kernels){
		this->pointsCount=pointsCount;
		this->gridSide=gridSide;
		this->cellSize=cellSize;
		this->offset=offset;
		this->kernels=kernels;
		gridSize = gridSide*gridSide*gridSide;
		lastUsedInd=new std::atomic<int>(0);
		list_Locs=new std::atomic_flag[gridSize];
		for(int i =0;i<gridSize;i++)list_Locs[i].clear();
	}
	void fillList(vmath::vec3 * pointsBuffer,int * gridBuffer,vmath::vec3 offset_vec){
		this->pointsBuffer=pointsBuffer;
		this->gridBuffer=gridBuffer;
		this->offset_vec=offset_vec;
		std::vector<int> cellsSizes =std::vector<int>(gridSize,0);
		
		*lastUsedInd=(gridSize)-1;
		
		for(int i=0;i<gridSize*2;i++)gridBuffer[i]=0;//init buffer
		//for(int i =0;i<gridSize;i++)list_Locs[i].clear();
		int pointsPerKernel = pointsCount / (kernels);
		int pointsPerKernelRem = pointsCount % kernels;

		//DBOUT("\n");
		//v.clear();
		for(int i=0;i<kernels;i++){
			int nPoints = pointsPerKernel;
			if((i)==(kernels-1))nPoints+=pointsPerKernelRem;
		//	v.emplace_back(std::thread(&GridListsOfListsParralOptimiser::GridListKernelParall,this,i,pointsPerKernel*i,nPoints));
			GridListKernelParall(i,pointsPerKernel*i, nPoints);
		}
		//for (auto& t : v)t.join();
		
	}

	void printList(vmath::vec3 * pointsBuffer,int * gridBuffer){
		std::ofstream myfile;
  myfile.open ("Grid_Buffer_SnapShot.txt"); 
		myfile<<"\nGrid Buffer:";
		for(int i=0;i<GRIDLIST_SIZE/4;i++){		
		myfile<<"\n"<<i<<":\t"<<gridBuffer[i];		
		} 
		myfile.close();
	}
protected:
	int pointsCount;int gridSide ; float cellSize;float offset;	int gridSize;int kernels;
	std::vector<std::thread> v;
	int * gridBuffer;
	vmath::vec3 * pointsBuffer;
	vmath::vec3 offset_vec;
	std::atomic<int>* lastUsedInd;
	std::atomic_flag* list_Locs;
	void GridListKernelParall(int globalIndex,int firstPoint,int nPoints){			
		for(int i =firstPoint;i<firstPoint+nPoints;i++){			
			int listInd = CellIndex(pointsBuffer+i)*2;	
			//std::stringstream sst;	sst<<" "<<listInd;DBOUT(sst.str().data());			
			int newMemory = ++(*lastUsedInd)*2;//alocating memory
			//while(list_Locs[listInd].test_and_set(std::memory_order_acquire));// aquire lock
				if(gridBuffer[listInd]==0){// pyrwiqt elemnt v spisyka
					gridBuffer[listInd] = newMemory;
					gridBuffer[listInd+1]++;//incrementing list size
					gridBuffer[newMemory]=i;// asigning new item value
					gridBuffer[newMemory+1]=0;
				}else{//poredniqt element
					int oldFirst = gridBuffer[listInd];
					gridBuffer[listInd] = newMemory;
					gridBuffer[listInd+1]++;//incrementing list size
					gridBuffer[newMemory]=i;// asigning new item value
					gridBuffer[newMemory+1]=oldFirst;
				}
			//list_Locs[listInd].clear(std::memory_order_release);//release lock
		}		
	}
	
	inline int CellIndex(vmath::vec3 * point){
	int cellIndex=0;
	cellIndex =std::max( std::min(((int)(((*point)[2]-offset_vec[2]+offset)/cellSize))*gridSide*gridSide,gridSide*gridSide*(gridSide-1)),0);
	cellIndex +=std::max( std::min(((int)(((*point)[1]-offset_vec[1]+offset)/cellSize))*gridSide,gridSide*(gridSide-1)),0);
	cellIndex +=std::max( std::min(((int)(((*point)[0]-offset_vec[0]+offset)/cellSize)),(gridSide-1)),0);
	return cellIndex;	
	}
};
class GridListsOfArraysOptimiser:public GridOptimiser{
	int pointsCount;int gridSide ; float cellSize;float offset;	int gridSize;int kernels;
public:	
	GridListsOfArraysOptimiser(int pointsCount,int gridSide , float cellSize,float offset,int kernels){
		this->pointsCount=pointsCount;
		this->gridSide=gridSide;
		this->cellSize=cellSize;
		this->offset=offset;
		this->kernels=kernels;
		gridSize = gridSide*gridSide*gridSide;		
	}
	void fillList(vmath::vec3 * pointsBuffer,int * gridBuffer,vmath::vec3 offset_vec){
		this->pointsBuffer=pointsBuffer;
		this->gridBuffer=gridBuffer;
		this->offset_vec=offset_vec;
		int gridSize = gridSide*gridSide*gridSide;
		std::vector<int> cellsSizes =std::vector<int>(gridSize,0);
		std::vector<std::forward_list<int>> cells = std::vector<std::forward_list<int>>(gridSize);
		

		for(int i=0;i<pointsCount;i++){
			int cellIndex = CellIndex(pointsBuffer+i);					
			cells[cellIndex].push_front(i);
			cellsSizes[cellIndex]++;
		}
		int lastUsedInd=(gridSize)*2-1;
		for(int i=0;i<gridSize;i++){
			gridBuffer[i*2]=lastUsedInd+1;
			gridBuffer[i*2+1]=cellsSizes[i];			
			for(std::forward_list<int>::iterator it = cells[i].begin();it!=cells[i].end();it++){
				gridBuffer[++lastUsedInd]=*it;
			}
		}				
	}

	void setPointers(vmath::vec3 * pointsBuffer,int * gridBuffer){
		this->pointsBuffer=pointsBuffer;
		this->gridBuffer=gridBuffer;		
	}
	void printList(vmath::vec3 * pointsBuffer,int * gridBuffer){
		std::ofstream myfile;
  myfile.open ("Grid_Buffer_SnapShot.txt"); 
		myfile<<"\nGrid Buffer:";
		for(int i=0;i<GRIDLIST_SIZE/4;i++){		
		myfile<<"\n"<<i<<":\t"<<gridBuffer[i];		
		} 
		myfile.close();
	}
private:
	vmath::vec3 offset_vec;
	int * gridBuffer;
	vmath::vec3 * pointsBuffer;
	
	inline int CellIndex(vmath::vec3 * point){
	int cellIndex=0;
	cellIndex =std::max( std::min(((int)(((*point)[2]-offset_vec[2]+offset)/cellSize))*gridSide*gridSide,gridSide*gridSide*(gridSide-1)),0);
	cellIndex +=std::max( std::min(((int)(((*point)[1]-offset_vec[1]+offset)/cellSize))*gridSide,gridSide*(gridSide-1)),0);
	cellIndex +=std::max( std::min(((int)(((*point)[0]-offset_vec[0]+offset)/cellSize)),(gridSide-1)),0);
	return cellIndex;	
	}
};
class OptimiserThreadPoolListsOfLists:public GridListsOfListsParralOptimiser{
public:
	OptimiserThreadPoolListsOfLists(int pointsCount,int gridSide , float cellSize,float offset,int nThreads):GridListsOfListsParralOptimiser(pointsCount,gridSide,cellSize,offset,nThreads){
		this->nThreads=nThreads;
		worker_Locs = new bool[nThreads];
		resetWorkers();
		parameters = new params[nThreads];
		for(int i =0;i<nThreads;i++){			
			threads.emplace_back(std::thread(&OptimiserThreadPoolListsOfLists::worker,this,i,&worker_Locs[i]));
		}
	}

	 void fillList(vmath::vec3 * pointsBuffer,int * gridBuffer,vmath::vec3 offset_vec){
		this->pointsBuffer=pointsBuffer;
		this->gridBuffer=gridBuffer;
		this->offset_vec=offset_vec;
		std::vector<int> cellsSizes =std::vector<int>(gridSize,0);
		
		*lastUsedInd=(gridSize)-1;
		
		for(int i=0;i<gridSize*2;i++)gridBuffer[i]=0;//init buffer

		int pointsPerKernel = pointsCount / (kernels);
		int pointsPerKernelRem = pointsCount % kernels;

		//DBOUT("\n");
		//for(int i =0;i<gridSize;i++)list_Locs[i].clear();
		resetWorkers();
		for(int i=0;i<kernels;i++){
			int nPoints = pointsPerKernel;
			if((i)==(kernels-1))nPoints+=pointsPerKernelRem;
		//	v.emplace_back(std::thread(&GridListsOfListsParralOptimiser::GridListKernelParall,this,i,pointsPerKernel*i,nPoints));
		//	GridListKernelParall(i,pointsPerKernel*i, nPoints);
			parameters[i].firstPoint =pointsPerKernel*i;
			parameters[i].nPoints =nPoints;
			worker_Locs[i]=(true);//puskame workera s teq parametri
		}		
	}

	void resetWorkers(){		
		for(int i=0;i<nThreads;i++){
			worker_Locs[i]=(false);
		}	
		//for(int i =0;i<gridSize;i++)list_Locs[i].clear();
	}
	bool is_ready(){
		bool ready = true;
		for(int i=0;i<nThreads;i++){
			if(worker_Locs[i]==true)ready=false;
		}
		return ready;
	}
	void join_Workers(){		
		for(int i=0;i<nThreads;i++){
			while(worker_Locs[i]==true);
		}		
	}
private:

	
	void worker(int id,bool *my_Lock){
		while(true){
			while((*my_Lock)==false);
			GridListKernelParall(id,parameters[id].firstPoint,parameters[id].nPoints);
			(*my_Lock)=(false);
		}
	}

	bool* worker_Locs;
	int nThreads;
	
	std::vector<std::thread> threads;
	struct params{
		int firstPoint;
		int nPoints;
	} *parameters;
};

class springmass_app : public sb7::application
{	
public:
	//GridListsOfListsOptimiser* gridOptimiser;
	GridOptimiser*gridOptimiser;
	 sb7::text_overlay   overlay;
    springmass_app()
        : m_iteration_index(0),
          m_update_program(0),
          m_render_program(0),
		  m_render_fluid_raytrace_program(0),
		  render_fluid_ray_trace_vao(0),
		  render_balls_vao(0),
		  m_render_wind_program(0),	
		  rayTraceCube_buffer(0),
		  balls_buffer(0),
          draw_points(true),   
		  draw_raytrace(false),
          iterations_per_frame(1)
    {
    }

    void init()
    {
        static const char title[] = "Asen OpenGl Partical Simulation";

        sb7::application::init();
	
        memcpy(info.title, title, sizeof(title));
    }
	void init_user_transforms(){
		wind_speed=1.80f;
		wind_dir = vmath::vec3(0.0f,-1.0f,0.0f);
		glass_pos = vmath::vec3(0.0f,0.0f,0.0f);
		light_pos=vmath::vec3(10.0f,10.0f,40.0f);
		m_matrix = vmath::mat4::identity();		
		v_matrix = vmath::lookat(vmath::vec3(0.0,0.0,50),vmath::vec3(0.0,0.0,0.0),vmath::vec3(0.0,1.0,0.0));
	}
    void startup(void)
    {

		glGenQueries(2, timerQueries);
#if (OPTIM_STRUCT ==1)	
		gridOptimiser =new GridListsOfArraysOptimiser(POINTS_TOTAL,GRID_SIDE,CELL_SIZE,GRID_OFFSET,1);		
#elif (OPTIM_STRUCT ==2)
		gridOptimiser =new GridListsOfListsOptimiser(POINTS_TOTAL,GRID_SIDE,CELL_SIZE,GRID_OFFSET);        
#elif (OPTIM_STRUCT ==3)
		gridOptimiser =new GridListsOfListsParralOptimiser(POINTS_TOTAL,GRID_SIDE,CELL_SIZE,GRID_OFFSET,1);	
#elif (OPTIM_STRUCT ==4)
		gridOptimiser =new OptimiserThreadPoolListsOfLists(POINTS_TOTAL,GRID_SIDE,CELL_SIZE,GRID_OFFSET,4);	
#endif
		
		int i, j,k;
		lastTime=0;frames=0;
		
        load_shaders();
		keyBuffer.q=0;
		draging=0;rotating=0;looking=0;
		init_user_transforms();
		proj_matrix = vmath::perspective(50.0f,(float)info.windowWidth / (float)info.windowHeight,0.1f,1000.0f);

		atractC =  vmath::vec3(105,0.1,0.001);
		repulsC =  vmath::vec3(99,0.1,0.001);

        vmath::vec3 * initial_positions = new vmath::vec3 [POINTS_TOTAL];
        vmath::vec4 * initial_velocities = new vmath::vec4 [POINTS_TOTAL];
		vmath::Tvec4<char> * initial_colors = new vmath::Tvec4<char> [POINTS_TOTAL];
		
		vmath::vec2 * initial_density_pressure = new vmath::vec2 [POINTS_TOTAL];
		vmath::vec2 * tex_coords = new vmath::vec2 [POINTS_TOTAL];
        int n = 0;
		vmath::vec4 vel = vmath::vec4(0.0,1.0,0.0,1.0);
		for (k = 0; k < POINTS_Z; k++) {
			float fk = (float)k / (float)POINTS_Z;
			vel = vel*vmath::rotate(360/(float)POINTS_Z,0.0f,0.0f,1.0f);
			for (j = 0; j < POINTS_Y; j++) {
				float fj = (float)j / (float)POINTS_Y;
				for (i = 0; i < POINTS_X; i++) {
					float fi = (float)i / (float)POINTS_X;



					initial_positions[n] = vmath::vec3(
						(fi - 0.5f) * (float)POINTS_X*0.9,
						(fj - 0.5f) * (float)POINTS_Y*0.8,
						(fk - 0.5f) * (float)POINTS_Z*0.9);

					//initial_velocities[n] = vmath::vec4(vel[0],vel[1],vel[2],1/25.0)*25;
					const float mas=1;
					initial_velocities[n] = i&2? vmath::vec4(0,0,0,mas):vmath::vec4(0,0,0,mas);
					initial_colors[n] =i&2? vmath::Tvec4<char>(255,180,0,255):vmath::Tvec4<char>(250,180,0,255);
					initial_density_pressure[n] =  vmath::vec2(1,1);
					n++;
				}
			}
		}

		
		glGenVertexArrays(1, &wind_dir_vao);		
		glBindVertexArray(wind_dir_vao);
		
		glGenBuffers(1,&wind_pointer_buff);
		glBindBuffer(GL_ARRAY_BUFFER,wind_pointer_buff);
		glBufferData(GL_ARRAY_BUFFER,4*3*2,NULL,GL_STATIC_DRAW);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
        glEnableVertexAttribArray(0);
		
		glBindVertexArray(0);

		

        glGenVertexArrays(2, m_vao);
        glGenBuffers(10, m_vbo);

			glBindBuffer(GL_ARRAY_BUFFER, m_vbo[COLORS]);
            glBufferData(GL_ARRAY_BUFFER, POINTS_TOTAL * sizeof(vmath::Tvec4<char>), initial_colors, GL_STATIC_DRAW);
            

        for (i = 0; i < 2; i++) {
            glBindVertexArray(m_vao[i]);

            glBindBuffer(GL_ARRAY_BUFFER, m_vbo[POSITION_A + i]);
		
        //    glBufferData(GL_ARRAY_BUFFER, POSITIONS_SIZE, initial_positions, GL_DYNAMIC_COPY);
			glBufferStorage(GL_ARRAY_BUFFER, POSITIONS_SIZE, initial_positions, GL_DYNAMIC_STORAGE_BIT|GL_MAP_READ_BIT|GL_MAP_PERSISTENT_BIT);
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, NULL);
            glEnableVertexAttribArray(0);

            glBindBuffer(GL_ARRAY_BUFFER, m_vbo[VELOCITY_A + i]);
            glBufferData(GL_ARRAY_BUFFER, POINTS_TOTAL * sizeof(vmath::vec4), initial_velocities, GL_DYNAMIC_COPY);
            glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 0, NULL);
            glEnableVertexAttribArray(1);			

			glBindBuffer(GL_ARRAY_BUFFER, m_vbo[DENSITY_A + i]);
            glBufferData(GL_ARRAY_BUFFER, POINTS_TOTAL * sizeof(vmath::vec2), initial_density_pressure, GL_DYNAMIC_COPY);
            glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 0, NULL);
            glEnableVertexAttribArray(2);
			
			//glBindBuffer(GL_ARRAY_BUFFER, m_vbo[COLORS]);
			//glVertexAttribPointer(3, 4, GL_BYTE, GL_FALSE, 0, NULL);
           // glEnableVertexAttribArray(3);
			
			glBindBuffer(GL_ARRAY_BUFFER, m_vbo[GRIDLIST_A + i]);
            //glBufferData(GL_ARRAY_BUFFER, GRIDLIST_SIZE, NULL, GL_DYNAMIC_COPY);
			glBufferStorage(GL_ARRAY_BUFFER, GRIDLIST_SIZE, NULL, GL_DYNAMIC_STORAGE_BIT|GL_MAP_WRITE_BIT|GL_MAP_PERSISTENT_BIT);

			#if(OPTIM_STRUCT >0&&OPTIM_STRUCT !=4)
			int * gridBuffertmp = (int *)glMapNamedBufferRange(m_vbo[GRIDLIST_A + i], 0, GRIDLIST_SIZE, GL_MAP_WRITE_BIT );
			vmath::vec3 * pointsBuffertmp = (vmath::vec3 *)glMapNamedBufferRange( m_vbo[POSITION_A +i], 0, POSITIONS_SIZE, GL_MAP_READ_BIT);
		
			gridOptimiser->fillList(pointsBuffertmp,gridBuffertmp,glass_pos);
			//gridOptimiser->printList(pointsBuffer,gridBuffer);std::exit(0); 
		
			glUnmapNamedBuffer(m_vbo[GRIDLIST_A + i]);
			glUnmapNamedBuffer(m_vbo[POSITION_A + i]);
#elif(OPTIM_STRUCT ==4)
			int * gridBuffertmp = (int *)glMapNamedBufferRange(m_vbo[GRIDLIST_A + i], 0, GRIDLIST_SIZE, GL_MAP_WRITE_BIT | GL_MAP_PERSISTENT_BIT|GL_MAP_FLUSH_EXPLICIT_BIT);
			vmath::vec3 * pointsBuffertmp = (vmath::vec3 *)glMapNamedBufferRange( m_vbo[POSITION_A +i], 0, POSITIONS_SIZE, GL_MAP_READ_BIT|GL_MAP_PERSISTENT_BIT);
		
			gridOptimiser->fillList(pointsBuffertmp,gridBuffertmp,glass_pos);
			//gridOptimiser->printList(pointsBuffer,gridBuffer);std::exit(0); 
		
			gridBuffer[i]=gridBuffertmp;
			pointsBuffer[i]=pointsBuffertmp;
			((OptimiserThreadPoolListsOfLists*)gridOptimiser)->join_Workers();
	
#endif
			
        }
		
        delete [] initial_velocities;
        delete [] initial_positions;
		delete [] tex_coords;
		delete [] initial_colors;
		delete [] initial_density_pressure;

        glGenTextures(2, m_pos_tbo);
        glBindTexture(GL_TEXTURE_BUFFER, m_pos_tbo[0]);
        glTexBuffer(GL_TEXTURE_BUFFER, GL_RGB32F, m_vbo[POSITION_A]);
        glBindTexture(GL_TEXTURE_BUFFER, m_pos_tbo[1]);
        glTexBuffer(GL_TEXTURE_BUFFER, GL_RGB32F, m_vbo[POSITION_B]);

		glGenTextures(2, m_vel_tbo);
        glBindTexture(GL_TEXTURE_BUFFER, m_vel_tbo[0]);
        glTexBuffer(GL_TEXTURE_BUFFER, GL_RGBA32F, m_vbo[VELOCITY_A]);
        glBindTexture(GL_TEXTURE_BUFFER, m_vel_tbo[1]);
        glTexBuffer(GL_TEXTURE_BUFFER, GL_RGBA32F, m_vbo[VELOCITY_B]);

		glGenTextures(2, m_GRIDLIST_tbo);
        glBindTexture(GL_TEXTURE_BUFFER, m_GRIDLIST_tbo[0]);
        glTexBuffer(GL_TEXTURE_BUFFER, GL_R32I, m_vbo[GRIDLIST_A]);
        glBindTexture(GL_TEXTURE_BUFFER, m_GRIDLIST_tbo[1]);
        glTexBuffer(GL_TEXTURE_BUFFER,GL_R32I, m_vbo[GRIDLIST_B]);

		glGenTextures(2, m_density_tbo);
        glBindTexture(GL_TEXTURE_BUFFER, m_density_tbo[0]);
        glTexBuffer(GL_TEXTURE_BUFFER, GL_RG32F, m_vbo[DENSITY_A]);
        glBindTexture(GL_TEXTURE_BUFFER, m_density_tbo[1]);
        glTexBuffer(GL_TEXTURE_BUFFER,GL_RG32F, m_vbo[DENSITY_B]);

		glGenTextures(1, &m_color_tbo);
        glBindTexture(GL_TEXTURE_BUFFER, m_color_tbo);
        glTexBuffer(GL_TEXTURE_BUFFER, GL_RGBA, m_vbo[COLORS]);
        

		glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
		glEnable(GL_DEPTH_TEST);
		glEnable(GL_CULL_FACE);
		
			init_Ball_Buffer();	
			init_RayTraceCube_Buffer();	
			overlay.init(128, 50);

	
    }

    void shutdown(void)
    {
        glDeleteProgram(m_update_program);
		glDeleteProgram(m_render_program);
		glDeleteProgram(m_render_wind_program);
		glDeleteProgram(m_render_fluid_raytrace_program);		
        glDeleteBuffers(10, m_vbo);
        glDeleteVertexArrays(2, m_vao);
    }

	void render_wind_dir(){
		glUseProgram(m_render_wind_program);
		glBindVertexArray(wind_dir_vao);
		glUniformMatrix4fv(proj_mat_loc_wind, 1, GL_FALSE, proj_matrix);
       
		vmath::mat4 mv_mat =v_matrix*m_matrix;
        glUniformMatrix4fv(mv_mat_loc_wind, 1, GL_FALSE, mv_mat);

		  
	//	vmath::vec3 wind_pointer[]={vmath::vec3(0),vmath::normalize(-wind_dir+vmath::vec3(0.2f,0.0f,0.0f)),vmath::normalize(-wind_dir-vmath::vec3(0.2f,0.0f,0.0f))};
		vmath::vec3 wind_pointer[]={-wind_dir*0.5f,wind_dir*0.5f};
		
		  	   
	   glNamedBufferData(wind_pointer_buff,4*3*2,wind_pointer,GL_STATIC_DRAW);
	 
	   
	  
	  // glDisable(GL_DEPTH_TEST);
	  // glDepthMask(false);
       glDrawArrays(GL_LINES, 0, 3);
	  // glDepthMask(true);
	  // glEnable(GL_DEPTH_TEST);        
	   glBindVertexArray(0);
	  
	  
	}

	void updateOverlay()
{
    char buffer[256];

    overlay.clear();
    sprintf(buffer, "%2.2fms / frame (%4.2f FPS)", 1000.0f / fps, fps);
	overlay.drawText(buffer, 0, 0);
//	memset(buffer,0,256);
//	sprintf(buffer, "atract = 1 / %2.2f*r^2 + %2.2f*r + %2.2f)",atractC[0],atractC[1],atractC[2]);
//	overlay.drawText(buffer, 0, 1);
	//memset(buffer,0,256);
//	sprintf(buffer, "repuls = 1 / %2.2f*r^2 + %2.2f*r + %2.2f)",repulsC[0],repulsC[1],repulsC[2]);
//	overlay.drawText(buffer, 0, 2);

	
    sprintf(buffer, "Optim: %3.2fms", optTime);
	overlay.drawText(buffer, 0, 1);
	sprintf(buffer, "Update: %3.2fms", updateTime);
	overlay.drawText(buffer, 0, 2);

    overlay.draw();
}

	void init_Ball_Buffer(){
		if(render_balls_vao)glDeleteVertexArrays(1,&render_balls_vao);
		glGenVertexArrays(1, &render_balls_vao);		
		glBindVertexArray(render_balls_vao);
		if(balls_buffer)glDeleteBuffers(1,&balls_buffer);
		glGenBuffers(1,&balls_buffer);
		glBindBuffer(GL_ARRAY_BUFFER,balls_buffer);
		VERT_PER_BALL = 0;

		const float PI = 3.14159265358979323846264f;
        const float R=0.3;
        const float step =PI/ 4;
		std::vector<float> vertices =  std::vector<float>();
		for(float i=0;i<=2*PI;i+=step){
		for(float j=-PI/2;j<PI/2+step;j+=step){
	
		
			vertices.push_back(R*sin(i+step)*cos(j));
			vertices.push_back(R*cos(i+step)*cos(j));
			vertices.push_back(R*sin(j));
	
			vertices.push_back(R*sin(i)*cos(j));
			vertices.push_back(R*cos(i)*cos(j));
			vertices.push_back(R*sin(j));
		
		}
	}
		VERT_PER_BALL = vertices.size();
		glBufferData(GL_ARRAY_BUFFER,4*VERT_PER_BALL,&*vertices.begin(),GL_STATIC_DRAW);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
        glEnableVertexAttribArray(0);		
		glBindVertexArray(0);
				
	}

	void init_RayTraceCube_Buffer(){
		if(render_fluid_ray_trace_vao)glDeleteVertexArrays(1,&render_fluid_ray_trace_vao);
		glGenVertexArrays(1, &render_fluid_ray_trace_vao);		
		glBindVertexArray(render_fluid_ray_trace_vao);
		if(rayTraceCube_buffer)glDeleteBuffers(1,&rayTraceCube_buffer);
		glGenBuffers(1,&rayTraceCube_buffer);
		glBindBuffer(GL_ARRAY_BUFFER,rayTraceCube_buffer);
		
		
		
		const float w=0.3,h=1.0,d=0.0;
		vmath::vec3 v[] = {
	vmath::vec3(-w, -h,  d), // front-bottom-left     0
    vmath::vec3( w, -h,  d), // front-bottom-right    1
    vmath::vec3(-w,  h,  d), // front-top-left        2
    vmath::vec3( w,  h,  d), // front-top-right       3

    vmath::vec3(-w, -h, -d), // back-bottom-left      4
    vmath::vec3( w, -h, -d), // back-bottom-right     5
    vmath::vec3(-w,  h, -d), // back-top-left         6
    vmath::vec3( w,  h, -d) // back-top-right        7
		};

//vmath::vec3  vertices[] = {v[7] , v[6] , v[3] , v[2] , v[0] , v[6] , v[4] ,	v[7] , v[5] , v[3] , v[1] , v[0] , v[5] , v[4]};

vmath::vec3  vertices[] = {v[0] , v[1] , v[2] , v[3] ,v[0]};
		
		
		glBufferData(GL_ARRAY_BUFFER,sizeof(vertices),vertices,GL_DYNAMIC_DRAW);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
        glEnableVertexAttribArray(0);		
		glBindVertexArray(0);			
	}


	void render_fps(float currentTime){	
    float nowTime = (currentTime);
		  if (nowTime > (lastTime + 0.25f))
    {
        fps = float(frames) / (nowTime - lastTime);
        frames = 0;
        lastTime = nowTime;
		GLuint upTm;
		glGetQueryObjectuiv(timerQueries[0], GL_QUERY_RESULT, &upTm);	
		updateTime = upTm/1000000.0;

    }
    updateOverlay();
    frames++;
	}

    void render(double t)
    {
        int i;		
        glUseProgram(m_update_program);

      //  glEnable(GL_RASTERIZER_DISCARD);
		glUniform1f(wind_speed_loc,wind_speed);
		glUniform3fv(wind_dir_loc,1,wind_dir);
		glUniform3fv(glass_pos_loc,1,glass_pos);
		glUniform3fv(atractC_loc,1,atractC);
		glUniform3fv(repulsC_loc,1,repulsC);
	
        for (i = iterations_per_frame; i != 0; --i)
        {			
            glBindVertexArray(m_vao[m_iteration_index & 1]);	

			
			std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
			
#if(OPTIM_STRUCT==4)		
			((OptimiserThreadPoolListsOfLists*)gridOptimiser)->join_Workers();	
			glFlushMappedNamedBufferRange(m_vbo[GRIDLIST_A + (m_iteration_index & 1)],0,GRIDLIST_SIZE);
			gridOptimiser->fillList(pointsBuffer[m_iteration_index & 1],gridBuffer[m_iteration_index & 1],glass_pos);
			
#elif(OPTIM_STRUCT >0)
			int * gridBuffer = (int *)glMapNamedBufferRange(m_vbo[GRIDLIST_A + (m_iteration_index & 1)], 0, GRIDLIST_SIZE, GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT);
			vmath::vec3 * pointsBuffer = (vmath::vec3 *)glMapNamedBufferRange( m_vbo[POSITION_A + (m_iteration_index & 1)], 0, POSITIONS_SIZE, GL_MAP_READ_BIT);		
			gridOptimiser->fillList(pointsBuffer,gridBuffer,glass_pos);
			//gridOptimiser->printList(pointsBuffer,gridBuffer);std::exit(0); 		
			glUnmapNamedBuffer(m_vbo[GRIDLIST_A + (m_iteration_index & 1)]);
			glUnmapNamedBuffer(m_vbo[POSITION_A + (m_iteration_index & 1)]);
#endif
			std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
			optTime = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count()/1000.0;
		

			glActiveTexture(GL_TEXTURE1);
            glBindTexture(GL_TEXTURE_BUFFER, m_vel_tbo[m_iteration_index & 1]);			
			glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_BUFFER, m_pos_tbo[m_iteration_index & 1]);
			glActiveTexture(GL_TEXTURE2);
            glBindTexture(GL_TEXTURE_BUFFER, m_GRIDLIST_tbo[m_iteration_index & 1]);
			glActiveTexture(GL_TEXTURE3);
            glBindTexture(GL_TEXTURE_BUFFER, m_density_tbo[m_iteration_index & 1]);
			//
            m_iteration_index++;
            glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 0, m_vbo[POSITION_A + (m_iteration_index & 1)]);
            glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 1, m_vbo[VELOCITY_A + (m_iteration_index & 1)]);
			glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 2, m_vbo[DENSITY_A  + (m_iteration_index & 1)]);
			glBeginQuery(GL_TIME_ELAPSED, timerQueries[0]);
            glBeginTransformFeedback(GL_POINTS);
            glDrawArrays(GL_POINTS, 0, POINTS_TOTAL);
            glEndTransformFeedback();
			glEndQuery(GL_TIME_ELAPSED);
			
        }

        glDisable(GL_RASTERIZER_DISCARD);

        static const GLfloat black[] = { 0.1f, 0.3f, 0.4f, 0.0f };

        glViewport(0, 0, info.windowWidth, info.windowHeight);
        glClearBufferfv(GL_COLOR, 0,black);
		static const GLfloat one = 1.0f; 
		glClearBufferfv(GL_DEPTH, 0, &one);
	 if (draw_points)
        {	
			glUseProgram(m_render_program);
		
		glUniformMatrix4fv(proj_mat_loc, 1, GL_FALSE, proj_matrix);
       
		vmath::mat4 mv_mat = v_matrix*m_matrix;
        glUniformMatrix4fv(mv_mat_loc, 1, GL_FALSE, mv_mat);
		glUniform3fv(light_pos_loc,1,light_pos);       
		
       
			//glPointSize(4);
          //  glDrawArrays(GL_POINTS, 0, POINTS_TOTAL);        
			glBindVertexArray(render_balls_vao);
			glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_BUFFER, m_pos_tbo[(m_iteration_index+1) & 1]);
			glActiveTexture(GL_TEXTURE1);
            glBindTexture(GL_TEXTURE_BUFFER, m_GRIDLIST_tbo[(m_iteration_index+1) & 1]);
			glActiveTexture(GL_TEXTURE2);
			glBindTexture(GL_TEXTURE_BUFFER, m_color_tbo);
			glActiveTexture(GL_TEXTURE3);
            glBindTexture(GL_TEXTURE_BUFFER, m_density_tbo[(m_iteration_index+1) & 1]);
			glDrawArraysInstanced(GL_TRIANGLE_STRIP, 0,VERT_PER_BALL/3, POINTS_TOTAL);		
		}
		if(draw_raytrace){
			
			glUseProgram(m_render_fluid_raytrace_program);		
			glUniformMatrix4fv(proj_mat_raytrace_loc, 1, GL_FALSE, proj_matrix);       
			vmath::mat4 mv_mat = v_matrix*m_matrix;
			glUniformMatrix4fv(mv_mat_raytrace_loc, 1, GL_FALSE, mv_mat);
			glUniform3fv(light_pos_raytrace_loc,1,light_pos);  
			glBindVertexArray(render_fluid_ray_trace_vao);
					
			glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_BUFFER, m_pos_tbo[m_iteration_index & 1]);
			glActiveTexture(GL_TEXTURE1);
            glBindTexture(GL_TEXTURE_BUFFER, m_vel_tbo[m_iteration_index & 1]);	
			glActiveTexture(GL_TEXTURE2);
            glBindTexture(GL_TEXTURE_BUFFER, m_GRIDLIST_tbo[m_iteration_index & 1]);
			glActiveTexture(GL_TEXTURE3);
            glBindTexture(GL_TEXTURE_BUFFER, m_density_tbo[m_iteration_index & 1]);
			glActiveTexture(GL_TEXTURE4);
			glBindTexture(GL_TEXTURE_BUFFER, m_color_tbo);
		
			glDisable(GL_DEPTH_TEST);
			glDrawArrays(GL_TRIANGLE_STRIP, 0,4);
			glEnable(GL_DEPTH_TEST);
		}
		render_wind_dir();
		glDisable(GL_CULL_FACE);
		render_fps(t);
		glEnable(GL_CULL_FACE);
    }
	
	void pointsToGridListsOfArrays(int pointsCount,vmath::vec3 * points,int * gridBuffer ,int gridSide , float cellSize,float offset){
		int gridSize = gridSide*gridSide*gridSide;
		std::vector<int> cellsSizes =std::vector<int>(gridSize,0);
		std::vector<std::forward_list<int>> cells = std::vector<std::forward_list<int>>(gridSize);
		

		for(int i=0;i<pointsCount;i++){
			int cellIndex = CellIndex(&points[i],cellSize,gridSide,offset);					
			cells[cellIndex].push_front(i);
			cellsSizes[cellIndex]++;
		}
		int lastUsedInd=(gridSize)*2-1;
		for(int i=0;i<gridSize;i++){
			gridBuffer[i*2]=lastUsedInd+1;
			gridBuffer[i*2+1]=cellsSizes[i];			
			for(std::forward_list<int>::iterator it = cells[i].begin();it!=cells[i].end();it++){
				gridBuffer[++lastUsedInd]=*it;
			}
		}
	}
		
	void pointsToGridListsOfListsParall(int pointsCount,vmath::vec3 * points,int * gridBuffer ,int gridSide , float cellSize,float offset,int kernels){
		int gridSize = gridSide*gridSide*gridSide;
		std::vector<int> cellsSizes =std::vector<int>(gridSize,0);
	
		std::atomic_flag* list_Locs = new std::atomic_flag[gridSize]; //keys to every list 	
		std::atomic<int> lastUsedInd=(gridSize)-1;
		
		for(int i=0;i<gridSize*2;i++)gridBuffer[i]=0;//init buffer

		int pointsPerKernel = pointsCount / (kernels);
		int pointsPerKernelRem = pointsCount % kernels;

	

		for(int i=0;i<kernels;i++){
			int nPoints = pointsPerKernel;
			if((i)==(kernels-1))nPoints+=pointsPerKernelRem;
			GridListKernelParall(i,gridBuffer,&lastUsedInd,list_Locs,cellSize,gridSide, offset,points,pointsPerKernel*i, nPoints);
		}
		
		
	}
	void GridListKernelParall(int globalIndex ,int * gridBuffer,std::atomic<int>* lastUsedInd,std::atomic_flag* list_Locs,int cellSize,int gridSide,float offset,vmath::vec3 * points,int firstPoint,int nPoints){		
		for(int i =firstPoint;i<firstPoint+nPoints;i++){
			int listInd = CellIndex(points+i,cellSize,gridSide,offset);
			int newMemory = ++(*lastUsedInd)*2;//alocating memory
			while(list_Locs[listInd].test_and_set(std::memory_order_acquire));// aquire lock
				if(gridBuffer[listInd]==0){// pyrwiqt elemnt v spisyka
					gridBuffer[listInd] = newMemory;
					gridBuffer[listInd+1]++;//incrementing list size
					gridBuffer[newMemory]=i;// asigning new item value
					gridBuffer[newMemory+1]=0;
				}else{//poredniqt element
					int oldFirst = gridBuffer[listInd];
					gridBuffer[listInd] = newMemory;
					gridBuffer[listInd+1]++;//incrementing list size
					gridBuffer[newMemory]=i;// asigning new item value
					gridBuffer[newMemory+1]=oldFirst;
				}
			list_Locs[listInd].clear(std::memory_order_release);//release lock
		}
		
		
	}
	
	inline int CellIndex(vmath::vec3 * point,float cellSize,int gridSide,float offset){
	int cellIndex=0;
	cellIndex =std::max( std::min(((int)floorf(((*point)[2]+offset)/cellSize))*gridSide*gridSide,gridSide*gridSide*(gridSide-1)),0);
	cellIndex +=std::max( std::min(((int)floorf(((*point)[1]+offset)/cellSize))*gridSide,gridSide*(gridSide-1)),0);
	cellIndex +=std::max( std::min(((int)floorf(((*point)[0]+offset)/cellSize)),(gridSide-1)),0);
	return cellIndex;	
	}

private:
	
	struct KeyBuffer{
	bool q;
	}keyBuffer;
    void onKey(int key, int action)
    { 
		switch (key)
            {
                case 'Q': keyBuffer.q=action;
                    break;              

			}
        if (action)
        {
            switch (key)
            {
                case 'R': load_shaders();
                    break;
                case 'L': draw_raytrace = !draw_raytrace;
                    break;
                case 'P': draw_points = !draw_points;
                    break;
				case 'T': draw_triangles = !draw_triangles;
                    break;
				case ']': wind_speed=wind_speed==0.0?0.01:wind_speed*1.5;
                    break;
                case '[': wind_speed*=0.7;
                    break;

				case 'A': atractC[0]*=1.1;
                    break;
                case 'Z': atractC[0]*=0.9;
					break;
				case 'S': atractC[1]*=1.1;
                    break;
                case 'X': atractC[1]*=0.9;
                    break;
				case 'D': atractC[2]*=1.1;
                    break;
                case 'C': atractC[2]*=0.9;
                    break;

				case 'F': repulsC[0]*=1.1;
                    break;
                case 'V': repulsC[0]*=0.9;
					break;
				case 'G': repulsC[1]*=1.1;
                    break;
                case 'B': repulsC[1]*=0.9;
                    break;
				case 'H': repulsC[2]*=1.1;
                    break;
                case 'N': repulsC[2]*=0.9;
                    break;

                case GLFW_KEY_KP_ADD: iterations_per_frame++;
                    break;
                case GLFW_KEY_KP_SUBTRACT: iterations_per_frame--;
                    break;
            }
        }
    }
	int lmx,lmy;
	bool draging,rotating,looking;
	void onMouseMove(int x,int y){		
		if (rotating){
			vmath::mat4 rotmat = vmath::rotate((-lmx+x)*0.3f, 0.0f, 1.0f, 0.0f)* vmath::rotate(-(lmy-y)*0.3f, 1.0f, 0.0f, 0.0f);
			if(keyBuffer.q){
				vmath::vec4 tmp_wind_dir =vmath::vec4(wind_dir,0.0f);
			tmp_wind_dir =tmp_wind_dir* rotmat;
			wind_dir = vmath::vec3(tmp_wind_dir[0],tmp_wind_dir[1],tmp_wind_dir[2]);
			}else{		
				m_matrix= rotmat*m_matrix;
			}
		}
		if (draging){
			if(keyBuffer.q){
			//	vmath::vec4 tmp_glass_pos =vmath::vec4(glass_pos,1.0f);
			//tmp_glass_pos =tmp_glass_pos* vmath::translate((-lmx+x)*1.0f, (lmy-y)*1.0f, 0.0f);
			//glass_pos = vmath::vec3(tmp_glass_pos[0],tmp_glass_pos[1],tmp_glass_pos[2]);
				glass_pos[0]-=-(-lmx+x)*0.1;
				glass_pos[1]+=(lmy-y)*0.1;
			}else{
		 v_matrix = vmath::translate((-lmx+x)*0.1f, -(-lmy+y)*0.1f, 0.0f)*v_matrix;	
			}
		}
		if (looking){
			vmath::mat4 lookrotmat = vmath::rotate((lmx-x)*0.4f, 0.0f, 1.0f, 0.0f)* vmath::rotate(-(lmy-y)*0.4f, 1.0f, 0.0f, 0.0f);
		 v_matrix= lookrotmat*v_matrix;		
		}
		 lmx=x;lmy=y;
	}
	void onMouseWheel(int pos){
		float c = 1.3f;
		if(pos<0)c=1.2f;
		else if(pos>0)c=-1.2f;
		v_matrix = vmath::translate(0.0f, 0.0f,c)*v_matrix;
	}
	void onMouseButton(int button,int action){
		if(button==0&&rotating||button==1&&draging){
		init_user_transforms();
		}
		if(button==0)draging=action;
		if(button==1)rotating=action;
		if(button==2)looking=action;
		
		//DBOUT("button:"<<button<<" action:"<<action);
	}
    void load_shaders()
    {
        GLuint vs;
        GLuint fs;
		GLuint gs;
        char buffer[1024];

        vs = sb7::shader::load("media/shaders/update.vs.glsl", GL_VERTEX_SHADER);

        if (m_update_program)
            glDeleteProgram(m_update_program);
        m_update_program = glCreateProgram();
        glAttachShader(m_update_program, vs);

        static const char * tf_varyings[] = 
        {
            "tf_position",
            "tf_velocity_mass",	
			"tf_density_pressure"
        };

        glTransformFeedbackVaryings(m_update_program, 3, tf_varyings, GL_SEPARATE_ATTRIBS);

        glLinkProgram(m_update_program);
		wind_speed_loc=glGetUniformLocation(m_update_program, "wind_speed");
		wind_dir_loc=glGetUniformLocation(m_update_program, "wind_dir");
		glass_pos_loc=glGetUniformLocation(m_update_program, "glass_pos");
		atractC_loc=glGetUniformLocation(m_update_program, "atractC");
		repulsC_loc=glGetUniformLocation(m_update_program, "repulsC");
		GRID_VOLUME_SIDE_loc=glGetUniformLocation(m_update_program, "GRID_VOLUME_SIDE");
        glGetShaderInfoLog(vs, 1024, NULL, buffer);
        glGetProgramInfoLog(m_update_program, 1024, NULL, buffer);

        glDeleteShader(vs);

        vs = sb7::shader::load("media/shaders/render.vs.sphere.glsl", GL_VERTEX_SHADER);
		//gs = sb7::shader::load("media/shaders/render.gs.glsl", GL_GEOMETRY_SHADER);
		//gs = sb7::shader::load("media/shaders/render.gs.sphere.glsl", GL_GEOMETRY_SHADER);
       // fs = sb7::shader::load("media/shaders/render.fs.glsl", GL_FRAGMENT_SHADER);
		fs = sb7::shader::load("media/shaders/render.fs.sphere.glsl", GL_FRAGMENT_SHADER);

        if (m_render_program)
            glDeleteProgram(m_render_program);
        m_render_program = glCreateProgram();
        glAttachShader(m_render_program, vs);
		//glAttachShader(m_render_program, gs);
        glAttachShader(m_render_program, fs);
        
        glLinkProgram(m_render_program);
		proj_mat_loc=glGetUniformLocation(m_render_program, "proj_mat");
		mv_mat_loc=glGetUniformLocation(m_render_program, "mv_mat");		
		light_pos_loc=glGetUniformLocation(m_render_program, "light_pos");
		

		vs = sb7::shader::load("media/shaders/render.wind.vs.glsl", GL_VERTEX_SHADER);
        fs = sb7::shader::load("media/shaders/render.wind.fs.glsl", GL_FRAGMENT_SHADER);
		gs = sb7::shader::load("media/shaders/render.wind.gs.glsl", GL_GEOMETRY_SHADER);

        if (m_render_wind_program)
            glDeleteProgram(m_render_wind_program);
        m_render_wind_program = glCreateProgram();
        glAttachShader(m_render_wind_program, vs);
        glAttachShader(m_render_wind_program, fs);
		glAttachShader(m_render_wind_program, gs);
        
        glLinkProgram(m_render_wind_program);
		proj_mat_loc_wind=glGetUniformLocation(m_render_wind_program, "proj_mat");
		mv_mat_loc_wind=glGetUniformLocation(m_render_wind_program, "mv_mat");
		


		vs = sb7::shader::load("media/shaders/render.fluid.raytrace.vs.glsl", GL_VERTEX_SHADER);
		//gs = sb7::shader::load("media/shaders/render.fluid.raytrace.gs.glsl", GL_GEOMETRY_SHADER); 
		fs = sb7::shader::load("media/shaders/render.fluid.raytrace.fs.glsl", GL_FRAGMENT_SHADER);

        if (m_render_fluid_raytrace_program)
            glDeleteProgram(m_render_fluid_raytrace_program);
        m_render_fluid_raytrace_program = glCreateProgram();
        glAttachShader(m_render_fluid_raytrace_program, vs);
		//glAttachShader(m_render_program, gs);
        glAttachShader(m_render_fluid_raytrace_program, fs);
        
        glLinkProgram(m_render_fluid_raytrace_program);
		proj_mat_raytrace_loc=glGetUniformLocation(m_render_fluid_raytrace_program, "proj_mat");
		mv_mat_raytrace_loc=glGetUniformLocation(m_render_fluid_raytrace_program, "mv_mat");		
		light_pos_raytrace_loc=glGetUniformLocation(m_render_fluid_raytrace_program, "light_pos");
		


		
    }
	GLuint			timerQueries[2];
    GLuint          m_vao[2];
    GLuint          m_vbo[10];
    GLuint          m_index_buffer;
	GLuint			m_index_triangles_buffer;
    GLuint          m_GRIDLIST_tbo[2];
	GLuint          m_pos_tbo[2];
	GLuint          m_vel_tbo[2];
	GLuint          m_density_tbo[2];
	GLuint          m_color_tbo;
	GLuint          m_con_tbo;
    GLuint          m_update_program;
    GLuint          m_render_program;
	GLuint          m_render_wind_program;
	GLuint          m_render_fluid_raytrace_program;
    GLuint          m_C_loc;
    GLuint          m_iteration_index;
	GLuint			wind_dir_vao;

    bool            draw_points;
    bool            draw_lines;
	bool	        draw_triangles;
	bool			draw_raytrace;
    int             iterations_per_frame;

	GLuint proj_mat_raytrace_loc;
	GLuint	mv_mat_raytrace_loc;
	GLuint	light_pos_raytrace_loc;		
	GLuint GRID_VOLUME_SIDE_loc ;
	GLuint mv_mat_loc ;
	GLuint proj_mat_loc ;
	GLuint wind_speed_loc ;
	GLuint wind_dir_loc ;
	GLuint glass_pos_loc ;
	GLuint mv_mat_loc_wind ;
	GLuint proj_mat_loc_wind ;
	GLuint wind_speed_loc_wind ;
	GLuint wind_dir_loc_wind ;
	GLuint atractC_loc ;
	GLuint repulsC_loc ;
	GLuint wind_pointer_buff;
	GLuint normal_buffer;
	GLuint light_pos_loc;
	GLuint tex_coord_buf;
	GLuint main_texture;
	GLuint balls_buffer;
	GLuint rayTraceCube_buffer;
	GLuint render_balls_vao;
	GLuint render_fluid_ray_trace_vao;
	int VERT_PER_BALL;
	vmath::mat4 proj_matrix;
	vmath::mat4 v_matrix;
	vmath::mat4 m_matrix;

	float wind_speed;
	vmath::vec3 wind_dir;
	vmath::vec3 light_pos;
	vmath::vec3 glass_pos;
	vmath::vec3 atractC;
	vmath::vec3 repulsC;

	int * gridBuffer[2] ;
	vmath::vec3 * pointsBuffer[2];
	  
	  float               fps;
	  float               optTime;
	  float               updateTime;
	  float lastTime ;
      int frames ;
};

DECLARE_MAIN(springmass_app);
