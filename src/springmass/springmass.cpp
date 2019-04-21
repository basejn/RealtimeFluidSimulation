/*
 * Asen Asenov
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
#include<string>
#include <mutex>
#include<vector>
#include<memory>

#define DBOUT( s )            \
{                              \
	std::wostringstream os_;    \
	os_ << s;                   \
	OutputDebugStringW(os_.str().c_str());  \
}
#ifdef DBOUT 
#include <Windows.h>
#include <iostream>
#include <sstream>
#endif

using namespace vmath;

enum BUFFER_TYPE_t
{
	POSITION_A,
	POSITION_B,
	VELOCITY_A,
	VELOCITY_B,
	DENSITY_A,
	DENSITY_B,
	COLORS,
	GRIDLIST_A,
	GRIDLIST_B,
	CONNECTION
};


enum
{
	POINTS_X = 15,
	POINTS_Y = 15,
	POINTS_Z = 15,
	POINTS_TOTAL = (POINTS_X * POINTS_Y * POINTS_Z),
	CONNECTIONS_TOTAL = (POINTS_X - 1) * POINTS_Y + (POINTS_Y - 1) * POINTS_X
};

#define INT_STATE_FROM_FILE 1
#define OPTIM_STRUCT  7 //0=no 1=array 2=lists 3=listsParral 4=listsParralThreadPool 5=listsParralThreadPoolArrays 6=arrayAllNeighbourIndsInCell 7=arrayAllNeighbourDataInCell

const int GRID_SIDE =  15;
const float GRID_VOLUME_SIDE = 20.0f; //+-10
const float GRID_OFFSET = 10.0f; //+-10
const float CELL_SIZE = GRID_VOLUME_SIDE / GRID_SIDE;
const int DENSITY_TEX_SIDE = 64;// trqbva da se promeni i v GeometryShadera
const int CELL_COUNT = GRID_SIDE * GRID_SIDE * GRID_SIDE;
#if (OPTIM_STRUCT ==1||OPTIM_STRUCT ==5)
const int GRIDLIST_SIZE = (GRID_SIDE * GRID_SIDE * GRID_SIDE * 3 + POINTS_TOTAL) * sizeof(int);
#elif (OPTIM_STRUCT ==6)
const int GRIDLIST_SIZE = (GRID_SIDE * GRID_SIDE * GRID_SIDE * 3 * (27*POINTS_TOTAL  / (GRID_SIDE * GRID_SIDE * GRID_SIDE)) ) * sizeof(int);
#elif (OPTIM_STRUCT ==7)
//const int GRIDLIST_SIZE = (GRID_SIDE * GRID_SIDE * GRID_SIDE * 2) * sizeof(int);
const int GRIDLIST_SIZE = (GRID_SIDE * GRID_SIDE * GRID_SIDE * 3 * (27 * POINTS_TOTAL / (GRID_SIDE * GRID_SIDE * GRID_SIDE))) * sizeof(int);
const int GRIDLIST_DATA_SIZE = (GRID_SIDE * GRID_SIDE * GRID_SIDE * 9 * (27 * POINTS_TOTAL / (GRID_SIDE * GRID_SIDE * GRID_SIDE))) * sizeof(float);
#else
const int GRIDLIST_SIZE = (GRID_SIDE*GRID_SIDE*GRID_SIDE*2+POINTS_TOTAL*2) * sizeof(int);
#endif
const int POSITIONS_SIZE = POINTS_TOTAL * sizeof(vmath::vec3);

class GridOptimiser
{
public:
	virtual void fillList(vmath::vec3* pointsBuffer, int* gridBuffer, vmath::vec3 offset_vec) = 0;
	virtual void join_Workers() = 0;
	virtual bool is_ready() = 0;
	float optTime;
};

class GridListsOfListsOptimiser :public GridOptimiser
{
	int pointsCount;
	int gridSide;
	float cellSize;
	float offset;
	int gridSize;
public:
	GridListsOfListsOptimiser(int pointsCount, int gridSide, float cellSize, float offset)
	{
		this->pointsCount = pointsCount;
		this->gridSide = gridSide;
		this->cellSize = cellSize;
		this->offset = offset;
		gridSize = gridSide * gridSide * gridSide;
		lastUsedInd = new std::atomic<int>(0);
	}

	void fillList(vmath::vec3* pointsBuffer, int* gridBuffer, vmath::vec3 offset_vec)
	{
		this->gridBuffer = gridBuffer;
		this->offset_vec = offset_vec;
		std::vector<int> cellsSizes = std::vector<int>(gridSize, 0);
		std::vector<std::forward_list<int>> cells = std::vector<std::forward_list<int>>(gridSize);
		for (int i = 0; i < pointsCount; i++)
		{
			int cellIndex = CellIndex(&(pointsBuffer[i]));
			cells[cellIndex].push_front(i);
			cellsSizes[cellIndex]++;
		}
		*lastUsedInd = ((gridSize)-1);
		std::vector<std::thread> v;
		for (int i = 0; i < gridSize; i++)
		{
			//v.emplace_back(std::thread(&GridListsOfListsOptimiser::GridListKernel,this,i,&cells[i],cellsSizes[i]));
			GridListKernel(i, &cells[i], cellsSizes[i]);
		}
		//	for (auto& t : v)t.join();
	}

private:
	vmath::vec3 offset_vec;
	int* gridBuffer;
	std::atomic<int>* lastUsedInd;

	void GridListKernel(int index, std::forward_list<int>* cell, int cellSize)
	{
		if ((*cell).empty())
		{
			gridBuffer[index * 2] = 0;
			gridBuffer[index * 2 + 1] = 0;//cellSize;	
		}
		else
		{
			int lastLocalUsedInd = gridBuffer[index * 2] = (++(*lastUsedInd)) * 2;
			gridBuffer[index * 2 + 1] = cellSize;
			for (std::forward_list<int>::iterator it = ++(*cell).begin(); it != (*cell).end(); it++)
			{
				gridBuffer[lastLocalUsedInd] = *it;//value
				gridBuffer[lastLocalUsedInd + 1] = (++(*lastUsedInd)) * 2;//new alocated index pointing to the next element
				lastLocalUsedInd = gridBuffer[lastLocalUsedInd + 1];//pointer to the next
			}
			gridBuffer[lastLocalUsedInd] = *((*cell).begin());//value
			gridBuffer[lastLocalUsedInd + 1] = 0;//terminator
		}
	}

	inline int CellIndex(vmath::vec3* point)
	{
		int cellIndex = 0;
		cellIndex = std::max(std::min(((int)(((*point)[2] - offset_vec[2] + offset) / cellSize)) * gridSide * gridSide, gridSide * gridSide * (gridSide - 1)), 0);
		cellIndex += std::max(std::min(((int)(((*point)[1] - offset_vec[1] + offset) / cellSize)) * gridSide, gridSide * (gridSide - 1)), 0);
		cellIndex += std::max(std::min(((int)(((*point)[0] - offset_vec[0] + offset) / cellSize)), (gridSide - 1)), 0);
		return cellIndex;
	}
};

class GridListsOfArraysOptimiser :public GridOptimiser
{
	int pointsCount;
	int gridSide;
	float cellSize;
	float offset;
	int gridSize;
	int kernels;
public:
	GridListsOfArraysOptimiser(int pointsCount, int gridSide, float cellSize, float offset, int kernels)
	{
		this->pointsCount = pointsCount;
		this->gridSide = gridSide;
		this->cellSize = cellSize;
		this->offset = offset;
		this->kernels = kernels;
		gridSize = gridSide * gridSide * gridSide;
	}

	void fillList(vmath::vec3* pointsBuffer, int* gridBuffer, vmath::vec3 offset_vec)
	{
		this->pointsBuffer = pointsBuffer;
		this->gridBuffer = gridBuffer;
		this->offset_vec = offset_vec;
		int gridSize = gridSide * gridSide * gridSide;
		std::vector<int> cellsSizes = std::vector<int>(gridSize, 0);
		std::vector<std::forward_list<int>> cells = std::vector<std::forward_list<int>>(gridSize);


		for (int i = 0; i < pointsCount; i++)
		{
			int cellIndex = CellIndex(pointsBuffer + i);
			cells[cellIndex].push_front(i);
			cellsSizes[cellIndex]++;
		}
		int lastUsedInd = (gridSize)* 2 - 1;
		for (int i = 0; i < gridSize; i++)
		{
			gridBuffer[i * 2] = lastUsedInd + 1;
			gridBuffer[i * 2 + 1] = cellsSizes[i];
			for (std::forward_list<int>::iterator it = cells[i].begin(); it != cells[i].end(); it++)
			{
				gridBuffer[++lastUsedInd] = *it;
			}
		}
	}

	void setPointers(vmath::vec3* pointsBuffer, int* gridBuffer)
	{
		this->pointsBuffer = pointsBuffer;
		this->gridBuffer = gridBuffer;
	}

	void printList(vmath::vec3* pointsBuffer, int* gridBuffer)
	{
		std::ofstream myfile;
		myfile.open("Grid_Buffer_SnapShot.txt");
		myfile << "\nGrid Buffer:";
		for (int i = 0; i < GRIDLIST_SIZE / 4; i++)
		{
			myfile << "\n" << i << ":\t" << gridBuffer[i];
		}
		myfile.close();
	}

private:
	vmath::vec3 offset_vec;
	int* gridBuffer;
	vmath::vec3* pointsBuffer;

	inline int CellIndex(vmath::vec3* point)
	{
		int cellIndex = 0;
		cellIndex = std::max(std::min(((int)(((*point)[2] - offset_vec[2] + offset) / cellSize)) * gridSide * gridSide, gridSide * gridSide * (gridSide - 1)), 0);
		cellIndex += std::max(std::min(((int)(((*point)[1] - offset_vec[1] + offset) / cellSize)) * gridSide, gridSide * (gridSide - 1)), 0);
		cellIndex += std::max(std::min(((int)(((*point)[0] - offset_vec[0] + offset) / cellSize)), (gridSide - 1)), 0);
		return cellIndex;
	}
};

class GridListsOfListsParralOptimiser :public GridOptimiser
{
public:
	GridListsOfListsParralOptimiser(int pointsCount, int gridSide, float cellSize, float offset, int kernels)
	{
		this->pointsCount = pointsCount;
		this->gridSide = gridSide;
		this->cellSize = cellSize;
		this->offset = offset;
		this->kernels = kernels;
		gridSize = gridSide * gridSide * gridSide;
		lastUsedInd = new std::atomic<int>(0);
		list_Locs = new std::atomic_flag[gridSize];
		for (int i = 0; i < gridSize; i++)list_Locs[i].clear();
	}

	void fillList(vmath::vec3* pointsBuffer, int* gridBuffer, vmath::vec3 offset_vec)
	{
		this->pointsBuffer = pointsBuffer;
		this->gridBuffer = gridBuffer;
		this->offset_vec = offset_vec;
		std::vector<int> cellsSizes = std::vector<int>(gridSize, 0);

		*lastUsedInd = (gridSize)-1;

		for (int i = 0; i < gridSize * 2; i++)gridBuffer[i] = 0;//init buffer
		//for(int i =0;i<gridSize;i++)list_Locs[i].clear();
		int pointsPerKernel = pointsCount / (kernels);
		int pointsPerKernelRem = pointsCount % kernels;

		//DBOUT("\n");
		//v.clear();
		for (int i = 0; i < kernels; i++)
		{
			int nPoints = pointsPerKernel;
			if ((i) == (kernels - 1))nPoints += pointsPerKernelRem;
			//	v.emplace_back(std::thread(&GridListsOfListsParralOptimiser::GridListKernelParall,this,i,pointsPerKernel*i,nPoints));
			GridListKernelParall(i, pointsPerKernel * i, nPoints);
		}
		//for (auto& t : v)t.join();
	}

	void printList(vmath::vec3* pointsBuffer, int* gridBuffer)
	{
		std::ofstream myfile;
		myfile.open("Grid_Buffer_SnapShot.txt");
		myfile << "\nGrid Buffer:";
		for (int i = 0; i < GRIDLIST_SIZE / 4; i++)
		{
			myfile << "\n" << i << ":\t" << gridBuffer[i];
		}
		myfile.close();
	}

protected:
	int pointsCount;
	int gridSide;
	float cellSize;
	float offset;
	int gridSize;
	int kernels;
	std::vector<std::thread> v;
	int* gridBuffer;
	vmath::vec3* pointsBuffer;
	vmath::vec3 offset_vec;
	std::atomic<int>* lastUsedInd;
	std::atomic_flag* list_Locs;

	void GridListKernelParall(int globalIndex, int firstPoint, int nPoints)
	{
		for (int i = firstPoint; i < firstPoint + nPoints; i++)
		{
			int listInd = CellIndex(pointsBuffer + i) * 2;
			//	std::stringstream sst;	sst<<"\n "<<listInd<<" thr:"<<globalIndex;DBOUT(sst.str().data());	

			int newMemory = ++(*lastUsedInd) * 2;//alocating memory
			while (list_Locs[listInd / 2].test_and_set(std::memory_order_acquire));// aquire lock
			if (gridBuffer[listInd] == 0)
			{// pyrwiqt elemnt v spisyka
				gridBuffer[listInd] = newMemory;
				gridBuffer[listInd + 1]++;//incrementing list size
				gridBuffer[newMemory] = i;// asigning new item value
				gridBuffer[newMemory + 1] = 0;
			}
			else
			{//poredniqt element
				int oldFirst = gridBuffer[listInd];
				gridBuffer[listInd] = newMemory;
				gridBuffer[listInd + 1]++;//incrementing list size
				gridBuffer[newMemory] = i;// asigning new item value
				gridBuffer[newMemory + 1] = oldFirst;
			}
			list_Locs[listInd / 2].clear(std::memory_order_release);//release lock
		}
	}

	inline int CellIndex(vmath::vec3* point)
	{
		int cellIndex = 0;
		cellIndex = std::max(std::min(((int)(((*point)[2] - offset_vec[2] + offset) / cellSize)) * gridSide * gridSide, gridSide * gridSide * (gridSide - 1)), 0);
		cellIndex += std::max(std::min(((int)(((*point)[1] - offset_vec[1] + offset) / cellSize)) * gridSide, gridSide * (gridSide - 1)), 0);
		cellIndex += std::max(std::min(((int)(((*point)[0] - offset_vec[0] + offset) / cellSize)), (gridSide - 1)), 0);
		return cellIndex;
	}
};

class ArraysFromListsThreadPoolOptimiser :public GridListsOfListsParralOptimiser
{
public:
	ArraysFromListsThreadPoolOptimiser(int pointsCount, int gridSide, float cellSize, float offset, int nThreads) : GridListsOfListsParralOptimiser(pointsCount, gridSide, cellSize, offset, nThreads)
	{
		this->nThreads = nThreads;
		worker_Locs = new std::atomic<bool>[nThreads];
		fillListTask_Locs = new std::atomic<bool>[nThreads];
		gridListKernelParall_Locs = new std::atomic<bool>[nThreads];
		gridBuffer = new int[pointsCount * 2 + gridSize * 2];
		resetWorkers();
		resetfillListTasks();
		parameters = new params[nThreads];
		for (int i = 0; i < nThreads; i++)
		{
			threads.emplace_back(std::thread(&ArraysFromListsThreadPoolOptimiser::worker, this, i, &worker_Locs[i], &fillListTask_Locs[i], &gridListKernelParall_Locs[i]));
		}
	}

	std::chrono::high_resolution_clock::time_point t1;

	void fillList(vmath::vec3* pointsBuffer, int* gridBuffer, vmath::vec3 offset_vec)
	{
		t1 = std::chrono::high_resolution_clock::now();

		this->pointsBuffer = pointsBuffer;
		//this->gridBuffer = gridBuffer;
		outGridBuffer = gridBuffer;

		this->offset_vec = offset_vec;
		//fillListTask();
		fillListTask_Locs[0] = (true);
	}

	void resetWorkers()
	{
		for (int i = 0; i < nThreads; i++)
		{
			worker_Locs[i].store(false);
			gridListKernelParall_Locs[i].store(false);
		}
	}

	void resetfillListTasks()
	{
		for (int i = 0; i < nThreads; i++)
		{
			fillListTask_Locs[i] = (false);
		}
	}

	bool is_ready()
	{
		for (int i = 0; i < nThreads; i++)
		{
			if (worker_Locs[i].load() == true)return false;
		}
		return true;
	}

	void join_Workers()
	{
		//DBOUT("\n \t\t Joining");		
		for (int i = 0; i < nThreads; i++)
		{
			while (fillListTask_Locs[i].load() == true);
			while (worker_Locs[i].load() == true);
		}
	}

private:
	void printList(std::string fileName, int* gridBuffer)
	{
		std::ofstream myfile;
		myfile.open(fileName);

		for (int i = 0; i < (GRID_SIDE * GRID_SIDE * GRID_SIDE * 2 + POINTS_TOTAL * 2); i++)
		{
			myfile << "\n" << i << ":\t" << gridBuffer[i];
		}
		myfile.close();
	}

	void worker(int id, std::atomic<bool>* my_Lock, std::atomic<bool>* my_fillListTaskLock, std::atomic<bool>* my_gridListKernelParallLock)
	{
		while (true)
		{
			while ((*my_Lock).load() == false)
			{
				if ((*my_fillListTaskLock) == true)
				{
					fillListTask();
				}
			}

			GridListKernelParall(id, parameters[id].firstPoint, parameters[id].nPoints);
			(*my_gridListKernelParallLock).store(true);

			if ((*my_fillListTaskLock) == true)
			{
				for (int i = 0; i < nThreads; i++)while (gridListKernelParall_Locs[i].load() == false);
				gridArraysFromListKernel();
				optTime = (std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - t1).count() / 1000000.0);
				(*my_fillListTaskLock) = (false);
			}
			(*my_Lock).store(false);
		}
	}

	void gridArraysFromListKernel()
	{
		//outGridBuffer

		int lastUsedIndex = gridSize * 2;
		for (int curInd = 0; curInd < gridSize; curInd++)
		{
			int curCelSize = gridBuffer[curInd * 2 + 1];
			int curCelListStart = gridBuffer[curInd * 2];
			outGridBuffer[curInd * 2 + 1] = curCelSize;
			int curArrayStartInd = lastUsedIndex;
			lastUsedIndex += curCelSize + 1;
			outGridBuffer[curInd * 2] = curArrayStartInd;
			outGridBuffer[lastUsedIndex - 1] = 0;

			int curentIteratingIndex = curCelListStart;
			int curentParticlePosIndex = curArrayStartInd;

			while (curentIteratingIndex != 0)
			{
				outGridBuffer[curentParticlePosIndex++] = gridBuffer[curentIteratingIndex];
				curentIteratingIndex = gridBuffer[curentIteratingIndex + 1];
			}
		}
	}

	void gridArrdfgaysFromListKernelPar()
	{
		int lastUsedIndex = gridSize * 2;
		for (int curInd = 0; curInd < gridSize; curInd++)
		{
			int curCelSize = gridBuffer[curInd * 2 + 1];
			int curCelListStart = gridBuffer[curInd * 2];
			outGridBuffer[curInd * 2 + 1] = curCelSize;
			int curArrayStartInd = lastUsedIndex;
			lastUsedIndex += curCelSize + 1;
			outGridBuffer[curInd * 2] = curArrayStartInd;
			outGridBuffer[lastUsedIndex - 1] = 0;
		}

		for (int curInd = 0; curInd < gridSize; curInd++)
		{
			int curentIteratingIndex = gridBuffer[curInd * 2];;
			int curentParticlePosIndex = outGridBuffer[curInd * 2];

			while (curentIteratingIndex != 0)
			{
				outGridBuffer[curentParticlePosIndex++] = gridBuffer[curentIteratingIndex];
				curentIteratingIndex = gridBuffer[curentIteratingIndex + 1];
			}
		}
	}

	void fillListTask()
	{
		*lastUsedInd = (gridSize)-1;

		//for(int i=0;i<gridSize*2;i++)gridBuffer[i]=0;//init buffer
		memset(gridBuffer, 0, gridSize * 2 * sizeof(int));
		int pointsPerKernel = pointsCount / (nThreads);
		int pointsPerKernelRem = pointsCount % nThreads;

		//DBOUT("\n");
		//resetWorkers();
		for (int i = 0; i < nThreads; i++)
		{
			int nPoints = pointsPerKernel;
			if ((i) == (nThreads - 1))nPoints += pointsPerKernelRem;
			//	v.emplace_back(std::thread(&GridListsOfListsParralOptimiser::GridListKernelParall,this,i,pointsPerKernel*i,nPoints));
			//	GridListKernelParall(i,pointsPerKernel*i, nPoints);
			parameters[i].firstPoint = pointsPerKernel * i;
			parameters[i].nPoints = nPoints;
			gridListKernelParall_Locs[i].store(false);
			worker_Locs[i] = (true);
		}
	}

	std::atomic<bool>* worker_Locs;
	std::atomic<bool>* gridListKernelParall_Locs;
	std::atomic<bool>* fillListTask_Locs;
	int nThreads;
	int* outGridBuffer;
	std::vector<std::thread> threads;

	struct params
	{
		int firstPoint;
		int nPoints;
	}*parameters;
};


class OptimiserThreadPoolListsOfLists :public GridListsOfListsParralOptimiser
{
public:
	OptimiserThreadPoolListsOfLists(int pointsCount, int gridSide, float cellSize, float offset, int nThreads) : GridListsOfListsParralOptimiser(pointsCount, gridSide, cellSize, offset, nThreads)
	{
		this->nThreads = nThreads;
		worker_Locs = new std::atomic<bool>[nThreads];
		fillListTask_Locs = new std::atomic<bool>[nThreads];
		resetWorkers();
		resetfillListTasks();
		parameters = new params[nThreads];
		for (int i = 0; i < nThreads; i++)
		{
			threads.emplace_back(std::thread(&OptimiserThreadPoolListsOfLists::worker, this, i, &worker_Locs[i], &fillListTask_Locs[i]));
		}
	}

	void fillList(vmath::vec3* pointsBuffer, int* gridBuffer, vmath::vec3 offset_vec)
	{
		this->pointsBuffer = pointsBuffer;
		this->gridBuffer = gridBuffer;
		this->offset_vec = offset_vec;
		//fillListTask();
		fillListTask_Locs[0] = (true);
	}

	void resetWorkers()
	{
		for (int i = 0; i < nThreads; i++)
		{
			worker_Locs[i].store(false);
		}
	}

	void resetfillListTasks()
	{
		for (int i = 0; i < nThreads; i++)
		{
			fillListTask_Locs[i] = (false);
		}
	}

	bool is_ready()
	{
		for (int i = 0; i < nThreads; i++)
		{
			if (worker_Locs[i].load() == true)return false;
		}
		return true;
	}

	void join_Workers()
	{
		//DBOUT("\n \t\t Joining");
		for (int i = 0; i < nThreads; i++)
		{
			while (fillListTask_Locs[i].load() == true);
			while (worker_Locs[i].load() == true);
		}
	}

private:
	void worker(int id, std::atomic<bool>* my_Lock, std::atomic<bool>* my_fillListTaskLock)
	{
		while (true)
		{
			while ((*my_Lock).load() == false)
			{
				if ((*my_fillListTaskLock) == true)
				{
					fillListTask();
					(*my_fillListTaskLock) = (false);
				}
			}

			GridListKernelParall(id, parameters[id].firstPoint, parameters[id].nPoints);


			(*my_Lock).store(false);
		}
	}

	void fillListTask()
	{
		*lastUsedInd = (gridSize)-1;

		//for(int i=0;i<gridSize*2;i++)gridBuffer[i]=0;//init buffer
		memset(gridBuffer, 0, gridSize * 2 * sizeof(int));
		int pointsPerKernel = pointsCount / (nThreads);
		int pointsPerKernelRem = pointsCount % nThreads;

		//DBOUT("\n");
		//resetWorkers();
		for (int i = 0; i < nThreads; i++)
		{
			int nPoints = pointsPerKernel;
			if ((i) == (nThreads - 1))nPoints += pointsPerKernelRem;
			//	v.emplace_back(std::thread(&GridListsOfListsParralOptimiser::GridListKernelParall,this,i,pointsPerKernel*i,nPoints));
			//	GridListKernelParall(i,pointsPerKernel*i, nPoints);
			parameters[i].firstPoint = pointsPerKernel * i;
			parameters[i].nPoints = nPoints;
			worker_Locs[i] = (true);
		}
	}

	std::atomic<bool>* worker_Locs;
	std::atomic<bool>* fillListTask_Locs;
	int nThreads;

	std::vector<std::thread> threads;

	struct params
	{
		int firstPoint;
		int nPoints;
	}*parameters;
};

class ArraysFromListsAllIndsPerCellThreadPoolOptimiser :public GridListsOfListsParralOptimiser
{
public:
	ArraysFromListsAllIndsPerCellThreadPoolOptimiser(int pointsCount, int gridSide, float cellSize, float offset, int nThreads) : GridListsOfListsParralOptimiser(pointsCount, gridSide, cellSize, offset, nThreads)
	{
		this->nThreads = nThreads;
		worker_Locs = new std::atomic<bool>[nThreads];
		fillListTask_Locs = new std::atomic<bool>[nThreads];
		gridListKernelParall_Locs = new std::atomic<bool>[nThreads];
		gridBuffer = new int[pointsCount * 2 + gridSize * 2];
		resetWorkers();
		resetfillListTasks();
		parameters = new params[nThreads];
		for (int i = 0; i < nThreads; i++)
		{
			threads.emplace_back(std::thread(&ArraysFromListsAllIndsPerCellThreadPoolOptimiser::worker, this, i, &worker_Locs[i], &fillListTask_Locs[i], &gridListKernelParall_Locs[i]));
		}
		initNeighbourIndsCache();
	}

	std::chrono::high_resolution_clock::time_point t1;

	void fillList(vmath::vec3* pointsBuffer, int* gridBuffer, vmath::vec3 offset_vec)
	{
		t1 = std::chrono::high_resolution_clock::now();

		this->pointsBuffer = pointsBuffer;
		//this->gridBuffer = gridBuffer;
		outGridBuffer = gridBuffer;

		this->offset_vec = offset_vec;
		//fillListTask();
		fillListTask_Locs[0] = (true);
	}

	void resetWorkers()
	{
		for (int i = 0; i < nThreads; i++)
		{
			worker_Locs[i].store(false);
			gridListKernelParall_Locs[i].store(false);
		}
	}

	void resetfillListTasks()
	{
		for (int i = 0; i < nThreads; i++)
		{
			fillListTask_Locs[i] = (false);
		}
	}

	bool is_ready()
	{
		for (int i = 0; i < nThreads; i++)
		{
			if (worker_Locs[i].load() == true)return false;
		}
		return true;
	}

	void join_Workers()
	{
		//DBOUT("\n \t\t Joining");		
		for (int i = 0; i < nThreads; i++)
		{
			while (fillListTask_Locs[i].load() == true);
			while (worker_Locs[i].load() == true);
		}
	}

private:
	void printList(std::string fileName, int* gridBuffer)
	{
		std::ofstream myfile;
		myfile.open(fileName);

		for (int i = 0; i < (GRID_SIDE * GRID_SIDE * GRID_SIDE * 2 + POINTS_TOTAL * 2); i++)
		{
			myfile << "\n" << i << ":\t" << gridBuffer[i];
		}
		myfile.close();
	}

	void worker(int id, std::atomic<bool>* my_Lock, std::atomic<bool>* my_fillListTaskLock, std::atomic<bool>* my_gridListKernelParallLock)
	{
		while (true)
		{
			while ((*my_Lock).load() == false)
			{
				if ((*my_fillListTaskLock) == true)
				{
					fillListTask(); // only one thread does this initialization job . 
				}
			}

			GridListKernelParall(id, parameters[id].firstPoint, parameters[id].nPoints); /// every thread doeas its own part of this job
			(*my_gridListKernelParallLock).store(true);

			if ((*my_fillListTaskLock) == true) // only one thread does this job after all are finished with GridListKernelParall job
			{
				for (int i = 0; i < nThreads; i++)while (gridListKernelParall_Locs[i].load() == false);// wait other threads to finish
				gridArraysAllIndsPerCellFromListKernel();
				optTime = (std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - t1).count() / 1000000.0);
				(*my_fillListTaskLock) = (false);
			}
			(*my_Lock).store(false);
		}
	}
	void initNeighbourIndsCache()
	{
		for (int i = 0; i < GRID_SIDE*GRID_SIDE*GRID_SIDE; i++){			
			neighbourIndCache[i] = genNeighbourInds(i);
		}
	}
	static std::vector<int> genNeighbourIndsOld(int i, const int GRID_SIDE = GRID_SIDE)
	{
		std::vector<int> inds{};
		//for (int i = 0; i < GRID_SIDE*GRID_SIDE*GRID_SIDE; i++)inds.push_back(i);
		//return inds;

		int indss[3 * 3 * 3];
		int ii = 0;
		indss[ii++] = i;
		indss[ii++] = i + GRID_SIDE;
		indss[ii++] = i - GRID_SIDE;
		indss[ii++] = i + 1;
		indss[ii++] = i - 1;
		indss[ii++] = i + GRID_SIDE + 1;
		indss[ii++] = i - (GRID_SIDE + 1);
		indss[ii++] = i + GRID_SIDE - 1;
		indss[ii++] = i - (GRID_SIDE - 1);
		i += GRID_SIDE*GRID_SIDE;
		indss[ii++] = i;
		indss[ii++] = i + GRID_SIDE;
		indss[ii++] = i - GRID_SIDE;
		indss[ii++] = i + 1;
		indss[ii++] = i - 1;
		indss[ii++] = i + GRID_SIDE + 1;
		indss[ii++] = i - (GRID_SIDE + 1);
		indss[ii++] = i + GRID_SIDE - 1;
		indss[ii++] = i - (GRID_SIDE - 1);
		i -= 2 * GRID_SIDE*GRID_SIDE;
		indss[ii++] = i;
		indss[ii++] = i + GRID_SIDE;
		indss[ii++] = i - GRID_SIDE;
		indss[ii++] = i + 1;
		indss[ii++] = i - 1;
		indss[ii++] = i + GRID_SIDE + 1;
		indss[ii++] = i - (GRID_SIDE + 1);
		indss[ii++] = i + GRID_SIDE - 1;
		indss[ii++] = i - (GRID_SIDE - 1);

		for (int i = 0; i < 3 * 3 * 3; i++)
		{
			if (indss[i] >= 0 && indss[i]<GRID_SIDE*GRID_SIDE*GRID_SIDE)
				inds.push_back(indss[i]);
		}

		return inds;
	}
	inline static  int pointToIndex(ivec3* point)
	{
		int cellIndex = 0;
		cellIndex = std::max(std::min(((((*point)[2]) )) * GRID_SIDE * GRID_SIDE, GRID_SIDE * GRID_SIDE * (GRID_SIDE - 1)), 0);
		cellIndex += std::max(std::min(((((*point)[1]) )) * GRID_SIDE, GRID_SIDE * (GRID_SIDE - 1)), 0);
		cellIndex += std::max(std::min(((((*point)[0]) )), (GRID_SIDE - 1)), 0);
		return cellIndex;
	}
	static ivec3 indexToPoint(int i, const int GRID_SIDE=GRID_SIDE)
	{
		int x, y, z;
		x = i % GRID_SIDE;
		y = (i / GRID_SIDE) % (GRID_SIDE);
		z = (i / (GRID_SIDE * GRID_SIDE)) % (GRID_SIDE);
		return ivec3(x, y, z );
	}
	std::vector<int> genNeighbourInds(int i)
	{
		std::vector<int> inds{};
				
		auto point = indexToPoint(i);

		//TODO Ima nqkyw byg. 
	
		int seta[GRID_SIDE*GRID_SIDE*GRID_SIDE]{0};
		for (size_t j = 0; j < 27; j++)// all neigbouring points
		{
			seta[pointToIndex(new ivec3(point + ivec3(-1, -1, -1) + indexToPoint(j,3)))] = 1;
		}
		/*
		int ofs = 1;
		seta[pointToIndex(new ivec3(point + ivec3(0, 0, 0)))] = 1;
		seta[pointToIndex(new ivec3(point + ivec3(0, -ofs, ofs)))] = 1;
		seta[pointToIndex(new ivec3(point + ivec3(0, -ofs, -ofs)))] = 1;
		seta[pointToIndex(new ivec3(point + ivec3(0, -ofs, 0)))] = 1;
		seta[pointToIndex(new ivec3(point + ivec3(0, ofs, ofs)))] = 1;
		seta[pointToIndex(new ivec3(point + ivec3(0, ofs, -ofs)))] = 1;
		seta[pointToIndex(new ivec3(point + ivec3(0, ofs, 0)))] = 1;
		seta[pointToIndex(new ivec3(point + ivec3(0, 0, ofs)))] = 1;
		seta[pointToIndex(new ivec3(point + ivec3(0, 0, -ofs)))] = 1;

		seta[pointToIndex(new ivec3(point + ivec3(ofs, 0, 0)))] = 1;
		seta[pointToIndex(new ivec3(point + ivec3(ofs, -ofs, ofs)))] = 1;
		seta[pointToIndex(new ivec3(point + ivec3(ofs, -ofs, -ofs)))] = 1;
		seta[pointToIndex(new ivec3(point + ivec3(ofs, -ofs, 0)))] = 1;
		seta[pointToIndex(new ivec3(point + ivec3(ofs, ofs, ofs)))] = 1;
		seta[pointToIndex(new ivec3(point + ivec3(ofs, ofs, -ofs)))] = 1;
		seta[pointToIndex(new ivec3(point + ivec3(ofs, ofs, 0)))] = 1;
		seta[pointToIndex(new ivec3(point + ivec3(ofs, 0, ofs)))] = 1;
		seta[pointToIndex(new ivec3(point + ivec3(ofs, 0, -ofs)))] = 1;

		seta[pointToIndex(new ivec3(point + ivec3(-ofs, 0, 0)))] = 1;
		seta[pointToIndex(new ivec3(point + ivec3(-ofs, -ofs, ofs)))] = 1;
		seta[pointToIndex(new ivec3(point + ivec3(-ofs, -ofs, -ofs)))] = 1;
		seta[pointToIndex(new ivec3(point + ivec3(-ofs, -ofs, 0)))] = 1;
		seta[pointToIndex(new ivec3(point + ivec3(-ofs, ofs, ofs)))] = 1;
		seta[pointToIndex(new ivec3(point + ivec3(-ofs, ofs, -ofs)))] = 1;
		seta[pointToIndex(new ivec3(point + ivec3(-ofs, ofs, 0)))] = 1;
		seta[pointToIndex(new ivec3(point + ivec3(-ofs, 0, ofs)))] = 1;
		seta[pointToIndex(new ivec3(point + ivec3(-ofs, 0, -ofs)))] = 1;
		*/
		for (int i = 0; i < GRID_SIDE*GRID_SIDE*GRID_SIDE; i++)
		{
			if (seta[i])
				inds.push_back(i);
		}

		return inds;
	}

	void gridArraysAllIndsPerCellFromListKernel()
	{
		int lastUsedIndex = gridSize * 2;
		for (int curInd = 0; curInd < gridSize; curInd++)
		{
			//calculate new array size : sum of all neighbouring cells sizes
			auto neighbours = &neighbourIndCache[curInd];// genNeighbourInds(curInd);			
			//auto neighbours1 = genNeighbourInds(curInd); auto neighbours = &neighbours1;

			int curCelSize = 0;
			for (auto neighbourCellInd : *neighbours)
			{
				curCelSize += gridBuffer[neighbourCellInd * 2 + 1];
			}

			//prepare new array
			int curentParticlePosIndex = lastUsedIndex; //new array position
			lastUsedIndex += curCelSize + 1;			//update new free memory position

			outGridBuffer[curentParticlePosIndex + curCelSize + 1] = 0;// null terminate last position of new array
			outGridBuffer[curInd * 2] = curentParticlePosIndex;//new array position
			outGridBuffer[curInd * 2 + 1] = curCelSize;//size of new array

			//fill indices from neighbours
			for (auto neighbourCellInd : *neighbours)
			{
				int curentIteratingIndex = gridBuffer[neighbourCellInd * 2];
				while (curentIteratingIndex != 0)
				{
					outGridBuffer[curentParticlePosIndex++] = gridBuffer[curentIteratingIndex];
					curentIteratingIndex = gridBuffer[curentIteratingIndex + 1];
				}
			}
			//std::sort(outGridBuffer + outGridBuffer[curInd * 2], outGridBuffer + outGridBuffer[curInd * 2] + outGridBuffer[curInd * 2 + 1],[](const int & a, const int & b) -> bool{return a < b;});
		}
	}

	void fillListTask()
	{
		*lastUsedInd = (gridSize)-1;

		//for(int i=0;i<gridSize*2;i++)gridBuffer[i]=0;//init buffer
		memset(gridBuffer, 0, gridSize * 2 * sizeof(int));
		int pointsPerKernel = pointsCount / (nThreads);
		int pointsPerKernelRem = pointsCount % nThreads;

		//DBOUT("\n");
		//resetWorkers();
		for (int i = 0; i < nThreads; i++)
		{
			int nPoints = pointsPerKernel;
			if ((i) == (nThreads - 1))nPoints += pointsPerKernelRem;
			//	v.emplace_back(std::thread(&GridListsOfListsParralOptimiser::GridListKernelParall,this,i,pointsPerKernel*i,nPoints));
			//	GridListKernelParall(i,pointsPerKernel*i, nPoints);
			parameters[i].firstPoint = pointsPerKernel * i;
			parameters[i].nPoints = nPoints;
			gridListKernelParall_Locs[i].store(false);
			worker_Locs[i] = (true);
		}
	}

	std::vector<int> neighbourIndCache[GRID_SIDE*GRID_SIDE*GRID_SIDE];
	std::atomic<bool>* worker_Locs;
	std::atomic<bool>* gridListKernelParall_Locs;
	std::atomic<bool>* fillListTask_Locs;
	int nThreads;
	int* outGridBuffer;
	std::vector<std::thread> threads;

	struct params
	{
		int firstPoint;
		int nPoints;
	}*parameters;
};
//Last Variant 
class ArraysFromListsAllDataPerCellThreadPoolOptimiser :public GridListsOfListsParralOptimiser
{
public:
	ArraysFromListsAllDataPerCellThreadPoolOptimiser(int pointsCount, int gridSide, float cellSize, float offset, int nThreads) : GridListsOfListsParralOptimiser(pointsCount, gridSide, cellSize, offset, nThreads)
	{
		this->nThreads = nThreads;
		worker_Locs = new std::atomic<bool>[nThreads];
		fillListTask_Locs = new std::atomic<bool>[nThreads];
		gridListKernelParall_Locs = new std::atomic<bool>[nThreads];
		gridBuffer = new int[pointsCount * 2 + gridSize * 2];
		resetWorkers();
		resetfillListTasks();
		parameters = new params[nThreads];
		for (int i = 0; i < nThreads; i++)
		{
			threads.emplace_back(std::thread(&ArraysFromListsAllDataPerCellThreadPoolOptimiser::worker, this, i, &worker_Locs[i], &fillListTask_Locs[i], &gridListKernelParall_Locs[i]));
		}
		initNeighbourIndsCache();
	}

	std::chrono::high_resolution_clock::time_point t1;

	void fillList(vmath::vec3* pointsBuffer, vec4* velocityBuffer, vec2* density_pBuffer, vmath::vec3 offset_vec, int* gridBuffer, float* gridDataBuffer)
	{
		t1 = std::chrono::high_resolution_clock::now();

		this->pointsBuffer = pointsBuffer;		
		this->outGridBuffer = gridBuffer;
		this->outGridDataBuffer = gridDataBuffer;
		this->velocityBuffer = velocityBuffer;
		this->density_pBuffer = density_pBuffer;

		this->offset_vec = offset_vec;
		//fillListTask();
		fillListTask_Locs[0] = (true);
	}

	void resetWorkers()
	{
		for (int i = 0; i < nThreads; i++)
		{
			worker_Locs[i].store(false);
			gridListKernelParall_Locs[i].store(false);
		}
	}

	void resetfillListTasks()
	{
		for (int i = 0; i < nThreads; i++)
		{
			fillListTask_Locs[i] = (false);
		}
	}

	bool is_ready()
	{
		for (int i = 0; i < nThreads; i++)
		{
			if (worker_Locs[i].load() == true)return false;
		}
		return true;
	}

	void join_Workers()
	{
		//DBOUT("\n \t\t Joining");		
		for (int i = 0; i < nThreads; i++)
		{
			while (fillListTask_Locs[i].load() == true);
			while (worker_Locs[i].load() == true);
		}
	}

private:
	void printList(std::string fileName, int* gridBuffer)
	{
		std::ofstream myfile;
		myfile.open(fileName);

		for (int i = 0; i < (GRID_SIDE * GRID_SIDE * GRID_SIDE * 2 + POINTS_TOTAL * 2); i++)
		{
			myfile << "\n" << i << ":\t" << gridBuffer[i];
		}
		myfile.close();
	}
	template<typename T,int len>
	void printArrayVectors(std::string fileName, vecN<T,len>* gridBuffer,int length)
	{
		std::ofstream myfile;
		myfile.open(fileName);

		for (int i = 0; i < length; i++) // sizeof(gridBuffer) / gridBuffer[0]
		{
			myfile << "\n" << i<<":" ;
			for (int j = 0; j < len; j++)
			{
				myfile <<"\t" << gridBuffer[i][j];
			}
		}
		myfile.close();
	}
	template<typename T>
	void printArray(std::string fileName, T* gridBuffer, int length,int stride)
	{
		std::ofstream myfile;
		myfile.open(fileName);

		for (int i = 0; i < length; i++) // sizeof(gridBuffer) / gridBuffer[0]
		{
			myfile << "\n" << i << ":";
			for (int j = 0; j < stride; j++)
			{
				myfile << "\t" << gridBuffer[i+j];
			}
		}
		myfile.close();
	}


	void worker(int id, std::atomic<bool>* my_Lock, std::atomic<bool>* my_fillListTaskLock, std::atomic<bool>* my_gridListKernelParallLock)
	{
		while (true)
		{
			while ((*my_Lock).load() == false)
			{
				if ((*my_fillListTaskLock) == true)
				{
					fillListTask(); // only one thread does this initialization job . 
				}
			}

			GridListKernelParall(id, parameters[id].firstPoint, parameters[id].nPoints); /// every thread doeas its own part of this job
			(*my_gridListKernelParallLock).store(true);

			if ((*my_fillListTaskLock) == true) // only one thread does this job after all are finished with GridListKernelParall job
			{
				for (int i = 0; i < nThreads; i++)while (gridListKernelParall_Locs[i].load() == false);// wait other threads to finish
				gridArraysAllIndsPerCellFromListKernel();
				optTime = (std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - t1).count() / 1000000.0);
				(*my_fillListTaskLock) = (false);
			}
			(*my_Lock).store(false);
		}
	}
	void initNeighbourIndsCache()
	{
		for (int i = 0; i < GRID_SIDE*GRID_SIDE*GRID_SIDE; i++){
			neighbourIndCache[i] = genNeighbourInds(i);
		}
	}
	static std::vector<int> genNeighbourIndsOld(int i, const int GRID_SIDE = GRID_SIDE)
	{
		std::vector<int> inds{};
		//for (int i = 0; i < GRID_SIDE*GRID_SIDE*GRID_SIDE; i++)inds.push_back(i);
		//return inds;

		int indss[3 * 3 * 3];
		int ii = 0;
		indss[ii++] = i;
		indss[ii++] = i + GRID_SIDE;
		indss[ii++] = i - GRID_SIDE;
		indss[ii++] = i + 1;
		indss[ii++] = i - 1;
		indss[ii++] = i + GRID_SIDE + 1;
		indss[ii++] = i - (GRID_SIDE + 1);
		indss[ii++] = i + GRID_SIDE - 1;
		indss[ii++] = i - (GRID_SIDE - 1);
		i += GRID_SIDE*GRID_SIDE;
		indss[ii++] = i;
		indss[ii++] = i + GRID_SIDE;
		indss[ii++] = i - GRID_SIDE;
		indss[ii++] = i + 1;
		indss[ii++] = i - 1;
		indss[ii++] = i + GRID_SIDE + 1;
		indss[ii++] = i - (GRID_SIDE + 1);
		indss[ii++] = i + GRID_SIDE - 1;
		indss[ii++] = i - (GRID_SIDE - 1);
		i -= 2 * GRID_SIDE*GRID_SIDE;
		indss[ii++] = i;
		indss[ii++] = i + GRID_SIDE;
		indss[ii++] = i - GRID_SIDE;
		indss[ii++] = i + 1;
		indss[ii++] = i - 1;
		indss[ii++] = i + GRID_SIDE + 1;
		indss[ii++] = i - (GRID_SIDE + 1);
		indss[ii++] = i + GRID_SIDE - 1;
		indss[ii++] = i - (GRID_SIDE - 1);

		for (int i = 0; i < 3 * 3 * 3; i++)
		{
			if (indss[i] >= 0 && indss[i]<GRID_SIDE*GRID_SIDE*GRID_SIDE)
				inds.push_back(indss[i]);
		}

		return inds;
	}
	inline static  int pointToIndex(ivec3* point)
	{
		int cellIndex = 0;
		cellIndex = std::max(std::min(((((*point)[2]))) * GRID_SIDE * GRID_SIDE, GRID_SIDE * GRID_SIDE * (GRID_SIDE - 1)), 0);
		cellIndex += std::max(std::min(((((*point)[1]))) * GRID_SIDE, GRID_SIDE * (GRID_SIDE - 1)), 0);
		cellIndex += std::max(std::min(((((*point)[0]))), (GRID_SIDE - 1)), 0);
		return cellIndex;
	}
	static ivec3 indexToPoint(int i, const int GRID_SIDE = GRID_SIDE)
	{
		int x, y, z;
		x = i % GRID_SIDE;
		y = (i / GRID_SIDE) % (GRID_SIDE);
		z = (i / (GRID_SIDE * GRID_SIDE)) % (GRID_SIDE);
		return ivec3(x, y, z);
	}
	std::vector<int> genNeighbourInds(int i)
	{
		std::vector<int> inds{};

		auto point = indexToPoint(i);

		

		int seta[GRID_SIDE*GRID_SIDE*GRID_SIDE]{0};
		for (size_t j = 0; j < 27; j++)// all neigbouring points
		{
			seta[pointToIndex(new ivec3(point + ivec3(-1, -1, -1) + indexToPoint(j, 3)))] = 1;
		}
	
		for (int i = 0; i < GRID_SIDE*GRID_SIDE*GRID_SIDE; i++)
		{
			if (seta[i])
				inds.push_back(i);
		}

		return inds;
	}
	void gridArraysAllIndsPerCellFromListKernel()
	{
		int lastUsedDataIndex = 0;
			int lastUsedIndex = gridSize * 3;
		for (int curInd = 0; curInd < gridSize; curInd++)
		{
			//calculate new array size : sum of all neighbouring cells sizes
			auto neighbours = &neighbourIndCache[curInd];// genNeighbourInds(curInd);			

			int curCelSize = 0;
			for (auto neighbourCellInd : *neighbours)
			{
				curCelSize += gridBuffer[neighbourCellInd * 2 + 1];
			}
			int curCellDataSize = curCelSize*10;

			//prepare new array
			int curentParticleDataPosIndex = lastUsedDataIndex; //new array position
			lastUsedDataIndex += curCellDataSize + 1;			//update new free memory position

				int curentParticlePosIndex = lastUsedIndex; //new array position
				lastUsedIndex += curCelSize + 1;			//update new free memory position

			outGridDataBuffer[curentParticleDataPosIndex + curCellDataSize + 1] = 0;

			outGridBuffer[curentParticlePosIndex + curCelSize + 1] = 0;
			outGridBuffer[curInd * 3] = curentParticlePosIndex ;//new array position
			outGridBuffer[curInd * 3 + 1] = curCelSize;//size of new array
			outGridBuffer[curInd * 3 + 2] = curentParticleDataPosIndex;//pointer to data starting in outGridDataBuffer[outGridBuffer[curInd * 3 + 2]]

			//fill indices from neighbours
			for (auto neighbourCellInd : *neighbours)
			{
				int curentIteratingIndex = gridBuffer[neighbourCellInd * 2];
				while (curentIteratingIndex != 0)
				{
					int index = gridBuffer[curentIteratingIndex];// particle ind
					outGridBuffer[curentParticlePosIndex++] = index;

					curentIteratingIndex = gridBuffer[curentIteratingIndex + 1];//move to next (ind,next)pair
					outGridDataBuffer[curentParticleDataPosIndex + 0] = this->pointsBuffer[index][0];
					outGridDataBuffer[curentParticleDataPosIndex + 1] = this->pointsBuffer[index][1];
					outGridDataBuffer[curentParticleDataPosIndex + 2] = this->pointsBuffer[index][2];

					outGridDataBuffer[curentParticleDataPosIndex + 3] = this->velocityBuffer[index][0];// TODO memcpy probably faster
					outGridDataBuffer[curentParticleDataPosIndex + 4] = this->velocityBuffer[index][1];
					outGridDataBuffer[curentParticleDataPosIndex + 5] = this->velocityBuffer[index][2];
					outGridDataBuffer[curentParticleDataPosIndex + 6] = this->velocityBuffer[index][4];

					outGridDataBuffer[curentParticleDataPosIndex + 7] = this->density_pBuffer[index][0];
					outGridDataBuffer[curentParticleDataPosIndex + 8] = this->density_pBuffer[index][1];
					outGridDataBuffer[curentParticleDataPosIndex + 9] = index;
					curentParticleDataPosIndex += 10;

				}
			}
		}
		//printArrayVectors("c:\\pointsBuffer.txt",pointsBuffer,POINTS_TOTAL);
		//printArray("c:\\outGridDataBuffer.txt", outGridDataBuffer, lastUsedDataIndex,9);
	}

	void fillListTask()
	{
		*lastUsedInd = (gridSize)-1;

		//for(int i=0;i<gridSize*2;i++)gridBuffer[i]=0;//init buffer
		memset(gridBuffer, 0, gridSize * 2 * sizeof(int));
		int pointsPerKernel = pointsCount / (nThreads);
		int pointsPerKernelRem = pointsCount % nThreads;

		//DBOUT("\n");
		//resetWorkers();
		for (int i = 0; i < nThreads; i++)
		{
			int nPoints = pointsPerKernel;
			if ((i) == (nThreads - 1))nPoints += pointsPerKernelRem;
			//	v.emplace_back(std::thread(&GridListsOfListsParralOptimiser::GridListKernelParall,this,i,pointsPerKernel*i,nPoints));
			//	GridListKernelParall(i,pointsPerKernel*i, nPoints);
			parameters[i].firstPoint = pointsPerKernel * i;
			parameters[i].nPoints = nPoints;
			gridListKernelParall_Locs[i].store(false);
			worker_Locs[i] = (true);
		}
	}

	std::vector<int> neighbourIndCache[GRID_SIDE*GRID_SIDE*GRID_SIDE];
	std::atomic<bool>* worker_Locs;
	std::atomic<bool>* gridListKernelParall_Locs;
	std::atomic<bool>* fillListTask_Locs;
	int nThreads;
	int* outGridBuffer;
	float* outGridDataBuffer;
	vec4* velocityBuffer;
	vec2* density_pBuffer;
	std::vector<std::thread> threads;

	struct params
	{
		int firstPoint;
		int nPoints;
	}*parameters;
};


class springmass_app : public sb7::application
{
public:
	//GridListsOfListsOptimiser* gridOptimiser;
	GridOptimiser* gridOptimiser;
	sb7::text_overlay overlay;

	springmass_app()
		: m_iteration_index(0),
		m_update_program(0),
		m_render_program(0),
		m_render_mask_program(0),
		m_render_fluid_raytrace_program(0),
		m_fill_DensityField_program(0),
		render_fluid_ray_trace_vao(0),
		render_balls_vao(0),
		m_render_wind_program(0),
		rayTraceCube_buffer(0),
		balls_buffer(0),
		draw_points(true),
		draw_raytrace(false),
		draw_skybox(true),
		iterations_per_frame(1)
	{
	}

	void init()
	{
		static const char title[] = "Asen OpenGl 4.3 Fluid SPH Simulation";

		sb7::application::init();
		info.windowHeight = 600;
		info.windowWidth = 600;
		int m = info.majorVersion;
		int mi = info.minorVersion;
		memcpy(info.title, title, sizeof(title));
		keyBuffer.e = false;
		keyBuffer.q = false;
		keyBuffer.w = false;
	}

	void init_user_transforms()
	{
		wind_speed = 1.80f;
		wind_dir = vmath::vec3(0.0f, -1.0f, 0.0f);
		glass_pos = vmath::vec3(0.0f, 0.0f, 0.0f);
		sphere1 = vmath::vec4(0.0f, 2.0f, 0.0f, 2);
		sphere1_CurentInterpol = vmath::vec4(sphere1);
		sphere1_speed = vmath::vec3(0.0f, 0.0f, 0.0f);
		light_pos = vmath::vec3(10.0f, 40.0f, 30.0f);
		m_matrix = vmath::mat4::identity();
		v_matrix = vmath::lookat(vmath::vec3(0.0, 0.0, 50), vmath::vec3(0.0, 0.0, 0.0), vmath::vec3(0.0, 1.0, 0.0));
	}

	void startup(void)
	{
		glGenQueries(2, timerQueries);
#if   (OPTIM_STRUCT ==1)
		gridOptimiser =new GridListsOfArraysOptimiser(POINTS_TOTAL,GRID_SIDE,CELL_SIZE,GRID_OFFSET,1);		
#elif (OPTIM_STRUCT ==2)
		gridOptimiser =new GridListsOfListsOptimiser(POINTS_TOTAL,GRID_SIDE,CELL_SIZE,GRID_OFFSET);        
#elif (OPTIM_STRUCT ==3)
		gridOptimiser =new GridListsOfListsParralOptimiser(POINTS_TOTAL,GRID_SIDE,CELL_SIZE,GRID_OFFSET,1);	
#elif (OPTIM_STRUCT ==4)
		gridOptimiser =new OptimiserThreadPoolListsOfLists(POINTS_TOTAL,GRID_SIDE,CELL_SIZE,GRID_OFFSET,1);	
#elif (OPTIM_STRUCT ==5)
		gridOptimiser = new ArraysFromListsThreadPoolOptimiser(POINTS_TOTAL, GRID_SIDE, CELL_SIZE, GRID_OFFSET, 1);
#elif (OPTIM_STRUCT ==6)
		gridOptimiser = new ArraysFromListsAllIndsPerCellThreadPoolOptimiser(POINTS_TOTAL, GRID_SIDE, CELL_SIZE, GRID_OFFSET, 1);
#elif (OPTIM_STRUCT ==7)
		gridOptimiser = new ArraysFromListsAllDataPerCellThreadPoolOptimiser(POINTS_TOTAL, GRID_SIDE, CELL_SIZE, GRID_OFFSET, 1);

#endif

		int i, j, k;
		lastTime = 0;
		frames = 0;

		load_shaders();
		keyBuffer.q = 0;
		keyBuffer.w = 0;
		draging = 0;
		rotating = 0;
		looking = 0;
		init_user_transforms();
		proj_matrix = vmath::perspective(50.0f, (float)info.windowWidth / (float)info.windowHeight, 0.1f, 1000.0f);

		atractC = vmath::vec3(105, 0.1, 0.001);
		repulsC = vmath::vec3(99, 0.1, 0.001);

		vmath::vec3* initial_positions = new vmath::vec3[POINTS_TOTAL];
		vmath::vec4* initial_velocities = new vmath::vec4[POINTS_TOTAL];
		vmath::Tvec4<char>* initial_colors = new vmath::Tvec4<char>[POINTS_TOTAL];

		vmath::vec2* initial_density_pressure = new vmath::vec2[POINTS_TOTAL];
		vmath::vec2* tex_coords = new vmath::vec2[POINTS_TOTAL];
		int n = 0;
		vmath::vec4 vel = vmath::vec4(0.0, 1.0, 0.0, 1.0);
		for (k = 0; k < POINTS_Z; k++)
		{
			float fk = (float)k / (float)POINTS_Z;
			vel = vel * vmath::rotate(360 / (float)POINTS_Z, 0.0f, 0.0f, 1.0f);
			for (j = 0; j < POINTS_Y; j++)
			{
				float fj = (float)j / (float)POINTS_Y;
				for (i = 0; i < POINTS_X; i++)
				{
					float fi = (float)i / (float)POINTS_X;

					initial_positions[n] = vmath::vec3(
						(fi - 0.5f) * (float)POINTS_X * 0.5,
						(fj - 0.5f) * (float)POINTS_Y * 0.5,
						(fk - 0.5f) * (float)POINTS_Z * 0.5);


					//initial_velocities[n] = vmath::vec4(vel[0],vel[1],vel[2],1/25.0)*25;
					const float mas = i < 7.5 ? 1 : 1;
					initial_velocities[n] = i & 2 ? vmath::vec4(0, 0, 0, mas) : vmath::vec4(0, 0, 0, mas);
					initial_colors[n] = i < 7.5 ? vmath::Tvec4<char>(255, 0, 0, 255) : vmath::Tvec4<char>(0, 0, 255, 255);
					initial_density_pressure[n] = vmath::vec2(1, 1);
					n++;
				}
			}
		}


#if (INT_STATE_FROM_FILE== 1)
		std::ifstream myfile;
		myfile.open("initParticlesState151515.dat", std::ifstream::binary);

		myfile.read((char*)initial_positions, sizeof(vmath::vec3) * POINTS_TOTAL);
		myfile.read((char*)initial_velocities, sizeof(vmath::vec4) * POINTS_TOTAL);
		myfile.read((char*)initial_density_pressure, sizeof(vmath::vec2) * POINTS_TOTAL);

		if (myfile)
			DBOUT("\nSiccessful read of file init Particle State")
		else
		DBOUT("\FAIL ERROR read of file init Particle State");
		myfile.close();
#endif

		glGenVertexArrays(1, &wind_dir_vao);
		glBindVertexArray(wind_dir_vao);

		glGenBuffers(1, &wind_pointer_buff);
		glBindBuffer(GL_ARRAY_BUFFER, wind_pointer_buff);
		glBufferData(GL_ARRAY_BUFFER, 4 * 3 * 2, NULL, GL_STATIC_DRAW);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
		glEnableVertexAttribArray(0);

		glBindVertexArray(0);


		glGenVertexArrays(2, m_vao);
		glGenBuffers(10, m_vbo);

		glBindBuffer(GL_ARRAY_BUFFER, m_vbo[COLORS]);
		glBufferData(GL_ARRAY_BUFFER, POINTS_TOTAL * sizeof(vmath::Tvec4<char>), initial_colors, GL_DYNAMIC_COPY);


		for (i = 0; i < 2; i++)
		{
			glBindVertexArray(m_vao[i]);

			glBindBuffer(GL_ARRAY_BUFFER, m_vbo[POSITION_A + i]);

			//    glBufferData(GL_ARRAY_BUFFER, POSITIONS_SIZE, initial_positions, GL_DYNAMIC_COPY);
			glBufferStorage(GL_ARRAY_BUFFER, POSITIONS_SIZE, initial_positions, GL_DYNAMIC_STORAGE_BIT | GL_MAP_READ_BIT | GL_MAP_PERSISTENT_BIT);
			glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, NULL);
			glEnableVertexAttribArray(0);

			glBindBuffer(GL_ARRAY_BUFFER, m_vbo[VELOCITY_A + i]);
			glBufferStorage(GL_ARRAY_BUFFER, POINTS_TOTAL * sizeof(vmath::vec4), initial_velocities, GL_DYNAMIC_STORAGE_BIT | GL_MAP_READ_BIT | GL_MAP_PERSISTENT_BIT);
			//glBufferData(GL_ARRAY_BUFFER, POINTS_TOTAL * sizeof(vmath::vec4), initial_velocities, GL_DYNAMIC_STORAGE_BIT | GL_MAP_READ_BIT | GL_MAP_PERSISTENT_BIT);
			glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 0, NULL);
			glEnableVertexAttribArray(1);

			glBindBuffer(GL_ARRAY_BUFFER, m_vbo[DENSITY_A + i]);
			glBufferStorage(GL_ARRAY_BUFFER, POINTS_TOTAL * sizeof(vmath::vec2), initial_density_pressure, GL_DYNAMIC_STORAGE_BIT | GL_MAP_READ_BIT | GL_MAP_PERSISTENT_BIT);
			//glBufferData(GL_ARRAY_BUFFER, POINTS_TOTAL * sizeof(vmath::vec2), initial_density_pressure, GL_DYNAMIC_STORAGE_BIT | GL_MAP_READ_BIT | GL_MAP_PERSISTENT_BIT);
			glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 0, NULL);
			glEnableVertexAttribArray(2);

			//glBindBuffer(GL_ARRAY_BUFFER, m_vbo[COLORS]);
			//glVertexAttribPointer(3, 4, GL_BYTE, GL_FALSE, 0, NULL);
			//  glEnableVertexAttribArray(3);

#if(OPTIM_STRUCT >0&&OPTIM_STRUCT <4)
			glBindBuffer(GL_ARRAY_BUFFER, m_vbo[GRIDLIST_A + i]);
			//glBufferData(GL_ARRAY_BUFFER, GRIDLIST_SIZE, NULL, GL_DYNAMIC_COPY);
			glBufferStorage(GL_ARRAY_BUFFER, GRIDLIST_SIZE, NULL, GL_DYNAMIC_STORAGE_BIT|GL_MAP_WRITE_BIT|GL_MAP_PERSISTENT_BIT);

			int * gridBuffertmp = (int *)glMapNamedBufferRange(m_vbo[GRIDLIST_A + i], 0, GRIDLIST_SIZE, GL_MAP_WRITE_BIT );
			vmath::vec3 * pointsBuffertmp = (vmath::vec3 *)glMapNamedBufferRange( m_vbo[POSITION_A +i], 0, POSITIONS_SIZE, GL_MAP_READ_BIT);

			gridOptimiser->fillList(pointsBuffertmp,gridBuffertmp,glass_pos);
			//gridOptimiser->printList(pointsBuffer,gridBuffer);std::exit(0); 

			glUnmapNamedBuffer(m_vbo[GRIDLIST_A + i]);
			glUnmapNamedBuffer(m_vbo[POSITION_A + i]);
#endif
		}

		delete[] initial_velocities;
		delete[] initial_positions;
		delete[] tex_coords;
		delete[] initial_colors;
		delete[] initial_density_pressure;

#if(OPTIM_STRUCT ==4||OPTIM_STRUCT ==5||OPTIM_STRUCT ==6)
		glGenBuffers(2, m_grdBufferChunks_vbo);
		glGenTextures(2, m_GRIDLIST_tbo);
		pointsBuffer[0] = (vmath::vec3 *)glMapNamedBufferRange(m_vbo[POSITION_A], 0, POSITIONS_SIZE, GL_MAP_READ_BIT | GL_MAP_PERSISTENT_BIT);
		pointsBuffer[1] = (vmath::vec3 *)glMapNamedBufferRange(m_vbo[POSITION_A + 1], 0, POSITIONS_SIZE, GL_MAP_READ_BIT | GL_MAP_PERSISTENT_BIT);
		for (int i = 0; i < 2; i++)
		{
			fence[i] = 0;
			glBindBuffer(GL_ARRAY_BUFFER, m_grdBufferChunks_vbo[i]);			
			glBufferStorage(GL_ARRAY_BUFFER, GRIDLIST_SIZE, NULL, GL_DYNAMIC_STORAGE_BIT | GL_MAP_WRITE_BIT | GL_MAP_PERSISTENT_BIT);

			int* gridBuffertmp = (int *)glMapNamedBufferRange(m_grdBufferChunks_vbo[i], 0, GRIDLIST_SIZE, GL_MAP_WRITE_BIT | GL_MAP_PERSISTENT_BIT | GL_MAP_FLUSH_EXPLICIT_BIT);

			gridOptimiser->fillList(pointsBuffer[i % 2], gridBuffertmp, glass_pos);
			gridBuffer[i] = gridBuffertmp;

			(gridOptimiser)->join_Workers();
			
			glBindTexture(GL_TEXTURE_BUFFER, m_GRIDLIST_tbo[i]);
			glTexBuffer(GL_TEXTURE_BUFFER, GL_R32I, m_grdBufferChunks_vbo[i]);

		}
#elif OPTIM_STRUCT==7
		glGenBuffers(2, m_grdBufferChunks_vbo);
		glGenTextures(2, m_GRIDLIST_tbo);
		glGenBuffers(2, m_GridDataBufferChunks_vbo);
		glGenTextures(2, m_GRID_DATA_tbo);

		pointsBuffer[0] = (vmath::vec3 *)glMapNamedBufferRange(m_vbo[POSITION_A], 0, POSITIONS_SIZE, GL_MAP_READ_BIT | GL_MAP_PERSISTENT_BIT);
		pointsBuffer[1] = (vmath::vec3 *)glMapNamedBufferRange(m_vbo[POSITION_A + 1], 0, POSITIONS_SIZE, GL_MAP_READ_BIT | GL_MAP_PERSISTENT_BIT);

		velosityBuffer[0] = (vmath::vec4 *)glMapNamedBufferRange(m_vbo[VELOCITY_A], 0, POINTS_TOTAL * sizeof(vmath::vec4), GL_MAP_READ_BIT | GL_MAP_PERSISTENT_BIT);
		velosityBuffer[1] = (vmath::vec4 *)glMapNamedBufferRange(m_vbo[VELOCITY_A + 1], 0, POINTS_TOTAL * sizeof(vmath::vec4), GL_MAP_READ_BIT | GL_MAP_PERSISTENT_BIT);

		density_pBuffer[0] = (vmath::vec2 *)glMapNamedBufferRange(m_vbo[DENSITY_A], 0, POINTS_TOTAL * sizeof(vmath::vec2), GL_MAP_READ_BIT | GL_MAP_PERSISTENT_BIT);
		density_pBuffer[1] = (vmath::vec2 *)glMapNamedBufferRange(m_vbo[DENSITY_A + 1], 0, POINTS_TOTAL * sizeof(vmath::vec2), GL_MAP_READ_BIT | GL_MAP_PERSISTENT_BIT);

		
		for (int i = 0; i < 2; i++)
		{
			fence[i] = 0;
			glBindBuffer(GL_ARRAY_BUFFER, m_grdBufferChunks_vbo[i]);
			glBufferStorage(GL_ARRAY_BUFFER, GRIDLIST_SIZE, NULL, GL_DYNAMIC_STORAGE_BIT | GL_MAP_WRITE_BIT | GL_MAP_PERSISTENT_BIT);
			int* gridBuffertmp = (int *)glMapNamedBufferRange(m_grdBufferChunks_vbo[i], 0, GRIDLIST_SIZE, GL_MAP_WRITE_BIT | GL_MAP_PERSISTENT_BIT | GL_MAP_FLUSH_EXPLICIT_BIT);

			glBindBuffer(GL_ARRAY_BUFFER, m_GridDataBufferChunks_vbo[i]);
			glBufferStorage(GL_ARRAY_BUFFER, GRIDLIST_DATA_SIZE, NULL, GL_DYNAMIC_STORAGE_BIT | GL_MAP_WRITE_BIT | GL_MAP_PERSISTENT_BIT);
			float* gridDataBuffertmp = (float *)glMapNamedBufferRange(m_GridDataBufferChunks_vbo[i], 0, GRIDLIST_DATA_SIZE, GL_MAP_WRITE_BIT | GL_MAP_PERSISTENT_BIT | GL_MAP_FLUSH_EXPLICIT_BIT);


			((ArraysFromListsAllDataPerCellThreadPoolOptimiser*)gridOptimiser)->fillList(pointsBuffer[i % 2], velosityBuffer[i % 2], density_pBuffer[i % 2], glass_pos, gridBuffertmp, gridDataBuffertmp);
			gridBuffer[i] = gridBuffertmp;
			gridDataBuffer[i] = gridDataBuffertmp;

			gridOptimiser->join_Workers();

			glBindTexture(GL_TEXTURE_BUFFER, m_GRIDLIST_tbo[i]);
			glTexBuffer(GL_TEXTURE_BUFFER, GL_R32I, m_grdBufferChunks_vbo[i]);
			glBindTexture(GL_TEXTURE_BUFFER, m_GRID_DATA_tbo[i]);
			glTexBuffer(GL_TEXTURE_BUFFER, GL_R32F, m_GridDataBufferChunks_vbo[i]);
		}
#endif

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

#if(OPTIM_STRUCT >0&&OPTIM_STRUCT <4)
		glGenTextures(2, m_GRIDLIST_tbo);
		glBindTexture(GL_TEXTURE_BUFFER, m_GRIDLIST_tbo[0]);
		glTexBuffer(GL_TEXTURE_BUFFER, GL_R32I, m_vbo[GRIDLIST_A]);
		glBindTexture(GL_TEXTURE_BUFFER, m_GRIDLIST_tbo[1]);
		glTexBuffer(GL_TEXTURE_BUFFER,GL_R32I, m_vbo[GRIDLIST_B]);
#endif
		glGenTextures(2, m_density_tbo);
		glBindTexture(GL_TEXTURE_BUFFER, m_density_tbo[0]);
		glTexBuffer(GL_TEXTURE_BUFFER, GL_RG32F, m_vbo[DENSITY_A]);
		glBindTexture(GL_TEXTURE_BUFFER, m_density_tbo[1]);
		glTexBuffer(GL_TEXTURE_BUFFER, GL_RG32F, m_vbo[DENSITY_B]);

		glGenTextures(1, &m_color_tbo);
		glBindTexture(GL_TEXTURE_BUFFER, m_color_tbo);
		glTexBuffer(GL_TEXTURE_BUFFER, GL_RGBA, m_vbo[COLORS]);


		//	glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
		glEnable(GL_DEPTH_TEST);
		glEnable(GL_CULL_FACE);
		glClearColor(0.1f, 0.3f, 0.4f, 0.0f);
		glEnable(GL_STENCIL_TEST);
		glClearStencil(0);
		init_mask_Buffer();
		init_Ball_Buffer();
		init_RayTraceCube_Buffer();
		init_SkyBox();
		overlay.init(128, 50);
		init_DensityFieldTexture();
	}

	void saveParticles(std::string filename)
	{
		vmath::vec3* initial_positions = new vmath::vec3[POINTS_TOTAL];
		vmath::vec4* initial_velocities = new vmath::vec4[POINTS_TOTAL];
		vmath::vec2* initial_density_pressure = new vmath::vec2[POINTS_TOTAL];

		glGetNamedBufferSubData(m_vbo[POSITION_A], 0, sizeof(vmath::vec3) * POINTS_TOTAL, initial_positions);
		glGetNamedBufferSubData(m_vbo[VELOCITY_A], 0, sizeof(vmath::vec4) * POINTS_TOTAL, initial_velocities);
		glGetNamedBufferSubData(m_vbo[DENSITY_A], 0, sizeof(vmath::vec2) * POINTS_TOTAL, initial_density_pressure);

		std::ofstream myfile;
		myfile.open(filename, std::ofstream::binary);
		myfile.write((char*)initial_positions, sizeof(vmath::vec3) * POINTS_TOTAL);
		myfile.write((char*)initial_velocities, sizeof(vmath::vec4) * POINTS_TOTAL);
		myfile.write((char*)initial_density_pressure, sizeof(vmath::vec2) * POINTS_TOTAL);
		myfile.close();
		delete initial_positions;
		delete initial_velocities;
		delete initial_density_pressure;
	}

	template <class T>
	std::vector<T, std::allocator<T>> vectorFromPointer(T* sourceArray, size_t arraySize)
	{
		std::vector<T, std::allocator<T>> targetVector();
		typename std::_Vector_base<T, std::allocator<T>>::_Vector_impl* vectorPtr =
			(typename std::_Vector_base<T, std::allocator<T>>::_Vector_impl *)((void *)&targetVector);
		vectorPtr->_M_start = sourceArray;
		vectorPtr->_M_finish = vectorPtr->_M_end_of_storage = vectorPtr->_M_start + arraySize;
		return targetVector;
	}

	void isAnyNanInf()
	{
		//std::unique_ptr<vmath::vec3[]> initial_positions(new vmath::vec3[POINTS_TOTAL]);
		//std::unique_ptr<vmath::vec4[]> initial_velocities(new vmath::vec4[POINTS_TOTAL]);
		//std::unique_ptr<vmath::vec2[]> initial_density_pressure(new vmath::vec2[POINTS_TOTAL]);

		std::vector<vmath::vec3> initial_positions(POINTS_TOTAL);
		std::vector<vmath::vec4> initial_velocities(POINTS_TOTAL);
		std::vector<vmath::vec2> initial_density_pressure(POINTS_TOTAL);

		glGetNamedBufferSubData(m_vbo[POSITION_A], 0, sizeof(vmath::vec3) * POINTS_TOTAL, initial_positions.data());
		glGetNamedBufferSubData(m_vbo[VELOCITY_A], 0, sizeof(vmath::vec4) * POINTS_TOTAL, initial_velocities.data());
		glGetNamedBufferSubData(m_vbo[DENSITY_A], 0, sizeof(vmath::vec2) * POINTS_TOTAL, initial_density_pressure.data());

		bool isNan = false;
		for (vmath::vec3 p : initial_positions)
		{
			if (std::isnan(p.x()))
				isNan = true;
			if (std::isnan(p.y()))
				isNan = true;
			if (std::isnan(p.z()))
				isNan = true;

			if (std::isinf(p.x()))
				isNan = true;
			if (std::isinf(p.y()))
				isNan = true;
			if (std::isinf(p.z()))
				isNan = true;
		}
		DBOUT("\nIsNan Pos : " << isNan);

		isNan = false;
		for (auto p : initial_velocities)
		{
			if (std::isnan(p.x()))
				isNan = true;
			if (std::isnan(p.y()))
				isNan = true;
			if (std::isnan(p.z()))
				isNan = true;
			if (std::isnan(p.w()))
				isNan = true;

			if (std::isinf(p.x()))
				isNan = true;
			if (std::isinf(p.y()))
				isNan = true;
			if (std::isinf(p.z()))
				isNan = true;
			if (std::isinf(p.w()))
				isNan = true;
		}
		DBOUT("\nIsNan Vel : " << isNan);

		isNan = false;
		for (auto p : initial_density_pressure)
		{
			if (std::isnan(p.x()))
				isNan = true;
			if (std::isnan(p.y()))
				isNan = true;

			if (std::isinf(p.x()))
				isNan = true;
			if (std::isinf(p.y()))
				isNan = true;
		}
		DBOUT("\nIsNan Pres : " << isNan);

		//for (auto p : pos)
	}

	void saveParticlesAsText(std::string filename)
	{
		vmath::vec3* initial_positions = new vmath::vec3[POINTS_TOTAL];
		vmath::vec4* initial_velocities = new vmath::vec4[POINTS_TOTAL];
		vmath::vec2* initial_density_pressure = new vmath::vec2[POINTS_TOTAL];

		glGetNamedBufferSubData(m_vbo[POSITION_A], 0, sizeof(vmath::vec3) * POINTS_TOTAL, initial_positions);
		glGetNamedBufferSubData(m_vbo[VELOCITY_A], 0, sizeof(vmath::vec4) * POINTS_TOTAL, initial_velocities);
		glGetNamedBufferSubData(m_vbo[DENSITY_A], 0, sizeof(vmath::vec2) * POINTS_TOTAL, initial_density_pressure);

		std::ofstream myfile;
		myfile.open(filename + "Positions.txt");
		myfile << "X" << "\t" << "Y" << "\t" << "Z" << std::endl;
		for (int i = 0; i < POINTS_TOTAL; i++)
		{
			myfile << initial_positions[i][0] << "\t" << initial_positions[i][1] << "\t" << initial_positions[i][2] << std::endl;
		}
		myfile.close();

		myfile.open(filename + "VelocityMass.txt");
		myfile << "X" << "\t" << "Y" << "\t" << "Z" << "\t" << "Mass" << std::endl;
		for (int i = 0; i < POINTS_TOTAL; i++)
		{
			myfile << initial_velocities[i][0] << "\t" << initial_velocities[i][1] << "\t" << initial_velocities[i][2] << "\t" << initial_velocities[i][3] << std::endl;
		}
		myfile.close();


		myfile.open(filename + "DensPress.txt");
		myfile << "Dens" << "\t" << "Pres" << std::endl;
		for (int i = 0; i < POINTS_TOTAL; i++)
		{
			myfile << initial_density_pressure[i][0] << "\t" << initial_density_pressure[i][1] << std::endl;
		}
		myfile.close();

		delete[] initial_positions;
		delete[] initial_velocities;
		delete[] initial_density_pressure;
	}

	template <typename T>
	T clip(const T& n, const T& lower, const T& upper)
	{
		return std::max(lower, std::min(n, upper));
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

	void render_wind_dir()
	{
		glUseProgram(m_render_wind_program);
		glBindVertexArray(wind_dir_vao);
		glUniformMatrix4fv(proj_mat_loc_wind, 1, GL_FALSE, proj_matrix);

		vmath::mat4 mv_mat = v_matrix * m_matrix;
		glUniformMatrix4fv(mv_mat_loc_wind, 1, GL_FALSE, mv_mat);


		//	vmath::vec3 wind_pointer[]={vmath::vec3(0),vmath::normalize(-wind_dir+vmath::vec3(0.2f,0.0f,0.0f)),vmath::normalize(-wind_dir-vmath::vec3(0.2f,0.0f,0.0f))};
		vmath::vec3 wind_pointer[] = { -wind_dir * 0.5f, wind_dir * 0.5f };


		glNamedBufferData(wind_pointer_buff, 4 * 3 * 2, wind_pointer, GL_STATIC_DRAW);


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


		sprintf(buffer, "OptimJoin: %3.2fms", optTime);
		overlay.drawText(buffer, 0, 1);
		sprintf(buffer, "Optim: %3.2fms", gridOptimiser->optTime);
		overlay.drawText(buffer, 0, 2);
		sprintf(buffer, "Update: %3.2fms", updateTime);
		overlay.drawText(buffer, 0, 3);
		if (stalling)
		{
			sprintf(buffer, "Stalling!!!");
			overlay.drawText(buffer, 0, 4);
		}

		overlay.draw();
	}

	void init_SkyBox()
	{
		envMapTex = sb7::ktx::file::load("media/textures/envmaps/mountaincube.ktx");
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glEnable(GL_TEXTURE_CUBE_MAP_SEAMLESS);
		glGenVertexArrays(1, &render_skybox_vao);
	}

	void init_Ball_Buffer()
	{
		if (render_balls_vao)glDeleteVertexArrays(1, &render_balls_vao);
		glGenVertexArrays(1, &render_balls_vao);
		glBindVertexArray(render_balls_vao);
		if (balls_buffer)glDeleteBuffers(1, &balls_buffer);
		glGenBuffers(1, &balls_buffer);
		glBindBuffer(GL_ARRAY_BUFFER, balls_buffer);
		VERT_PER_BALL = 0;

		const float PI = 3.14159265358979323846264f;
		const float R = 0.3;
		const float step = PI / 4;
		std::vector<float> vertices = std::vector<float>();
		for (float i = 0; i <= 2 * PI; i += step)
		{
			for (float j = -PI / 2; j < PI / 2 + step; j += step)
			{
				vertices.push_back(R * sin(i + step) * cos(j));
				vertices.push_back(R * cos(i + step) * cos(j));
				vertices.push_back(R * sin(j));

				vertices.push_back(R * sin(i) * cos(j));
				vertices.push_back(R * cos(i) * cos(j));
				vertices.push_back(R * sin(j));
			}
		}
		VERT_PER_BALL = vertices.size();
		glBufferData(GL_ARRAY_BUFFER, 4 * VERT_PER_BALL, &*vertices.begin(), GL_STATIC_DRAW);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
		glEnableVertexAttribArray(0);
		glBindVertexArray(0);
	}

	void init_RayTraceCube_Buffer()
	{
		if (render_fluid_ray_trace_vao)glDeleteVertexArrays(1, &render_fluid_ray_trace_vao);
		glGenVertexArrays(1, &render_fluid_ray_trace_vao);
		glBindVertexArray(render_fluid_ray_trace_vao);
		if (rayTraceCube_buffer)glDeleteBuffers(1, &rayTraceCube_buffer);
		glGenBuffers(1, &rayTraceCube_buffer);
		glBindBuffer(GL_ARRAY_BUFFER, rayTraceCube_buffer);


		const float w = 1, h = 1, d = -2.14450692050;
		vmath::vec3 v[] = {
			vmath::vec3(-w, -h, d), // front-bottom-left     0
			vmath::vec3(w, -h, d), // front-bottom-right    1
			vmath::vec3(-w, h, d), // front-top-left        2
			vmath::vec3(w, h, d), // front-top-right       3

			//   vmath::vec3(-w, -h, -d), // back-bottom-left      4
			//   vmath::vec3( w, -h, -d), // back-bottom-right     5
			//   vmath::vec3(-w,  h, -d), // back-top-left         6
			//   vmath::vec3( w,  h, -d) // back-top-right        7
		};

		//vmath::vec3  vertices[] = {v[7] , v[6] , v[3] , v[2] , v[0] , v[6] , v[4] ,	v[7] , v[5] , v[3] , v[1] , v[0] , v[5] , v[4]};

		vmath::vec3 vertices[] = { v[0], v[1], v[2], v[3], v[0] };


		glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_DYNAMIC_DRAW);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
		glEnableVertexAttribArray(0);
		glBindVertexArray(0);
	}

	void init_mask_Buffer()
	{
		glGenVertexArrays(1, &mask_vao);
		glBindVertexArray(mask_vao);
		glBindBuffer(GL_ARRAY_BUFFER, m_vbo[POSITION_A + (m_iteration_index + 1) & 1]);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, NULL);
		glEnableVertexAttribArray(0);
	}

	void init_DensityFieldTexture()
	{
		glGenVertexArrays(1, &fill_density_Texture_VAO);
		glBindVertexArray(fill_density_Texture_VAO);
		glGenTextures(1, &density_Field_Texture);
		glBindTexture(GL_TEXTURE_3D, density_Field_Texture);
		glTexStorage3D(GL_TEXTURE_3D, 1, GL_R32F, DENSITY_TEX_SIDE, DENSITY_TEX_SIDE, DENSITY_TEX_SIDE);
		glTexParameterf(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameterf(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameterf(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
		glTexParameterf(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
		glTexParameterf(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_BORDER);

		glGenFramebuffers(1, &density_Texture_FBO);
		glBindFramebuffer(GL_FRAMEBUFFER, density_Texture_FBO);
		glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,density_Field_Texture, 0);
		static const GLuint draw_buffers[] = { GL_COLOR_ATTACHMENT0 };
		glDrawBuffers(1, draw_buffers);
		glBindFramebuffer(GL_FRAMEBUFFER, 0);

		glBindTexture(GL_TEXTURE_BUFFER, 0);
		glBindTexture(GL_TEXTURE_3D, 0);
		glBindVertexArray(0);
	}

	void fillDensityFieldTexture()
	{
		glBindFramebuffer(GL_FRAMEBUFFER, density_Texture_FBO);
		glViewport(0, 0, DENSITY_TEX_SIDE, DENSITY_TEX_SIDE);
		glBindVertexArray(fill_density_Texture_VAO);
		glUseProgram(m_fill_DensityField_program);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_BUFFER, m_pos_tbo[m_iteration_index & 1]);
		glActiveTexture(GL_TEXTURE1);
		glBindTexture(GL_TEXTURE_BUFFER, m_vel_tbo[m_iteration_index & 1]);
		glActiveTexture(GL_TEXTURE2);
		glBindTexture(GL_TEXTURE_BUFFER, m_GRIDLIST_tbo[m_iteration_index & 1]);
#if OPTIM_STRUCT==7
		glActiveTexture(GL_TEXTURE3);
		glBindTexture(GL_TEXTURE_BUFFER, m_GRID_DATA_tbo[m_iteration_index & 1]);
#endif
		glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		glViewport(0, 0, info.windowWidth, info.windowHeight);
	}

	void render_SkyBox()
	{
		glUseProgram(m_render_skybox_program);
		glBindVertexArray(render_skybox_vao);
		glDisable(GL_DEPTH_TEST);
		vmath::mat4 mv_mat = v_matrix * m_matrix;
		glUniformMatrix4fv(mv_mat_skybox_loc, 1, GL_FALSE, mv_mat);
		glUniformMatrix4fv(proj_mat_skybox_loc, 1, GL_FALSE, proj_matrix);
		glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
		glEnable(GL_DEPTH_TEST);
	}

	void render_fps(float currentTime)
	{
		float nowTime = (currentTime);
		if (nowTime > (lastTime + 0.25f))
		{
			fps = float(frames) / (nowTime - lastTime);
			frames = 0;
			lastTime = nowTime;
			GLuint upTm;
			glGetQueryObjectuiv(timerQueries[0], GL_QUERY_RESULT, &upTm);
			updateTime = upTm / 1000000.0;
		}
		updateOverlay();
		frames++;
	}

	void render(double t)
	{
		glUseProgram(m_update_program);
		glEnable(GL_RASTERIZER_DISCARD);
		glUniform1f(wind_speed_loc, wind_speed);
		glUniform3fv(wind_dir_loc, 1, wind_dir);
		glUniform3fv(glass_pos_loc, 1, glass_pos);


		glUniform3fv(atractC_loc, 1, atractC);
		glUniform3fv(repulsC_loc, 1, repulsC);

		for (int i = iterations_per_frame; i != 0; --i)

		{
			m_iteration_index++;

			if (requestRain > 0)
			{
				rain((m_iteration_index)& 1);
				requestRain--;
			}

			vec4 tmp_sphere1_CurentInterpol = mix(sphere1_CurentInterpol, sphere1, 0.01);
			vec3 tmp_sphere1_speed = *((vec3*)&tmp_sphere1_CurentInterpol) - (*((vec3*)&sphere1_CurentInterpol));
			sphere1_CurentInterpol = tmp_sphere1_CurentInterpol;
			glUniform4fv(sphere1_loc, 1, sphere1_CurentInterpol);
			glUniform3fv(sphere1_speed_loc, 1, tmp_sphere1_speed);
			glBindVertexArray(m_vao[m_iteration_index & 1]);

			std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

#if(OPTIM_STRUCT==4||OPTIM_STRUCT==5||OPTIM_STRUCT==6 || OPTIM_STRUCT==7)
			//glFinish();		

			//m_iteration_index+=CHUNK_COUNT/2 ;//5;
			//std::this_thread::sleep_for(std::chrono::milliseconds(10));
			if (fence[(m_iteration_index)& 1] != 0)
			{
				glGetSynciv(fence[(m_iteration_index)& 1], GL_SYNC_STATUS, sizeof(int), nullptr, &isSignaled);
				stalling = isSignaled == GL_UNSIGNALED;
				glClientWaitSync(fence[(m_iteration_index)& 1], 0, GL_TIMEOUT_IGNORED);
				glGetSynciv(fence[(m_iteration_index)& 1], GL_SYNC_STATUS, sizeof(int), nullptr, &isSignaled);
				glDeleteSync(fence[(m_iteration_index)& 1]);
			}


			gridOptimiser->join_Workers();
			glFlushMappedNamedBufferRange(m_grdBufferChunks_vbo[((m_iteration_index)& 1)], 0, GRIDLIST_SIZE);

#if OPTIM_STRUCT == 7
			glFlushMappedNamedBufferRange(m_GridDataBufferChunks_vbo[((m_iteration_index)& 1)], 0, GRIDLIST_DATA_SIZE);			
			((ArraysFromListsAllDataPerCellThreadPoolOptimiser*)gridOptimiser)->fillList(pointsBuffer[(m_iteration_index)& 1], velosityBuffer[(m_iteration_index)& 1], density_pBuffer[(m_iteration_index)& 1], glass_pos, gridBuffer[(m_iteration_index + 1) & 1], gridDataBuffer[(m_iteration_index + 1) & 1]);
#else
			gridOptimiser->fillList(pointsBuffer[(m_iteration_index)& 1], gridBuffer[(m_iteration_index + 1) & 1], glass_pos);
#endif
					


#elif(OPTIM_STRUCT >0)
			int * gridBuffer = (int *)glMapNamedBufferRange(m_vbo[GRIDLIST_A + (m_iteration_index & 1)], 0, GRIDLIST_SIZE, GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT);
			vmath::vec3 * pointsBuffer = (vmath::vec3 *)glMapNamedBufferRange( m_vbo[POSITION_A + (m_iteration_index & 1)], 0, POSITIONS_SIZE, GL_MAP_READ_BIT);		
			gridOptimiser->fillList(pointsBuffer,gridBuffer,glass_pos);
			//gridOptimiser->printList(pointsBuffer,gridBuffer);std::exit(0); 		
			glUnmapNamedBuffer(m_vbo[GRIDLIST_A + (m_iteration_index & 1)]);
			glUnmapNamedBuffer(m_vbo[POSITION_A + (m_iteration_index & 1)]);
#endif
			std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
			optTime = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() / 1000.0;//OptimJoin

			
			glActiveTexture(GL_TEXTURE0);
			glBindTexture(GL_TEXTURE_BUFFER, m_pos_tbo[m_iteration_index & 1]);
			glActiveTexture(GL_TEXTURE1);
			glBindTexture(GL_TEXTURE_BUFFER, m_vel_tbo[m_iteration_index & 1]);
			glActiveTexture(GL_TEXTURE2);
			glBindTexture(GL_TEXTURE_BUFFER, m_GRIDLIST_tbo[m_iteration_index & 1]);
			glActiveTexture(GL_TEXTURE3);
			glBindTexture(GL_TEXTURE_BUFFER, m_density_tbo[m_iteration_index & 1]);
#if OPTIM_STRUCT == 7
			glActiveTexture(GL_TEXTURE4);
			glBindTexture(GL_TEXTURE_BUFFER, m_GRID_DATA_tbo[m_iteration_index & 1]);
#endif
			//
			//m_iteration_index++;//the only increment
			glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 0, m_vbo[POSITION_A + ((m_iteration_index + 1) & 1)]);
			glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 1, m_vbo[VELOCITY_A + ((m_iteration_index + 1) & 1)]);
			glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 2, m_vbo[DENSITY_A + ((m_iteration_index + 1) & 1)]);
			glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 3, m_vbo[COLORS]);

			glBeginQuery(GL_TIME_ELAPSED, timerQueries[0]);
			glBeginTransformFeedback(GL_POINTS);
			glDrawArrays(GL_POINTS, 0, POINTS_TOTAL);
			glEndTransformFeedback();
			glEndQuery(GL_TIME_ELAPSED);
			fence[(m_iteration_index + 1) & 1] = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
		}

		glDisable(GL_RASTERIZER_DISCARD);

		glViewport(0, 0, info.windowWidth, info.windowHeight);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

		if (draw_skybox)
		{
			render_SkyBox();
		}
		if (draw_points)
		{
			glUseProgram(m_render_program);
			glUniformMatrix4fv(proj_mat_loc, 1, GL_FALSE, proj_matrix);
			vmath::mat4 mv_mat = v_matrix * m_matrix;
			glUniformMatrix4fv(mv_mat_loc, 1, GL_FALSE, mv_mat);
			glUniform3fv(light_pos_loc, 1, light_pos);

			glBindVertexArray(render_balls_vao);
			glActiveTexture(GL_TEXTURE0);
			glBindTexture(GL_TEXTURE_BUFFER, m_pos_tbo[(m_iteration_index)& 1]);
			glActiveTexture(GL_TEXTURE1);
			glBindTexture(GL_TEXTURE_BUFFER, m_GRIDLIST_tbo[(m_iteration_index)& 1]);
			glActiveTexture(GL_TEXTURE2);
			glBindTexture(GL_TEXTURE_BUFFER, m_color_tbo);
			glActiveTexture(GL_TEXTURE3);
			glBindTexture(GL_TEXTURE_BUFFER, m_density_tbo[(m_iteration_index)& 1]);
			glDrawArraysInstanced(GL_TRIANGLE_STRIP, 0, VERT_PER_BALL / 3, POINTS_TOTAL);
		}
		if (draw_raytrace)
		{
			//m_iteration_index--;
			fillDensityFieldTexture();
			glDisable(GL_DEPTH_TEST);
			/*glEnable(GL_STENCIL_TEST);

			glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
			glStencilMask(0xFF);
			glStencilFunc(GL_ALWAYS, 1, 0xFF);
			glColorMask(false,false,false,false);
			glBindVertexArray(mask_vao);
			glUseProgram(m_render_mask_program);
			vmath::mat4 mvp_mat = proj_matrix*v_matrix*m_matrix;
			glUniformMatrix4fv(mvp_mat_mask_loc, 1, GL_FALSE, mvp_mat);
			glPointSize(30);
			glDrawArrays(GL_POINTS, 0,POINTS_TOTAL);


			glStencilFunc(GL_EQUAL, 1, 0xFF);
			glStencilMask(0x00);
			glColorMask(true,true,true,true);*/
			glUseProgram(m_render_fluid_raytrace_program);
			glUniformMatrix4fv(proj_mat_raytrace_loc, 1, GL_FALSE, proj_matrix);
			vmath::mat4 mv_mat = v_matrix * m_matrix;
			glUniformMatrix4fv(mv_mat_raytrace_loc, 1, GL_FALSE, mv_mat);
			glUniform3fv(light_pos_raytrace_loc, 1, light_pos);
			glUniform3fv(glass_pos_raytrace_loc, 1, glass_pos);
			glUniform4fv(sphere1_rayTrace_loc, 1, vmath::vec4(sphere1_CurentInterpol[0], sphere1_CurentInterpol[1], sphere1_CurentInterpol[2], sphere1_CurentInterpol[3] * 1.7));
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
			glActiveTexture(GL_TEXTURE5);
			glBindTexture(GL_TEXTURE_CUBE_MAP, envMapTex);
			glActiveTexture(GL_TEXTURE6);
			glBindTexture(GL_TEXTURE_3D, density_Field_Texture);
			//m_iteration_index++;
			glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
			glEnable(GL_DEPTH_TEST);
			glStencilMask(0xFF);
			glDisable(GL_STENCIL_TEST);
		}


		render_wind_dir();
		glDisable(GL_CULL_FACE);
		render_fps(t);
		glEnable(GL_CULL_FACE);
	}

	void pointsToGridListsOfArrays(int pointsCount, vmath::vec3* points, int* gridBuffer, int gridSide, float cellSize, float offset)
	{
		int gridSize = gridSide * gridSide * gridSide;
		std::vector<int> cellsSizes = std::vector<int>(gridSize, 0);
		std::vector<std::forward_list<int>> cells = std::vector<std::forward_list<int>>(gridSize);


		for (int i = 0; i < pointsCount; i++)
		{
			int cellIndex = CellIndex(&points[i], cellSize, gridSide, offset);
			cells[cellIndex].push_front(i);
			cellsSizes[cellIndex]++;
		}
		int lastUsedInd = (gridSize)* 2 - 1;
		for (int i = 0; i < gridSize; i++)
		{
			gridBuffer[i * 2] = lastUsedInd + 1;
			gridBuffer[i * 2 + 1] = cellsSizes[i];
			for (std::forward_list<int>::iterator it = cells[i].begin(); it != cells[i].end(); it++)
			{
				gridBuffer[++lastUsedInd] = *it;
			}
		}
	}

	void pointsToGridListsOfListsParall(int pointsCount, vmath::vec3* points, int* gridBuffer, int gridSide, float cellSize, float offset, int kernels)
	{
		int gridSize = gridSide * gridSide * gridSide;
		std::vector<int> cellsSizes = std::vector<int>(gridSize, 0);

		std::atomic_flag* list_Locs = new std::atomic_flag[gridSize]; //keys to every list 	
		std::atomic<int> lastUsedInd = (gridSize)-1;

		for (int i = 0; i < gridSize * 2; i++)gridBuffer[i] = 0;//init buffer

		int pointsPerKernel = pointsCount / (kernels);
		int pointsPerKernelRem = pointsCount % kernels;


		for (int i = 0; i < kernels; i++)
		{
			int nPoints = pointsPerKernel;
			if ((i) == (kernels - 1))nPoints += pointsPerKernelRem;
			GridListKernelParall(i, gridBuffer, &lastUsedInd, list_Locs, cellSize, gridSide, offset, points, pointsPerKernel * i, nPoints);
		}
	}

	void GridListKernelParall(int globalIndex, int* gridBuffer, std::atomic<int>* lastUsedInd, std::atomic_flag* list_Locs, int cellSize, int gridSide, float offset, vmath::vec3* points, int firstPoint, int nPoints)
	{
		for (int i = firstPoint; i < firstPoint + nPoints; i++)
		{
			int listInd = CellIndex(points + i, cellSize, gridSide, offset);
			int newMemory = ++(*lastUsedInd) * 2;//alocating memory
			while (list_Locs[listInd].test_and_set(std::memory_order_acquire));// aquire lock
			if (gridBuffer[listInd] == 0)
			{// pyrwiqt elemnt v spisyka
				gridBuffer[listInd] = newMemory;
				gridBuffer[listInd + 1]++;//incrementing list size
				gridBuffer[newMemory] = i;// asigning new item value
				gridBuffer[newMemory + 1] = 0;
			}
			else
			{//poredniqt element
				int oldFirst = gridBuffer[listInd];
				gridBuffer[listInd] = newMemory;
				gridBuffer[listInd + 1]++;//incrementing list size
				gridBuffer[newMemory] = i;// asigning new item value
				gridBuffer[newMemory + 1] = oldFirst;
			}
			list_Locs[listInd].clear(std::memory_order_release);//release lock
		}
	}

	inline int CellIndex(vmath::vec3* point, float cellSize, int gridSide, float offset)
	{
		int cellIndex = 0;
		cellIndex = std::max(std::min(((int)floorf(((*point)[2] + offset) / cellSize)) * gridSide * gridSide, gridSide * gridSide * (gridSide - 1)), 0);
		cellIndex += std::max(std::min(((int)floorf(((*point)[1] + offset) / cellSize)) * gridSide, gridSide * (gridSide - 1)), 0);
		cellIndex += std::max(std::min(((int)floorf(((*point)[0] + offset) / cellSize)), (gridSide - 1)), 0);
		return cellIndex;
	}

	void rain(int bufer)
	{
		float a = rand() / (float)RAND_MAX;

		for (int i = 0; i < POINTS_TOTAL; i++)
		{
			if (vmath::length(pointsBuffer[bufer][i] - vec3(0, -4, 0)) < 3)
			{
				pointsBuffer[bufer][i] = vec3(pointsBuffer[bufer][i].x(), pointsBuffer[bufer][i].y() + 13, pointsBuffer[bufer][i].z());
			}
		}
	}

private:

	struct KeyBuffer
	{
		bool q;
		bool w;
		bool e;
	} keyBuffer;

	void onKey(int key, int action)
	{
		switch (key)
		{
		case 'Q': keyBuffer.q = action;
			break;
		case 'W': keyBuffer.w = action;
			break;
		case 'E': keyBuffer.e = action;
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
			case 'S': draw_skybox = !draw_skybox;
				break;
			case ']': wind_speed = wind_speed == 0.0 ? 0.01 : wind_speed * 1.5;
				break;
			case '[': wind_speed *= 0.7;
				break;

			case 'A': atractC[0] *= 1.1;
				break;
			case 'Z': atractC[0] *= 0.9;
				break;
				//case 'S': atractC[1]*=1.1;
				break;
			case 'X': atractC[1] *= 0.9;
				break;
			case 'D': atractC[2] *= 1.1;
				break;
			case 'C': atractC[2] *= 0.9;
				break;
			case 'F': repulsC[0] *= 1.1;
				break;
			case 'V': repulsC[0] *= 0.9;
				break;
			case 'G': repulsC[1] *= 1.1;
				break;
				// case 'B': repulsC[1]*=0.9;
				//      break;
				//case 'H': repulsC[2]*=1.1;
				//     break;
			case 'N': repulsC[2] *= 0.9;
				break;
			case 'B': saveParticles("initParticlesState.dat");
				break;
			case 'H': saveParticlesAsText("textDump");
				break;
			case 'I': isAnyNanInf();
				break;
			case 'M':
			{
						requestRain = 2;
			}
				break;
			case GLFW_KEY_KP_ADD: iterations_per_frame++;
				break;
			case GLFW_KEY_KP_SUBTRACT: iterations_per_frame = iterations_per_frame > 0 ? iterations_per_frame - 1 : 0;
				break;
			}
		}
	}

	int lmx, lmy;
	bool draging, rotating, looking;

	void onMouseMove(int x, int y)
	{
		if (rotating)
		{
			vmath::mat4 rotmat = vmath::rotate((-lmx + x) * 0.3f, 0.0f, 1.0f, 0.0f) * vmath::rotate(-(lmy - y) * 0.3f, 1.0f, 0.0f, 0.0f);
			if (keyBuffer.q)
			{
				vmath::vec4 tmp_wind_dir = vmath::vec4(wind_dir, 0.0f);
				tmp_wind_dir = tmp_wind_dir * rotmat;
				wind_dir = vmath::vec3(tmp_wind_dir[0], tmp_wind_dir[1], tmp_wind_dir[2]);
			}
			else if (keyBuffer.w)
			{
				vmath::vec4 tmp_light_pos = vmath::vec4(light_pos, 0.0f);
				tmp_light_pos = tmp_light_pos * rotmat;
				light_pos = vmath::vec3(tmp_light_pos[0], tmp_light_pos[1], tmp_light_pos[2]);
			}
			else
			{
				m_matrix = rotmat * m_matrix;
			}
		}
		if (draging)
		{
			if (keyBuffer.q)
			{
				//	vmath::vec4 tmp_glass_pos =vmath::vec4(glass_pos,1.0f);
				//tmp_glass_pos =tmp_glass_pos* vmath::translate((-lmx+x)*1.0f, (lmy-y)*1.0f, 0.0f);
				//glass_pos = vmath::vec3(tmp_glass_pos[0],tmp_glass_pos[1],tmp_glass_pos[2]);
				glass_pos[0] -= -(-lmx + x) * 0.1;
				glass_pos[1] += (lmy - y) * 0.1;
			}
			else if (keyBuffer.e)
			{
				vmath::vec2 dmouse((-lmx + x) * 0.1, (lmy - y) * 0.1);
				//	dmouse = vmath::clamp(dmouse,vmath::vec2(-0.2),vmath::vec2(0.2));

				vmath::vec3 newSpPos = vmath::vec3(*((vmath::vec3*)&sphere1));
				newSpPos += vmath::vec3(dmouse, 0);
				sphere1_speed = newSpPos - vmath::vec3(*((vmath::vec3*)&sphere1));
				//if (length(sphere1 - sphere1_CurentInterpol)<0.001)sphere1_CurentInterpol = vec4(sphere1);
				sphere1 = vmath::vec4(newSpPos, sphere1[3]);
			}
			else
			{
				v_matrix = vmath::translate((-lmx + x) * 0.1f, -(-lmy + y) * 0.1f, 0.0f) * v_matrix;
			}
		}
		if (looking)
		{
			vmath::mat4 lookrotmat = vmath::rotate(-(lmx - x) * 0.4f, 0.0f, 1.0f, 0.0f) * vmath::rotate(-(lmy - y) * 0.4f, 1.0f, 0.0f, 0.0f);
			v_matrix = lookrotmat * v_matrix;
		}
		lmx = x;
		lmy = y;
	}

	void onMouseWheel(int pos)
	{
		if (keyBuffer.w)
		{
			light_pos = pos > 0 ? light_pos * 1.2 : light_pos * 0.8;
		}
		else if (keyBuffer.e)
		{
			float c = 1.0f;
			if (pos < 0)c = 0.1f;
			else if (pos > 0)c = -0.1f;
			sphere1[3] += c;
		}
		else
		{
			float c = 1.3f;
			if (pos < 0)c = 1.2f;
			else if (pos > 0)c = -1.2f;
			v_matrix = vmath::translate(0.0f, 0.0f, c) * v_matrix;
		}
	}

	void onMouseButton(int button, int action)
	{
		if (button == 0 && rotating || button == 1 && draging)
		{
			if (keyBuffer.q)wind_dir = vmath::vec3(0, -1, 0);
			else
				init_user_transforms();
		}
		if (button == 0)draging = action;
		if (button == 1)rotating = action;
		if (button == 2)looking = action;

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

		static const char* tf_varyings[] =
		{
			"tf_position",
			"tf_velocity_mass",
			"tf_density_pressure",
			"tf_color"
		};

		glTransformFeedbackVaryings(m_update_program, 4, tf_varyings, GL_SEPARATE_ATTRIBS);

		glLinkProgram(m_update_program);
		wind_speed_loc = glGetUniformLocation(m_update_program, "wind_speed");
		wind_dir_loc = glGetUniformLocation(m_update_program, "wind_dir");
		glass_pos_loc = glGetUniformLocation(m_update_program, "glass_pos");
		sphere1_loc = glGetUniformLocation(m_update_program, "sphere1");
		sphere1_speed_loc = glGetUniformLocation(m_update_program, "sphere1_speed");
		atractC_loc = glGetUniformLocation(m_update_program, "atractC");
		repulsC_loc = glGetUniformLocation(m_update_program, "repulsC");
		GRID_VOLUME_SIDE_loc = glGetUniformLocation(m_update_program, "GRID_VOLUME_SIDE");
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
		proj_mat_loc = glGetUniformLocation(m_render_program, "proj_mat");
		mv_mat_loc = glGetUniformLocation(m_render_program, "mv_mat");
		light_pos_loc = glGetUniformLocation(m_render_program, "light_pos");


		glDeleteShader(vs);
		glDeleteShader(fs);
		vs = sb7::shader::load("media/shaders/render_mask.vs.glsl", GL_VERTEX_SHADER);
		fs = sb7::shader::load("media/shaders/render_mask.fs.glsl", GL_FRAGMENT_SHADER);

		if (m_render_mask_program)
			glDeleteProgram(m_render_mask_program);
		m_render_mask_program = glCreateProgram();
		glAttachShader(m_render_mask_program, vs);
		// glAttachShader(m_render_mask_program, fs);        
		glLinkProgram(m_render_mask_program);
		mvp_mat_mask_loc = glGetUniformLocation(m_render_mask_program, "mvp_mat");


		glDeleteShader(vs);
		glDeleteShader(fs);
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
		proj_mat_loc_wind = glGetUniformLocation(m_render_wind_program, "proj_mat");
		mv_mat_loc_wind = glGetUniformLocation(m_render_wind_program, "mv_mat");


		glDeleteShader(vs);
		glDeleteShader(fs);
		glDeleteShader(gs);
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
		proj_mat_raytrace_loc = glGetUniformLocation(m_render_fluid_raytrace_program, "proj_mat");
		mv_mat_raytrace_loc = glGetUniformLocation(m_render_fluid_raytrace_program, "mv_mat");
		light_pos_raytrace_loc = glGetUniformLocation(m_render_fluid_raytrace_program, "light_pos");
		glass_pos_raytrace_loc = glGetUniformLocation(m_render_fluid_raytrace_program, "glass_pos");
		sphere1_rayTrace_loc = glGetUniformLocation(m_render_fluid_raytrace_program, "sphere1");
		glDeleteShader(vs);
		glDeleteShader(fs);

		vs = sb7::shader::load("media/shaders/skybox.vs.glsl", GL_VERTEX_SHADER);
		fs = sb7::shader::load("media/shaders/skybox.fs.glsl", GL_FRAGMENT_SHADER);

		m_render_skybox_program = glCreateProgram();
		glAttachShader(m_render_skybox_program, vs);
		glAttachShader(m_render_skybox_program, fs);
		glLinkProgram(m_render_skybox_program);
		mv_mat_skybox_loc = glGetUniformLocation(m_render_skybox_program, "view_matrix");
		proj_mat_skybox_loc = glGetUniformLocation(m_render_skybox_program, "proj_matrix");
		glDeleteShader(vs);
		glDeleteShader(fs);

		vs = sb7::shader::load("media/shaders/fill.density.field.vs.glsl", GL_VERTEX_SHADER);
		gs = sb7::shader::load("media/shaders/fill.density.field.gs.glsl", GL_GEOMETRY_SHADER);
		fs = sb7::shader::load("media/shaders/fill.density.field.fs.glsl", GL_FRAGMENT_SHADER);


		if (m_fill_DensityField_program)
			glDeleteProgram(m_fill_DensityField_program);
		m_fill_DensityField_program = glCreateProgram();
		glAttachShader(m_fill_DensityField_program, vs);
		glAttachShader(m_fill_DensityField_program, fs);
		glAttachShader(m_fill_DensityField_program, gs);

		glLinkProgram(m_fill_DensityField_program);

		glDeleteShader(vs);
		glDeleteShader(fs);
		glDeleteShader(gs);
	}

	GLuint mask_vao;
	GLuint timerQueries[2];
	GLuint m_vao[2];
	GLuint m_vbo[8];
	GLuint m_grdBufferChunks_vbo[2];
	GLuint m_GridDataBufferChunks_vbo[2];
	GLuint m_index_buffer;
	GLuint m_index_triangles_buffer;
	GLuint m_GRIDLIST_tbo[2];
	GLuint m_GRID_DATA_tbo[2];
	GLuint m_pos_tbo[2];
	GLuint m_vel_tbo[2];
	GLuint m_density_tbo[2];
	GLuint m_color_tbo;
	GLuint m_con_tbo;
	GLuint m_update_program;
	GLuint m_render_program;
	GLuint m_render_mask_program;
	GLuint m_render_wind_program;
	GLuint m_render_fluid_raytrace_program;
	GLuint m_render_skybox_program;
	GLuint m_fill_DensityField_program;
	GLuint m_C_loc;
	GLuint m_iteration_index;
	GLuint wind_dir_vao;
	GLuint density_Texture_FBO;
	GLuint fill_density_Texture_VAO;

	bool draw_points;
	bool draw_lines;
	bool draw_triangles;
	bool draw_raytrace;
	bool draw_skybox;
	int iterations_per_frame;
	int requestRain;

	GLuint mv_mat_skybox_loc;
	GLuint proj_mat_skybox_loc;
	GLuint proj_mat_raytrace_loc;
	GLuint mv_mat_raytrace_loc;
	GLuint mvp_mat_mask_loc;
	GLuint light_pos_raytrace_loc;
	GLuint glass_pos_raytrace_loc;
	GLuint GRID_VOLUME_SIDE_loc;
	GLuint mv_mat_loc;
	GLuint proj_mat_loc;
	GLuint wind_speed_loc;
	GLuint wind_dir_loc;
	GLuint glass_pos_loc;
	GLuint sphere1_loc;
	GLuint sphere1_rayTrace_loc;
	GLuint sphere1_speed_loc;
	GLuint mv_mat_loc_wind;
	GLuint proj_mat_loc_wind;
	GLuint wind_speed_loc_wind;
	GLuint wind_dir_loc_wind;
	GLuint atractC_loc;
	GLuint repulsC_loc;
	GLuint wind_pointer_buff;
	GLuint normal_buffer;
	GLuint light_pos_loc;
	GLuint tex_coord_buf;
	GLuint density_Field_Texture;
	GLuint density_Field_Buffer;
	GLuint main_texture;
	GLuint balls_buffer;
	GLuint rayTraceCube_buffer;
	GLuint render_balls_vao;
	GLuint render_fluid_ray_trace_vao;
	GLuint render_skybox_vao;

	GLsync fence[2];
	GLint isSignaled;
	bool stalling;

	GLuint envMapTex;

	int VERT_PER_BALL;
	vmath::mat4 proj_matrix;
	vmath::mat4 v_matrix;
	vmath::mat4 m_matrix;

	float wind_speed;
	vmath::vec3 wind_dir;
	vmath::vec3 light_pos;
	vmath::vec3 glass_pos;
	vmath::vec4 sphere1;
	vmath::vec4 sphere1_CurentInterpol;
	vmath::vec3 sphere1_speed;
	vmath::vec3 atractC;
	vmath::vec3 repulsC;

	int* gridBuffer[2];
	float* gridDataBuffer[2];
	vmath::vec3* pointsBuffer[2];
	
	vmath::vec4* velosityBuffer[2];	
	vmath::vec2* density_pBuffer[2];


	float fps;
	float optTime;
	float updateTime;
	float lastTime;
	int frames;
};

DECLARE_MAIN(springmass_app);
