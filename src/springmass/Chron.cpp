#include <chrono>
#include<string>
#include<functional>

template<typename T = std::function<void(double)>>
class Chron
{
public:
	std::chrono::high_resolution_clock::time_point tStart;
	T callback;
	Chron(T callback) : callback(std::move(callback))  {
		tStart = std::chrono::high_resolution_clock::now();		
	}

	~Chron() {
		auto tEnd = std::chrono::high_resolution_clock::now();
		auto t = std::chrono::duration_cast<std::chrono::microseconds>(tEnd - tStart).count() / 1000.0;
		callback(t);
	}

};

Chron(void(*)(double))->Chron<void(*)(double)>;
Chron(std::function<void(double)>)->Chron<std::function<void(double)>>;

