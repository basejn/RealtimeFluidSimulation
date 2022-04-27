#include <chrono>
#include<string>

class Chron
{
public:
	std::chrono::high_resolution_clock::time_point tStart;
	void(*callback)(double);
	Chron(void(*callback)(double)) {
		tStart = std::chrono::high_resolution_clock::now();
		this->callback = callback;
	}

	~Chron() {
		auto tEnd = std::chrono::high_resolution_clock::now();
		auto t = std::chrono::duration_cast<std::chrono::microseconds>(tEnd - tStart).count() / 1000.0;
		callback(t);
	}

};