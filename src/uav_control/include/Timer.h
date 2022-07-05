#pragma once 

#include <chrono>

class Timer
{
	public:
	// constructor
	Timer();

	// enums
	enum class Status{INACTIVE, ACTIVE, DONE};

	// setters
	void setTimeout(double milliseconds);

	// public methods
	void start();
    void reset();
	Status check();

	private:
	std::chrono::steady_clock::time_point start_time;
	double msecs_duration;
	Status status;
};
