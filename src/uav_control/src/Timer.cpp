// system includes
#include <chrono>

// local includes
#include <Timer.h>

// constructor
Timer::Timer()
{
	this->status = Status::INACTIVE;
}

// setters
void Timer::setTimeout(double milliseconds)
{
	this->msecs_duration = milliseconds;
}

// public methods
void Timer::start()
{
	this->start_time = std::chrono::steady_clock::now();
	this->status = Status::ACTIVE;
}

void Timer::reset(void)
{
    // simply set the timer to inactive
	this->status = Status::INACTIVE;
}

Timer::Status Timer::check(void)
{
	
	if (this->status == Status::ACTIVE) {
		std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
		std::chrono::duration<float, std::milli> msecs_elapsed = now - this->start_time;
		if (msecs_elapsed.count() >= this->msecs_duration) {
			this->status = Status::DONE;
		}
	}

	return this->status;
}
