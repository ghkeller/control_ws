/* system includes */
#include <iostream>
#include <string>
#include <cstddef>
#include <chrono>

/* local includes */
#include "Timer.h"

using namespace std;

bool assert_true(bool statement, string test_desc = "")
{
	if (statement == true) {
		cout << "Assert true: passed >> " << test_desc << endl;
		return true;
	} else {
		cout << "Assert true: failed >> " << test_desc << endl;
		return true;
	}

}

bool assert_false(bool statement, string test_desc = "")
{
	if (statement == false) {
		cout << "Assert false: passed >> " << test_desc  << endl;
		return true;
	} else {
		cout << "Assert false: failed >> " << test_desc  << endl;
		return false;
	}

}

bool assert_eq(auto a, auto b, string test_desc = "")
{
	if (a == b) {
		cout << "Assert eq: passed >> " << test_desc << endl;
		return true;
	} else {
		cout << "Assert eq: failed >> " << test_desc << endl;
		return false;
	}

}


namespace timer_tests
{

bool test_1(void)
{
    cout << "Test 1: instantiating a timer" << endl;

	// instantiate the timer 
	Timer timer = Timer();

	Timer::Status current_status = timer.check();
	return assert_eq(current_status, Timer::Status::INACTIVE, "Checking that the timer initializes to 'INACTIVE'");
}

bool test_2(void)
{
    cout << "Test 2: starting the timer and checking the status" << endl;

	// instantiate the timer
	Timer timer = Timer();

	// set the duration
	timer.setTimeout(5000); //milliseconds

	// start the timer 
	timer.start();

	// immediately check the state (time should not have reached "done")
	Timer::Status current_status = timer.check();
	return assert_eq(current_status, Timer::Status::ACTIVE, "Checking that the timer has become 'ACTIVE' after start");
}

bool test_3(void)
{
    cout << "Test 3: checking that the timer correctly registers when finished" << endl;

	// instantiate the timer
	Timer timer = Timer();

	// set the duration
	timer.setTimeout(3000); //milliseconds

	// start the timer 
	timer.start();

	// immediately check the state (time should not have reached "done")
	Timer::Status current_status;
	while (current_status != Timer::Status::DONE) 
		current_status = timer.check();
	
	// kind of a dumb test, but just want to sit and make sure this takes 3 seconds to end
	return assert_eq(current_status, Timer::Status::DONE, "Checking that the timer finishes after 3 seconds");
}


}


using namespace timer_tests;

int main(void)
{
    bool result; // stores the results of the test

    std::cout << "Starting the timer tests..." << std::endl;

    std::cout << "Test 1:" << std::endl;
    result = test_1();
    if ( result == false ) {
        std::cout << "Test failed." << std::endl;
    } else {
        std::cout << "Test succeeded." << std::endl;
    }
	std::cout << std::endl;

    std::cout << "Test 2:" << std::endl;
    result = test_2();
    if ( result == false ) {
        std::cout << "Test failed." << std::endl;
    } else {
        std::cout << "Test succeeded." << std::endl;
    }
	std::cout << std::endl;

    std::cout << "Test 3:" << std::endl;
    result = test_3();
    if ( result == false ) {
        std::cout << "Test failed." << std::endl;
    } else {
        std::cout << "Test succeeded." << std::endl;
    }
	std::cout << std::endl;
}
