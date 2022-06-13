#include <iostream>
#include <string>
#include <queue>
#include <vector>

#include "ExampleFlight.h"
#include "Flight.h"

namespace testing
{

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

bool assert_neq(auto a, auto b, string test_desc = "")
{
	if (a != b) {
		cout << "Assert neq: passed >> " << test_desc << endl;
		return true;
	} else {
		cout << "Assert neq: failed >> " << test_desc << endl;
		return false;
	}

}


bool assert_nullptr(auto a, string test_desc = "")
{
	if (a == nullptr) {
		cout << "Assert nullptr: passed >> " << test_desc << endl;
		return true;
	} else {
		cout << "Assert nullptr: failed >> " << test_desc << endl;
		return false;
	}

}

} // namespace testing END

using namespace testing;

namespace flight_class_tests
{
	int flightClassInstantiation_test_1()
	{
		ExampleFlight flight();

		return 1;
	}

} // namespace method_tests END


using namespace flight_class_tests;

int main( void )
{

	int result;

    std::cout << "Test 1: instantiate a Flight object" << std::endl;
    result = flightClassInstantiation_test_1();
    if ( result == false ) {
        std::cout << "Test failed." << std::endl;
    } else {
        std::cout << "Test succeeded." << std::endl;
    }
	std::cout << std::endl;
}
