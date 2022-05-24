// position_target_scheme_test.cpp: test suite for the interfacing with 
// MAVLink APIs

#include <iostream>
#include "PositionTargetScheme.h"

namespace position_target_scheme_tests
{

int test_1(void) {
    std::cout << "Test 1: instantiating an instance of the PositionTargetScheme object." << std::endl;

    PositionTargetScheme pts;

    std::cout << &pts << std::endl;

    return 0;
}

}

using namespace position_target_scheme_tests;

int main(void)
{
    int result; // stores the results of the test

    std::cout << "Starting the position target scheme tests..." << std::endl;

    std::cout << "Test 1:" << std::endl;
    result = test_1();
    if ( result < 0 ) {
        std::cout << "Test failed." << std::endl;
    } else {
        std::cout << "Test succeeded." << std::endl;
    }

}


