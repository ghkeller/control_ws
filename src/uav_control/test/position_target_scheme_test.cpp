// position_target_scheme_test.cpp: test suite for the interfacing with 
// MAVLink APIs

/* system includes */
#include <iostream>
#include <string>
#include <cstddef>
#include <mavros_msgs/PositionTarget.h>

/* local includes */
#include "PositionTargetScheme.h"
#include <ros/types.h>

/*

PositionTarget_()
  : header()
  , coordinate_frame(0)
  , type_mask(0)
  , position()
  , velocity()
  , acceleration_or_force()
  , yaw(0.0)
  , yaw_rate(0.0)  {
  }

enum { 
    FRAME_LOCAL_NED = 1u, 
    FRAME_LOCAL_OFFSET_NED = 7u, 
    FRAME_BODY_NED = 8u, 
    FRAME_BODY_OFFSET_NED = 9u, 
    IGNORE_PX = 1u, 
    IGNORE_PY = 2u, 
    IGNORE_PZ = 4u, 
    IGNORE_VX = 8u, 
    IGNORE_VY = 16u, 
    IGNORE_VZ = 32u, 
    IGNORE_AFX = 64u, 
    IGNORE_AFY = 128u, 
    IGNORE_AFZ = 256u, 
    FORCE = 512u, 
    IGNORE_YAW = 1024u, 
    IGNORE_YAW_RATE = 2048u, 
  }; 


*/
using namespace mavros_msgs;

namespace position_target_scheme_tests
{

int test_1(void) {
    std::cout << "Test 1: instantiating an instance of the PositionTargetScheme object." << std::endl;

    PositionTargetScheme pts;

    std::cout << &pts << std::endl;

    return 0;
}

int test_2(void) {
    std::cout << "Test 2: attempting to set a member attribute of the object." << std::endl;

    PositionTargetScheme pts;

	pts.setName("");

    std::cout << pts.getName() << std::endl;

    return 0;
}

int test_3(void) {
    std::cout << "Test 3: Attempting to set the ignore flags for the message struct." << std::endl;

    PositionTargetScheme pts;
	uint16_t flags = 4;
	pts.setIgnoreFlags(flags);

    std::cout << pts.getName() << std::endl;

    return 0;
}

int test_4(void) {
    std::cout << "Test 4: Print out the ignore flags." << std::endl;

    PositionTargetScheme pts;
	string _name = "main_sp_scheme";
	uint16_t _flags = PositionTarget::IGNORE_VX | 
						PositionTarget::IGNORE_VY | 
						PositionTarget::IGNORE_VZ | 
						PositionTarget::IGNORE_AFX | 
						PositionTarget::IGNORE_AFY | 
						PositionTarget::IGNORE_AFZ;
	uint16_t _coord_frame = PositionTarget::FRAME_LOCAL_NED; 
	pts.setName(_name);
	pts.setIgnoreFlags(_flags);
	pts.setCoordinateFrame(_coord_frame);

    std::cout << pts.getName() << std::endl;
	std::cout << "IGNORE_FLAGS: " << pts.getIgnoreFlags() << std::endl;
	std::cout << "COORD_FRAME: " << pts.getCoordinateFrame() << std::endl;

    return 0;
}


int test_5(void) {
    std::cout << "Test 5: add one setpoint to the setpoint queue." << std::endl;

    PositionTargetScheme pts;
	string _name = "main_sp_scheme";
	uint16_t _flags = PositionTarget::IGNORE_VX | 
						PositionTarget::IGNORE_VY | 
						PositionTarget::IGNORE_VZ | 
						PositionTarget::IGNORE_AFX | 
						PositionTarget::IGNORE_AFY | 
						PositionTarget::IGNORE_AFZ;
	uint16_t _coord_frame = PositionTarget::FRAME_LOCAL_NED; 
	pts.setName(_name);
	pts.setIgnoreFlags(_flags);
	pts.setCoordinateFrame(_coord_frame);

    std::cout << pts.getName() << std::endl;
	std::cout << "IGNORE_FLAGS: " << pts.getIgnoreFlags() << std::endl;

	mavros_msgs::PositionTarget sp;
	bool success = pts.addSetpointToQueue(sp);

	if (success == true) {
		std::cout << "successfully added setpoint to the queue" << std::endl;
	} else {
		std::cerr << "was not successful in adding setpoint to the queue" << std::endl;
	}

    return 0;
}

int test_6(void) {
    std::cout << "Test 6: get the setpoint from the queue of size one" << std::endl;

    PositionTargetScheme pts;
	string _name = "main_sp_scheme";
	uint16_t _flags = PositionTarget::IGNORE_VX | 
						PositionTarget::IGNORE_VY | 
						PositionTarget::IGNORE_VZ | 
						PositionTarget::IGNORE_AFX | 
						PositionTarget::IGNORE_AFY | 
						PositionTarget::IGNORE_AFZ;
	uint16_t _coord_frame = PositionTarget::FRAME_LOCAL_NED; 
	pts.setName(_name);
	pts.setIgnoreFlags(_flags);
	pts.setCoordinateFrame(_coord_frame);

    std::cout << pts.getName() << std::endl;
	std::cout << "IGNORE_FLAGS: " << pts.getIgnoreFlags() << std::endl;

	mavros_msgs::PositionTarget sp;
	sp.position.x = 3;
	bool success = pts.addSetpointToQueue(sp);

	if (success == true) {
		std::cout << "successfully added setpoint to the queue" << std::endl;
	} else {
		std::cerr << "was not successful in adding setpoint to the queue" << std::endl;
	}

	if (pts.queueEmpty() == false) {
		mavros_msgs::PositionTarget sp_out = pts.nextSetpoint();
		//print the info of the setpoint we just got
		std::cout << sp_out.position << endl;

	} else {
		std::cerr << "cannot get sp from queue -- the queue is empty" << std::endl;
	}

    return 0;
}


} //namespace position_target_scheme_tests END

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

    std::cout << "Test 2:" << std::endl;
    result = test_2();
    if ( result < 0 ) {
        std::cout << "Test failed." << std::endl;
    } else {
        std::cout << "Test succeeded." << std::endl;
    }

    std::cout << "Test 3:" << std::endl;
    result = test_3();
    if ( result < 0 ) {
        std::cout << "Test failed." << std::endl;
    } else {
        std::cout << "Test succeeded." << std::endl;
    }

    std::cout << "Test 4:" << std::endl;
    result = test_4();
    if ( result < 0 ) {
        std::cout << "Test failed." << std::endl;
    } else {
        std::cout << "Test succeeded." << std::endl;
    }

    std::cout << "Test 5:" << std::endl;
    result = test_5();
    if ( result < 0 ) {
        std::cout << "Test failed." << std::endl;
    } else {
        std::cout << "Test succeeded." << std::endl;
    }

    std::cout << "Test 6:" << std::endl;
    result = test_6();
    if ( result < 0 ) {
        std::cout << "Test failed." << std::endl;
    } else {
        std::cout << "Test succeeded." << std::endl;
    }
}

