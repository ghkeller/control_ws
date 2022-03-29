#include <ros/ros.h>
#include <mavros_msgs/PositionTarget.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <memory.h>
#include <unistd.h>


#define KEYCODE_w 0x77 
#define KEYCODE_a 0x61
#define KEYCODE_s 0x73
#define KEYCODE_d 0x64
#define KEYCODE_right 0x43 
#define KEYCODE_left 0x44
#define KEYCODE_up 0x41
#define KEYCODE_down 0x42

#define VEL_MAX 5.0f
#define YAW_RATE_MAX 0.2f

class TeleopMavros
{
public:
  TeleopMavros();
  void keyLoop();

private:

  
  ros::NodeHandle nh_;
  ros::Publisher pos_pub_;
  
};

TeleopMavros::TeleopMavros()
{
	pos_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("mavthread/setpoint_raw/local", 1);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
	(void)sig;
	tcsetattr(kfd, TCSANOW, &cooked);
	ros::shutdown();
	exit(0);
}


int main(int argc, char** argv)
{
   ros::init(argc, argv, "teleop_mavros");
  TeleopMavros teleop_mavros;

  signal(SIGINT,quit);

  teleop_mavros.keyLoop();
  
  return(0);
}


void TeleopMavros::keyLoop()
{
  char c;
  bool dirty=false;


  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the vehicle.");


  for(;;)
  {
    // get the next event from the keyboard  
    if(::read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    mavros_msgs::PositionTarget tp;
    tp.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    tp.type_mask = mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY |
                  mavros_msgs::PositionTarget::IGNORE_PZ | mavros_msgs::PositionTarget::IGNORE_AFX |
                  mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ |
                  mavros_msgs::PositionTarget::IGNORE_YAW;
    tp.position.x = 0.0f;
    tp.position.y = 0.0f;
    tp.position.z = 0.0f;
    tp.acceleration_or_force.x = 0.0f;
    tp.acceleration_or_force.y = 0.0f;
    tp.acceleration_or_force.z = 0.0f;
    tp.velocity.x = 0.0f;
    tp.velocity.y = 0.0f;
    tp.velocity.z = 0.0f;
    tp.yaw_rate = 0.0f;

    switch(c)
    {
      case KEYCODE_w:
        std::cout << "UP" << std::endl;
        tp.velocity.z = VEL_MAX;
        dirty = true;
        break;
      case KEYCODE_s:
        // ROS_DEBUG("RIGHT");
        std::cout << "DOWN" << std::endl;
        tp.velocity.z = -VEL_MAX;
        dirty = true;
        break;
      case KEYCODE_a:
        // ROS_DEBUG("UP");
        std::cout << "YAW LEFT" << std::endl;
        tp.yaw_rate = YAW_RATE_MAX;
        dirty = true;
        break;
      case KEYCODE_d:
        // ROS_DEBUG("DOWN");
        std::cout << "YAW RIGHT" << std::endl;
        tp.yaw_rate = -YAW_RATE_MAX;
        dirty = true;
        break;
      case KEYCODE_left:
        std::cout << "ROLL LEFT" << std::endl;
        tp.velocity.x = -VEL_MAX;
        dirty = true;
        break;
      case KEYCODE_right:
        // ROS_DEBUG("RIGHT");
        std::cout << "ROLL RIGHT" << std::endl;
        tp.velocity.x = VEL_MAX;
        dirty = true;
        break;
      case KEYCODE_up:
        // ROS_DEBUG("UP");
        std::cout << "PITCH FORWARD" << std::endl;
        tp.velocity.y = VEL_MAX;
        dirty = true;
        break;
      case KEYCODE_down:
        // ROS_DEBUG("DOWN");
        std::cout << "PITCH BACKWARD" << std::endl;
        tp.velocity.y = -VEL_MAX;
        dirty = true;
        break;
    }

    if(dirty ==true)
    {
      pos_pub_.publish(tp);    
      dirty=false;
    }
  }


  return;
}
