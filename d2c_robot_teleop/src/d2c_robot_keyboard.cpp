#include <ros/ros.h>
#include "d2c_robot_msgs/D2cRobot.h"

#include <stdio.h>
#include <unistd.h>
#include <termios.h>

#include <map>

// Map for movement keys
std::map<char, std::vector<float>> moveBindings
{
  {'q', {0.0}},  // Initialize
  {'w', {1.0}}, // Serving motion
  {'e', {2.0}},  // Cleaning motion
  
 
};

// Reminder message
const char* msg = R"(
Hi! this is Serving Robot Controller!
---------------------------
Motion Command :
q : Initialize
w : Serving Motion
e : Cleaning Motion
           
anything else : Initialize

CTRL-C to quit
)";

float x(0); 
char key(' ');

// For non-blocking keyboard inputs
int getch(void)
{
  int ch;
  struct termios oldt;
  struct termios newt;

  // Store old settings, and copy to new settings
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  // Make required changes and apply the settings
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);

  // Get the current character
  ch = getchar();

  // Reapply old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return ch;
}

int main(int argc, char** argv)
{
  // Init ROS node
  ros::init(argc, argv, "d2c_robot_keyboard");
  ros::NodeHandle nh;

  // Init cmd_vel publisher
  ros::Publisher pub = nh.advertise<d2c_robot_msgs::D2cRobot>("d2c_robot_msg", 1);


  d2c_robot_msgs::D2cRobot d2c;

  printf("%s", msg);
  //printf("\rCurrent: speed %f\tturn %f | Awaiting command...\r", speed, turn);

  while(true){

    // Get the pressed key
    key = getch();

    // If the key corresponds to a key in moveBindings
    if (moveBindings.count(key) == 1)
    {
      x = moveBindings[key][0];
      printf("\rLast command: %c   ", key);
    }

    // Otherwise, set the robot to initialize position
    else
    {
      x = 0.0;

      // If ctrl-C (^C) was pressed, terminate the program
      if (key == '\x03')
      {
        printf("\n\n I LOVE ROBOT \n\n ");
        break;
      }

      printf("\rInvalid command! %c", key);
    }
    ROS_INFO("%f", x);
    // Update the Twist message
    d2c.motion_command = x;
    
    // Publish it and resolve any remaining callbacks
    pub.publish(d2c);
    
    ros::spinOnce();
  }

  return 0;
}