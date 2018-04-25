/*
 * Hercules Skid Steering
 * 
 * Created By: Jeovanny Reyes
 * Created On: 2/24/18 
 * 
 * Input: geometry_msgs/Twist that contain translation and rotation commands
 * Output: Physical movements of Hercules' wheels
 * 
 * Publisher: None
 * Subscriber: "cmd_vel" receiving a geometry msg from "herc_nav.py"
 * 
 * Navigation Method:
 *                    Translations: Forward linear.x =
 *                                  Back    linear.x = 
 *                                  
 *                    Rotations   : Right   angular.z = 
 *                                  Left    angular.z = 
 * 
 * Raytheon Radar Guided Rescue Robot
 * Cal State LA
 * 
 */

#include <Hercules.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle nh; // instantiating node handle to create publishers and subscribers
// This also takes care of our serial port communication

const int rob_spd = 20; // Set at 20 percent of robot speed 

void wheel_cb( const geometry_msgs::Twist& cmd_msg){ // message name cmd_msg
  // Insert how we want hercules wheels to move here

  float linear = cmd_msg.linear.x;
  float angular = cmd_msg.angular.z;

  if (cmd_msg.linear.x == 20 && cmd_msg.angular.z == 0){ // Makes robot go forward
    MOTOR.setSpeedDir(rob_spd,DIRF); 
  }
  if (cmd_msg.linear.x == 0 && cmd_msg.angular.z == 20){ // Makes robot rotate
    MOTOR.setSpeedDir1(rob_spd-10,DIRF);
    MOTOR.setSpeedDir2(rob_spd,DIRF);
  }
  if(cmd_msg.linear.x == 0 && cmd_msg.angular.z == 0){ // Makes robot stop
    MOTOR.setStop1();
    MOTOR.setStop2(); 
  }
  
}

ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("cmd_vel" , wheel_cb);

void setup()
{
  nh.initNode();
  nh.subscribe(sub_cmd_vel); // Subscribing to topic "cmd_vel" with nickname sub_cmd_vel
}

void loop()
{
  nh.spinOnce();
  delay(1);
}
