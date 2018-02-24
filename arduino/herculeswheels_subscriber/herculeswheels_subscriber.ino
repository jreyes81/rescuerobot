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
 *                    Translations: Forward linear.x = 1
 *                                  Back    linear.x = -1
 *                                  
 *                    Rotations   : Right   angular.z = 1
 *                                  Left    angular.z = -1
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

void wheel_cb( const geometry_msgs::Twist& cmd_msg){ // message name cmd_msg
  // Insert how we want hercules wheels to move here
  //digitalWrite(13, HIGH-digitalRead(13));   // blink the led
}


//ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb );
ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("cmd_vel" , wheel_cb);

void setup()
{
  //pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub_cmd_vel); // Subscribing to topic "cmd_vel" with nickname sub_cmd_vel
}

void loop()
{
  nh.spinOnce();
  delay(1);
}
