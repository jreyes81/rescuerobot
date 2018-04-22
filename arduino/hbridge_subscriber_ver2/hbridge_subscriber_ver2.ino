/*
 * Hercules Skid Steering
 * 
 * Created By: Jeovanny Reyes
 * Created On: 2/24/18 
 * 
 * Input: geometry_msgs/Twist that contain translation and rotation commands
 * Output: Physical movements of Hercules' wheels
 * 
 * Publisher: "scan_once_mode" sends out boolean True to radar display
 * Subscriber: "cmd_vel" receiving a geometry msg from "herc_nav.py"
 * 
 * Navigation Method:
 *                    Translations: Forward linear.x = 20
 *                                  Back    linear.x = ?
 *                                  
 *                    Rotations   : Right   angular.z = 20
 *                                  Left    angular.z = -20
 * 
 * TIP for Debugging: run "roscore" on one seperate terminal
 *                    run "rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600"
 *                    to have this run based on manual commands do 
 *                    "rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'"
 *                    where the linear and angular parameters can be channged   
 *                    Note: _port may be different when adrduino is connected 
 *                    and baud rate parameter can be changed to satisfy data transfer rate
 *                    
 *                    DO NOT PUT DELAYS IN THE FUNCTIONS THAT MAKE THE ROBOT MOVE!!!!!!!
 * 
 * Raytheon Radar Guided Rescue Robot
 * Cal State LA
 * 
 */

#include <ros.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle nh; // instantiating node handle to create publishers and subscribers
// This also takes care of our serial port communication

int ENA = 30;//teensy pin 30. For Left Motor
int ENB = 2; //teensy pin 2. For Right Motor

//right motors
//int IN1 = 10;
int IN2 = 9; // Only using this one!!

//left motors
//int IN3 = 7;
int IN4 = 6; // Only using this one!!

float linear = 0; // Contains linear velocity commands
float angular = 0; // Contains angular velocity commands
float to_stop = 0;

void straight() // Function to make robot go straight
{
    analogWrite(ENA, 200); // This sets the speed of the left motor
    analogWrite(ENB, 200); // This sets the speed of the right motor
    digitalWrite(IN2, HIGH);
    digitalWrite(IN4, HIGH);  // Might need to put a delay to make robot go forwards for a few seconds
  }

void right_turn() // Function to make robot go right
{
    analogWrite(ENA, 250); // For left motor
    //digitalWrite(ENB, HIGH);
    analogWrite(ENB, -250);
    digitalWrite(IN4, HIGH);
    digitalWrite(IN2, HIGH);
  }

void left_turn() // Function to make robot go left
{
  analogWrite(ENA, -250);
  analogWrite(ENB, 250);
  digitalWrite(IN2, LOW);
  digitalWrite(IN4, HIGH);
  }

void stop_robot() // Function to make robot stop
{
  analogWrite(ENA, 5);
  analogWrite(ENB, 5);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN4, HIGH);
  }

void wheel_cb( const geometry_msgs::Twist& cmd_msg) // message name cmd_msg. Used to be const instead of int
{
  linear = cmd_msg.linear.x;
  angular = cmd_msg.angular.z;
  to_stop = cmd_msg.linear.z;

  if (linear == 20 && angular == 0)// Makes robot go forward
  {
    straight();
  }

  else if (linear == 0 && angular == 20)// Makes robot go right
  {
    right_turn();
  }

  else if (linear == 0 && angular == -20) // Makes robot go left
    {
      left_turn();
    }
    
  else if(linear == 0 && angular == 0 ) // Has robot stop moving
    { 
      stop_robot(); // Function to make robot stop
    }
}
 
ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("cmd_vel" , wheel_cb); 

void setup()
{
  nh.initNode();
  nh.subscribe(sub_cmd_vel); // Subscribing to topic "cmd_vel" with nickname sub_cmd_vel
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void loop()
{
  nh.spinOnce(); // callback gets implemented
}
