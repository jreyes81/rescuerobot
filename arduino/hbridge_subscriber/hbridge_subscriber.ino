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
 *                                  Left    angular.z = ?
 * 
 * Raytheon Radar Guided Rescue Robot
 * Cal State LA
 * 
 */

//#include <Hercules.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8.h>

ros::NodeHandle nh; // instantiating node handle to create publishers and subscribers
// This also takes care of our serial port communication
std_msgs::Int8 int_msg; // Creating message instance

const int rob_spd = 200; // Set at 20 percent of robot speed 
//pins that will control speed of motors, enable PWM

int ENA = 30;//teensy pin 30

int ENB = 2; //teensy pin 2

//right motors

int IN1 = 10;

int IN2 = 9;

//left motors

int IN3 = 7;

int IN4 = 6;


void wheel_cb( const geometry_msgs::Twist& cmd_msg){ // message name cmd_msg
  // Insert how we want hercules wheels to move here

  float linear = cmd_msg.linear.x;
  float angular = cmd_msg.angular.z;
  int_msg.data = 1; //Sends info about scan mode to have robot 
  // not scan!

  if (linear == 20 && angular == 0){ // Makes robot go forward
    digitalWrite(ENA, HIGH);

    digitalWrite(ENB, HIGH);

    digitalWrite(IN1, LOW);

    digitalWrite(IN2, HIGH);

    digitalWrite(IN3, LOW);

    digitalWrite(IN4, HIGH);  // Might need to put a delay to make robot go forwards for a few seconds
    //delay(1000);
    int_msg.data = 2; // sends True boolean to have robot
    // resume scanning once it is done moving  
    // To control speed add analogwrite()
     
  }
  if (linear == 0 && angular == 20){ // Makes robot rotate right
    digitalWrite(ENA, HIGH);

    digitalWrite(ENB, LOW);

    //digitalWrite(IN1, LOW);

    //digitalWrite(IN2, HIGH);

    digitalWrite(IN3, HIGH);

    digitalWrite(IN4, HIGH);

    //delay(1000);
    int_msg.data = 2; // sends True boolean to have robot
    // resume scanning once it is done moving 
    
  }
  if(linear == 0 && angular == 0 && int_msg.data == 1){ // Makes robot stop
    digitalWrite(ENA, LOW);

    digitalWrite(ENB, LOW);
  }
  
}

ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("cmd_vel" , wheel_cb); 
ros::Publisher scan_once_return("scan_once_return", &int_msg);

void setup()
{
  nh.initNode();
  nh.subscribe(sub_cmd_vel); // Subscribing to topic "cmd_vel" with nickname sub_cmd_vel
  nh.advertise(scan_once_return); // Publishing to topic "scan_once_mode"
  //define the mode of the pins, are they pins sending data to outside peripherals? or recieving data? .....

  pinMode(ENA, OUTPUT);

  pinMode(ENB, OUTPUT);

  pinMode(IN1, OUTPUT);

  pinMode(IN2, OUTPUT);

  pinMode(IN3, OUTPUT);

  pinMode(IN4, OUTPUT);
}

void loop()
{
  //nh.spinOnce();
  scan_once_return.publish( &int_msg );
  nh.spinOnce();
  delay(1);
}
