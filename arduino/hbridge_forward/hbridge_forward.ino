
/*
 * Hercules Skid Steering
 * 
 * Created By: Jeovanny Reyes
 * Created On: 2/24/18 
 * 
 * Input: None
 * Output: Physical movements of Hercules' wheels
 * 
 * Publisher: None
 * Subscriber: None
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

//#include <Hercules.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle nh; // instantiating node handle to create publishers and subscribers
// This also takes care of our serial port communication

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


void forward() // Going forward
{
    digitalWrite(ENA, HIGH); // Going straight ahead
    digitalWrite(ENB, HIGH);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }

void wheel_cb( const geometry_msgs::Twist& cmd_msg){ // message name cmd_msg
  // Insert how we want hercules wheels to move here

  float linear = cmd_msg.linear.x;
  float angular = cmd_msg.angular.z;

  if (linear == 20 && cmd_msg.angular.z == 0){ // Makes robot go forward
    //digitalWrite(ENA, HIGH);

    //digitalWrite(ENB, HIGH);

    //digitalWrite(IN1, LOW);

    //digitalWrite(IN2, HIGH);

    //digitalWrite(IN3, LOW);

    //digitalWrite(IN4, HIGH); 

    // To control speed add analogwrite()
     
  //}
  //if (cmd_msg.linear.x == 0 && cmd_msg.angular.z == 20){ // Makes robot rotate right
    //digitalWrite(ENA, HIGH);

    //digitalWrite(ENB, LOW);

    //digitalWrite(IN1, LOW);

    //digitalWrite(IN2, HIGH);

    //digitalWrite(IN3, HIGH);

    //digitalWrite(IN4, HIGH);
 // }
  //if(cmd_msg.linear.x == 0 && cmd_msg.angular.z == 0){ // Makes robot stop
    //digitalWrite(ENA, LOW);

    //digitalWrite(ENB, LOW);
  //}
  
//}

//ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("cmd_vel" , wheel_cb);

void setup()
{
  nh.initNode();
  //nh.subscribe(sub_cmd_vel); // Subscribing to topic "cmd_vel" with nickname sub_cmd_vel
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
  nh.spinOnce();
  delay(1); // In milliseconds
}
