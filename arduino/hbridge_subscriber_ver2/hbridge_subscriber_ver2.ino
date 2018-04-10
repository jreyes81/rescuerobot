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
 * TIP for Debugging: run "roscore" on one seperate terminal
 *                    run "rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600"
 *                    Note: _port may be different when adrduino is connected 
 *                    and baud rate parameter can be changed to satisfy data transfer rate
 * 
 * Raytheon Radar Guided Rescue Robot
 * Cal State LA
 * 
 */

//#include <Hercules.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

//class hbridge_controller
//{
  //public:
    //ros::NodeHandle nh;
    //ros::Subscriber
  
  //}

ros::NodeHandle nh; // instantiating node handle to create publishers and subscribers
// This also takes care of our serial port communication

std_msgs::Float32 int_msg; // Creating message instance

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
    analogWrite(ENA, 150); // This sets the speed of the left motor
    analogWrite(ENB, 150); // This sets the speed of the right motor
    //digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    //digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);  // Might need to put a delay to make robot go forwards for a few seconds
    //int_msg.data = 2 //send value of 2 to have robot scan again in "Data Collection Mode"
  }

void right_turn() // Function to make robot go right
{
    analogWrite(ENA, 150); // For left motor
    //delay(2000); Does not work here!!
    analogWrite(ENB, -150);
    //delay(2000); Does not work here !!
    digitalWrite(IN4, HIGH);
    //delay(2000); // Waits 2 seconds before left wheel starts moving!
    digitalWrite(IN2, HIGH);
    
  }

void left_turn() // Function to make robot go left
{
  analogWrite(ENA, -150);
  analogWrite(ENB, 150);
  //digitalWrite(ENB, HIGH); // For left motor
  //analogWrite(ENB, 200); // This sets the speed of the left motor
  //digitalWrite(ENA, LOW); 
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

ros::Publisher scan_once_return("scan_once_return", &int_msg);

void wheel_cb( const geometry_msgs::Twist& cmd_msg){ // message name cmd_msg. Used to be const instead of int
  linear = cmd_msg.linear.x;
  angular = cmd_msg.angular.z;
  to_stop = cmd_msg.linear.z;
  //int_msg.data = 1; //Sends info about scan mode to have robot not scan!

  if (linear == 20){
    left_turn();
    delay(2000);
    if (angular == 20){
      right_turn();
      delay(2000);
      if (to_stop == -1){
        straight();
        delay(2000);
      }
    }
  }

  if (linear == 20 && angular == 0)// Makes robot go forward
  {
    right_turn();
    //delay(20);
    //straight(); //robot goes straight
    //delay(20); // For 2 seconds
    stop_robot(); // Robot then stops
    int_msg.data = 2; // State data to make robot scan again
    scan_once_return.publish( &int_msg ); // state info of 2 (robot scan again)
    delay(1);
    }

   if (linear == 0 && angular == 20)// Makes robot rotate right
   {
    straight();
    right_turn(); // robot rotates right
    delay(1000); // For 1 second
    straight(); // robot goes straight
    delay(1000); // For 1 second
    left_turn();
    delay(1000);
    straight();
    delay(1000);
    right_turn();
    delay(1000);
    straight();
    delay(2000);
    stop_robot();
    int_msg.data = 2; // State data to make robot scan again
    scan_once_return.publish( &int_msg ); // state info of 2 (robot scan again)
    delay(1);
    }

    if(linear == 0 && angular == 0 ) // Has robot not move and stop moving when stop button is pressed
    { 
      stop_robot(); // Function to make robot stop
      int_msg.data = 2; // State data to make robot scan again
    }
}
 

ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("cmd_vel" , wheel_cb); 
//ros::Publisher scan_once_return("scan_once_return", &int_msg);

void setup()
{
  nh.initNode();
  nh.advertise(scan_once_return); // Publishing to topic "scan_once_mode"
  nh.subscribe(sub_cmd_vel); // Subscribing to topic "cmd_vel" with nickname sub_cmd_vel
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  //pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  //pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  //int_msg.data = 1; //Sends info about scan mode to have robot not scan!
}

void loop()
{
  //scan_once_return.publish( &int_msg );
  nh.spinOnce(); // callback gets implemented
  //scan_once_return.publish( &int_msg ); // state info of 2 (robot scan again)
  //delay(1);
}
