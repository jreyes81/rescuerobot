/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int servofeed = A14; // analog pin
int pos = 0;    // variable to store the servo position
int temp = 0; // variable that will store the analog feedback signal
int count = 0; // To keep count of data points
int servo_pos = 0;

void setup() {
  myservo.attach(5);  // attaches the servo on pin 9 to the servo object
  Serial.begin(9600);
}

void loop() {
  for (pos = 15; pos <= 165; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    count = count + 1; // increment
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(30);                       // waits 15ms for the servo to reach the position
    temp = analogRead(servofeed);
    servo_pos = ((0.3956*temp)-75.165); // Angle of servo
    Serial.print(servo_pos);
    Serial.print(",");
    Serial.println(count);
    delay(30);
  }
  delay (100);
  for (pos = 165; pos >= 15; pos -= 1) { // goes from 180 degrees to 0 degrees
    count = count + 1;
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(30);                       // waits 15ms for the servo to reach the position
    temp = analogRead(servofeed);
    servo_pos = ((0.3956*temp)-75.165);
    Serial.println(servo_pos);
    Serial.print(",");
    Serial.println(count);
    delay(30);
  }
  delay(100);
}

