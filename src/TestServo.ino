#include <Servo.h>

Servo sweeper;  // create servo object to control a servo
int pos = 0;


void setup() {
  sweeper.attach(5);
  Serial.begin(9600);
}


// INCREMENTS 90 FUNCTION (0 to 90 to 180 and back)
void increments90() {
  sweeper.write(0);
  delay(1000);
  sweeper.write(90);
  delay(1000);
  sweeper.write(180);
  delay(1000);
  sweeper.write(90);
  delay(1000);
}


// SERVO SWEEP FUNCTIONS (0 to 180 and back)
// increment of 20 degrees
void sweep20() {
  // sweep motion from 0 to 180 degress
  for (pos = 0; pos <= 180; pos += 20) { 
    sweeper.write(pos);              // tell servo to go to position in variable 'pos'
    Serial.print("Position: ");
    Serial.println(pos);
    delay(500);                       // waits half a second
  }
  // back from 180 degrees to 0
  for (pos = 180; pos >= 0; pos -= 20) { 
    sweeper.write(pos);
    Serial.print("Position: ");
    Serial.println(pos);             
    delay(500);                       
  }
}

// increment of 1 degree
void sweep1() {
  // sweep motion from 0 to 180 degress
  for (pos = 0; pos <= 180; pos += 1) { 
    sweeper.write(pos);              // tell servo to go to position in variable 'pos'
    Serial.print("Position: ");
    Serial.println(pos);
    delay(500);                       // waits half a second
  }
  // back from 180 degrees to 0
  for (pos = 180; pos >= 0; pos -= 1) { 
    sweeper.write(pos);
    Serial.print("Position: ");
    Serial.println(pos);             
    delay(500);                       
  }
}

void loop() {
  sweep20();
}



