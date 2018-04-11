/*
* HC-SR04 ultrasound sensor - simple detection code without ROS
* Created By: Jeovanny Reyes
* Raytheon Radar Guided Rescue Robot
* Cal State LA
*
*/
 
// Defines Trig and Echo pins of the Ultrasonic Sensor
const int trigPin = 4; //digital pin
const int echoPin = 3; // digital pin
int count = 0;

// Variables for the duration and the distance
long duration;
int distance;


void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
}

void loop() {
  count++;
  distance = calculateDistance(); // Ultrasound publishing
  Serial.print(distance);
  Serial.print("cm");
  Serial.print(",");
  Serial.println(count);
  
  delay(30);
}

// Function for calculating the distance measured by the Ultrasonic sensor
float calculateDistance(){ 
  
  digitalWrite(trigPin, LOW); 
  delayMicroseconds(2);
  
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH); 
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  duration = pulseIn(echoPin, HIGH); // Reads the echoPin, returns the sound wave travel time in microseconds
  distance= duration*0.034/2;
  return distance; // Distance is returned in cm

}
