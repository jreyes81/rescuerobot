const unsigned int echo = 3;
const unsigned int trigr = 4;

double timing;
double distance;
double speedofsound = (0.0343/2);

void setup() {
  pinMode(echo, INPUT);
  pinMode(trigr, OUTPUT);

  Serial.begin(9600);
}

void PrintDistance() {
  digitalWrite(trigr, LOW);
  delayMicroseconds(1);
  digitalWrite(trigr, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigr, LOW);

  timing = pulseIn(echo, HIGH);
  distance = speedofsound*timing;
  Serial.print("Distance: ");
  Serial.println(distance);
  Serial.println("");
  delay(1000);
}

void loop() {
  PrintDistance();

}
