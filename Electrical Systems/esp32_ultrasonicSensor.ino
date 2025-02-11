const int trigPin = 5;
const int echoPin = 18;

//define sound speed in cm/uS
#define soundSpeed 0.034

long duration;
float distance;

void setup()
{
  Serial.begin(115200); //start serial communication
  pinMode(trigPin, OUTPUT); //trigger pin is an output
  pinMode(echoPin, INPUT); //echo pin is an input
}

void loop()
{
  //clear the trigger pin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  //set trigger pin hogh for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  //read the echo pin, return sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  //calculate the distance
  distance = duration*soundSpeed/2;

  //print distance in serial monitor
  Serial.print("Distance (cm) : ");
  Serial.println(distance);

  delay(1000);
}
