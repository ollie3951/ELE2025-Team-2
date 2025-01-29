//Define pin connections
#define APin 3
#define BPin 2
#define M1A 9
#define M1B 11

//Define counter variables
volatile unsigned long ACount = 0;
volatile unsigned long BCount = 0;

//Variable to hold previous ticks so we can compare with current to determine when a full rotation (~948) has occured
unsigned int prevTicks = 0;

//Variable which holds current direction of rotation of the motor
bool clockwise = true;

void setup() 
{
  //Declare inputs
  pinMode(APin, INPUT);
  pinMode(BPin, INPUT);
  
  //Attach interupts
  //Attach interrupt to APin, which will cause function ATick to execute on
  //rising pulse
  attachInterrupt(digitalPinToInterrupt(APin), ATick, RISING);
  //Attach interrupt to BPin, which will cause function BTick to execute on
  //rising pulse
  attachInterrupt(digitalPinToInterrupt(BPin), BTick, RISING);
  
  //Open serial monitor
  Serial.begin(9600);
  
  //Start motors
  analogWrite(M1A, 25);
}


void loop() 
{
  //printing current ticks to serial monitor for troubleshooting
  Serial.print("A: " + String(ACount));
  Serial.print(" B: " + String(BCount) + "\n");


if((ACount - prevTicks) >= 948) //if a full rotation has occured since last time motor direction switched
{
  //stop motor for half a second
  analogWrite(M1A, 0);
  analogWrite(M1B, 0);
  delay(500);
 
  if(clockwise) //if previously clockwise, rotate anticlockwise
  {
    analogWrite(M1A, 0);
    analogWrite(M1B, 25);
    prevTicks = ACount; //update prevTicks variable with current count to allow next full rotation
    clockwise =  false;
  }
  else //if previously anticlockwise, rotate clockwise
  {
    analogWrite(M1B, 0);
    analogWrite(M1A, 25);
    prevTicks = ACount; //update prevTicks variable with current count to allow next full rotation
    clockwise =  true;
  }
}



}
void ATick()
{
  ACount++; //Increment ACount
}

void BTick()
{
  BCount++; //Increment BCount
}
