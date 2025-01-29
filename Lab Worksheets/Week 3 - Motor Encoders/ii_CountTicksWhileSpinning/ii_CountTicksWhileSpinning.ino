//Define pin connections
#define APin 3
#define BPin 2
#define M1A 9
#define M1B 10

//Define counter variables
volatile unsigned long ACount = 0;
volatile unsigned long BCount = 0;

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
  Serial.print("A: " + String(ACount));
  Serial.print(" B: " + String(BCount) + "\n");
}

void ATick()
{
  ACount++; //Increment ACount
}

void BTick()
{
  BCount++; //Increment BCount
}
