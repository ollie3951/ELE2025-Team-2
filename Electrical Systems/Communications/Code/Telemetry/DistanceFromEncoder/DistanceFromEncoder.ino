//Define pin connections
#define APin 17
#define BPin 16

//Declare variables
volatile unsigned long ACount = 0;
volatile unsigned long BCount = 0;

//variables
float distanceCM = 0; //initialise at zero distance


void setup()
{
  //Declare inputs
  pinMode(APin, INPUT);
  pinMode(BPin, INPUT);
  //Attach interrupts
  //Attach interrupt to APin, which will cause function ATick to execute on
  //rising pulse
  attachInterrupt(digitalPinToInterrupt(APin), ATick, RISING);
  //Attach interrupt to BPin, which will cause function BTick to execute on
  //rising pulse
  attachInterrupt(digitalPinToInterrupt(BPin), BTick, RISING);
  //Open serial monitor
  Serial.begin(115200);
}


void loop()
{


  distanceCM = 0.0217*ACount;

  Serial.println(distanceCM);
  delay(200);


}

//ISRs
void ATick(){
ACount++; //Increment ACount
}
void BTick(){
BCount++; //Increment BCount
}
