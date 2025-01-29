
//Code to count and display number of rising edges produced by motor encoder on serial monitor

//Define pin connections
#define APin 3
#define BPin 2

//Declare variables
volatile unsigned long ACount = 0;
volatile unsigned long BCount = 0;


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
  Serial.begin(9600);
}


void loop()
{
  //Print tick count variables
  Serial.print("A: " + String(ACount));
  Serial.print(" B: " + String(BCount) + "\n");
  delay(200); //Delay to keep serial monitor readable
}

//ISRs
void ATick()
{
  ACount++; //Increment ACount
}

void BTick()
{
  BCount++; //Increment BCount
}
