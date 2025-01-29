
//code which uses proportional control loop to control motor speed using motor encoder

//Pin definitions
#define M1A 9
#define EncoderA 3

//Encoder tick counter
volatile unsigned long ACounter = 0;

//Proportional control variables
int maxPWM = 100; //Maximum PWM value
int targetTicks = 5; //Set point
int timePeriod = 10; //Time period pulses are measured over
int proportionalGain = maxPWM/targetTicks; //Gain
int output; //Calculated in loop
int error; //Calculated in loop

void setup() 
{
  //Declare inputs
  pinMode(EncoderA, INPUT);

  //Attach interrupt
  attachInterrupt(digitalPinToInterrupt(EncoderA), ATick, RISING);
}


void loop() 
{
  //Record current number of pulses counted
  unsigned long startingTicks = ACounter;
  
  //Count pulses for desired period of time
  delay(timePeriod);
  
  //Find number of pulses that occured during delay
  unsigned long tickDifference = ACounter - startingTicks;
  
  //Calculate error in measured pulses vs target pulses
  error = targetTicks - tickDifference;
  
  //Calculate output as PWM %
  output = error * proportionalGain;
  
  //Convert PWM % to analogWrite value and write to motor pin
  PWMMotor(output);
}


//Convert PWM % to analogWrite value
void PWMMotor(int PWMValue)
{
  int writeValue;
  
  if(PWMValue > 0) //Check that value is greater than 0
  { 
  writeValue = 255/100 * PWMValue;
  }
  else{writeValue = 0;}
  
  analogWrite(M1A, writeValue);
}

//ISR to count encoder pulses
void ATick()
{
  ACounter++;
}
