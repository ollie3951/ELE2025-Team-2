
//state change pin
#define statePin 2

//state variable
bool state = 0; //0 is remote control, 1 is autonomous

//prevTime to prevent button bouncing
unsigned long prevTime = 0;

int transmitData[1];

void setup() {
  
  Serial.begin(9600);

  pinMode(statePin, INPUT);

  attachInterrupt(digitalPinToInterrupt(statePin), isr_stateChange, FALLING);

}

void loop() {

  transmitData[0]=state;

  if(state==0)
  {
    Serial.print(state); Serial.println(" Remote Control");
  }
  else
  {
    Serial.print(state); Serial.println(" Autonomous");
  }

}


void isr_stateChange()
{
  if(millis()-prevTime>1000) //if more than a second has passed since last state change
  {
    prevTime = millis(); //update previous time 
    state = !state; //change state
  }
}
