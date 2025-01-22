//Week 2 - DC Motors Lab Worksheet
//linear acceleration of motor, braking, and coasting


#define M1A 9 //M1A is connected to Arduino pin 9
#define M1B 11 //M1B is connected to Arduio pin 11

int i = 0; //indexing variable
int accDelay = 50; //time in ms between consecutive increases in motor speed

void setup() 
{
  //setting motor control pins as outputs:
  pinMode(M1A, OUTPUT);
  pinMode(M1B, OUTPUT);
}


void loop() 
{
  //If motor is stopped, gradually increase speed until motor is at full speed
  if(i<255)
  {
    while(i!=255)
    {
    analogWrite(M1A, i);
    analogWrite(M1B, 0);

    delay(accDelay);

    i++;
    }


    //1. BRAKING FOR 5 SECONDS AFTER REACHING MAX MOTOR SPEED
    //digitalWrite(M1A, 0); 
    //delay(5000);
    //-------------------------------------------------------


    //2. COASTING FOR 5 SECOND AFTER REACHING MAX MOTOR SPEED
    digitalWrite(M1B, 1);
    delay(5000);
    digitalWrite(M1B, 0); //returning to forward motion of motor
    //-------------------------------------------------------
  }


    //If motor is at max speed, gradually decrease until motor stops
    if(i==255)
  {
    while(i!=0)
    {
    analogWrite(M1A, i);
    analogWrite(M1B, 0);

    delay(accDelay);

    i--;
    }
  }

}
