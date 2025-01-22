#define M1A 9 //M1A is connected to Arduino pin 9
#define M1B 11 //M1B is connected to Arduio pin 11


void setup() 
{
  //Motor control pins as outputs
  pinMode(M1A, OUTPUT);
  pinMode(M1B, OUTPUT);

}

void loop() 
{
  //Forward at 20% for 5 seconds
  analogWrite(M1A, 51);
  analogWrite(M1B, 0);
  delay(5000);

  //Brake for 0.5 seconds
  analogWrite(M1A, 0);
  delay(500);

  //Backwards at 20% for 5 seconds
  analogWrite(M1B, 51);
  delay(5000);

  //Brake for 5 seconds
  analogWrite(M1B, 0);
  delay(5000);

}
