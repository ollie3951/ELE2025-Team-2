//pin definitions for joystick
const int xAxis = A5;
const int yAxis = A3;

//variables to store joystick values
int xVal;
int yVal;
int mapYVal;

void setup() 
{

  //open serial monitor
  Serial.begin(9600);

  //joystick pins as inputs
  pinMode(xAxis, INPUT);
  pinMode(yAxis, INPUT);
}

void loop() 
{

  //Reading joystick values
  xVal = analogRead(xAxis);
  yVal = analogRead(yAxis);

  mapYVal = map(yVal, 0, 1023, 1023, 0); //invert y-axis value to work with the already written code. Difference between the joystick used for testing and adafruit one

  
  //printing joystick values to serial monitor for debugging
  char buffer[50];
  sprintf(buffer, "x-axis: %d | y-axis: %d", xVal, mapYVal);
  Serial.println(buffer);


  delay(30); //wait before sending next change to servo angle

}
