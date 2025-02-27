//Ollie, 29/01/2025
//Practicing wireless communication using joystick and servo

//Code for Arduino UNO transmitter

//importing libraries---------------------------

//INSTALL LIBRARIES FROM: https://github.com/nRF24/RF24

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//--------------------------

//starting radio object of class RF24
RF24 radio(9, 8); //CE and CSN connected to digital pins 9 and 8 respectively

//channel/pipe address which RF24 modules agree to commuicate through (SAME ACROSS TRANSMITTER AND RECEIVER CODE)
const byte address[6] = "TEAM2"; //This address can be any 5 charcter string e.g. "123AB"


//pin definitions for joystick
const int xAxis = A5;
const int yAxis = A3;

//variables to store joystick values
int xVal;
int yVal;

//variables for servo motor
int mapVal = 5; //holds mapped angle change before sign is added
int angleChange = 0; //hold how much angle will be changed in this iteration
int servoAngle = 90; //initially at 90 degrees

void setup() 
{

  //open serial monitor
  Serial.begin(9600);

  //joystick pins as inputs
  pinMode(xAxis, INPUT);
  pinMode(yAxis, INPUT);

  //beginning radio communication
  radio.begin();
  //setting radio channel using previously defined channel address
  radio.openWritingPipe(address); //writing pipe because this is transmitter code
  //setting power amplifier level. For this test setting to min, but if using higher value in robot to increase range consider adding bypass capacitor between +3.3V and GND to stabilise voltage
  radio.setPALevel(RF24_PA_MIN);
  //setting this module as the transmitter:
  radio.stopListening();
  

}

void loop() 
{

  //Reading joystick values
  xVal = analogRead(xAxis);
  yVal = analogRead(yAxis);

  
  //printing joystick values to serial monitor for debugging
  char buffer[50];
  sprintf(buffer, "x-axis: %d | y-axis: %d", xVal, yVal);
  Serial.println(buffer);
  

  //mapping y-axis value to a change in angle of fixed servo
  mapVal = map(yVal, 0, 1023, 0, 20);
  //if mapVal==(9, 10, 11) no change in servo angle is required since joystick is in neutral position.
  if(mapVal<9) //if joystick has been pushed up, increase servo angle
  {
    angleChange = 10 - mapVal; //greatest angle change when joystick is pushed entirely upwards
    servoAngle = servoAngle + angleChange; //upwards should increase servo angle
  }
  else if(mapVal>11) //if joystick is pushed downwards
  {
    angleChange = mapVal - 10; //greatest angle change when joystick is pushed entirely downwards
    servoAngle = servoAngle - angleChange; //downwards should decrease servo angle
  }

  //creating a maximum and minimum value for servo angle
  if(servoAngle>180) //setting max angle of 180 degrees
  {
    servoAngle = 180;
  }
  if(servoAngle<0) //setting minimum value of 0 degrees
  {
    servoAngle = 0;
  }
 
  
  //sending this servo angle value to the receiver
  radio.write(&servoAngle, sizeof(servoAngle));


  delay(30); //wait before sending next change to servo angle

  //Serial.println(servoAngle); //printing servo angle to serial monitor for debugging

}
