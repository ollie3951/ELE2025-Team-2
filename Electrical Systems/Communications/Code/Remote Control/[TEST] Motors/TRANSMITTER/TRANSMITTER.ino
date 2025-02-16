//Ollie, 16/02/2025
//Practicing controlling motor speed and direction remotely

//Code for Arduino UNO transmitter

//importing libraries

//INSTALL LIBRARIES FROM: https://github.com/nRF24/RF24

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//--------------------------

//starting radio object of class RF24
RF24 radio(9, 8); //CE and CSN connected to digital pins 9 and 8 respectively

//channel/pipe address which RF24 modules agree to communicate through (SAME ACROSS TRANSMITTER AND RECEIVER CODE)
const byte address[6] = "TEAM2"; //This address can be any 5 charcter string e.g. "123AB"


//pin definitions for joystick
const int xAxis = A5;
const int yAxis = A3;

//variables to store joystick values
int xVal;
int yVal;

//variables for controlling the motors
int motorSpeed; //speed and forward/reverse
int motorDirection; //left, right or straight?


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

  //Reading joystick values, in range 0-1023. full speed forward=0, full speed reverse= 1023, full right=1023, full left=0.
  xVal = analogRead(xAxis);
  yVal = analogRead(yAxis);
    
  //printing joystick values to serial monitor for debugging
  char buffer[50];
  sprintf(buffer, "x-axis: %d | y-axis: %d", xVal, yVal);
  Serial.println(buffer);

  //using joystick readings to get values for motor control. FULL PROCESSING OF THESE READINGS WILL BE DONE AT RECEIVER.
  motorSpeed = yVal; //y axis controls motor speed
  motorDirection = xVal; //x axis controls motor direction
  

  //sending speed value to the receiver
  radio.write(&motorSpeed, sizeof(motorSpeed));

  //sending direction value to the receiver
  radio.write(&motorDirection, sizeof(motorDirection));

  delay(30); //delay between sending new data to receiver

}
