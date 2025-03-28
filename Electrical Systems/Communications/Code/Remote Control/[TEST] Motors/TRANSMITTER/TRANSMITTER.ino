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

//array to store values to be transmitted
int transmitData[2]; //first element is xVal, second element is yVal
//xVal controls motor direction, yVal motor speed

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

  //Reading joystick values, in range 0-1023. full speed forward=0, full speed reverse=1023, full right=1023, full left=0.
  transmitData[0] = analogRead(xAxis);
  transmitData[1] = analogRead(yAxis);
    
  //printing joystick values to serial monitor for debugging
  char buffer[50];
  sprintf(buffer, "x-axis: %d | y-axis: %d", transmitData[0], transmitData[1]);
  Serial.println(buffer);

  //sending all data to the receiver
  radio.write(&transmitData, sizeof(transmitData));

  delay(20); //delay between sending new data to receiver

}
