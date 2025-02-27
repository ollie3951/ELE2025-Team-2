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

//state change pin
#define statePin 2

//state variable
bool state = 0; //0 is remote control, 1 is autonomous

//prevTime to prevent button bouncing
unsigned long prevTime = 0;

//pin definitions for joystick
const int xAxis = A5;
const int yAxis = A3;

//array to store values to be transmitted
int transmitData[3]; //first element is xVal, second element is yVal, third element is robot state
//xVal controls motor direction, yVal motor speed

void setup() 
{
  //open serial monitor
  //Serial.begin(9600);
  //Serial.println("Serial Monitor Open");

  //state change interrupt
  pinMode(statePin, INPUT);
  attachInterrupt(digitalPinToInterrupt(statePin), isr_stateChange, FALLING);

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
  //setting robot state
  transmitData[2] = state;
    
  //printing joystick values to serial monitor for debugging
  //char buffer[50];
  //sprintf(buffer, "x-axis: %d | y-axis: %d | state: %d", transmitData[0], transmitData[1], transmitData[2]);
  //Serial.println(buffer);

  //sending all data to the receiver
  radio.write(&transmitData, sizeof(transmitData));

  delay(20); //delay between sending new data to receiver

}


//Interrupt for changing state of robot between remote control and autonomous
void isr_stateChange()
{
  if(millis()-prevTime>1000) //if more than a second has passed since last state change
  {
    prevTime = millis(); //update previous time 
    state = !state; //change state
  }
}
