//ELE2025 Robot Project Team 2 : Ríannan Mottram, Oliver Ross, Rhys Greaves, Yihan Mei

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

//potentiometer pin for servo arm
#define potPin A1

//state variable
bool state = 0; //0 is remote control, 1 is autonomous

//prevTime to prevent button bouncing
unsigned long prevTime = 0;

//pin definitions for joystick
const int xAxis = A5;
const int yAxis = A3;

//array to store values to be transmitted
int transmitData[4]; //first element is xVal, second element is yVal, third element is robot state, fourth element is servo arm position
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

  //potentiometer pin for servo lifter as input
  pinMode(potPin, INPUT);

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
  transmitData[1] = map(analogRead(yAxis), 0, 1023, 1023, 0); //invert y-axis so it works like joystick used for testing
  //setting robot state
  transmitData[2] = state;
  //finding position of servo arm
  transmitData[3] = analogRead(potPin);
    
  //printing joystick values to serial monitor for debugging
  //char buffer[70];
  //sprintf(buffer, "x-axis: %d | y-axis: %d | state: %d | Pot Reading: %d", transmitData[0], transmitData[1], transmitData[2], transmitData[3]);
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
