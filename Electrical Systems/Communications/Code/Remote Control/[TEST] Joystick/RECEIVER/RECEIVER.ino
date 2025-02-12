//Ollie, 29/01/2025
//Practicing wireless communication using NRF24L01 module

//Code for Arduino MEGA receiver

//importing libraries---------------------------

//INSTALL LIBRARIES FROM: https://github.com/nRF24/RF24

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//servo library

#include <Servo.h>

//--------------------------

//defining pins---------------------

#define servoPin 4

//----------------------------------

//creating a servo object
Servo myServo;

int servoAngle = 90; //integer to hold incoming servo angle

//starting radio object of class RF24
RF24 radio(7, 8); //CE and CSN connected to digital pins 7 and 8 respectively

//channel/pipe address which RF24 modules agree to commuicate through (SAME ACROSS TRANSMITTER AND RECEIVER CODE)
const byte address[6] = "TEAM2"; //This address can be any 5 charcter string e.g. "123AB"


void setup() 
{

  Serial.begin(9600); //begin serial monitor at 9600 baud rate

  myServo.attach(servoPin); //identify pwm pin for servo

  //beginning radio communication
  radio.begin();
  //setting radio channel using previously defined channel address
  radio.openReadingPipe(0, address); //writing pipe because this is receiver code
  //setting power amplifier level. For this test setting to min, but if using higher value in robot to increase range consider adding bypass capacitor between +3.3V and GND to stabilise voltage
  radio.setPALevel(RF24_PA_MIN);
  //setting this module as the receiver:
  radio.startListening();

  //indicate radio communication from transmitter is about to begin
  Serial.println("Receiving...\n");

}

void loop()
{

  if(radio.available()) //if data has been received
  {
    radio.read(&servoAngle, sizeof(servoAngle)); //read the value received
    Serial.println(servoAngle); //print to serial monitor on a new line
  }

  myServo.write(servoAngle);
  
} 
