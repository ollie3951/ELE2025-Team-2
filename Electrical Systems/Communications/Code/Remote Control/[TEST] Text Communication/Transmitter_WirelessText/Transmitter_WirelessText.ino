//Ollie, 29/01/2025
//Practicing wireless communication using NRF24L01 module

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
const byte address[6] = "00001"; //This address can be any 5 charcter string e.g. "123AB"

int i = 1; //indicates number of message sent through channel


void setup() 
{

  Serial.begin(9600); //troubleshooting by checking message sent

  //beginning radio communication
  //radio.begin();
  //radio.begin();
  if (! radio.begin()) {
    Serial.println("Failed to connect NRF");
    while (1);
  }
  //setting radio channel using previously defined channel address
  radio.openWritingPipe(address); //writing pipe because this is transmitter code
  //setting power amplifier level. For this test setting to min, but if using higher value in robot to increase range consider adding bypass capacitor between +3.3V and GND to stabilise voltage
  radio.setPALevel(RF24_PA_MIN);
  //setting this module as the transmitter:
  radio.stopListening();
}

void loop()
{

  //const char text[] = "Data received from transmitter"; //creating array of charcters to send to receiver
  char textNumbered[50];
  sprintf(textNumbered, "Message %d : Received", i);
  Serial.println(textNumbered);
  
  
  radio.write(&textNumbered, sizeof(textNumbered)); //sending the message through channel to the receiver
  Serial.println(i);
  delay(1000);
  i = i + 1; //increment i for next message 
} 
