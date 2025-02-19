//Ollie, 19/02/2025
//Practicing remote control of motors

//Code for Arduino MEGA receiver

//importing libraries---------------------------

//INSTALL LIBRARIES FROM: https://github.com/nRF24/RF24

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//pins for motor control
#define M1A 9 //M1A is connected to Arduino pin 9
#define M1B 11 //M1B is connected to Arduio pin 11
#define M2A 10 //M2A is connected to Arduino pin 10
#define M2B 12 //M2B is connected to Arduino pin 12
//M1A and M1B for right motor
//M2A and M2B for left motor


//array for receiving radio data from remote controller
int receiveData[2] = {512, 512}; //initialise braking to prevent immediate motion after turning on
//first element is xVal(direction), second element is yVal(speed)

//variable to give how much speed of wheels should be changed for turn based on joystick x-axis reading
turnAmount = 0; //initially assume to turning

//variables for output to motors
int rightOutputA = 0; //start braking
int rightOutputB = 0; //start braking
int leftOutputA = 0; //start braking
int leftOutputB = 0; //start braking

//starting radio object of class RF24
RF24 radio(7, 8); //CE and CSN connected to digital pins 7 and 8 respectively

//channel/pipe address which RF24 modules agree to commuicate through (SAME ACROSS TRANSMITTER AND RECEIVER CODE)
const byte address[6] = "TEAM2"; //This address can be any 5 charcter string e.g. "123AB"



void setup()
{
  Serial.begin(9600); //begin serial monitor at 9600 baud rate

  //beginning radio communication
  radio.begin();
  //setting radio channel using previously defined channel address
  radio.openReadingPipe(0, address); //writing pipe because this is receiver code
  //setting power amplifier level. For this test setting to min, but if using higher value in robot to increase range consider adding bypass capacitor between +3.3V and GND to stabilise voltage
  radio.setPALevel(RF24_PA_MIN);
  //setting this module as the receiver:
  radio.startListening();

  //Motor control pins as outputs
  pinMode(M1A, OUTPUT);
  pinMode(M1B, OUTPUT);
  pinMode(M2A, OUTPUT);
  pinMode(M2B, OUTPUT);
}



void loop()
{
  if(radio.available()) //if data has been received
  {
    radio.read(&receiveData, sizeof(receiveData)); //read the data array received
    
    //print received data to serial monitor for debugging
    Serial.print("Motor Speed: ");
    Serial.println(receiveData[1]);
    Serial.print("Motor Direction: ");
    Serial.println(receiveData[0]);
  }

  //code which translates the received data into actual remote control motion of the motors -----------------------------------------------------------

  //getting turnAmount value
  
  if(receiveData[0]<500) //left turn indicated, should make turnAmount +ve and proportional to receiveData[0]
  {
    turnAmount = (1023-receiveData[0])/100 //max turn is +-10
  }
  else if(receiveData[0]>530) //if right turn indicated, make turnAmount -ve and proportional to receiveData[0]
  {
    turnAmount = -(receiveData[0]/100); //max turn is +-10
  }
  else //when joystick is central on x-axis, make robot go straight/no turning
  {
    turnAmount = 0; 
  }
  
  if(receiveData[1]<500) //forward when joystick is pushed forwards along y-axis
  {
    rightOutputA = (1023-receiveData[1])/16 + turnAmount; //make speed of motor depend on how far forward joystick is pushed
    rightOutputB= 0;
    leftOutputA = 0;
    leftOutputB = (1023-receiveData[1])/16 - turnAmount; //make speed of motor depend on how far forward joystick is pushed

    //setting limits to prevent values outside range 0-255 for motors
    //MAX 255
    if(rightOutputA > 255)
    {
      rightOutputA = 255;
    }
      if(leftOutputB > 255)
    {
      leftOutputB = 255;
    }
    //MIN 0
    if(rightOutputA < 0)
    {
      rightOutputA = 0;
    }
    if(leftOutputB < 0)
    {
      leftOutputB = 0;
    }
    
  }
  else if(receiveData[1]>530) //reverse when joystick is pushed backwards along y-axis
  {
    rightOutputA = 0;
    rightOutputB = receiveData[1]/16 + turnAmount; //make speed of motor depend on how far backward joystick is pushed
    leftOutputA = receiveData[1]/16 - turnAmount; //make speed of motor depend on how far backward joystick is pushed
    leftOutputB = 0;

    //setting limits to prevent values outside range 0-255 for motors
    //MAX 255
    if(rightOutputB > 255)
    {
      rightOutputB = 255;
    }
      if(leftOutputA > 255)
    {
      leftOutputA = 255;
    }
    //MIN 0
    if(rightOutputB < 0)
    {
      rightOutputB = 0;
    }
    if(leftOutputA < 0)
    {
      leftOutputA = 0;
    }
  }
  else //braking when y-axis value is close to the center point, turnAmount here alone to allow turning on the spot
  {
    //default is staying still
    rightOutputA = 0;
    rightOutputB = 0;
    leftOutputA = 0;
    leftOutputB = 0;

    //if right turn is indicated by joystick
    if(turnAmount<0)
    {
      rightOutputA = 0; //right wheel reverse
      rightOutputB = -turnAmount;
      leftOutputA = 0; //left wheel forward
      leftOutputB = -turnAmount;
    }

    //if left turn is indicated by joystick
    if(turnAmount>0)
    {
      rightOutputA = turnAmount; //right wheel forward
      rightOutputB = 0;
      leftOutputA = turnAmount; //left wheel reverse
      leftOutputB = 0;
    }

  }
    analogWrite(M1A,rightOutputA);
    analogWrite(M1B,rightOutputB);
    analogWrite(M2A,leftOutputA);
    analogWrite(M2B,leftOutputB);
    
  //----------------------------------------------------------------------------------------------------------------------------------
 
   delay(50);
}
