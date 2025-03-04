//ELE2025 Robot Project Team 2 : Ríannan Mottram, Oliver Ross, Rhys Greaves, Yihan Mei

//Code for Arduino MEGA receiver

//importing libraries---------------------------

//INSTALL LIBRARIES FROM: https://github.com/nRF24/RF24

//remote control libraries
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//Line following library
#include <QTRSensors.h>

//ToF libraries
#include <Wire.h>
#include "Adafruit_VL6180X.h"


//pins for motor control
#define M1A 9 //M1A is connected to Arduino pin 9
#define M1B 11 //M1B is connected to Arduio pin 11
#define M2A 10 //M2A is connected to Arduino pin 10
#define M2B 12 //M2B is connected to Arduino pin 12
//M1A and M1B for right motor
//M2A and M2B for left motor


//array for receiving radio data from remote controller
int receiveData[3] = {512, 512, 0}; //initialise braking to prevent immediate motion after turning on, and in remote control state
//first element is xVal(direction), second element is yVal(speed), third element is robot state

//variable to give how much speed of wheels should be changed for turn based on joystick x-axis reading
int turnAmount = 0; //initially assume to turning

//variables for output to motors
int rightOutputA = 0; //start braking
int rightOutputB = 0; //start braking
int leftOutputA = 0; //start braking
int leftOutputB = 0; //start braking

//starting radio object of class RF24
RF24 radio(7, 8); //CE and CSN connected to digital pins 7 and 8 respectively

//channel/pipe address which RF24 modules agree to commuicate through (SAME ACROSS TRANSMITTER AND RECEIVER CODE)
const byte address[6] = "TEAM2"; //This address can be any 5 charcter string e.g. "123AB"

//creating ToF object
Adafruit_VL6180X vl = Adafruit_VL6180X(); 


//Max speed for line following
const int maxSpeed = 230; //value in range 0-255, ensure high enough to complete course in 10 seconds

// PD Properties for line following
const double Kp = 0.0657; //proportional term, maxspeed/3500 gives max motor speed when error is maximum
const double Kd = 0.657; //derivative term, initial value of 10*Kp
int lastError = 0; //hold the last error for implementing the derivative term
const int goal = 3500; //goal is for sensor array to be positioned with the middle on the line 
int adjustment; //holds motor adjustment

//implementing line following sensor
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];


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

  //initialising ToF sensor
  if (! vl.begin()) {
    Serial.println("Failed to find sensor");
    while (1);
  }
  Serial.println("Sensor found!");

  // configure the line sensor
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){33, 34, 35, 36, 37, 38, 39, 40}, SensorCount);
  //qtr.setEmitterPin(2);

  calibrateLineSensor(); //calls custom function to calibrate the line sensor to the environment
}



void loop()
{ 
  if(radio.available()) //if data has been received
  {
    radio.read(&receiveData, sizeof(receiveData)); //read the data array received
    //Serial.println(receiveData[0]); //Turning
    //Serial.println(receiveData[1]); //Forward/reverse
    //Serial.println(receiveData[2]); //State
  }


  if(receiveData[2]==0) //when robot is in remote control state
  {
    //getting turnAmount value
    if(receiveData[0]<490) //left turn indicated, should make turnAmount -ve and proportional to receiveData[0]
    {
      turnAmount = -(1023-receiveData[0])/15; //max turn is +-70
    }
    else if(receiveData[0]>530) //if right turn indicated, make turnAmount +ve and proportional to receiveData[0]
    {
      turnAmount = +(receiveData[0]/15); //max turn is +-70
    }
    else //when joystick is central on x-axis, make robot go straight/no turning
    {
      turnAmount = 0; 
    }
  
    if(receiveData[1]<500) //forward when joystick is pushed forwards along y-axis
    {
      rightOutputA = 0;
      rightOutputB = constrain((1023-receiveData[1])/12 + turnAmount, 0, 255); //make speed of motor depend on how far forward joystick is pushed
      leftOutputA = constrain((1023-receiveData[1])/12 - turnAmount, 0, 255); //make speed of motor depend on how far forward joystick is pushed
      leftOutputB = 0;
    }
    else if(receiveData[1]>530) //reverse when joystick is pushed backwards along y-axis
    {
      rightOutputA = constrain(receiveData[1]/12 + turnAmount, 0, 255); //make speed of motor depend on how far backward joystick is pushed
      rightOutputB = 0;
      leftOutputA = 0;
      leftOutputB = constrain(receiveData[1]/12 - turnAmount, 0, 255); //make speed of motor depend on how far backward joystick is pushed
    }
    else //braking when y-axis value is close to the center point, turnAmount here alone to allow turning on the spot
    {
      //default is staying still
      rightOutputA = 0;
      rightOutputB = 0;
      leftOutputA = 0;
      leftOutputB = 0;

      //if left turn is indicated by joystick
      if(turnAmount<0)
      {
        //Serial.println(turnAmount);
        rightOutputA = -turnAmount; //right wheel forward
        rightOutputB = 0;
        leftOutputA = -turnAmount; //left wheel reverse; 
        leftOutputB = 0;
      }

      //if right turn is indicated by joystick
      if(turnAmount>0)
      {
        //Serial.println(turnAmount);
        rightOutputA = 0;
        rightOutputB = turnAmount; //right wheel reverse
        leftOutputA = 0;
        leftOutputB = turnAmount; //left wheel forward
      }
    }
  }
  //----------------------------------------------------------------------------------------------------------------------------------
  else //if the robot is in autonomous mode
  {
    
    //read distance from ToF to allow stopping at wall
    uint8_t range = vl.readRange(); 

    //PID procedure for line following:
    //read position of sensor array over line. 3500 means right in the middle.
    uint16_t position1 = qtr.readLineBlack(sensorValues);
    // calculate how far off the sensor array position is from the goal of 3500
    int error = goal - position1;
    //Serial.print("Error"); Serial.println(error); //check error for debugging
    //calculate motor adjustment required using PD terms
    adjustment = Kp*error + Kd*(error - lastError);
    //Serial.print("\n Adjustment"); Serial.println(adjustment); //check adjustment for debugging
    //store error for the next iteration
    lastError = error;
    //adjust motor outputs
    rightOutputA = 0;
    rightOutputB = constrain(maxSpeed - adjustment, 0, maxSpeed); //constrain ensures after adjustment is added value stays on range 0-maxSpeed
    leftOutputA = constrain(maxSpeed + adjustment, 0, maxSpeed); //IF ROBOT PUSHES AWAY FROM LINE, SWITCH + AND - HERE
    leftOutputB = 0;
    //IF FULLY OFF LINE, PIVOT ON THE SPOT TO FIND LINE AGAIN IMMEDIATELY
    if(position1==0) //if veering off line to left
    {
      //left wheel forward, right wheel reverse
      rightOutputA = 150;
      rightOutputB = 0;
      leftOutputA = 150;
      leftOutputB = 0;
    }
    if(position1==7000) //if veering off line to right
    {
      //left wheel reverse, right wheel forward
      rightOutputA = 0;
      rightOutputB = 150;
      leftOutputA = 0;
      leftOutputB = 150;
    }
    //End of PID line following procedure
    
    /*
    //Descreasing speed when approaching wall, and stopping at 20 mm from wall
    //Serial.println(range); //checking range value for debugging
    if(range<80)
    {
      rightOutputA=0;
      rightOutputB=constrain(rightOutputB-rightOutputB*((-1/60)*range+4/3), 0, 255); //MIGHT STOP EARLY DUE TO NOT ENOUGH TORQUE. NEED TO DETERMINE MIN TORQUE
      leftOutputA=constrain(leftOutputA-leftOutputA*((-1/60)*range+4/3), 0, 255);
      leftOutputB=0;
    }
    */
    
  }
  //output to motors after variables have been calculated by either remote control branch or autonomous branch
  analogWrite(M1A,rightOutputA);
  analogWrite(M1B,rightOutputB);
  analogWrite(M2A,leftOutputA);
  analogWrite(M2B,leftOutputB);
}



// FUNCTIONS -----------------------------------------------------------------------------------------------------

void calibrateLineSensor() //function which carries out calibration process for line sensor
{
  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  //UNCOMMENT TO SEE RESULTS OF CALIBRATION IN SERIAL MONITOR
  /*
  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
  */
  
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------
