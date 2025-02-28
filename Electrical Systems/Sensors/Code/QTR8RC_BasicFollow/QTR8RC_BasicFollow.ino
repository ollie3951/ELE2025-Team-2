#include <QTRSensors.h>

//pins for motor control
#define M1A 9 //M1A is connected to Arduino pin 9
#define M1B 11 //M1B is connected to Arduio pin 11
#define M2A 10 //M2A is connected to Arduino pin 10
#define M2B 12 //M2B is connected to Arduino pin 12
//M1A and M1B for right motor
//M2A and M2B for left motor

//variables for output to motors
int rightOutputA = 0; //start braking
int rightOutputB = 0; //start braking
int leftOutputA = 0; //start braking
int leftOutputB = 0; //start braking

int turnAmount = 0; //holds how much motors will turn

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

void setup()
{

  //Motor control pins as outputs
  pinMode(M1A, OUTPUT);
  pinMode(M1B, OUTPUT);
  pinMode(M2A, OUTPUT);
  pinMode(M2B, OUTPUT);
  
  // configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){33, 34, 35, 36, 37, 38, 39, 40}, SensorCount);
  //qtr.setEmitterPin(2);

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
}

void loop()
{
  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 5000 (for a white line, use readLineWhite() instead)
  uint16_t position1 = qtr.readLineBlack(sensorValues);

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum
  // reflectance and 1000 means minimum reflectance, followed by the line
  // position
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position1);

  if(position1>4000) //robot veers off left of the line
  {
    turnAmount = (position1-3500)/70;
    //left wheel faster than right to turn right back onto line
    rightOutputA = 0;
    rightOutputB = 50-turnAmount;
    leftOutputA = 50+turnAmount;
    leftOutputB = 0;

    if(position1==7000)
    {
      rightOutputA = 50 +turnAmount;
      rightOutputB = 0;
      leftOutputA = 50 +turnAmount;
      leftOutputB = 0;
    }
  }
  else if(position1<3000) //robot veers off right of line
  {
    turnAmount = (3500-position1)/70;
    
    rightOutputA = 0;
    rightOutputB = 50+turnAmount;
    leftOutputA = 50-turnAmount;
    leftOutputB = 0;

    if(position1==0)
    {
      rightOutputA = 0;
      rightOutputB = 50+turnAmount;
      leftOutputA = 0;
      leftOutputB = 50 +turnAmount;
    }
  }
  else
  {
    rightOutputA = 0;
    rightOutputB = 50;
    leftOutputA = 50;
    leftOutputB = 0;
  }


  //output to motors after variables have been calculated by either remote control branch or autonomous branch
  analogWrite(M1A,rightOutputA);
  analogWrite(M1B,rightOutputB);
  analogWrite(M2A,leftOutputA);
  analogWrite(M2B,leftOutputB);

  delay(150);
}
