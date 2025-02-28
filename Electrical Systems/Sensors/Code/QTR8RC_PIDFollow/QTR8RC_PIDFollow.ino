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

//Max speed of motors for robot
const int maxSpeed = 50; //value in range 0-255, ensure high enough to complete course in 10 seconds

// PD Properties
const double Kp = maxSpeed/3500; //proportional term, maxspeed/3500 gives max motor speed when error is maximum
const double Kd = 10*Kp; //derivative term, initial value of 10*Kp
int lastError = 0; //hold the last error for implementing the derivative term
const int goal = 3500; //goal is for sensor array to be positioned with the middle on the line 

//line following sensor parameters
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
  
  // configure the line sensor
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){33, 34, 35, 36, 37, 38, 39, 40}, SensorCount);
  //qtr.setEmitterPin(2);

  calibrateLineSensor(); //calls custom function to calibrate the line sensor to the environment
}



void loop()
{
  //read position of sensor array over line. 3500 means right in the middle.
  uint16_t position1 = qtr.readLineBlack(sensorValues);

  // calculate how far off the sensor array position is from the goal of 3500
  int error = goal - position1;

  //calculate motor adjustment required using PD terms
  int adjustment = Kp*error + Kd*(error - lastError);

  //store error for the next iteration
  lastError = error;

  //adjust motor outputs
  rightOutputA = 0;
  rightOutputB = constrain(maxSpeed + adjustment, 0, maxSpeed); //constrain ensures after adjustment is added value stays on range 0-maxSpeed
  leftOutputA = constrain(maxSpeed - adjustment, 0, maxSpeed); //IF ROBOT PUSHES AWAY FROM LINE, SWITCH + AND - HERE
  leftOutputB = 0;

  //output to motors
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
