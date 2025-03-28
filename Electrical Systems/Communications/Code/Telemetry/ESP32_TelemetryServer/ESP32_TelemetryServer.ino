#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>


//defining pins for motor encoder
#define APin 17
#define BPin 16

//defining pins for ultrasonic sensor
const int trigPin = 32;
const int echoPin = 33;

//define sound speed in cm/uS
#define soundSpeed 0.034

//variables for ultrasonic sensor
long durationUS;
float distanceUS;

//Declare variables to hold pulses for motor encoder
volatile unsigned long ACount = 0;
volatile unsigned long BCount = 0;
unsigned long prevACount = 0;
int diffACount = 0;

//variable to hold total distance travelled by robot
float distanceCM = 0; //initialise at zero distance

//variable to hold rpm of one wheel of robot
float rpm = 0; //initially zero

// Replace with your network credentials
const char* ssid = "Ollie’s iPhone 14";
const char* password = "olliehotspot";

#define DHTPIN 27     // Digital pin connected to the DHT sensor

#define DHTTYPE    DHT11     // DHT 11

DHT dht(DHTPIN, DHTTYPE);

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

String readDHTTemperature() { //function that reads temperature
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  //float t = dht.readTemperature(true);
  // Check if any reads failed and exit early (to try again).
  if (isnan(t)) {    
    Serial.println("Failed to read from DHT sensor!");
    return "--";
  }
  else {
    //Serial.println(t);
    return String(t);
  }
}

String readDHTHumidity() { //function that reads humidity
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  if (isnan(h)) {
    Serial.println("Failed to read from DHT sensor!");
    return "--";
  }
  else {
    //Serial.println(h);
    return String(h);
  }
}


// Replaces placeholders with values
String processor(const String& var){
  if(var == "TIME"){
    return String(millis()/1000);
  }
  return String();
  if(var == "RPM"){
    return String(rpm);
  }
  if(var == "OBSTRUCTION"){
    return String(distanceUS);
  }
}



void setup(){

  //Declare inputs for motor encoding tracking distance
  pinMode(APin, INPUT);
  pinMode(BPin, INPUT);
  //Attach interrupts
  //Attach interrupt to APin, which will cause function ATick to execute on
  //rising pulse
  attachInterrupt(digitalPinToInterrupt(APin), ATick, RISING);
  //Attach interrupt to BPin, which will cause function BTick to execute on
  //rising pulse
  attachInterrupt(digitalPinToInterrupt(BPin), BTick, RISING);

  //configuring GPIOs for ultrasonic sensor
  pinMode(trigPin, OUTPUT); //trigger pin is an output
  pinMode(echoPin, INPUT); //echo pin is an input


  // Serial port for debugging purposes
  Serial.begin(115200);
  
  dht.begin();

  if(!LittleFS.begin()){
  Serial.println("An Error has occurred while mounting LittleFS");
  return;
  }

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }

  // Print ESP32 Local IP Address
  Serial.println(WiFi.localIP());

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(LittleFS, "/index.html");
  });
  server.on("/temperature", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", readDHTTemperature().c_str());
  });
  server.on("/humidity", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", readDHTHumidity().c_str());
  });
    server.on("/distance", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(distanceCM).c_str());
  });
    server.on("/time", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(millis()/1000).c_str());
  });
  server.on("/rpm", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(rpm).c_str());
  });
  server.on("/obstruction", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(distanceUS).c_str());
  });

  // Start server
  server.begin();
}
 
void loop(){


  distanceCM = 0.0217*ACount; //update distance travelled

  prevACount = ACount; //save value of ACount before waiting 0.25 seconds to see how quickly it is changing
  delay(250); //for calculating rpm
  diffACount = ACount - prevACount; //find how many new pulses are recorded in 0.25 seconds
  rpm = (240*diffACount/920); //use rate of change of ACount to find motor rpm
  
  //Serial.println(rpm);


  // ULTRASONIC SENSOR CODE --------------------------------
  //clear the trigger pin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  //set trigger pin high for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  //read the echo pin, return sound wave travel time in microseconds
  durationUS = pulseIn(echoPin, HIGH);
  //calculate the distance
  distanceUS = durationUS*soundSpeed/2;

  //print distance in serial monitor
  //Serial.print("Distance (cm) : ");
  //Serial.println(distanceUS);
  //-------------------------------------------------------------
  
}


//ISRs for counting pulses
void ATick(){
ACount++; //Increment ACount
}
void BTick(){
BCount++; //Increment BCount
}



