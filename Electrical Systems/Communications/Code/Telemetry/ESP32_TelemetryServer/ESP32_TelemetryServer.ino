#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>


//defining pins for motor encoder
#define APin 17
#define BPin 16

//Declare variables to hold pulses for motor encoder
volatile unsigned long ACount = 0;
volatile unsigned long BCount = 0;

//variable to hold total distance travelled by robot
float distanceCM = 0; //initialise at zero distance

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
    Serial.println(t);
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
    Serial.println(h);
    return String(h);
  }
}


// Replaces placeholders with values
String processor(const String& var){
  if(var == "TIME"){
    return String(millis()/1000);
  }
  return String();
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

  // Start server
  server.begin();
}
 
void loop(){

  distanceCM = 0.0217*ACount;
  delay(100); //0.1 second delay between updates of distance

  
}


//ISRs for counting pulses
void ATick(){
ACount++; //Increment ACount
}
void BTick(){
BCount++; //Increment BCount
}


