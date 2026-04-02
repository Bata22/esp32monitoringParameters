#include "credencials.h"
#include <SparkFun_VL53L1X.h>
#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "heartRate.h"
#include <HX711_ADC.h>
#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>

#include <ArduinoJson.h>


const char *ssid = "WIFINAME"; //network name hostpot 
const char *password = "WIFIPASS"; // passwoord hostspot 
IPAddress staticIP(IPADDRES);

WebServer server(80);

#endif
#define MAX_BRIGHTNESS 255

//pins:
const int HX711_dout = 15; //mcu > HX711 dout pin
const int HX711_sck = 5; //mcu > HX711 sck pin

SFEVL53L1X distanceSensor;
HX711_ADC LoadCell(HX711_dout, HX711_sck);
MAX30105 particleSensor;



float distance = 0;
const int calVal_calVal_eepromAdress = 0;
unsigned long t = 0;
float mh;
float weight;
float bmi;

uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
int32_t bufferLength = 100; //data length
int32_t spo2; 
int8_t validSPO2; 
int32_t heartRate; 
int8_t validHeartRate; 

const byte RATE_SIZE = 4; 
byte rates[RATE_SIZE]; 
byte rateSpot = 0;
long lastBeat = 0; 

float beatsPerMinute;
int beatAvg;

 byte pulseLED = 11; //Must be on PWM pin
 byte readLED = 13; //Blinks with each data read


void setup() {
  Wire.begin();
  Serial.begin(115200);
  if (WiFi.config(staticIP) == false) {
   Serial.println("Configuration failed.");
 }
 
  delay(1);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

// odavde senzori


  if (MDNS.begin("esp32")) {
    Serial.println("MDNS responder started");
  }
  server.on("/readSensors", HTTP_GET, sensorData);
  server.begin();
  Serial.println("HTTP server started");
   if (!particleSensor.begin(Wire, I2C_SPEED_FAST)){//Use default I2C port, 400kHz speed
     Serial.println("MAX30105 was not found. Please check wiring/power.");
     while (1);
  }

 
   float calibrationValue; 
   EEPROM.begin(512);
  
  particleSensor.setup(); //Default setting for sensor
  particleSensor.setPulseAmplitudeRed(0x0A); 
  particleSensor.setPulseAmplitudeGreen(0); 
  Serial.println();
  Serial.println("Starting...");                          
  EEPROM.get(calVal_calVal_eepromAdress, calibrationValue); // uncomment this if you want to fetch this value from eeprom

  LoadCell.begin();
  
  unsigned long stabilizingtime = 3000; // tare preciscion can be improved by adding a few seconds of stabilizing time 
  boolean _tare = true; //set this to false if you don't want tare to be performed in the next step
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
  }
  else {
    LoadCell.setCalFactor(calibrationValue); // set calibration factor (float)
    Serial.println("Startup is complete");
  }
  while (!LoadCell.update());
  Serial.print("Calibration value: ");
  Serial.println(LoadCell.getCalFactor());
  Serial.print("HX711 measured conversion time ms: ");
  Serial.println(LoadCell.getConversionTime());
  Serial.print("HX711 measured sampling rate HZ: ");
  Serial.println(LoadCell.getSPS());
  Serial.print("HX711 measured settlingtime ms: ");
  Serial.println(LoadCell.getSettlingTime());
  Serial.println("Note that the settling time may increase significantly if you use delay() in your sketch!");
  if (LoadCell.getSPS() < 7) {
    Serial.println("!!Sampling rate is lower than specification, check MCU>HX711 wiring and pin designations");
  }
  else if (LoadCell.getSPS() > 100) {
    Serial.println("!!Sampling rate is higher than specification, check MCU>HX711 wiring and pin designations");
  }
}

void loop() {
  server.handleClient();
  delay(1);
  mh = mesureHight();
  
  long irValue = particleSensor.getIR();

  if (checkForBeat(irValue) == true) {
    // Otkrili smo otkucaj srca!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20) {
      rates[rateSpot++] = (byte)beatsPerMinute; 
      rateSpot %= RATE_SIZE;
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }

  for (byte i = 0; i < 100; i++) {
    while (particleSensor.available() == false)
      particleSensor.check(); 

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); 
  

  }
   
  maxim_heart_rate_and_oxygen_saturation(irBuffer, 100, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  
  
 
  Serial.print("SPO2= ");
  Serial.print(spo2);
  Serial.print("|Valid= ");
  Serial.println(validSPO2);
  Serial.print("HR= ");
  Serial.print(heartRate);
  Serial.print(" |Valid= ");
  Serial.println(validHeartRate);


  
  static boolean newDataReady = 0;
  const int serialPrintInterval = 0; //increase value to slow down serial print activity

  // check for new data/start next conversion:
  if (LoadCell.update()) newDataReady = true;

 t:
  if (newDataReady) {
   
    if (millis() > t + serialPrintInterval) {
       weight =  (LoadCell.getData()/ 1000);
      Serial.print("Load_cell output val: ");
      Serial.println(weight);
      newDataReady = 0;
      t = millis();
    }
  }
  bmi = Bmi(mh,weight);
  
  Serial.print("Visina = ");
  Serial.print(mh);
  Serial.print("Bmi = ");
  Serial.print(bmi);  
  

  if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 't') LoadCell.tareNoDelay();
  }

  // check if last tare operation is complete:
  if (LoadCell.getTareStatus() == true) {
    Serial.println("Tare complete");
  }

}

float mesureHight() {
  if (distanceSensor.begin() != 0){
    Serial.println("Sensor Faled");
    Serial.println("Check wiring");
    while(1);
  }
  distanceSensor.setDistanceModeLong();
  distanceSensor.startRanging();

  while(!distanceSensor.checkForDataReady()){
    delay(1);
  }
  distance = distanceSensor.getDistance();
  distanceSensor.clearInterrupt();
  distanceSensor.stopRanging();
  float distanceCm = distance /10.00;

  return distanceCm;

}

void sensorData() {
   StaticJsonDocument<200> jsonDoc;
    jsonDoc["height"] = mh;
    jsonDoc["weight"] = weight;
    jsonDoc["bmi"] = bmi;
    jsonDoc["heartRate"] = heartRate;
    jsonDoc["spo2"] = spo2;

     String response;
  serializeJson(jsonDoc, response);
  server.send(200, "application/json", response);
  }
float Bmi(float height, float weight) {
  float heightBmi = height / 100;
  float bmi = weight/ (heightBmi*heightBmi);
  return bmi;

}
