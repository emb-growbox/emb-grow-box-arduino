/*
  Rui Santos
  Complete project details at our blog.
    - ESP32: https://RandomNerdTutorials.com/esp32-firebase-realtime-database/
    - ESP8266: https://RandomNerdTutorials.com/esp8266-nodemcu-firebase-realtime-database/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
  Based in the RTDB Basic Example by Firebase-ESP-Client library by mobizt
  https://github.com/mobizt/Firebase-ESP-Client/blob/main/examples/RTDB/Basic/Basic.ino
*/
#include <Arduino.h>
#if defined(ESP32)
#include <WiFi.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#endif
#include <Firebase_ESP_Client.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <SoftwareSerial.h>
#if !defined(ESP8266)
#error This code is designed to run on ESP8266 and ESP8266-based boards! Please check your Tools->Board setting.
#endif

// These define's must be placed at the beginning before #include "ESP8266TimerInterrupt.h"
// _TIMERINTERRUPT_LOGLEVEL_ from 0 to 4
// Don't define _TIMERINTERRUPT_LOGLEVEL_ > 0. Only for special ISR debugging only. Can hang the system.
#define TIMER_INTERRUPT_DEBUG 0
#define _TIMERINTERRUPT_LOGLEVEL_ 0

// Select a Timer Clock
#define USING_TIM_DIV1 false   // for shortest and most accurate timer
#define USING_TIM_DIV16 false  // for medium time and medium accurate timer
#define USING_TIM_DIV256 true  // for longest timer but least accurate. Default

#include "ESP8266TimerInterrupt.h"

// //Provide the token generation process info.
#include "addons/TokenHelper.h"
// //Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

// Insert your network credentials
#define WIFI_SSID "AndroidAP6DCA"
#define WIFI_PASSWORD "ufpb8543"

#define TIMER_INTERVAL_MS 1000

// // Insert Firebase project API Key
#define API_KEY "AIzaSyBK36Dn0e1SgospSI_c-BsJv2z9HzhLmvw"

// // Insert RTDB URLefine the RTDB URL */
#define DATABASE_URL "https://emb-grow-box-default-rtdb.europe-west1.firebasedatabase.app"
const byte rxPin = 16;
const byte txPin = 15;

struct __attribute__((packed)) sensorData {
  char control;
  int8_t light;
  float temperature;
  int8_t humidity;
  int8_t soilMoisture;
  int8_t waterTank;
  int8_t irrigation;
};

struct __attribute__((packed)) controlsPolling {
  int8_t lightPower;
  int8_t irrigation;
  time_t updateDate;
};

struct __attribute__((packed)) controls {
  char control;
  int8_t lightPower;
  int8_t irrigation;
};

// //Define Firebase Data object
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");
volatile int count = 0;
volatile bool signupOK = false;
volatile bool shouldFetchControl = false;
volatile bool shouldFetchSensor = false;
volatile int sensorCounter = 0;
volatile int newData = 0;
SoftwareSerial mySerial(rxPin, txPin);
controlsPolling controlsPollingData;

ESP8266Timer FBTimer;

void setup() {
  Serial.begin(115200);
  mySerial.begin(115200);
  setUpWifi();
  setUpFirebase();
  setUpTimeClient();
  setUpFBTimer();
  controlsPollingData.updateDate = 0;
}

void setUpWifi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();
}

void setUpTimeClient() {
  timeClient.begin();
  timeClient.setTimeOffset(3600);
}

void setUpFirebase() {

  /* Assign the api key (required) */
  config.api_key = API_KEY;

  // /* Assign the RTDB URL (required) */
  config.database_url = DATABASE_URL;

  /* Sign up */
  if (Firebase.signUp(&config, &auth, "", "")) {
    Serial.println("ok");
    signupOK = true;
  } else {
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }

  // /* Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback;  //see addons/TokenHelper.h

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
}

void IRAM_ATTR TimerHandler() {
  shouldFetchControl = true;
  sensorCounter++;
  if (sensorCounter >= 4) {
    shouldFetchSensor = true;
  }
}

void setUpFBTimer() {
  // Interval in microsecs
  if (FBTimer.attachInterruptInterval(TIMER_INTERVAL_MS * 1000, TimerHandler)) {
    Serial.println("Starting  FBTimer OK");
  } else {
    Serial.println("Can't set ITimer correctly. Select another freq. or interval");
  }
}

void loop() {
  if (shouldFetchControl) {
    readControllFromFirebase();
    shouldFetchControl = false;
  }
  if (shouldFetchSensor) {
    tryToReceiveSensorData();
    shouldFetchSensor=false;
    sensorCounter=0;
  }
}

void tryToReceiveSensorData() {
  sensorData data;
  if (Serial.available() > 0 && newData == 0) {
    if (Serial.read() == '<') {
      newData = 1;
      Serial.println("Stared new data");
      int8_t* ptr = (int8_t*)&(data);
      ptr++;
      int counter = 0;
      int finish = 0;
      while (counter < sizeof(sensorData) - 1) {
        while (Serial.available() <= 0)  //waits until there is new input
        {
          //do nothing
        }
        *ptr = Serial.read();
        ptr++;
        counter++;
      }
      Serial.print("Size: ");
      Serial.print(sizeof(data));
      Serial.print(" Light: ");
      Serial.print(data.light);
      Serial.print(" Temperature: ");
      Serial.print(data.temperature);
      Serial.print(" Soil: ");
      Serial.print(data.soilMoisture);
      Serial.print(" Humidity: ");
      Serial.print(data.humidity);
      Serial.println();
      writeToFirebase(data);
      newData = 0;
      counter = 0;
    } else {
      //do nothing
    }
  }
}

void readControllFromFirebase() {
  FirebaseJson jVal;
  time_t newUpdateDate = 0;
  int irrigation = 0;
  int lightPower = 0;
  if (Firebase.RTDB.getJSON(&fbdo, "/controls", &jVal)) {
    FirebaseJsonData updateDateResult;
    jVal.get(updateDateResult, "updateDate");
    newUpdateDate = updateDateResult.to<time_t>();
    if (controlsPollingData.updateDate != newUpdateDate) {
      FirebaseJsonData irrigationResult;
      jVal.get(irrigationResult, "irrigation");
      FirebaseJsonData lightPowerResult;
      jVal.get(lightPowerResult, "lightPower");
      controlsPollingData.updateDate = newUpdateDate;
      controlsPollingData.irrigation = irrigationResult.to<int8_t>();
      controlsPollingData.lightPower = lightPowerResult.to<int8_t>();
      sendDataToMsp(controlsPollingData);
    }
  }
}


void sendDataToMsp(controlsPolling data) {
  controls dataToSend;
  dataToSend.irrigation = data.irrigation;
  dataToSend.lightPower = data.lightPower;
  dataToSend.control = '>';
  Serial.println("Sending Data...");
  mySerial.write(dataToSend.control);
  mySerial.write(dataToSend.lightPower);
  mySerial.write(dataToSend.irrigation);
}

void writeToFirebase(sensorData data) {
  FirebaseJson json;
  int succeeded = 0;
  int numberOfTries = 0;
  while (succeeded == 0 && numberOfTries < 5) {
    if (Firebase.ready() && signupOK) {
      timeClient.update();
      //the time will be in seconds, so in react js we need to multiply by 1000 to covnert to real date
      time_t epochTime = timeClient.getEpochTime();
      json.set("temperature", data.temperature);
      json.set("soilMoisture", data.soilMoisture);
      json.set("date", epochTime);
      json.set("lightPower", data.light);
      json.set("humidity", data.humidity);
      json.set("waterTank", data.waterTank);

      if (Firebase.RTDB.pushJSON(&fbdo, "sensorsData", &json)) {
        succeeded = 1;
        numberOfTries = 0;
      } else {
        numberOfTries++;
      }
    }
  }
}