#include <WiFi.h>
#include <DHT.h>
#include <Firebase_ESP_Client.h>
#include <addons/TokenHelper.h>
#include <addons/RTDBHelper.h>
#include <Adafruit_Sensor.h>

#define WIFI_SSID         "Jilver"
#define WIFI_PASSWORD     "1098820841"
#define DATABASE_URL      "https://greenpid-default-rtdb.firebaseio.com"
#define API_KEY           "AIzaSyAs9Sax04m7HvqYVHVhb-1DzzMqIe2Y5OU"
#define USER_EMAIL        "jpacheco3@udi.edu.co"
#define USER_PASSWORD     "123456789"

#define PIN_RX            16
#define PIN_TX            17
#define DHT_TYPE          DHT22
#define PIN_DHT1          14
#define PIN_DHT2          22
#define PIN_AIR_QUALITY   32
#define PIN_WATER_LEVEL   35
#define PIN_GROUND_LEVEL  34
#define PIN_FAN_1         19
#define PIN_FAN_2         21
#define PIN_VALVE         23

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

TaskHandle_t Task1;

DHT dht1(PIN_DHT1, DHT_TYPE);
DHT dht2(PIN_DHT2, DHT_TYPE);

int DHTDelay = 2500;

// Firebase paths.
String sensors = "/sensors";
String variables = "/variables";

// Default process variables.
double kp = 231;
double ki = 7.2;
double kd = 1.04;
double tempSetpoint = 32;
double relativeSetpoint = 0;
double groundSetpoint = 0;
double waterSetpoint = 25;
double airQualitySetpoint = 0;

// PID variables.
unsigned long currentTime, prevTime;
double elapsedTime;
double currentError, lastError, accumulativeError, rateError;
double PID, errTemp1, errTemp2;

// Sensor variables.
double temp1, temp2, relative1, relative2;
int ppm, groundPercentage, waterPercentage;

boolean connectWifi() {
  boolean isConnect = true;
  int iteration = 0;
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    if (iteration > 10) {
      isConnect = false;
      break;
    }
    iteration++;
    delay(1000);
  }
  return isConnect;
}

void setup() {
  xTaskCreatePinnedToCore(
    CONTROL,
    "Task_1",
    10000,
    NULL,
    1,
    &Task1,
    0);
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, PIN_RX, PIN_TX);
  boolean wifiConnected = connectWifi();
  if (wifiConnected) {
    // Serial.println("[Wi-Fi]: Connected");
    config.api_key = API_KEY;
    config.database_url = DATABASE_URL;
    auth.user.email = USER_EMAIL;
    auth.user.password = USER_PASSWORD;
    fbdo.setResponseSize(4096);
    config.token_status_callback = tokenStatusCallback;
    config.max_token_generation_retry = 15;
    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);
  }
  dht1.begin();
  dht2.begin();
  pinMode(PIN_FAN_1, OUTPUT);
  pinMode(PIN_FAN_2, OUTPUT);
  pinMode(PIN_VALVE, OUTPUT);
}

void loop() {
  if (Firebase.ready()) {
    if (!isnan(temp1)) {
      FirebaseJson JSONTemperature_1;
      JSONTemperature_1.setDoubleDigits(3);
      JSONTemperature_1.set("value", temp1);
      JSONTemperature_1.set("ts/.sv", "timestamp");
      Firebase.RTDB.pushJSON(&fbdo, String(sensors + "/temperature_1").c_str(), &JSONTemperature_1);
    }
    if (!isnan(temp2)) {
      FirebaseJson JSONTemperature_2;
      JSONTemperature_2.setDoubleDigits(3);
      JSONTemperature_2.set("value", temp2);
      JSONTemperature_2.set("ts/.sv", "timestamp");
      Firebase.RTDB.pushJSON(&fbdo, String(sensors + "/temperature_2").c_str(), &JSONTemperature_2);
    }
    if (!isnan(relative1)) {
      FirebaseJson JSONRelative1;
      JSONRelative1.setDoubleDigits(3);
      JSONRelative1.set("value", relative1);
      JSONRelative1.set("ts/.sv", "timestamp");
      Firebase.RTDB.pushJSON(&fbdo, String(sensors + "/relative_humidity_1").c_str(), &JSONRelative1);
    }
    if (!isnan(relative2)) {
      FirebaseJson JSONRelative2;
      JSONRelative2.setDoubleDigits(3);
      JSONRelative2.set("value", relative2);
      JSONRelative2.set("ts/.sv", "timestamp");
      Firebase.RTDB.pushJSON(&fbdo, String(sensors + "/relative_humidity_2").c_str(), &JSONRelative2);
    }
    if (groundPercentage >= 0 && groundPercentage <= 100) {
      FirebaseJson JSONGroundHumidity;
      JSONGroundHumidity.setDoubleDigits(3);
      JSONGroundHumidity.set("value", groundPercentage);
      JSONGroundHumidity.set("ts/.sv", "timestamp");
      Firebase.RTDB.pushJSON(&fbdo, String(sensors + "/ground_humidity").c_str(), &JSONGroundHumidity);
    }
    if (waterPercentage >= 0 && waterPercentage <= 100) {
      FirebaseJson JSONWaterLevel;
      JSONWaterLevel.setDoubleDigits(3);
      JSONWaterLevel.set("value", waterPercentage);
      JSONWaterLevel.set("ts/.sv", "timestamp");
      Firebase.RTDB.pushJSON(&fbdo, String(sensors + "/water_level").c_str(), &JSONWaterLevel);
    }
    if (ppm >= 10 && ppm <= 1000) {
      FirebaseJson JSONAirQuality;
      JSONAirQuality.setDoubleDigits(3);
      JSONAirQuality.set("value", ppm);
      JSONAirQuality.set("ts/.sv", "timestamp");
      Firebase.RTDB.pushJSON(&fbdo, String(sensors + "/air_quality").c_str(), &JSONAirQuality);
    }
    delay(DHTDelay * 4);
  }
}

void CONTROL(void *parameter) {
  while (true) {
    temp1 = dht1.readTemperature();
    relative1 = dht1.readHumidity();
    temp2 = dht2.readTemperature();
    relative2 = dht2.readHumidity();
    ppm = map(analogRead(PIN_AIR_QUALITY), 0, 4095, 10, 1000);
    groundPercentage = map(analogRead(PIN_GROUND_LEVEL), 0, 4095, 100, 0);
    waterPercentage = map(analogRead(PIN_WATER_LEVEL), 0, 4095, 100, 0);

    currentTime = millis();
    elapsedTime = (double)(currentTime - prevTime);

    // Verifying if temp values are numbers.
    if (!isnan(temp1)) {
      errTemp1 = tempSetpoint - temp1;
    }
    if (!isnan(temp2)) {
      errTemp2 = tempSetpoint - temp2;
    }

    //Calculating the PID current error.
    if ((!isnan(errTemp1)) && (!isnan(errTemp2))) {
      currentError = ((errTemp1 + errTemp2) / 2);
    }
    if ((!isnan(errTemp1)) && (isnan(errTemp2))) {
      currentError = errTemp1;
    }
    if ((isnan(errTemp1)) && (!isnan(errTemp2))) {
      currentError = errTemp2;
    }

    if (!isnan(currentError)) {
      accumulativeError += currentError * elapsedTime;
      rateError = (currentError - lastError) / elapsedTime;
      if (currentError > 10) {
        PID = kp * currentError + 0 + kd * rateError;
      } else {
        PID = kp * currentError + ki * accumulativeError + kd * rateError;
      }
      lastError = currentError;
      prevTime = currentTime;
      Serial2.print(PID);
    }
    delay(DHTDelay);
  }
}
