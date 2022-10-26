#include <WiFi.h>
#include <DHT.h>
#include <FirebaseESP32.h>
#include <addons/TokenHelper.h>
#include <addons/RTDBHelper.h>
#include <Adafruit_Sensor.h>
#include <esp_task_wdt.h>

#define WIFI_SSID "JILVER"                                           // Your network SSID
#define WIFI_PASSWORD "1098820841"                                   // your network password
#define DATABASE_URL "https://greenpid-default-rtdb.firebaseio.com"  // Database URL        <-- DON'T TOUCH! -->
#define API_KEY "AIzaSyAs9Sax04m7HvqYVHVhb-1DzzMqIe2Y5OU"            // Database api key    <-- DON'T TOUCH! -->
#define USER_EMAIL "jpacheco3@udi.edu.co"                            // User email          <-- DON'T TOUCH! -->
#define USER_PASSWORD "123456789"                                    // User password       <-- DON'T TOUCH! -->
#define SENSORS_PATH "/sensors"                                      // Sensors path        <-- DON'T TOUCH! -->
#define VARIABLES_PATH "/variables"                                  // Variables Path      <-- DON'T TOUCH! -->
#define PIN_RX 16
#define PIN_TX 17
#define DHT_TYPE DHT22
#define PIN_DHT1 14
#define PIN_DHT2 22
#define PIN_AIR_QUALITY 32
#define PIN_WATER_LEVEL 35
#define PIN_GROUND_LEVEL 34
#define PIN_FAN_1 19
#define PIN_FAN_2 21
#define PIN_VALVE 23

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

TaskHandle_t Task1;

// Firebase paths
String sensors = "/sensors";
String variables = "/variables";

// Default process variables
String(str_temperatureSP) = "37";
String(str_relativeHumiditySP) = "75";
String(str_groundHumiditySP) = "30";
String(str_irrigationSP) = "25";
String(str_airQualitySP) = "20";
String(str_kp) = "4.7398";
String(str_ki) = "0.01243";
String(str_kd) = "0";

float kp = str_kp.toFloat();
float ki = str_ki.toFloat();
float kd = str_kd.toFloat();
float relativeHumiditySP = str_relativeHumiditySP.toFloat();
float temperatureSP = str_temperatureSP.toFloat();
float airQualitySP = str_airQualitySP.toFloat();
float groundHumiditySP = str_groundHumiditySP.toFloat();
float irrigationSP = str_irrigationSP.toFloat();

// PID variables
int mode = 1;      // Switch between 1 for open loop system or 2 for close loop system (PID)
int Time = 10000;  // for open loop system (step response delay).
int power = 0;

unsigned long currentTime = 0, prevMillis = 0;
float elapsedTime, TimeMillis, prevTime;
float lastError = 0, accumulativeError = 0, rateError = 0;

float currentError = 0;
float errTemp1 = 0, errTemp2 = 0, PID_value = 0, PID_error = 0;
float PID_P = 0, PID_I = 0, PID_D = 0;

// Sensor variables
int tempRead_delay = 2500;
float temp1 = 0, temp2 = 0, relative1 = 0, relative2 = 0;
int ppm = 0, groundPercentage = 0, waterLevel = 0;

DHT dht1(PIN_DHT1, DHT_TYPE);
DHT dht2(PIN_DHT2, DHT_TYPE);

void setup() {
  xTaskCreatePinnedToCore(
    UPLOAD,
    "Task_1",
    10000,
    NULL,
    2,
    &Task1,
    0);
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, PIN_RX, PIN_TX);
  boolean wifiConnected = connectWifi();
  if (wifiConnected) {
    Serial.println("[WiFi]: Connected.");
    config.api_key = API_KEY;
    config.database_url = DATABASE_URL;
    auth.user.email = USER_EMAIL;
    auth.user.password = USER_PASSWORD;
    fbdo.setResponseSize(4096);
    config.token_status_callback = tokenStatusCallback;
    config.max_token_generation_retry = 15;
    Serial.println("[Firebase]: Initializing...");
    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);
    Serial.println("[Firebase]: Initialized.");
  }

  dht1.begin();
  dht2.begin();

  pinMode(PIN_FAN_1, OUTPUT);
  pinMode(PIN_FAN_2, OUTPUT);
  pinMode(PIN_VALVE, OUTPUT);
}

boolean connectWifi() {
  boolean isConnect = true;
  int iteration = 0;
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WiFi]: Connecting...");
    if (iteration > 10) {
      isConnect = false;
      break;
    }
    iteration++;
    delay(1000);
  }
  return isConnect;
}

void loop() {
  currentTime = millis();

  if (currentTime - prevTime >= tempRead_delay) {
    prevTime += tempRead_delay;
    temp1 = dht1.readTemperature();
    temp2 = dht2.readTemperature();
    relative1 = dht1.readHumidity();
    relative2 = dht2.readHumidity();
    ppm = map(analogRead(PIN_AIR_QUALITY), 0, 4095, 10, 1000);
    groundPercentage = map(analogRead(PIN_GROUND_LEVEL), 0, 4095, 100, 0);
    waterLevel = map(analogRead(PIN_WATER_LEVEL), 0, 4095, 0, 100);

    // ----------------------------------------------------------------------------- //
    // ----------------------------- OPEN LOOP SYSTEM ------------------------------ //
    // ----------------------------------------------------------------------------- //

    if (mode == 1) {
      if (currentTime <= Time) {
        power = 0;
      } else {
        power = 100;
      }
    }

    // ----------------------------------------------------------------------------- //
    // ---------------------------- CLOSE LOOP SYSTEM ------------------------------ //
    // ----------------------------------------------------------------------------- //

    if (mode == 2) {
      if (!isnan(temp1)) {
        errTemp1 = temperatureSP - temp1;
      }
      if (!isnan(temp1)) {
        errTemp2 = temperatureSP - temp2;
      }

      // Calculate the system current error.
      if (!isnan(errTemp1)) {
        currentError = errTemp1;
      }
      if (!isnan(errTemp2)) {
        currentError = errTemp2;
      }
      if ((!isnan(errTemp1)) && (!isnan(errTemp2))) {
        currentError = (errTemp1 + errTemp2) / 2;
      }

      if (!isnan(currentError)) {
        if (currentError > 10) { PID_I = 0; }

        PID_P = kp * currentError;
        PID_I = PID_I + (ki * currentError);
        prevTime = TimeMillis;
        TimeMillis = millis();
        elapsedTime = (TimeMillis - prevTime) / 2500;
        rateError = (currentError - lastError) / elapsedTime;
        PID_D = kd * rateError;
        PID_value = PID_P + PID_I + PID_D;


        if (PID_value <= 0) { PID_value = 0; }
        if (PID_value >= 100) { PID_value = 100; }
        power = PID_value;
        
        lastError = currentError;

        Serial.println("------------ PID VARIABLES -------------");
        Serial.println("Err Temperatura 1 [Pin 14]: " + String(errTemp1) + " °C");
        Serial.println("Err Temperatura 2 [Pin 22]: " + String(errTemp2) + " °C");
        Serial.println("PID Error: " + String(currentError));
        Serial.println("Elapsed time: " + String(elapsedTime));
        Serial.println("Accumulative Err: " + String(accumulativeError));
        Serial.println("Rate Err: " + String(rateError));
        Serial.println("PID_P: " + String(PID_P));
        Serial.println("PID_I: " + String(PID_I));
        Serial.println("PID_D: " + String(PID_D));
        Serial.println("PID_value: " + String(PID_value));
        Serial.println("Power: " + String(power));
        Serial.println("------------ END PID VARIABLES -------------");
      }
    }

    Serial2.print(power);

    Serial.println("------------ SENSOR VARIABLES -------------");
    Serial.println("Temperatura 1 [Pin 14]: " + String(temp1) + " °C");
    Serial.println("Temperatura 2 [Pin 22]: " + String(temp2) + " °C");
    Serial.println("Humedad relativa 1 [Pin 14]: " + String(relative1) + " % RH");
    Serial.println("Humedad relativa 2 [Pin 22]: " + String(relative2) + " % RH");
    Serial.println("Calidad de aire [Pin 32]: " + String(ppm) + " ppm");
    Serial.println("Humedad Suelo [Pin 34]: " + String(groundPercentage) + " %");
    Serial.println("Nivel de agua [Pin 35]: " + String(waterLevel) + " %");
    Serial.println("------------ END SENSOR VARIABLES -------------");
  }
}

void UPLOAD(void *parameter) {
  while (true) {
    if (Firebase.ready()) {
      str_temperatureSP = (Firebase.RTDB.getFloat(&fbdo, String(variables + "/temperature_sp")) ? String(fbdo.to<String>()).c_str() : str_temperatureSP);
      str_kp = (Firebase.RTDB.getFloat(&fbdo, String(variables + "/kp")) ? String(fbdo.to<String>()).c_str() : str_kp);
      str_ki = (Firebase.RTDB.getFloat(&fbdo, String(variables + "/ki")) ? String(fbdo.to<String>()).c_str() : str_ki);
      str_kd = (Firebase.RTDB.getFloat(&fbdo, String(variables + "/kd")) ? String(fbdo.to<String>()).c_str() : str_kd);
      str_relativeHumiditySP = (Firebase.RTDB.getFloat(&fbdo, String(variables + "/relative_humidity_sp")) ? String(fbdo.to<String>()).c_str() : str_relativeHumiditySP);
      str_airQualitySP = (Firebase.RTDB.getFloat(&fbdo, String(variables + "/air_quality_sp")) ? String(fbdo.to<String>()).c_str() : str_airQualitySP);
      str_groundHumiditySP = (Firebase.RTDB.getFloat(&fbdo, String(variables + "/ground_humidity_sp")) ? String(fbdo.to<String>()).c_str() : str_groundHumiditySP);
      str_irrigationSP = (Firebase.RTDB.getFloat(&fbdo, String(variables + "/irrigation_sp")) ? String(fbdo.to<String>()).c_str() : str_irrigationSP);

      temperatureSP = str_temperatureSP.toFloat();
      relativeHumiditySP = str_relativeHumiditySP.toFloat();
      kp = str_kp.toFloat();
      ki = str_ki.toFloat();
      kd = str_kd.toFloat();
      airQualitySP = str_airQualitySP.toFloat();
      groundHumiditySP = str_groundHumiditySP.toFloat();
      irrigationSP = str_irrigationSP.toFloat();

      delay(tempRead_delay / 5);

      Serial.println("------------ PROCESS VARIABLES -------------");
      Serial.println("Temperatura STR SP: " + str_temperatureSP);
      Serial.println("Temperatura SP:  " + String(temperatureSP));
      Serial.println("KP: " + str_kp);
      Serial.println("KI: " + str_ki);
      Serial.println("KD: " + str_kd);
      Serial.println("Humedad relativa SP: " + str_relativeHumiditySP);
      Serial.println("Calidad de aire SP: " + str_airQualitySP);
      Serial.println("Humedad de suelo SP: " + str_groundHumiditySP);
      Serial.println("Nivel de agua SP: " + str_irrigationSP);
      Serial.println("------------ END PROCESS VARIABLES -------------");



      if (!isnan(temp1)) {
        FirebaseJson JSONTemperature1;
        JSONTemperature1.setDoubleDigits(2);
        JSONTemperature1.set("value", temp1);
        JSONTemperature1.set("ts/.sv", "timestamp");
        Firebase.RTDB.pushJSON(&fbdo, String(sensors + "/temperature_1"), &JSONTemperature1);  // Pushing Temperature DHT located at PIN 14
      }
      delay(tempRead_delay / 10);
      if (!isnan(temp2)) {
        FirebaseJson JSONTemperature2;
        JSONTemperature2.setDoubleDigits(2);
        JSONTemperature2.set("value", temp2);
        JSONTemperature2.set("ts/.sv", "timestamp");
        Firebase.RTDB.pushJSON(&fbdo, String(sensors + "/temperature_2"), &JSONTemperature2);  // Pushing Temperature DHT located at PIN 22
      }
      delay(tempRead_delay / 10);
      if (!isnan(relative1)) {
        FirebaseJson JSONRelative1;
        JSONRelative1.setDoubleDigits(2);
        JSONRelative1.set("value", relative1);
        JSONRelative1.set("ts/.sv", "timestamp");
        Firebase.RTDB.pushJSON(&fbdo, String(sensors + "/relative_humidity_1"), &JSONRelative1);  // Pushing Relative Humidity DHT located at PIN 14
      }
      delay(tempRead_delay / 10);
      if (!isnan(relative2)) {
        FirebaseJson JSONRelative2;
        JSONRelative2.setDoubleDigits(2);
        JSONRelative2.set("value", relative2);
        JSONRelative2.set("ts/.sv", "timestamp");
        Firebase.RTDB.pushJSON(&fbdo, String(sensors + "/relative_humidity_2"), &JSONRelative2);  // Pushing Relative Humidity DHT located at PIN 22
      }
      delay(tempRead_delay / 10);
      if (groundPercentage >= 0 && groundPercentage <= 100) {
        FirebaseJson JSONGroundHumidity;
        JSONGroundHumidity.setDoubleDigits(2);
        JSONGroundHumidity.set("value", groundPercentage);
        JSONGroundHumidity.set("ts/.sv", "timestamp");
        Firebase.RTDB.pushJSON(&fbdo, String(sensors + "/ground_humidity"), &JSONGroundHumidity);  // Pushing Ground Humidity Percentage located at PIN 34
      }
      delay(tempRead_delay / 10);
      if (waterLevel >= 0 && waterLevel <= 100) {
        FirebaseJson JSONWaterLevel;
        JSONWaterLevel.setDoubleDigits(2);
        JSONWaterLevel.set("value", waterLevel);
        JSONWaterLevel.set("ts/.sv", "timestamp");
        Firebase.RTDB.pushJSON(&fbdo, String(sensors + "/water_level"), &JSONWaterLevel);  // Pushing Water Level Percentage located at PIN 35
      }
      delay(tempRead_delay / 10);
      if (ppm >= 10 && ppm <= 1000) {
        FirebaseJson JSONAirQuality;
        JSONAirQuality.setDoubleDigits(2);
        JSONAirQuality.set("value", ppm);
        JSONAirQuality.set("ts/.sv", "timestamp");
        Firebase.RTDB.pushJSON(&fbdo, String(sensors + "/air_quality"), &JSONAirQuality);  // Pushing Air Quality Concentration located at PIN 32
      }
      delay(tempRead_delay * 3.2);
    }
  }
}