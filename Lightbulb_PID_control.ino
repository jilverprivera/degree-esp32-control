/*
*********************************************************************************
  TITLE: PID Control for a agroindustrial prototype (Greenhouse) using ESP32 & Firebase RTDB
  AUTHOR: Jilver Pacheco Rivera.
  DESCRIPTION: 50W Lightbulb PID control with a digital dimmer using 4n26 optocoupler, MOC3021 opto-triac and BTA16 triac.
  twiiter: https://twitter.com/Jilverprivera
  github: https://github.com/jilverprivera
  © 2022
* ******************************************************************************
*/

#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <addons/TokenHelper.h>
#include <addons/RTDBHelper.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>

#define WIFI_SSID "JILVER"
#define WIFI_PASSWORD "1098820841"

#define DATABASE_URL "https://greenpid-default-rtdb.firebaseio.com"
#define API_KEY "AIzaSyAs9Sax04m7HvqYVHVhb-1DzzMqIe2Y5OU"
#define USER_EMAIL "jpacheco3@udi.edu.co"
#define USER_PASSWORD "123456789"

#define DTH_PIN_1         24
#define DHT_PIN_2         25
#define DHT_TYPE          DHT22
#define MQ135_PIN         32
#define WATER_PIN         33
#define GROUND_PIN        34
#define FAN1_PIN          35
#define FAN2_PIN          36

#define FIRING_PIN        26
#define INTERRUPT_PIN     27

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

TaskHandle_t Task1;

DHT dht1(DTH_PIN_1, DHT_TYPE); // --> DHT 1
DHT dht2(DHT_PIN_2, DHT_TYPE); // --> DHT 2

// --- DB Paths --- //
String path = "/sensors";
String path_temperature_1 = path + "/temperature_1";
String path_temperature_2 = path + "/temperature_2";
String path_relative_1 = path + "/relative_humidity_1";
String path_relative_2 = path + "/relative_humidity_2";
String path_ground = path + "/ground_humidity";
String path_air_quality = path + "/air_quality";
String path_water_level = path + "/water_level";

// --- ACTUATORS --- //
int rpm_fan_1 = 0;
int rpm_fan_2 = 0;
int freq = 5000;
int channel_1 = 0;
int channel_2 = 1;
int resolution = 12;

// --- SENSORS --- //
float temp_1 = 0;
float relative_humidity_1 = 0;
float ppm = 0;
int ground_max_value = 4095;
float ground_percentage = 0;
float ground_humidity = 0;
int water_max_level = 4095;
float water_percentage = 0;
float water_level = 0;

// --- Millis --- //
unsigned long previousMillis = 0;
unsigned long currentMillis = 0;

// --- CONSTANTS --- //
int maximum_firing_delay = 9000;
int temp_read_Delay = 2000;

// --- PID Variables --- //
int setpoint = 32;
float PID_error = 0;
int PID_value = 0;
float previous_error = 0;
float elapsedTime, Time, timePrev;

// --- PID Constants --- //
int kp = 203;
int ki = 7.2;
int kd = 1.04;
int PID_p = 0;
int PID_i = 0;
int PID_d = 0;

/*
  void loop -> push each json obj to Firebase RTDB and get the variables.
  void loop2 -> PID control and sensor reading.
*/

void setup() {
  xTaskCreatePinnedToCore(
    loop2,
    "Task_1",
    10000,
    NULL,
    1,
    &Task1,
    0
  );
  Serial.begin(115200);
  pinMode(FIRING_PIN, OUTPUT);
  attachInterrupt(INTERRUPT_PIN, ZERO_CROSS, RISING);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("[Wi-Fi] Status: Connecting...");
    delay(1000);
  }
  Serial.println("[Wi-Fi] Status: Connected.");
  config.api_key = API_KEY;
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;
  config.database_url = DATABASE_URL;
  fbdo.setResponseSize(4096);
  config.token_status_callback = tokenStatusCallback;
  config.max_token_generation_retry = 10;
  Firebase.begin(&config, &auth);
  Serial.println("[Firebase] Status: Connected.");
  Firebase.reconnectWiFi(true);

  dht1.begin();

  
}

void loop() {
  if (Firebase.ready()) {
    //    FirebaseJson jVariables;
    //    Serial.printf("Get json ref... %s\n", Firebase.RTDB.getJSON(&fbdo, "/variables", &jVal) ? jVal.raw() : fbdo.errorReason().c_str());
    //    Serial.println(jVal.raw());
    //    jVal.toString(Serial, true);
    //    //    Serial.println("Running on: " +  String(xPortGetCoreID()));
    if (isnan(temp_1)) {
      Serial.println("Error. Failed to read TEMPERATURE from DHT sensor 1.");
    } else {
      FirebaseJson JSONTemperature;
      JSONTemperature.setDoubleDigits(3);
      JSONTemperature.set("value", temp_1);
      JSONTemperature.set("ts/.sv", "timestamp");
      Serial.printf("Set JSONTemperature... %s\n", Firebase.RTDB.pushJSON(&fbdo, path_temperature_1.c_str(), &JSONTemperature) ? "ok" : fbdo.errorReason().c_str());
    }
    if (isnan(relative_humidity_1)) {
      Serial.println("Error. Failed to read RELATIVE HUMIDITY from DHT sensor 1.");
    } else {
      FirebaseJson JSONRelative;
      JSONRelative.setDoubleDigits(3);
      JSONRelative.set("value", relative_humidity_1);
      JSONRelative.set("ts/.sv", "timestamp");
      Serial.printf("Set JSONRelative... %s\n", Firebase.RTDB.pushJSON(&fbdo, path_relative_1.c_str(), &JSONRelative) ? "ok" : fbdo.errorReason().c_str());
    }
    if (ground_percentage >= 0 && ground_percentage <= 100) {
      FirebaseJson JSONGroundHumidity;
      JSONGroundHumidity.setDoubleDigits(3);
      JSONGroundHumidity.set("value", ground_percentage);
      JSONGroundHumidity.set("ts/.sv", "timestamp");
      Serial.printf("Set JSONGroundHumidity... %s\n", Firebase.RTDB.pushJSON(&fbdo, path_ground.c_str(), &JSONGroundHumidity) ? "ok" : fbdo.errorReason().c_str());
    } else {
      Serial.println("Error. Failed to read GROUND HUMIDITY from HI-69 sensor.");
    }
    if (ppm >= 0 && ppm <= 1300) {
      FirebaseJson JSONAirQuality;
      JSONAirQuality.setDoubleDigits(3);
      JSONAirQuality.set("value", ppm);
      JSONAirQuality.set("ts/.sv", "timestamp");
      Serial.printf("Set JSONAirQuality... %s\n", Firebase.RTDB.pushJSON(&fbdo, path_air_quality.c_str(), &JSONAirQuality) ? "ok" : fbdo.errorReason().c_str());
    } else {
      Serial.println("Error. Failed to read PPM's from MQ135 sensor.");
    }
    delay(10000);
  }
}

void loop2(void *parameter) {
  while (true) {
    currentMillis = millis();
    if (currentMillis - previousMillis >= temp_read_Delay) {
      previousMillis += temp_read_Delay;
      temp_1 = dht1.readTemperature();
      relative_humidity_1 = dht1.readHumidity();
      ppm = analogRead(MQ135_PIN);
      ground_humidity = analogRead(GROUND_PIN);
      ground_percentage = (100 - (ground_humidity * 100) / ground_max_value);

      PID_error = setpoint - temp_1;
      if (PID_error > 5) {
        PID_i = 0;
      }
      PID_p = kp * PID_error;
      PID_i = PID_i + (ki * PID_error);
      timePrev = Time;
      Time = millis();
      elapsedTime = (Time - timePrev) / 1000;
      PID_d = kd * ((PID_error - previous_error) / elapsedTime);
      PID_value = PID_p + PID_i + PID_d;
      if (PID_value < 0) {
        PID_value = 0;
      }
      if (PID_value > 9000) {
        PID_value = 9000;
      }
    }
  }
}

// --- ISR --- //
void ZERO_CROSS() {
  delayMicroseconds(maximum_firing_delay - PID_value);
  digitalWrite(FIRING_PIN, HIGH); // disparo
  delayMicroseconds(100);
  digitalWrite(FIRING_PIN, LOW);
}
