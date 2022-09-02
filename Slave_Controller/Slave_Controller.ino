int ZERO_CROSS_PIN = 2;
int FIRING_PIN1 = 3;
int FIRING_PIN2 = 4;

int DHTDelay = 2500;
String PID_value = "";
float PID_value_toFloat = 0;
int Max_Firing_Delay = 7400;
int Min_Firing_Delay = 1350;
int Interval_Delay = 833;

void setup() {
  Serial.begin(9600);
  pinMode(FIRING_PIN1, OUTPUT);
  pinMode(FIRING_PIN2, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ZERO_CROSS_PIN), ZERO_CROSS_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ZERO_CROSS_PIN), ZERO_CROSS_ISR_2, RISING);
}

void loop() {
  if (Serial.available()) {
    PID_value = Serial.readString();
    PID_value_toFloat = PID_value.toFloat();
    if (PID_value_toFloat < Min_Firing_Delay) {
      PID_value = Min_Firing_Delay;
    }
    if (PID_value_toFloat > Max_Firing_Delay) {
      PID_value = Max_Firing_Delay;
    }
    PID_value = "";
    delay(DHTDelay);
  }
  PID_value = "";
}

void ZERO_CROSS_ISR() {
  delayMicroseconds(Max_Firing_Delay - PID_value_toFloat);
  digitalWrite(FIRING_PIN1, HIGH);
  delayMicroseconds(Interval_Delay);
  digitalWrite(FIRING_PIN1, LOW);
}

void ZERO_CROSS_ISR_2() {
  delayMicroseconds(Max_Firing_Delay - PID_value_toFloat);
  digitalWrite(FIRING_PIN2, HIGH);
  delayMicroseconds(Interval_Delay);
  digitalWrite(FIRING_PIN2, LOW);
}
