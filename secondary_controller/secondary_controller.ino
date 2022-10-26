#define zeroCrossPIN 2
#define firingPIN1 3
#define firingPIN2 4

String(str_PID_value) = "";

float PID_value = 0;
int value = 0;
int tempRead_delay = 2500;

int maxFiring_delay = 7400;
int minFiring_delay = 10;
int interval_delay = 833;

void setup() {
  Serial.begin(9600);
  pinMode(firingPIN1, OUTPUT);
  pinMode(firingPIN2, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(zeroCrossPIN), zero_cross_ISR, RISING);
}

void loop() {
  currentTime = millis();
  if (currentTime - prevTime >= tempRead_delay) {
    prevTime += tempRead_delay;
    if (Serial.available()) {
      str_PID_value = Serial.readString();
      PID_value = str_PID_value.toFloat();
    }
    if (PID_value >= 0 && PID_value <= 100) {
      value = map(PID_value, 0, 100, maxFiring_delay, minFiring_delay);
    }
    Serial.print(value);
    Serial.print(" ");
    Serial.print(PID_value);
    Serial.print(" ");
    Serial.println(sp);
  }
}

void zero_cross_ISR() {
  delayMicroseconds(value);
  digitalWrite(firingPIN1, HIGH);
  digitalWrite(firingPIN2, HIGH);
  delayMicroseconds(interval_delay);
  digitalWrite(firingPIN1, LOW);
  digitalWrite(firingPIN2, LOW);
}