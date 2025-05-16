//Code for the SD2 STABILUS_POWER Team Presentation & Expo
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

// === ADXL345 Setup ===
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// === State Reporting ===
String doorStateStr = "Idle";
String doorReason   = "Startup";

// === Smart Button Settings ===
const int smartButton1Pin = 11;
const int smartButton2Pin = 12;
const unsigned long longPressTime = 1000; // 1 s for long press
unsigned long pressStartTime1 = 0;
unsigned long pressStartTime2 = 0;
bool button1Pressed = false;
bool button2Pressed = false;

bool smartOpenCommand = false;
bool smartCloseCommand = false;

// === MD10 R2 Pins ===
const int motorPWM = 5;
const int motorDIR = 4;

// === Encoder Pins ===
const int hall1 = 3;
const int hall2 = 2;
volatile long encoderPos = 0;

// === Remote Buttons ===
const int remoteOpenPin  = 6;
const int remoteClosePin = 7;

// === Latch Pins ===
const int latchSigPin = 10;
const int latchNCPin  = 9;

// === Ultrasonic ===
const int trigPin = 47;
const int echoPin = 49;

// === Bounds & Speeds ===
const int maxPosition = 535;
const int minPosition = 0;
const int positionBuffer = 5;
const int openSpeed  = 110;
const int closeSpeed = 130;

// === Current Sensor ===
const int currentPin = A0;
const float VREF = 5.0;
const float sensitivity = 0.066;
const float zeroCurrentVoltage = VREF / 2.0;

// === Obstacle & Stall ===
bool doorMoving = false;
bool movingOpen = false;
bool obstacleDetected = false;
long lastEncoderValue = 0;
int stallCounter = 0;
const int stallLimit = 6;

// === Helpers ===
void stopMotor(const char* reason) {
  analogWrite(motorPWM, 0);
  digitalWrite(motorDIR, LOW);
  pinMode(motorDIR, INPUT);
  doorMoving   = false;
  doorStateStr = "Idle";
  doorReason   = reason;
  Serial.print("[STALL] ");
  Serial.println(reason);
}

void moveMotor(int baseSpeed, const char* reason) {
  pinMode(motorDIR, OUTPUT);
  digitalWrite(motorDIR, baseSpeed > 0 ? LOW : HIGH);
  analogWrite(motorPWM, abs(baseSpeed));
  doorMoving   = true;
  movingOpen   = baseSpeed > 0;
  doorStateStr = movingOpen ? "Opening" : "Closing";
  doorReason   = reason;
  Serial.print("[ACTION] ");
  Serial.println(reason);
}

void unlockLatch() {
  Serial.println("[EVENT] Unlocking latch");
  digitalWrite(latchSigPin, HIGH);
  delay(300);
  digitalWrite(latchSigPin, LOW);
  delay(500);
}

void handleEncoder() {
  bool A = digitalRead(hall1);
  bool B = digitalRead(hall2);
  static uint8_t lastEncoded = 0;
  uint8_t currentEncoded = (A << 1) | B;
  int8_t transition = (lastEncoded << 2) | currentEncoded;
  switch (transition) {
    case 0b0001: case 0b0111: case 0b1110: case 0b1000: encoderPos++; break;
    case 0b0010: case 0b0100: case 0b1101: case 0b1011: encoderPos--; break;
    default: break;
  }
  if (encoderPos < minPosition) encoderPos = minPosition;
  lastEncoded = currentEncoded;
}

float readCurrent() {
  int adc = analogRead(currentPin);
  float voltage = (adc / 1023.0) * VREF;
  return (voltage - zeroCurrentVoltage) / sensitivity;
}

bool readLatchNC() {
  pinMode(latchNCPin, INPUT);
  delayMicroseconds(10);
  bool state = digitalRead(latchNCPin);
  pinMode(latchNCPin, OUTPUT);
  digitalWrite(latchNCPin, LOW);
  return state == HIGH;
}

float readUltrasonicDistance() {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return -1;
  return duration * 0.0343 / 2.0;
}

void updateSmartButton(int pin, unsigned long &pressStart, bool &buttonPressed, bool &openCmd, bool &closeCmd) {
  bool currentState = digitalRead(pin);
  if (currentState == LOW && !buttonPressed) {
    buttonPressed = true;
    pressStart = millis();
  }
  else if (currentState == HIGH && buttonPressed) {
    buttonPressed = false;
    unsigned long duration = millis() - pressStart;
    if (duration < longPressTime) {
      openCmd = true;
      if (!readLatchNC()) unlockLatch();
    }
  }
  if (buttonPressed && (millis() - pressStart >= longPressTime)) {
    closeCmd = true;
    buttonPressed = false;
  }
}

void handleSmartButtons() {
  updateSmartButton(smartButton1Pin, pressStartTime1, button1Pressed, smartOpenCommand, smartCloseCommand);
  updateSmartButton(smartButton2Pin, pressStartTime2, button2Pressed, smartOpenCommand, smartCloseCommand);
}

void setup() {
  // speed up PWM on pin 5
  TCCR2B = TCCR2B & 0b11111000 | 0x02;

  Serial.begin(9600);
  while (!Serial) { }

  // ADXL345 init
  if (!accel.begin()) {
    Serial.println("ERROR: ADXL345 not detected");
    while (1) delay(10);
  }
  accel.setRange(ADXL345_RANGE_16_G);

  // pin modes
  pinMode(motorPWM, OUTPUT);
  pinMode(motorDIR, OUTPUT);
  pinMode(hall1, INPUT_PULLUP);
  pinMode(hall2, INPUT_PULLUP);
  pinMode(remoteOpenPin, INPUT_PULLUP);
  pinMode(remoteClosePin, INPUT_PULLUP);
  pinMode(latchSigPin, OUTPUT);
  digitalWrite(latchSigPin, LOW);
  pinMode(latchNCPin, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(smartButton1Pin, INPUT_PULLUP);
  pinMode(smartButton2Pin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(hall1), handleEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(hall2), handleEncoder, CHANGE);

  Serial.println("Setup complete");
  doorStateStr = "Idle";
  doorReason   = "Ready";
}

void loop() {
  handleSmartButtons();

  bool openPressed  = digitalRead(remoteOpenPin)  == LOW;
  bool closePressed = digitalRead(remoteClosePin) == LOW;
  bool latchUnlocked = readLatchNC();
  float distance    = readUltrasonicDistance();
  obstacleDetected  = (distance > 40 && distance < 140);

  if (!latchUnlocked) encoderPos = 0;

  if (doorMoving) {
    if ((movingOpen  && openPressed  && !lastOpenPressed) ||
        (!movingOpen && closePressed && !lastClosePressed)) {
      stopMotor("Manual stop");
      justStopped = true;
    }
    else if (obstacleDetected) {
      stopMotor("Obstacle");
      justStopped = true;
    }
  }

  if (!doorMoving && !justStopped && !obstacleDetected) {
    if ((openPressed || smartOpenCommand) && encoderPos < maxPosition - positionBuffer) {
      if (!latchUnlocked) unlockLatch();
      doorReason = openPressed ? "Remote Open" : "Smart Open";
      moveMotor(openSpeed, doorReason.c_str());
      smartOpenCommand = false;
    }
    else if ((closePressed || smartCloseCommand) && encoderPos > minPosition + positionBuffer) {
      doorReason = closePressed ? "Remote Close" : "Smart Close";
      moveMotor(-closeSpeed, doorReason.c_str());
      smartCloseCommand = false;
    }
  }

  if (!openPressed && !closePressed) justStopped = false;

  if (doorMoving) {
    if (movingOpen && encoderPos >= maxPosition - positionBuffer) {
      stopMotor("Reached open limit");
      encoderPos = maxPosition;
    }
    else if (!movingOpen && encoderPos <= minPosition + positionBuffer) {
      stopMotor("Reached close limit");
      encoderPos = minPosition;
    }
  }

  if (doorMoving) {
    if (encoderPos == lastEncoderValue) {
      stallCounter++;
      if (stallCounter >= stallLimit) {
        stopMotor("Stall");
        justStopped = true;
        stallCounter = 0;
      }
    } else stallCounter = 0;
    lastEncoderValue = encoderPos;
  }

  lastOpenPressed  = openPressed;
  lastClosePressed = closePressed;

  // Telemetry outputs:
  Serial.print("[POS] ");     
  Serial.println(encoderPos);

  float current = readCurrent();
  Serial.print("[CURRENT] ");
  Serial.println(current, 2);

  Serial.print("[STATE] ");
  Serial.println(doorStateStr);
  Serial.print("[REASON] ");
  Serial.println(doorReason);

  Serial.print("[OBSTACLE] ");
  Serial.println(obstacleDetected ? "YES" : "NO");

  sensors_event_t ev;
  accel.getEvent(&ev);
  float pitch = atan2(ev.acceleration.x, sqrt(ev.acceleration.y*ev.acceleration.y + ev.acceleration.z*ev.acceleration.z)) * 180.0 / PI;
  Serial.print("[INCLINE] ");
  Serial.println(pitch, 1);

  delay(100);
}
