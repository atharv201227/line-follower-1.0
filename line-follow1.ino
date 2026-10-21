// ======================================================
// ESP32 HIGH-SPEED 5-SENSOR LINE FOLLOWER
// STEP 2 - PID CONTROL
// ======================================================

// ---------------- TB6612FNG ----------------

#define PWMA 25
#define AIN1 26
#define AIN2 27

#define PWMB 14
#define BIN1 32
#define BIN2 33

#define STBY 13

// ---------------- IR SENSORS ----------------

#define S1 34   // Far Left
#define S2 35   // Left
#define S3 36   // Center
#define S4 39   // Right
#define S5 4    // Far Right


// ======================================================
// SETTINGS
// ======================================================

// Change this if your sensor gives the opposite reading.
//
// If BLACK LINE = LOW:
#define LINE_STATE LOW

// If your sensor gives BLACK = HIGH, change to:
// #define LINE_STATE HIGH


// Starting speed.
// DO NOT start at 255.
int baseSpeed = 120;

int maxSpeed = 255;


// ======================================================
// PID VALUES
// ======================================================

// Start with these values.
// We will tune them after testing.

float Kp = 10.0; //30.0  // Responds to current error magnitude
float Ki = 10.0;   //was actualy 0.0  good=10.0 very good = 8.0  //Addresses accumulated past errors
float Kd = 70.0;   // 18.0 // Predicts future error based on the rate of change

float error = 0;
float lastError = 0;
float integral = 0;


// Last direction in which the line was seen
int lastDirection = 0;


// ======================================================
// SETUP
// ======================================================

void setup() {

  Serial.begin(115200);

  // Motor pins
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(STBY, OUTPUT);

  // IR sensors
  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);
  pinMode(S5, INPUT);

  // Enable motor driver
  digitalWrite(STBY, HIGH);

  stopMotors();

  delay(2000);

  Serial.println("================================");
  Serial.println("  5 SENSOR PID LINE FOLLOWER");
  Serial.println("================================");
}


// ======================================================
// MAIN LOOP
// ======================================================

void loop() {

  int s1 = digitalRead(S1);
  int s2 = digitalRead(S2);
  int s3 = digitalRead(S3);
  int s4 = digitalRead(S4);
  int s5 = digitalRead(S5);


  // ----------------------------------------------------
  // CONVERT SENSOR VALUES
  // ----------------------------------------------------

  bool leftOuter  = (s1 == LINE_STATE);
  bool leftInner  = (s2 == LINE_STATE);
  bool center     = (s3 == LINE_STATE);
  bool rightInner = (s4 == LINE_STATE);
  bool rightOuter = (s5 == LINE_STATE);


  // ----------------------------------------------------
  // CALCULATE LINE POSITION
  //
  //       -4   -2    0    +2   +4
  //        S1   S2   S3    S4   S5
  // ----------------------------------------------------

  int total = 0;
  int weighted = 0;

  if (leftOuter) {
    weighted += -4;
    total++;
  }

  if (leftInner) {
    weighted += -2;
    total++;
  }

  if (center) {
    weighted += 0;
    total++;
  }

  if (rightInner) {
    weighted += 2;
    total++;
  }

  if (rightOuter) {
    weighted += 4;
    total++;
  }


  // ----------------------------------------------------
  // LINE FOUND
  // ----------------------------------------------------

  if (total > 0) {

    error = (float)weighted / total;

    // Remember direction
    if (error < 0) {
      lastDirection = -1;
    }
    else if (error > 0) {
      lastDirection = 1;
    }
    else {
      lastDirection = 0;
    }

  }

  // ----------------------------------------------------
  // LINE LOST
  // ----------------------------------------------------

  else {

    // Keep turning toward where the line disappeared

    if (lastDirection < 0) {
      error = -5;
    }
    else if (lastDirection > 0) {
      error = 5;
    }
    else {
      error = 0;
    }
  }


  // ====================================================
  // PID
  // ====================================================

  integral += error;

  // Prevent integral wind-up
  integral = constrain(integral, -20, 20);

  float derivative = error - lastError;

  float correction =
      (Kp * error) +
      (Ki * integral) +
      (Kd * derivative);

  lastError = error;


  // ====================================================
  // MOTOR SPEED
  // ====================================================

  int leftSpeed  = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;

  leftSpeed = constrain(leftSpeed, -maxSpeed, maxSpeed);
  rightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);


  setMotor(leftSpeed, rightSpeed);


  // ====================================================
  // DEBUG
  // ====================================================

  Serial.print("Sensors: ");

  Serial.print(s1);
  Serial.print(" ");
  Serial.print(s2);
  Serial.print(" ");
  Serial.print(s3);
  Serial.print(" ");
  Serial.print(s4);
  Serial.print(" ");
  Serial.print(s5);

  Serial.print(" | Error: ");
  Serial.print(error);

  Serial.print(" | L: ");
  Serial.print(leftSpeed);

  Serial.print(" | R: ");
  Serial.println(rightSpeed);


  delay(2);
}


// ======================================================
// MOTOR CONTROL
// ======================================================

void setMotor(int leftSpeed, int rightSpeed) {

  // ---------------- LEFT MOTOR ----------------

  if (leftSpeed >= 0) {

    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);

  }
  else {

    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);

    leftSpeed = -leftSpeed;
  }


  // ---------------- RIGHT MOTOR ----------------

  if (rightSpeed >= 0) {

    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);

  }
  else {

    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);

    rightSpeed = -rightSpeed;
  }


  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);


  analogWrite(PWMA, leftSpeed);
  analogWrite(PWMB, rightSpeed);
}


// ======================================================
// STOP
// ======================================================

void stopMotors() {

  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);

  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}