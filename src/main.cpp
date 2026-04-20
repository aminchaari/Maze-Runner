#include <Arduino.h>

// ================== PIN CONFIG ==================
//start pin
#define START_PIN 4
// Motor Right (Matches main code IN1/IN2)
#define R_IN1 26
#define R_IN2 27
// Motor Left (Matches main code IN3/IN4)
#define L_IN1 32
#define L_IN2 14

// Encoder Right
#define ENC_R_A 16
#define ENC_R_B 17
// Encoder Left
#define ENC_L_A 25
#define ENC_L_B 33

// Ultrasoncic Sensor 
#define TRIG_FRONT 5
#define ECHO_FRONT 18

#define TRIG_LEFT  19
#define ECHO_LEFT  34

#define TRIG_RIGHT 23
#define ECHO_RIGHT 35
bool started = false;


// PWM Channels
const int CH_R1 = 0; const int CH_R2 = 1;
const int CH_L1 = 2; const int CH_L2 = 3;

// ================== ROBOT PARAMS ==================

float wheel_diameter = 90.69;    // mm 
float wheel_spacing  = 178.6;    // mm (This is your DBW)
int encoder_cpr = 411;           // 4X decoding CPR to match your main TPR

// ================== GLOBALS ==================

volatile long encL = 0;
volatile long encR = 0;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// ================== INTERRUPTS ==================
void IRAM_ATTR isr_right_A() {
  if (digitalRead(ENC_R_A) == digitalRead(ENC_R_B)) encR++; else encR--;
}
void IRAM_ATTR isr_right_B() {
  if (digitalRead(ENC_R_A) == digitalRead(ENC_R_B)) encR--; else encR++;
}
void IRAM_ATTR isr_left_A() {
  if (digitalRead(ENC_L_A) == digitalRead(ENC_L_B)) encL++; else encL--;
}
void IRAM_ATTR isr_left_B() {
  if (digitalRead(ENC_L_A) == digitalRead(ENC_L_B)) encL--; else encL++;
}

//Ultrasonic distance functions
long getFrontDistance() {
  digitalWrite(TRIG_FRONT, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_FRONT, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_FRONT, LOW);
  long duration = pulseIn(ECHO_FRONT, HIGH, 30000); // 30ms timeout
  if (duration == 0) return 50; // no echo = no wall
  return duration * 0.034 / 2; // Convert to cm
}

long getLeftDistance() {
  digitalWrite(TRIG_LEFT, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_LEFT, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_LEFT, LOW);
  long duration = pulseIn(ECHO_LEFT, HIGH, 30000); // 30ms timeout
  if (duration == 0) return 50; // no echo = no wall
  return duration * 0.034 / 2; // Convert to cm
}

long getRightDistance() {
  digitalWrite(TRIG_RIGHT, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_RIGHT, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_RIGHT, LOW);
  long duration = pulseIn(ECHO_RIGHT, HIGH, 30000); // 30ms timeout
  if (duration == 0) return 50; // no echo = no wall
  return duration * 0.034 / 2; // Convert to cm
}

// ================== UTILS ==================

float mmToCounts(float mm) {
  float circumference = PI * wheel_diameter;
  return (mm / circumference) * encoder_cpr;
}

void setMotor(int ch1, int ch2, int speed) {
  if (speed >= 0) {
    ledcWrite(ch1, speed);
    ledcWrite(ch2, 0);
  } else {
    ledcWrite(ch1, 0);
    ledcWrite(ch2, abs(speed));
  }
}

void stopMotors() {
  ledcWrite(CH_L1, 0); ledcWrite(CH_L2, 0);
  ledcWrite(CH_R1, 0); ledcWrite(CH_R2, 0);
}

// ================== MOTION PROFILE ==================

int rampSpeed(long current, long target, int maxSpeed) {
  float ratio = (float)current / target;
  int minSpeed = 80; // Minimum PWM to overcome static friction

  // Acceleration (0 → 0.3)
  if (ratio < 0.3)
    return minSpeed + (maxSpeed - minSpeed) * (ratio / 0.3);

  // Deceleration (0.7 → 1)
  if (ratio > 0.7)
    return minSpeed + (maxSpeed - minSpeed) * ((1.0 - ratio) / 0.3);

  // Constant
  return maxSpeed;
}

// ================== MOVE FORWARD (WITH PID) ==================

void moveForward(float distance_mm, int maxSpeed = 150) {
  portENTER_CRITICAL(&mux);
  encL = 0; encR = 0;
  portEXIT_CRITICAL(&mux);

  long target = mmToCounts(distance_mm);

  // --- PID CONSTANTS --- 
  // You will likely need to adjust these!
  float Kp = 1.2;   // Proportional: Reacts to current error
  float Ki = 0.05;  // Integral: Reacts to accumulated past error
  float Kd = 0.5;   // Derivative: Reacts to how fast the error is changing

  long prev_error = 0;
  long integral = 0;

  while (true) {
    portENTER_CRITICAL(&mux);
    long currentL = abs(encL);
    long currentR = abs(encR);
    portEXIT_CRITICAL(&mux);

    long avg = (currentL + currentR) / 2;
    if (avg >= target) break;

    // Get the base speed from your ramp up/down profile
    int base_speed = rampSpeed(avg, target, maxSpeed);

    // 1. Calculate Error (If Left is ahead, error is positive)
    long error = currentL - currentR;

    // 2. Accumulate Integral and calculate Derivative
    integral += error;
    long derivative = error - prev_error;

    // 3. Compute the PID Correction value
    int correction = (Kp * error) + (Ki * integral) + (Kd * derivative);

    // 4. Apply Correction
    // If Left is ahead (positive correction) -> Left slows down, Right speeds up
    int speedL = base_speed + correction;
    int speedR = base_speed - correction;

    // 5. Constrain speeds to safe 8-bit PWM ranges (0 to 255)
    speedL = constrain(speedL, 0, 255);
    speedR = constrain(speedR, 0, 255);

    // Note: We removed your static 0.9326 ratio because PID handles it now!
    setMotor(CH_L1, CH_L2, speedL);
    setMotor(CH_R1, CH_R2, speedR);

    // Save current error for the next loop iteration
    prev_error = error;
    
    delay(5);
  }
  stopMotors();
}

// ================== ROTATE (WITH PID) ==================

void rotate(float angle_deg, int maxSpeed = 150) {
  portENTER_CRITICAL(&mux);
  encL = 0; encR = 0;
  portEXIT_CRITICAL(&mux);

  // Calculate absolute arc length each wheel needs to travel
  float arc = (PI * wheel_spacing * abs(angle_deg)) / 360.0;
  long target = mmToCounts(arc);

  // --- DIRECTION HANDLING ---
  // Standard: Positive angle = Right turn (Left goes FWD, Right goes REV)
  // If your robot turns the wrong way, just swap the 1 and -1 in these two lines!
  int dirL = (angle_deg > 0) ? 1 : -1;
  int dirR = (angle_deg > 0) ? -1 : 1; 

  // --- PID CONSTANTS --- 
  // You can tune these separately from your moveForward PID if needed
  float Kp = 1.2;  
  float Ki = 0.05; 
  float Kd = 0.5;  

  long prev_error = 0;
  long integral = 0;

  while (true) {
    portENTER_CRITICAL(&mux);
    long currentL = abs(encL);
    long currentR = abs(encR);
    portEXIT_CRITICAL(&mux);

    long avg = (currentL + currentR) / 2;
    if (avg >= target) break;

    // Get the base speed magnitude
    int base_speed = rampSpeed(avg, target, maxSpeed);

    // 1. Calculate Error (Comparing absolute distances traveled)
    // If Left has spun further than Right, error is positive
    long error = currentL - currentR;

    // 2. Accumulate Integral and calculate Derivative
    integral += error;
    long derivative = error - prev_error;

    // 3. Compute the PID Correction
    int correction = (Kp * error) + (Ki * integral) + (Kd * derivative);

    // 4. Apply Correction to the Speed Magnitudes
    // If Left is ahead (positive correction) -> decrease Left, increase Right
    int magL = base_speed + correction;
    int magR = base_speed - correction;

    // 5. Constrain magnitudes to safe 8-bit PWM limits (0 to 255)
    magL = constrain(magL, 0, 255);
    magR = constrain(magR, 0, 255);

    // 6. Apply the directional signs and send to motors
    setMotor(CH_L1, CH_L2, magL * dirL);
    setMotor(CH_R1, CH_R2, magR * dirR);

    // Save error for the next loop
    prev_error = error;
    
    delay(5);
  }
  stopMotors();
}

// ================== SETUP (CALIBRATION) ==================

void setup() {
  Serial.begin(9600);
  
  // --- HARDWARE CONFIGURATION ---
  ledcSetup(CH_R1, 2000, 8); ledcSetup(CH_R2, 2000, 8);
  ledcSetup(CH_L1, 2000, 8); ledcSetup(CH_L2, 2000, 8);

  ledcAttachPin(R_IN1, CH_R1); ledcAttachPin(R_IN2, CH_R2);
  ledcAttachPin(L_IN1, CH_L1); ledcAttachPin(L_IN2, CH_L2);

  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);

  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);

  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);

  pinMode(ENC_R_A, INPUT); pinMode(ENC_R_B, INPUT);
  pinMode(ENC_L_A, INPUT); pinMode(ENC_L_B, INPUT);
  //pin setup start
  pinMode(START_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isr_right_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_B), isr_right_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isr_left_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_B), isr_left_B, CHANGE);
  
  
  delay(3000); // Wait 3 seconds to put the robot on the ground
}

// ================== MACROS ==================

void MCF(){
  moveForward(400);
  stopMotors();
}

void MCR(){
  rotate(90);
  moveForward(400);
  stopMotors();
}

void MCL(){
  rotate(-90);
  moveForward(400);
  stopMotors();
}

long getFrontAvg() {
  long sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += getFrontDistance();
    delay(5); // petit délai pour stabiliser le capteur
  }
  return sum / 10;
}

long getRightAvg() {
  long sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += getRightDistance();
    delay(5);
  }
  return sum / 10;
}

long getLeftAvg() {
  long sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += getLeftDistance();
    delay(5);
  }
  return sum / 10;
}

// ================== LOOP ==================

void loop() {
   if (digitalRead(START_PIN) == HIGH && !started) {
    started = true;
   }
    if (!started) {
    stopMotors();
    return;
  }

  delay(100);

  long front = getFrontAvg();
  long right = getRightAvg();
  long left  = getLeftAvg();

  int wallDist = 25;      // cm
  int frontThresh = 15;   // cm

  if (right > wallDist) {
    Serial.println("Turning Right");
    rotate(100);
    delay(5);
    moveForward(400); // 1 cell
  }
  else if (front > frontThresh) {
    Serial.println("Moving Forward");
    moveForward(400);  // 1 cell
  }
  else {
    Serial.println("Turning Left");
    rotate(-100);
   // delay(5);
   // moveForward(400);
  }

 delay(50);
}