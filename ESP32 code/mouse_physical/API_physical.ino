// =========================================================

// API_physical.ino

// CENTER PID + ENCODER STRAIGHT BALANCE FOR FORWARD MOTION

// Tagged Telemetry Compatible with Python Dashboard

// =========================================================



#include <Arduino.h>

#include <Wire.h>

#include <SparkFun_VL53L5CX_Library.h>

#include <VL53L0X.h>

#include "bt_stub.h"

extern BluetoothSerial SerialBT;



// --------------------------------------------------------

// PIN MAP

// --------------------------------------------------------

#define A_IN1   4

#define A_IN2   13

#define B_IN1   27

#define B_IN2   16

#define STBY    33

#define PWM_A   26

#define PWM_B   25



// Encoders

#define ENC_AA  35

#define ENC_AB  34

#define ENC_BA  14

#define ENC_BB  32



// TOF Pins (XSHUT)

#define FRONT_RESET_PIN 23

#define XSHUT_LEFT  5

#define XSHUT_RIGHT 17



// I2C pins

#define I2C_SDA 21

#define I2C_SCL 22



// --------------------------------------------------------

// GLOBALS

// --------------------------------------------------------

SparkFun_VL53L5CX tofFront;

VL53L5CX_ResultsData tofFrontData;



VL53L0X tofLeft;

VL53L0X tofRight;



volatile long encA = 0;

volatile long encB = 0;



// --------------------------------------------------------

// SPEED PROFILES

// --------------------------------------------------------

int MOTOR_SPEED_EXPLORE = 250;

int MOTOR_SPEED_FAST   = 250;



int MOTOR_SPEED = MOTOR_SPEED_EXPLORE;



// --------------------------------------------------------

// WALL / DISTANCE CONSTANTS

// --------------------------------------------------------

const int FRONT_WALL_MM = 110;

const int SIDE_WALL_MM  = 255;



const int CENTER_TARGET = 145;

const int WALL_MAX     = 255;



// --------------------------------------------------------

// PIN AVOIDANCE GLOBALS

// --------------------------------------------------------

float filtered_pin_offset = 0.0f;



// --------------------------------------------------------

// PIN AVOIDANCE GLOBALS

// --------------------------------------------------------

const int PIN_DIS_THRESHOLD = 170;

const int PIN_DIVERSION_TICKS = 410;

const float MAX_PIN_SHIFT = 90.0f; // Tune this: higher = harder swerve



bool pin_avoiding = false;

long pin_start_tick = 0;

int pin_shift_dir = 0;

float current_pin_offset = 0.0f;



// --------------------------------------------------------

// CENTER PID GAINS

// --------------------------------------------------------

float Kp_center = 1.2;

float Ki_center = 0.00;

float Kd_center = 0.5;



float prevErr_center = 0;

float integral_center = 0;



// --------------------------------------------------------

// BALANCE GAINS (ENCODER)

// --------------------------------------------------------

float Kp_balance = 0.20f;

float Kd_balance = 0.10f;



long prev_err_balance = 0;

// --------------------------------------------------------

// ENCODER ISR

// --------------------------------------------------------

void IRAM_ATTR encA_ISR() { encA++; }

void IRAM_ATTR encB_ISR() { encB++; }



// --------------------------------------------------------

// SUPPORT

// --------------------------------------------------------

void resetEncoders() {

  encA = 0;

  encB = 0;

}



long avgTurnCounts() {

  return (abs(encA) + abs(encB)) / 2;

}



// --------------------------------------------------------

// MOTOR SETUP

// --------------------------------------------------------

void motorsInit() {

  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, HIGH);



  pinMode(A_IN1, OUTPUT);

  pinMode(A_IN2, OUTPUT);

  pinMode(B_IN1, OUTPUT);

  pinMode(B_IN2, OUTPUT);

  pinMode(PWM_A, OUTPUT);

  pinMode(PWM_B, OUTPUT);



  ledcAttachPin(PWM_A, 0);

  ledcAttachPin(PWM_B, 1);

  ledcSetup(0, 20000, 8);

  ledcSetup(1, 20000, 8);



  pinMode(ENC_AA, INPUT_PULLUP);

  pinMode(ENC_AB, INPUT_PULLUP);

  pinMode(ENC_BA, INPUT_PULLUP);

  pinMode(ENC_BB, INPUT_PULLUP);



  attachInterrupt(digitalPinToInterrupt(ENC_AA), encA_ISR, RISING);

  attachInterrupt(digitalPinToInterrupt(ENC_BA), encB_ISR, RISING);

}



// --------------------------------------------------------

// MOTOR CONTROL

// --------------------------------------------------------

void motorsStop() {

  ledcWrite(0, 0);

  ledcWrite(1, 0);

}



void motorA_forward(int spd) {

  digitalWrite(A_IN1, HIGH);

  digitalWrite(A_IN2, LOW);

  ledcWrite(0, spd);

}



void motorA_backward(int spd) {

  digitalWrite(A_IN1, LOW);

  digitalWrite(A_IN2, HIGH);

  ledcWrite(0, spd);

}



void motorB_forward(int spd) {

  digitalWrite(B_IN1, HIGH);

  digitalWrite(B_IN2, LOW);

  ledcWrite(1, spd);

}



void motorB_backward(int spd) {

  digitalWrite(B_IN1, LOW);

  digitalWrite(B_IN2, HIGH);

  ledcWrite(1, spd);

}



// --------------------------------------------------------

// TOF INIT

// --------------------------------------------------------



void tofInit() {



  Wire.begin(I2C_SDA, I2C_SCL);

  Wire.setClock(400000);   // Fast upload



  pinMode(XSHUT_LEFT, OUTPUT);

  pinMode(XSHUT_RIGHT, OUTPUT);



  digitalWrite(XSHUT_LEFT, LOW);

  digitalWrite(XSHUT_RIGHT, LOW);



  // VL53L5CX init

  if (!tofFront.begin(0x29, Wire)) {

    Serial.println("VL53L5CX INIT FAILED");

    return;

  }



  tofFront.setAddress(0x30);



  // Keep your original accuracy settings

  tofFront.setResolution(8 * 8);

  tofFront.setRangingFrequency(15);



  tofFront.startRanging();



  // Left

  digitalWrite(XSHUT_LEFT, HIGH);

  if (tofLeft.init()) {

    tofLeft.setAddress(0x31);

    tofLeft.setMeasurementTimingBudget(10000);

    tofLeft.setTimeout(50);

  }



  // Right

  digitalWrite(XSHUT_RIGHT, HIGH);

  if (tofRight.init()) {

    tofRight.setAddress(0x32);

    tofRight.setMeasurementTimingBudget(10000);

    tofRight.setTimeout(50);

  }

}

// --------------------------------------------------------

// SENSOR HELPERS

// --------------------------------------------------------

int getFront() {



  static int lastValid = 4000;

  static int cached_res = 0;



  if (!tofFront.isDataReady())

    return lastValid;



  tofFront.getRangingData(&tofFrontData);



  if (cached_res == 0) {

    cached_res = tofFront.getResolution();

    if (cached_res != 16 && cached_res != 64) cached_res = 64;

  }



  // --- TELEMETRY INJECTION ---

  SerialBT.print("FPIX");

  for (int i = 0; i < cached_res; i++) {

    SerialBT.print(",");

    SerialBT.print(tofFrontData.distance_mm[i]);

  }

  SerialBT.println();



  // ==========================================================

  // 1. STANDARD FRONT WALL DETECTION (CENTER PIXELS)

  // ==========================================================

  int d1, d2, d3, d4;

  if (cached_res == 16) {

    d1 = tofFrontData.distance_mm[5];  d2 = tofFrontData.distance_mm[6];

    d3 = tofFrontData.distance_mm[9];  d4 = tofFrontData.distance_mm[10];

  } else {

    d1 = tofFrontData.distance_mm[27]; d2 = tofFrontData.distance_mm[28];

    d3 = tofFrontData.distance_mm[35]; d4 = tofFrontData.distance_mm[36];

  }



  int sum = 0, count = 0;

  if (d1 > 0 && d1 < 4000) { sum += d1; count++; }

  if (d2 > 0 && d2 < 4000) { sum += d2; count++; }

  if (d3 > 0 && d3 < 4000) { sum += d3; count++; }

  if (d4 > 0 && d4 < 4000) { sum += d4; count++; }



  lastValid = (count > 0) ? (sum / count) : 4000;



  // ==========================================================

  // 2. PIN DETECTION & COLUMN AVERAGING LOGIC

  // ==========================================================

  long currentTicks = (abs(encA) + abs(encB)) / 2;



  // If already dodging, execute the 410-tick diversion profile

  if (pin_avoiding) {

      long elapsed = currentTicks - pin_start_tick;

     

      if (elapsed < PIN_DIVERSION_TICKS) {

          // Gradual Shift: Increases to MAX_PIN_SHIFT at halfway, then decreases

          float mag = 0;

          long half_ticks = PIN_DIVERSION_TICKS / 2;

         

          if (elapsed < half_ticks) {

              mag = ((float)elapsed / half_ticks) * MAX_PIN_SHIFT;

          } else {

              mag = ((float)(PIN_DIVERSION_TICKS - elapsed) / half_ticks) * MAX_PIN_SHIFT;

          }

          current_pin_offset = mag * pin_shift_dir;

      } else {

          // Diversion complete, restore immediately

          pin_avoiding = false;

          current_pin_offset = 0.0f;

      }

  }

  else if (cached_res == 64) {

      // Look for a new pin by averaging columns (ignoring top 2 rows)

      long colSum[8] = {0};

      int colCount[8] = {0};



      for (int c = 0; c < 8; c++) {

          for (int r = 2; r < 8; r++) { // Ignore r=0 and r=1

              int d = tofFrontData.distance_mm[r * 8 + c];

              if (d > 0 && d < 4000) {

                  colSum[c] += d;

                  colCount[c]++;

              }

          }

      }



      int minCol = -1;

      int minAvg = 4000;



      // Find the column with the lowest average

      for (int c = 0; c < 8; c++) {

          int colAvg = (colCount[c] > 0) ? (colSum[c] / colCount[c]) : 4000;

          if (colAvg < minAvg) {

              minAvg = colAvg;

              minCol = c;

          }

      }



      // If the lowest column hits the threshold, trigger the diversion!

      if (minCol != -1 && minAvg <= PIN_DIS_THRESHOLD) {

          pin_avoiding = true;

          pin_start_tick = currentTicks;

         

          if (minCol <= 3) {

              // Pin is on LEFT (Cols 0-3) -> Divert RIGHT (positive offset)

              pin_shift_dir = 1;

          } else {

              // Pin is on RIGHT (Cols 4-7) -> Divert LEFT (negative offset)

              pin_shift_dir = -1;

          }

      }

  }



  return lastValid;

}

int getLeft() {

  return tofLeft.readRangeSingleMillimeters();

}



int getRight() {

  return tofRight.readRangeSingleMillimeters();

}



bool wallFront() { return getFront() < FRONT_WALL_MM; }

bool wallLeft()  { return getLeft()  < SIDE_WALL_MM; }

bool wallRight() { return getRight() < SIDE_WALL_MM; }



// --------------------------------------------------------

// CENTER PID (DYNAMIC TARGET ENABLED)

// --------------------------------------------------------

// Added targetShift parameter.

// Negative shift steers bot RIGHT. Positive shift steers bot LEFT.

float centerPID(int leftD, int rightD, float targetShift) {



  float error = 0.0f;



  bool leftWall  = (leftD  > 0 && leftD  < SIDE_WALL_MM);

  bool rightWall = (rightD > 0 && rightD < SIDE_WALL_MM);



  // NO WALL → NO CENTER PID (But still allow pin shifting in open space!)

  if (!leftWall && !rightWall) {

    integral_center = 0;

    prevErr_center = 0;

    // If in open space but dodging a pin, rely entirely on the targetShift

    return (Kp_center * targetShift);

  }



  // ONE WALL

  if (leftWall && !rightWall) {

    error = -(float)(165 - leftD) + targetShift;

  }

  else if (!leftWall && rightWall) {

    error = -(float)(rightD - 165) + targetShift;

  }

  // BOTH WALLS

  else {

    // Multiply shift by 2 because we are overriding two physical walls

    error = (float)(leftD - rightD) + (targetShift * 2.0f);

  }



  // PID MATH

  integral_center += error;

  float derivative = error - prevErr_center;

  prevErr_center = error;



  float corr = (Kp_center * error) +

               (Ki_center * integral_center) +

               (Kd_center * derivative);



  return corr;

}

// --------------------------------------------------------

// STRAIGHT BALANCE (ENCODER)

// --------------------------------------------------------

float balanceStraight() {

  long diff = encA - encB;

  long d_err = diff - prev_err_balance;

  prev_err_balance = diff;



  float correction = (Kp_balance * diff) + (Kd_balance * d_err);



  return correction;

}



// --------------------------------------------------------

// ADVANCED PIN DETECTION (BACKGROUND DEPTH COMPARISON)

// --------------------------------------------------------

float filtered_pin_shift = 0.0f;



float getPinTargetShift() {

  if (tofFront.getResolution() != 64) return 0.0f;



  // 1. Calculate the "Background Wall" distance using the extreme Left/Right edges.

  // A thin pin won't trigger the outer edges (Cols 0 and 7). A solid wall will.

  int bg_sum = 0, bg_count = 0;

  for (int r = 3; r <= 5; r++) {

      int dL = tofFrontData.distance_mm[r * 8 + 0]; // Leftmost column

      int dR = tofFrontData.distance_mm[r * 8 + 7]; // Rightmost column

      if (dL > 0 && dL < 4000) { bg_sum += dL; bg_count++; }

      if (dR > 0 && dR < 4000) { bg_sum += dR; bg_count++; }

  }

  int bg_dist = (bg_count > 0) ? (bg_sum / bg_count) : 4000;



  // 2. Scan the inner columns for the Pin (Cols 1 to 6)

  int min_left = 4000;

  int min_right = 4000;



  for (int r = 3; r <= 6; r++) {

      // Left side of the path (Cols 1, 2, 3)

      for (int c = 1; c <= 3; c++) {

          int d = tofFrontData.distance_mm[r * 8 + c];

          if (d > 0 && d < min_left) min_left = d;

      }

      // Right side of the path (Cols 4, 5, 6)

      for (int c = 4; c <= 6; c++) {

          int d = tofFrontData.distance_mm[r * 8 + c];

          if (d > 0 && d < min_right) min_right = d;

      }

  }



  float raw_shift = 0.0f;

  const int PIN_THRESHOLD = 300; // mm: Start looking at the pin earlier

  const float MAX_SHIFT = 200.0f; // Boosted shift to easily overpower physical side walls



  int overall_min = min(min_left, min_right);



  // 3. CORE LOGIC: Is the object significantly closer than the background?

  // If the background wall is at 300mm and pin is at 150mm, diff is 150mm -> It's a pin!

  // If it's just a flat wall, diff is ~0mm -> Ignore!

  if (overall_min < PIN_THRESHOLD && (bg_dist - overall_min) > 75) {

     

      if (min_left < min_right) {

          // Pin LEFT -> Steer RIGHT (Negative Shift)

          // Float math gives a perfectly smooth ramp from 0 to MAX_SHIFT

          float ratio = (float)(PIN_THRESHOLD - min_left) / (float)(PIN_THRESHOLD - 30);

          raw_shift = -(ratio * MAX_SHIFT);

      }

      else {

          // Pin RIGHT -> Steer LEFT (Positive Shift)

          float ratio = (float)(PIN_THRESHOLD - min_right) / (float)(PIN_THRESHOLD - 30);

          raw_shift = (ratio * MAX_SHIFT);

      }

  }



  // 4. Fast-reacting filter (70% new data, 30% old data)

  // This prevents the delay you were seeing while still smoothing out noise.

  filtered_pin_shift = (0.3f * filtered_pin_shift) + (0.7f * raw_shift);

 

  return filtered_pin_shift;

}

// --------------------------------------------------------

// MOVE FORWARD — CENTER + BALANCE

// --------------------------------------------------------



// --------------------------------------------------------

// NODE MOVE CONSTANTS

// --------------------------------------------------------

extern const int VIRTUAL_TICKS_PER_CELL = 1559; // Used for math to map the grid

const int TICKS_TO_CENTER_NODE = 695;    // Ticks to drive to center of junction after wall change is detected



// --------------------------------------------------------

// MOVE FORWARD TO NEXT NODE — CONTINUOUS WITH PID & TELEMETRY

// --------------------------------------------------------

bool moveToNextNode(long &ticks_moved, int &cells_moved, bool &corridorLeft, bool &corridorRight) {



  resetEncoders();

  delay(20); // Settle TOF sensors



  bool corridorLocked = false;

  bool nodeFound = false;

  bool blockedFront = false;

 

  long nodeDetectedTick = 0;

  long active_centering_ticks = TICKS_TO_CENTER_NODE; // Dynamic centering target



  int changeDebounce = 0;



  motorA_forward(MOTOR_SPEED);

  motorB_forward(MOTOR_SPEED);



  unsigned long start = millis();

  unsigned long lastTOF = 0;



  int L = getLeft();

  int R = getRight();

  int prevL = L;

  int prevR = R;



  // CONTINUOUS SINGLE LOOP

  while (true) {

    if (millis() - start > 8000) {

      motorsStop();

      break;

    }



    bool newTofData = false;

    if (millis() - lastTOF >= 25) {

      lastTOF = millis();

      L = getLeft();

      R = getRight();

      newTofData = true;

    }



    long currentTicks = (abs(encA) + abs(encB)) / 2;



    // IMMEDIATE STOP ON FRONT WALL

    if (wallFront()) {

      blockedFront = true;

      if (!nodeFound) {

        nodeFound = true;

        nodeDetectedTick = currentTicks;

        SerialBT.println("LOG: NODE DETECTED (FRONT WALL)");

      }

      break;

    }



    if (newTofData) {

        // ==========================================================

        // RULE 1: CAPTURE CORRIDOR AT 300 TICKS

        // Lock the true corridor walls. Never overwrite this.

        // ==========================================================

        if (currentTicks >= 180 && !corridorLocked) {

            corridorLeft = (L > 0 && L < SIDE_WALL_MM);

            corridorRight = (R > 0 && R < SIDE_WALL_MM);

            corridorLocked = true;

        }



        // ==========================================================

        // RULE 2: SCAN CONTINUOUSLY FOR NODE

        // ==========================================================

        if (corridorLocked && !nodeFound) {

            bool currentLeftWall = (L > 0 && L < SIDE_WALL_MM);

            bool currentRightWall = (R > 0 && R < SIDE_WALL_MM);



            // BOUNDED PEG DETECTOR: Only valid if the reading drops into the physical wall range.

            // Fixes the "turned immediately in open space" bug.

            bool hitPegLeft = (prevL > 0 && prevL - L > 60 && L < SIDE_WALL_MM + 40);

            bool hitPegRight = (prevR > 0 && prevR - R > 60 && R < SIDE_WALL_MM + 40);



            bool patternChanged = (currentLeftWall != corridorLeft || currentRightWall != corridorRight);



            if (patternChanged || hitPegLeft || hitPegRight) {

               

                // Is it an opening? (Wall disappeared)

                bool isOpening = (!currentLeftWall && corridorLeft) || (!currentRightWall && corridorRight);

               

                if (isOpening && !hitPegLeft && !hitPegRight) {

                    changeDebounce++;

                    if (changeDebounce >= 2) {

                        nodeFound = true;

                        nodeDetectedTick = currentTicks;

                        active_centering_ticks = TICKS_TO_CENTER_NODE; // Standard center

                        SerialBT.println("LOG: NODE DETECTED (OPENING)");

                    }

                } else {

                    // It is a Peg or New Wall!

                    // FOV FIX: We see the peg ~5cm early. We MUST add ticks to the center target

                    // so we don't stop short and turn too early.

                    nodeFound = true;

                    nodeDetectedTick = currentTicks;

                    active_centering_ticks = TICKS_TO_CENTER_NODE + 160; // Drive further to compensate

                    SerialBT.println("LOG: NODE DETECTED (PEG)");

                }

            } else {

                changeDebounce = 0;

            }

        }

       

        prevL = L;

        prevR = R;

    }



    // ==========================================================

    // RULE 3: CENTERING COOLDOWN

    // ==========================================================

    if (nodeFound && !blockedFront) {

        if (currentTicks >= nodeDetectedTick + active_centering_ticks) {

            break;

        }

    }



    // 1. Get fresh data and calculate the gradual pin dodge target

    float pinShift = 0.0f;

   

    // CUTOFF SWITCH: Only calculate and apply pin shifting if we haven't found a node yet.

    if (!nodeFound) {

        if (newTofData) {

            int frontD = getFront(); // Refreshes tofFrontData safely

            pinShift = getPinTargetShift();

        } else {

            pinShift = filtered_pin_shift; // Keep using last known shift between readings

        }

    } else {

        // If node is found, instantly kill the shift so wall detection and turning stabilize!

        pinShift = 0.0f;

        filtered_pin_shift = 0.0f; // Reset the global filter too so it doesn't carry over to the next cell

    }



    // 2. Feed the dynamic target directly into your PID

    float centerCorr  = centerPID(L, R, pinShift);

    float balanceCorr = balanceStraight();

    float totalCorr = constrain(centerCorr + balanceCorr, -80, 80);



    int leftSPD  = constrain(MOTOR_SPEED - totalCorr, 80, 255);

    int rightSPD = constrain(MOTOR_SPEED + totalCorr, 80, 255);



    ledcWrite(0, leftSPD);

    ledcWrite(1, rightSPD);



    // --- TELEMETRY ---

    if (newTofData) {

        int frontD = getFront();

        float err = (float)(L - R);

        btSendPID(L, R, err, centerCorr, balanceCorr, totalCorr, (frontD < FRONT_WALL_MM));

        SerialBT.print("F,"); SerialBT.print(frontD);

        SerialBT.print(",L,"); SerialBT.print(L);

        SerialBT.print(",R,"); SerialBT.println(R);

    }

  }



  motorsStop();



  if (!corridorLocked) {

      corridorLeft = (getLeft() > 0 && getLeft() < SIDE_WALL_MM);

      corridorRight = (getRight() > 0 && getRight() < SIDE_WALL_MM);

  }



  extern bool g_isFastRun;

  if (g_isFastRun) {

      delay(150);

  } else {

      delay(60);

  }



  ticks_moved = (abs(encA) + abs(encB)) / 2;

 

  // CORE FIX: Increase the rounding offset (850 instead of 759) to prevent dropping

  // the cell count when the bot stops early at an open junction!

  cells_moved = (ticks_moved + 850) / VIRTUAL_TICKS_PER_CELL;



  SerialBT.print("ENC_NODE,");

  SerialBT.print(ticks_moved);

  SerialBT.print(",C:");

  SerialBT.println(cells_moved);



  if (cells_moved == 0) return false;

  return true;

}

// --------------------------------------------------------

// TURNING (ENCODER-BASED)

// --------------------------------------------------------

const int TURN_SPEED = 150;

const int TURN_TICKS_90 = 473;



void gyroTurnDegrees(float targetDeg, bool left) {



  resetEncoders();



  if (left) {

    motorA_backward(TURN_SPEED);

    motorB_forward(TURN_SPEED);

  } else {

    motorA_forward(TURN_SPEED);

    motorB_backward(TURN_SPEED);

  }



  unsigned long t0 = millis();



  long targetTicks = (long)(TURN_TICKS_90 * (targetDeg / 90.0f));



  while (avgTurnCounts() < targetTicks) {

    if (millis() - t0 > 3500) break;

  }



  motorsStop();



  // ROOT FIX: Settle rotational momentum only in Phase 3

  extern bool g_isFastRun;

  if (g_isFastRun) {

      delay(150);

  }



  // RESET PID + ENCODERS AFTER TURN

  integral_center = 0;

  prevErr_center = 0;

  prev_err_balance = 0;

  encA = 0;

  encB = 0;

}



void turnLeft()   { gyroTurnDegrees(90.0f, true);  }

void turnRight()  { gyroTurnDegrees(90.0f, false); }

void turnAround() { gyroTurnDegrees(190.0f, false); }

// --------------------------------------------------------

// SPEED CONTROL

// --------------------------------------------------------

bool g_isFastRun = false;



void setExploreSpeed() {

  MOTOR_SPEED = MOTOR_SPEED_EXPLORE;

  g_isFastRun = false;

}



void setFastSpeed() {

  MOTOR_SPEED = MOTOR_SPEED_FAST;

  g_isFastRun = true;

}



// --------------------------------------------------------

// FRONT ADJUST

// --------------------------------------------------------

void frontAdjust() {



  const int TARGET = FRONT_WALL_MM; // 132



  unsigned long t0 = millis();



  while (true) {



    int d = getFront();



    // Exit if no wall or wall too far

    if (d <= 0 || d > 200)

      break;



    // Stop when target reached

    if (d <= TARGET)

      break;



    // Timeout safety

    if (millis() - t0 > 1200)

      break;



    motorA_forward(210);

    motorB_forward(210);



    delay(50);

    motorsStop();

    delay(6);

    delay(0);

  }



  motorsStop();

}



// --------------------------------------------------------

// END OF FILE

// --------------------------------------------------------