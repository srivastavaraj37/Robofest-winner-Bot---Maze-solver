#pragma once
#include <Arduino.h>
#include <stdint.h>

// ============================
// Bluetooth API
// ============================

void btInit();

void btSendLog(const char* msg);
void btSendPose(int x, int y, int ori);

// Calibration Telemetry
// Format: CALIB,MEASURED,LEARNED,LASTTICKS,TARGETTICKS
void btSendCalib(float measuredW, float learnedW, long ticks, int targetTicks);

void btSendCell(int x, int y, uint8_t wallsMask, uint8_t visited);
void btSendPathPoint(int x, int y);
void btSendPathEnd();

// PID Telemetry
// Format: PID,L,R,ERR,CENTER,BALANCE,TOTAL,FRONT
void btSendPID(int L, int R, float err,
               float centerCorr, float balanceCorr,
               float totalCorr, int front);

// Optional but recommended
void btFlush();
