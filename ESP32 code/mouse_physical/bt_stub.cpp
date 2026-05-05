#include "bt_stub.h"
#include <BluetoothSerial.h>

// =====================================================
// GLOBAL BLUETOOTH OBJECT
// =====================================================
BluetoothSerial SerialBT;

// =====================================================
// INIT
// =====================================================
void btInit()
{
    if (!SerialBT.begin("Micromouse")) {
        Serial.println("BT init failed");
    } else {
        Serial.println("BT ready, name: Micromouse");
        SerialBT.println("LOG,Bluetooth Online");
    }
}

// =====================================================
// LOGGING
// Format: LOG,message
// =====================================================
void btSendLog(const char* msg)
{
    SerialBT.print("LOG,");
    SerialBT.println(msg);
}

// =====================================================
// BOT POSE
// Format: POSE,X,Y,ORI
// =====================================================
void btSendPose(int x, int y, int ori)
{
    SerialBT.print("POSE,");
    SerialBT.print(x); SerialBT.print(",");
    SerialBT.print(y); SerialBT.print(",");
    SerialBT.println(ori);
}

// =====================================================
// CALIBRATION
// Format: CALIB,MEASURED,LEARNED,LASTTICKS,TARGETTICKS
// =====================================================
void btSendCalib(float measuredW, float learnedW, long ticks, int targetTicks)
{
    SerialBT.printf("CALIB,%.1f,%.1f,%ld,%d\n",
                    measuredW,
                    learnedW,
                    ticks,
                    targetTicks);
}

// =====================================================
// MAZE CELL
// Format: CELL,X,Y,WALLMASK,VISITED
// =====================================================
void btSendCell(int x, int y, uint8_t wallsMask, uint8_t visited)
{
    SerialBT.print("CELL,");
    SerialBT.print(x); SerialBT.print(",");
    SerialBT.print(y); SerialBT.print(",");
    SerialBT.print(wallsMask); SerialBT.print(",");
    SerialBT.println(visited);
}

// =====================================================
// FINAL PATH
// Format: PATH,X,Y
// =====================================================
void btSendPathPoint(int x, int y)
{
    SerialBT.print("PATH,");
    SerialBT.print(x); SerialBT.print(",");
    SerialBT.println(y);
}

void btSendPathEnd()
{
    SerialBT.println("PATHEND");
}

// =====================================================
// PID TELEMETRY
// Format: PID,L,R,ERR,CENTER,BALANCE,TOTAL,FRONT
// =====================================================
void btSendPID(int L, int R, float err,
               float centerCorr, float balanceCorr,
               float totalCorr, int front)
{
    SerialBT.print("PID,");
    SerialBT.print(L); SerialBT.print(",");
    SerialBT.print(R); SerialBT.print(",");
    SerialBT.print(err); SerialBT.print(",");
    SerialBT.print(centerCorr); SerialBT.print(",");
    SerialBT.print(balanceCorr); SerialBT.print(",");
    SerialBT.print(totalCorr); SerialBT.print(",");
    SerialBT.println(front);
}

// =====================================================
// FLUSH
// =====================================================
void btFlush()
{
    SerialBT.flush();
}
