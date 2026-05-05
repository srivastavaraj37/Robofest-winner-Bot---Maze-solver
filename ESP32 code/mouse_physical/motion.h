#pragma once
#include <Arduino.h>
#include "maze_map.h"

// From API_physical.ino
// Replace bool moveForward(); with:
bool moveToNextNode(long &ticks_moved, int &cells_moved, bool &corridorLeft, bool &corridorRight);
extern const int VIRTUAL_TICKS_PER_CELL;
bool wallFront();
bool wallLeft();
bool wallRight();
void turnLeft();
void turnRight();
void turnAround();

// ------------------------------------------------------------
// SIMPLE, CONSISTENT TURNING LOGIC (Option A — Your Style)
// ------------------------------------------------------------
inline void turnToDirAndUpdate(int &ori, int targetDir) {

    int diff = (targetDir - ori) & 3;

    // Already facing correct direction?
    if (diff == 0) return;

    // Turn right once
    if (diff == 1) {
        turnRight();
        ori = (ori + 1) & 3;
        return;
    }

    // Turn 180° in one smooth motion
    if (diff == 2) {
        turnAround();
        ori = (ori + 2) & 3;
        return;
    }

    // Turn left once
    if (diff == 3) {
        turnLeft();
        ori = (ori + 3) & 3;
        return;
    }
}