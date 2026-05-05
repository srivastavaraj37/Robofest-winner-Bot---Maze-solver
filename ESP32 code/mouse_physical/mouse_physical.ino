// 44444=========================================================
// mouse_physical.ino ( full exploration )
// FULL PHYSICAL MICROMOUSE CONTROLLER (10×10 grid)
// Hybrid Trémaux B2 + Floodfill + DFS Optimization
// PHASE 1: Exploration
// PHASE 2: DFS Optimal Path ( multiple optimal paths )
// PHASE 3: Fast Run
// =========================================================

#include <Arduino.h>
#include "maze_map.h"
#include "pathfinding.h"
#include "motion.h"
#include "bt_stub.h"
#include <BluetoothSerial.h>


// ================= GLOBAL MEMORY BUFFERS =================
static MazeMap maze;
static int distGoal[MAX_W][MAX_H];

// -----------------------------------------------
// Start Position (Micromouse Standard
// -----------------------------------------------
int gMazeW = MAX_W;
int gMazeH = MAX_H;

const int START_X   = 13;
const int START_Y   = 13;
const int START_ORI = 0; // 0=N,1=E,2=S,3=W

// Bluetooth
extern BluetoothSerial SerialBT;

// ================= RESET DIAGNOSTICS =================
static int gResetReason = 0;

// Manual button
#define BUTTON_PIN 2

// ----------------------------------------------------------
// BT Mapping helpers
// ----------------------------------------------------------
inline void sendMappedCell(int x, int y, uint8_t wallsMask, uint8_t visited)
{
    btSendCell(x, y, wallsMask, visited);
}

inline void sendMappedPose(int x, int y, int ori)
{
    btSendPose(x, y, ori);
}

// ----------------------------------------------------------
// WAIT BUTTON
// ----------------------------------------------------------
void waitForButtonPressBlocking() {
    while (digitalRead(BUTTON_PIN) == HIGH) delay(10);
    delay(40);
    while (digitalRead(BUTTON_PIN) == LOW) delay(10);
    delay(40);
}

// =========================================================
// SETUP
// =========================================================
void setup() {
    
    Serial.begin(115200);
    delay(300);
    // Capture reset reason BEFORE anything else
    gResetReason = esp_reset_reason();

    Serial.print("Reset reason: ");
    Serial.println(esp_reset_reason());

    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(0, INPUT_PULLUP);

    motorsInit();
    tofInit();
    btInit();
    
    // Send reset reason via Bluetooth
    char resetMsg[40];
    sprintf(resetMsg, "RESET REASON: %d", gResetReason);
    btSendLog(resetMsg);

    Serial.println("READY – waiting for START...");
    btSendLog("READY – waiting for START");
}

// =========================================================
// LOOP
// =========================================================
bool START_ALLOWED = false;

void loop() {

    if (!START_ALLOWED) {

        if (SerialBT.available()) {
            String cmd = SerialBT.readStringUntil('\n');
            cmd.trim();
            if (cmd.equalsIgnoreCase("R")) {
                START_ALLOWED = true;
                btSendLog("Bluetooth START received");
                delay(200);
            }
        }

        if (digitalRead(0) == 0) {
            START_ALLOWED = true;
            btSendLog("Button START received");
            delay(300);
        }

        return;
    }

    runController();

    btSendLog("Maze complete. Locking system.");
    while (1) delay(1000);
}

// =========================================================
// MAIN CONTROLLER
// =========================================================
void runController()
{
    // ------------------------------------------
    // Maze model + starting cell
    // ------------------------------------------
    maze.clear();

    int x   = START_X;
    int y   = START_Y;
    int ori = START_ORI;

    // Seal ONLY the back wall of the starting box. 
    // This maintains the boundary but leaves the sides wide open for Cell Z!
    maze.setWall(x, y, (ori + 2) & 3); 

    int goals[4][2];
    int goalCount = 0;

    maze.computeDistancesTo(goals, goalCount, distGoal);
    maze.markVisit(x, y);
    maze.isNode[x][y] = true; 
    sendMappedPose(x, y, ori);

    // =====================================================
    // PHASE 1 — HYBRID TRÉMAUX (FULL EXPLORE, EXIT LATCH)
    // =====================================================
    btSendLog("PHASE 1: Hybrid Trémaux Exploration START");
    setExploreSpeed();

    bool exitGoalSet = false;
    bool GOAL_SET = false;

    int steps = 0;
    int lastVisited = 1;
    int noGrowth = 0;

    // --------- track previous cell + lastDir ------
    int prevX = x;
    int prevY = y;
    int lastDir = -1;
    // -------------------------------------------------------

    while (true)
    {
        // (The ORIGIN RETURN CHECK has been completely deleted so the bot can cross the origin freely)

        prevX = x;
        prevY = y;

        // ==================================================
        // EXIT CHECK — MUST HAPPEN BEFORE ANY MOVE DECISION
        // ==================================================
        if (!GOAL_SET)
        {
            int dF = getFront();
            int dL = getLeft();
            int dR = getRight();

            // Restored YOUR perfectly tuned physical exit logic
            bool exitDetected =
                (dR > 500) &&
                (dF > 800) &&
                (dL > 350);

            if (exitDetected)
            {
                btSendLog("EXIT detected — 180 turn NOW, 1 step");

                // CORE FIX: Seal the physical edge of the maze! 
                maze.setWall(x, y, ori);               // Seal virtual front
                maze.setWall(x, y, (ori + 1) & 3);     // Seal virtual right
                maze.setWall(x, y, (ori + 3) & 3);     // Seal virtual left

                // Lock goal at THIS cell
                GOAL_SET = true;
                exitGoalSet = true;

                goals[0][0] = x;
                goals[0][1] = y;
                goalCount = 1;
                maze.computeDistancesTo(goals, goalCount, distGoal);

                // Mark incoming traversal cleanly
                if (lastDir >= 0)
                {
                    maze.markEdge(prevX, prevY, lastDir);
                }

                // Turn 180 degrees cleanly
                int targetDir = (ori + 2) & 3;
                turnToDirAndUpdate(ori, targetDir);

                // Move to the next node and map the distance
                long ticks = 0;
                int cells_moved = 0;
                bool cLeft = false, cRight = false;
                if (moveToNextNode(ticks, cells_moved, cLeft, cRight))
                {
                    for (int step = 0; step < cells_moved; step++) {
                        int px = x;
                        int py = y;
                        
                        x += DX[ori];
                        y += DY[ori];
                        maze.markVisit(x, y);
                        
                        // Mark the edge we are taking back inside the maze
                        maze.markEdge(px, py, ori);

                        if (step < cells_moved - 1) {
                            if (cRight) maze.setWall(x, y, (ori + 1) & 3); 
                            if (cLeft)  maze.setWall(x, y, (ori + 3) & 3); 
                            
                            // Send intermediate cell to Bluetooth so UI updates correctly
                            uint8_t wMask = 0;
                            for (int d = 0; d < 4; d++) if (maze.walls[x][y][d]) wMask |= (1 << d);
                            sendMappedCell(x, y, wMask, maze.visitCount[x][y]);
                        }
                    }
                    sendMappedPose(x, y, ori);

                    // Reset Trémaux state cleanly
                    lastDir = -1;
                    prevX = x;
                    prevY = y;
                }
                // Skip Trémaux this cycle
                continue;
            }
        } 
        
        // ------------------------------------------
        // 1. READ WALLS + UPDATE MAZE MODEL
        // ------------------------------------------
        maze.isNode[x][y] = true; 

        bool f = wallFront();
        bool l = wallLeft();
        bool r = wallRight();

        // CORE FIX: ORIGIN VOID SEAL
        // If the bot just started and sees the massive void on either side,
        // force BOTH side walls to strictly seal the physical boundary!
        if (steps == 0 && x == START_X && y == START_Y) {
            int rawL = getLeft();
            int rawR = getRight();
            if (rawL > 1500 || rawR > 1500) {
                btSendLog("Origin void detected! Forcing side walls.");
                l = true;
                r = true;
            }
        }

        if (f) maze.setWall(x, y, ori);
        if (l) maze.setWall(x, y, (ori + 3) & 3);
        if (r) maze.setWall(x, y, (ori + 1) & 3);

        uint8_t wallsMask = 0;
        for (int d = 0; d < 4; d++)
            if (maze.walls[x][y][d]) wallsMask |= (1 << d);

        sendMappedCell(x, y, wallsMask, maze.visitCount[x][y]);

        // ==========================================
        // DYNAMIC MAZE COMPLETION CHECK
        // ==========================================
        if (isExplorationComplete(maze)) {
            btSendLog("ALL ACCESSIBLE CELLS VISITED! Maze closed.");
            break; // Immediately exit Phase 1 Exploration!
        }

        // ------------------------------------------
        // 2. HYBRID TRÉMAUX DECISION
        // ------------------------------------------
        IntCell next;
        bool ok = chooseTremauxNext(maze, distGoal, x, y, ori, next);

        if (!ok) {
            btSendLog("No moves left — exploration ends");
            break;
        }

        // ------------------------------------------
        // 3. TURN + MOVE (ONLY MOVE POINT)
        // ------------------------------------------
        turnToDirAndUpdate(ori, next.dir);
        frontAdjust();

        long ticks = 0;
        int cells_moved = 0;
        bool cLeft = false, cRight = false;
        if (!moveToNextNode(ticks, cells_moved, cLeft, cRight)) {
            maze.setWall(x, y, next.dir);
            btSendLog("Move blocked – wall added");
            continue;
        }

        // ------------------------------------------
        // 4. UPDATE POSITION (NODE BASED MULTI-CELL)
        // ------------------------------------------
        for (int step = 0; step < cells_moved; step++) {
            int px = x;
            int py = y;
            
            x += DX[ori];
            y += DY[ori];

            maze.markVisit(x, y);
            
            // ROOT FIX: markEdge is automatically symmetric! Call it exactly ONCE per cell.
            maze.markEdge(px, py, ori);
            
            if (step < cells_moved - 1) {
                if (cRight) maze.setWall(x, y, (ori + 1) & 3); 
                if (cLeft)  maze.setWall(x, y, (ori + 3) & 3); 
                
                // Send intermediate cell to Bluetooth so UI updates correctly
                uint8_t wMask = 0;
                for (int d = 0; d < 4; d++) if (maze.walls[x][y][d]) wMask |= (1 << d);
                sendMappedCell(x, y, wMask, maze.visitCount[x][y]);
            }
        }

        sendMappedPose(x, y, ori);

        if ((steps % 8) == 0)
            maze.computeDistancesTo(goals, goalCount, distGoal);

        steps++;

        // ------------------------------------------
        // 5. STALL DETECTION
        // ------------------------------------------
        int visitedCells = 0;
        for (int i = 0; i < gMazeW; i++)
            for (int j = 0; j < gMazeH; j++)
                if (maze.visitCount[i][j] > 0)
                    visitedCells++;

        if (visitedCells == lastVisited)
            noGrowth++;
        else {
            noGrowth = 0;
            lastVisited = visitedCells;
        }

        if (noGrowth > 200) {
            btSendLog("Exploration stagnated — stopping");
            break;
        }

        delay(0);
    }

    // ------------------------------------------
    // SAFETY FALLBACK GOALS
    // ------------------------------------------
    if (!exitGoalSet)
    {
        btSendLog("No EXIT found — using fallback goals");

        goals[0][0] = gMazeW - 1; goals[0][1] = gMazeH - 1;
        goals[1][0] = gMazeW - 1; goals[1][1] = 0;
        goals[2][0] = 0;          goals[2][1] = gMazeH - 1;
        goalCount = 3;

        maze.computeDistancesTo(goals, goalCount, distGoal);
    }

    // =====================================================
    // PHASE 2 — DFS OPTIMIZATION
    // =====================================================
    btSendLog("PHASE 2: Press button for DFS Optimization");
    waitForButtonPressBlocking();

    // ROOT FIX: Moved to static to prevent 8KB ESP32 Stack Overflow at 20x20
    static Path finalPath;
    finalPath.len = 0;

    bool okDFS = dfsOptimizeOnVisited(
                    maze,
                    goals,
                    goalCount,
                    distGoal,
                    START_X,
                    START_Y,
                    finalPath );

    if (!okDFS || finalPath.len < 2)
    {
        btSendLog("DFS failed — using shortest known path");
        shortestPathOnKnownMap(maze, goals, goalCount, finalPath);
    }
    else {
        btSendLog("DFS optimal path computed");
    }

    if (finalPath.len < 2) {
        btSendLog("NO PATH AVAILABLE — ABORT");
        return;
    }

    btSendLog("Sending final path to app");

    for (int i = 0; i < finalPath.len; i++) {
        int px = finalPath.cells[i].x;
        int py = finalPath.cells[i].y;
        btSendPathPoint(px, py);
    }
    btSendPathEnd();

    // =====================================================
    // PHASE 3 — FAST RUN
    // =====================================================
    btSendLog("PHASE 3: Press button for FAST RUN");
    waitForButtonPressBlocking();

    btSendLog("FAST RUN STARTED");
    setFastSpeed();

    x = START_X;
    y = START_Y;
    ori = START_ORI;
    sendMappedPose(x, y, ori);

    // --------------------------------------------
    // PHASE 3 — DYNAMIC SHORTEST PATH TO FIXED GOAL
    // --------------------------------------------

    // Always use floodfill as source of truth
    maze.computeDistancesOnVisitedTo(goals, goalCount, distGoal);

    while (true)
    {
        // -----------------------------
        // STOP CONDITION (REAL LOGIC)
        // -----------------------------
        if (distGoal[x][y] >= INF_DIST) {
            btSendLog("GOAL UNREACHABLE — STOPPING");
            break;
        }

        // Reached goal
        if (distGoal[x][y] == 0) {
            btSendLog("GOAL REACHED");
            break;
        }

        // -----------------------------
        // FIND BEST NEXT STEP (WITH MOMENTUM)
        // -----------------------------
        int bestDir = -1;
        int bestD   = distGoal[x][y];
        int bestTurnPenalty = 255;

        for (int d = 0; d < 4; d++) {
            if (maze.isBlocked(x, y, d)) continue;

            int nx = x + DX[d];
            int ny = y + DY[d];

            if (!maze.inBounds(nx, ny)) continue;

            int nDist = distGoal[nx][ny];
            
            // Calculate how hard the turn is (0=Straight, 1=Left/Right, 2=U-Turn)
            int relDir = (d - ori) & 3;
            int turnPenalty = (relDir == 0) ? 0 : (relDir == 2) ? 2 : 1;

            // CORE FIX: Only go to cells that are closer to the goal.
            // If two paths are equally close, ALWAYS pick the one with the least turning!
            if (nDist < distGoal[x][y]) {
                if (nDist < bestD || (nDist == bestD && turnPenalty < bestTurnPenalty)) {
                    bestD   = nDist;
                    bestTurnPenalty = turnPenalty;
                    bestDir = d;
                }
            }
        }

        // Safety
        if (bestDir < 0) {
            btSendLog("NO DESCENT EDGE — STOPPING");
            break;
        }

        // -----------------------------
        // MOVE
        // -----------------------------
        turnToDirAndUpdate(ori, bestDir);
        frontAdjust();

        long ticks = 0;
        int cells_moved = 0;
        bool cLeft = false, cRight = false;
        if (!moveToNextNode(ticks, cells_moved, cLeft, cRight)) {
            btSendLog("BLOCKED — UPDATING MAP");

            // Remove this edge from graph
            maze.setWall(x, y, bestDir);

            int bx = x + DX[bestDir];
            int by = y + DY[bestDir];
            if (maze.inBounds(bx, by)) {
                maze.setWall(bx, by, (bestDir + 2) & 3);
            }

            // Recompute floodfill
            maze.computeDistancesOnVisitedTo(goals, goalCount, distGoal);

            delay(1);  // ESP32 watchdog / BT yield
            continue;
        }

        // -----------------------------
        // MOVE SUCCESS (NODE MULTI-CELL)
        // -----------------------------
        for (int step = 0; step < cells_moved; step++) {
            x += DX[ori];
            y += DY[ori];
            maze.markVisit(x, y);
            
            if (step < cells_moved - 1) {
                // Send intermediate cell to Bluetooth so UI updates correctly
                uint8_t wMask = 0;
                for (int d = 0; d < 4; d++) if (maze.walls[x][y][d]) wMask |= (1 << d);
                sendMappedCell(x, y, wMask, maze.visitCount[x][y]);
            }
        }
        
        sendMappedPose(x, y, ori);

        delay(1);  // ESP32 watchdog / BT yield
        delay(0);
    }


    btSendLog("FAST RUN COMPLETE");
    Serial.println("LOG: All phases completed successfully");
}