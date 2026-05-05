#include <Arduino.h>
#include "maze_map.h"

struct DFSState {
    int x, y, ori;
    float curT;
    int fwds, turns;
    uint8_t childIndex;
};

// MMS API (from API.ino)
void log(String message);
void setColor(int x, int y, char color);
void clearColor(int x, int y);
void setText(int x, int y, String text);
void clearText(int x, int y);
void setWall(int x, int y, char direction);
bool wallFront();
bool wallLeft();
bool wallRight();
bool moveForward();
bool wasReset();
void ackReset();
void turnLeft();
void turnRight();

// ================= DFS GLOBAL BUFFERS =================
// Move heavy DFS arrays off stack to prevent stack overflow
static DFSState dfsStack[MAX_W * MAX_H];
static bool dfsVisited[MAX_W][MAX_H];
static Path dfsCurPath;

// ================= TREMAUX GLOBAL BUFFERS =================
static int tremauxFDist[MAX_W][MAX_H];
static int tremauxQX[MAX_W * MAX_H];
static int tremauxQY[MAX_W * MAX_H];

// --------------------------------------
// BASIC CELL STRUCT FOR TREMAUX OUTPUT
// --------------------------------------
struct IntCell {
    int x;
    int y;
    int dir;
};


inline int manhattan(int a, int b, int c, int d) {
    return abs(a - c) + abs(b - d);
}

inline bool inGoalArea(int x, int y, int W, int H) {
    return (x == W - 1 && y == H - 1);
}

// ======================================================
// FLOODFILL VISUALIZATION
// ======================================================
inline void updateFloodVisual(const MazeMap &maze,
                              int dist[MAX_W][MAX_H])
{
    for (int i = 0; i < gMazeW; i++)
        for (int j = 0; j < gMazeH; j++) {
            int d = dist[i][j];
            String s = (d >= INF_DIST ? " " : String(d));
            setText(i, j, s);
        }
}

// ======================================================
// PATH VISUALIZATION
// ======================================================
inline void colorPlan(const Path &p, char c) {
    for (int i = 0; i < p.len; i++) {
        int x = p.cells[i].x;
        int y = p.cells[i].y;
        if (x >= 0 && x < gMazeW && y >= 0 && y < gMazeH)
            setColor(x, y, c);
    }
}

inline void clearPlanVisual(const Path &p, const MazeMap &maze) {
    for (int i = 0; i < p.len; i++) {
        int x = p.cells[i].x;
        int y = p.cells[i].y;
        if (x >= 0 && x < gMazeW && y >= 0 && y < gMazeH) {
            if (maze.visitCount[x][y] == 0)
                clearColor(x, y);
        }
    }
}
inline bool isExplorationComplete(const MazeMap &maze) {
    // Look at every cell in the grid...
    for (int x = 0; x < MAX_W; x++) {
        for (int y = 0; y < MAX_H; y++) {
            
            // CORE FIX: ONLY evaluate true Nodes! This ignores phantom openings in intermediate cells.
            if (maze.isNode[x][y]) {
                // Check all 4 openings (North, East, South, West)
                for (int d = 0; d < 4; d++) {
                    
                    // CORE FIX: Ignore the blind spot directly behind the starting position.
                    // This allows early termination if the origin is an entry doorway.
                    if (x == START_X && y == START_Y && d == ((START_ORI + 2) & 3)) {
                        continue; 
                    }

                    // If the opening is NOT closed by a physical wall...
                    if (!maze.isBlocked(x, y, d)) {
                        int nx = x + DX[d];
                        int ny = y + DY[d];
                        
                        // Check if this open path leads to an unvisited cell
                        if (maze.inBounds(nx, ny) && maze.visitCount[nx][ny] == 0) {
                            // We found an opening that leads to an unknown area!
                            // The maze is NOT fully closed yet.
                            return false; 
                        }
                    }
                }
            }
        }
    }
    
    // If we checked all visited cells and every single opening was either 
    // a physical wall or led to another visited cell... the maze is closed!
    return true;
}
// =====================================================================
//  B2: AGGRESSIVE HYBRID TRÉMAUX CHOOSER (BEST FOR COMPETITIONS)
// =====================================================================
//
// 1. Always follow Trémaux hierarchy:
//      mark=0  → explore new
//      mark=1  → return if no 0
//      mark=2  → only if forced
//
// 2. Inside ANY mark group, choose direction with SMALLEST distGoal
//    (aggressive goal bias — speeds exploration dramatically)
//
// =====================================================================

// ------------------------------------------------------------
// LEAF-FIRST FRONTIER + WAVEFRONT + ORIGIN AVOIDANCE TRÉMAUX
// DOMINANT RULE: If any unvisited neighbor has gain==1, go there first
// O(N) per step
// ------------------------------------------------------------
// ------------------------------------------------------------
// BFS + TRÉMAUX EXPLORATION
// 1. Seed BFS from all unvisited cells.
// 2. Obey Trémaux edge marks strictly (0 > 1 > 2).
// 3. Use BFS gradient to break ties and find nearest unknown.
// ------------------------------------------------------------
// =====================================================================
// FRONTIER BFS + TRÉMAUX CHOOSER (LOOP BREAKER)
// =====================================================================
inline bool chooseTremauxNext(const MazeMap &maze,
                              int distGoal[MAX_W][MAX_H], // unused
                              int x, int y, int ori,
                              IntCell &out)
{
    // 1. BFS TO NEAREST UNVISITED CELL (CELL Z)
    static int fDist[MAX_W][MAX_H];
    for (int i = 0; i < MAX_W; i++) {
        for (int j = 0; j < MAX_H; j++) {
            fDist[i][j] = INF_DIST;
        }
    }

    static int qx[MAX_W * MAX_H];
    static int qy[MAX_W * MAX_H];
    int qh = 0, qt = 0;

    // Seed BFS with ALL unvisited cells
    for (int i = 0; i < MAX_W; i++) {
        for (int j = 0; j < MAX_H; j++) {
            if (maze.visitCount[i][j] == 0) {
                fDist[i][j] = 0;
                qx[qh] = i;
                qy[qh] = j;
                qh++;
            }
        }
    }

    // Run BFS backwards from unvisited cells
    while (qt < qh) {
        int cx = qx[qt];
        int cy = qy[qt];
        qt++;
        int base = fDist[cx][cy];

        for (int d = 0; d < 4; d++) {
            int nx = cx + DX[d];
            int ny = cy + DY[d];
            if (maze.inBounds(nx, ny)) {
                int od = (d + 2) & 3; // Check the wall looking back from nx to cx
                if (!maze.isBlocked(nx, ny, od)) {
                    if (fDist[nx][ny] > base + 1) {
                        fDist[nx][ny] = base + 1;
                        qx[qh] = nx;
                        qy[qh] = ny;
                        qh++;
                    }
                }
            }
        }
    }

    int bestD = -1;
    int bestDist = INF_DIST;
    int bestTurnPenalty = 255;
    uint8_t bestMark = 255;

    // 2. Follow the BFS gradient directly to Cell Z
    for (int d = 0; d < 4; d++) {
        if (maze.isBlocked(x, y, d)) continue;
        int nx = x + DX[d];
        int ny = y + DY[d];
        if (!maze.inBounds(nx, ny)) continue;

        int d_to_unvisited = fDist[nx][ny];
        int relDir = (d - ori) & 3;
        int turnPenalty = (relDir == 0) ? 0 : (relDir == 2) ? 2 : 1;

        if (d_to_unvisited < INF_DIST) {
            if (d_to_unvisited < bestDist || 
               (d_to_unvisited == bestDist && turnPenalty < bestTurnPenalty)) {
                bestDist = d_to_unvisited;
                bestTurnPenalty = turnPenalty;
                bestD = d;
            }
        }
    }

    // If BFS found a path to an unvisited cell, override Trémaux and go there!
    if (bestD != -1) {
        out.x = x + DX[bestD];
        out.y = y + DY[bestD];
        out.dir = bestD;
        return true;
    }

    // 3. FALLBACK TO PURE TRÉMAUX (If all accessible cells are visited)
    bestTurnPenalty = 255;
    bestD = -1;

    for (int d = 0; d < 4; d++) {
        if (maze.isBlocked(x, y, d)) continue;
        int nx = x + DX[d];
        int ny = y + DY[d];
        if (!maze.inBounds(nx, ny)) continue;

        uint8_t m = maze.edgeMark(x, y, d);
        int relDir = (d - ori) & 3;
        int turnPenalty = (relDir == 0) ? 0 : (relDir == 2) ? 2 : 1;

        if (m < bestMark || (m == bestMark && turnPenalty < bestTurnPenalty)) {
            bestMark = m;
            bestTurnPenalty = turnPenalty;
            bestD = d;
        }
    }

    if (bestD == -1) return false;

    out.x = x + DX[bestD];
    out.y = y + DY[bestD];
    out.dir = bestD;
    
    return true;
}
// =====================================================================
// SHORTEST PATH ON KNOWN MAP (USED IF DFS FAILS)
// =====================================================================
inline bool shortestPathOnKnownMap(const MazeMap &maze,
                                   const int goals[][2],
                                   int goalCount,
                                   Path &out)
{
    out.len = 0;
    static int dist[MAX_W][MAX_H];
    maze.computeDistancesTo(goals, goalCount, dist);

    if (dist[0][0] >= INF_DIST) return false;

    int x = 0, y = 0;

    while (out.len < MAX_W * MAX_H) {
        out.cells[out.len++] = {x, y, -1};
        if (dist[x][y] == 0) break;

        int bestD = dist[x][y];
        int bx = x, by = y;

        for (int d = 0; d < 4; d++) {
            if (!maze.isBlocked(x, y, d)) {
                int nx = x + DX[d], ny = y + DY[d];
                if (maze.inBounds(nx, ny)) {
                    if (dist[nx][ny] < bestD) {
                        bestD = dist[nx][ny];
                        bx = nx; by = ny;
                    }
                }
            }
        }

        if (bestD >= dist[x][y]) break;
        x = bx; y = by;
    }

    return (out.len > 1);
}

// =====================================================================
// CLEAN DFS OPTIMIZER (no contradictions with hybrid exploration)
// =====================================================================

inline bool dfsOptimizeOnVisited(const MazeMap &maze,
                                 const int goals[][2],
                                 int goalCount,
                                 int distGoal[MAX_W][MAX_H],
                                 int sx, int sy,
                                 Path &best)
{
    best.len = 0;
    if (!maze.inBounds(sx, sy)) return false;
    if (maze.visitCount[sx][sy] == 0) return false;

    static bool isGoal[MAX_W][MAX_H];

    for (int i = 0; i < MAX_W; i++)
        for (int j = 0; j < MAX_H; j++)
            dfsVisited[i][j] = isGoal[i][j] = false;

    for (int g = 0; g < goalCount; g++) {
        int gx = goals[g][0], gy = goals[g][1];
        if (maze.inBounds(gx, gy))
            isGoal[gx][gy] = true;
    }

    float bestTime = 1e30f;
    int bestFwds = 1000000000;
    long nodes = 0;
    const long maxNodes = 200000;

    int sp = 0;
    dfsStack[0] = { sx, sy, 0, 0.0f, 0, 0, 0 };

    dfsVisited[sx][sy] = true;
    dfsCurPath.len = 1;
    dfsCurPath.cells[0] = { sx, sy, -1 };

    while (sp >= 0) {

        if ((nodes & 63) == 0) delay(0);

        if (++nodes > maxNodes) break;

        DFSState &st = dfsStack[sp];
        int x = st.x, y = st.y, ori = st.ori;

        // goal found
        if (isGoal[x][y]) {
            if (st.curT < bestTime ||
               (fabs(st.curT - bestTime) < 1e-6f && st.fwds < bestFwds))
            {
                bestTime = st.curT;
                bestFwds = st.fwds;
                best.len = dfsCurPath.len;
                for (int i = 0; i < dfsCurPath.len; i++)
                    best.cells[i] = dfsCurPath.cells[i];
            }
            dfsVisited[x][y] = false;
            sp--;
            dfsCurPath.len--;
            continue;
        }

        // generate neighbors
        struct NB {
            float cost;
            int turns;
            int d;
            int nx, ny;
            int h;
        } neigh[4];

        int N = 0;
        for (int d = 0; d < 4; d++) {
            if (maze.isBlocked(x, y, d)) continue;

            int nx = x + DX[d], ny = y + DY[d];
            if (!maze.inBounds(nx, ny)) continue;
            if (maze.visitCount[nx][ny] == 0) continue;
            if (dfsVisited[nx][ny]) continue;

            int diff = (d - ori) & 3;
            int tcount = (diff == 0 ? 0 : (diff == 2 ? 2 : 1));
            float tcost = tcount * 0.45f;

            int hh = distGoal[nx][ny];
            if (hh >= INF_DIST)
                hh = abs(nx - goals[0][0]) + abs(ny - goals[0][1]);

            neigh[N++] = { tcost, tcount, d, nx, ny, hh };
        }

        // sort by cost then heuristic
        for (int i = 0; i < N; i++)
            for (int j = i + 1; j < N; j++)
                if (neigh[j].cost < neigh[i].cost ||
                   (fabs(neigh[j].cost - neigh[i].cost) < 1e-6f &&
                    neigh[j].h < neigh[i].h))
                {
                    NB t = neigh[i];
                    neigh[i] = neigh[j];
                    neigh[j] = t;
                }

        bool pushed = false;
        while (st.childIndex < N) {
            NB &nb = neigh[st.childIndex++];

            float newT = st.curT + nb.cost + 1.0f;
            if (newT >= bestTime) continue;

            if (sp + 1 >= MAX_W * MAX_H) break;

            DFSState &child = dfsStack[++sp];
            child = { nb.nx, nb.ny, nb.d, newT,
                      st.fwds + 1,
                      st.turns + nb.turns,
                      0 };

            dfsVisited[nb.nx][nb.ny] = true;
            dfsCurPath.cells[dfsCurPath.len++] = { nb.nx, nb.ny, nb.d };

            pushed = true;
            break;
        }

        if (pushed) continue;

        dfsVisited[x][y] = false;
        sp--;
        dfsCurPath.len--;
    }

    return (best.len > 1);
}