#pragma once
#include <Arduino.h>

// =========================================================
// MAZE MAP SYSTEM — OPTIMIZED FOR MICROMOUSE COMPETITION
// ---------------------------------------------------------
// FEATURES:
//  - Perfect symmetric walls
//  - Perfect symmetric edge marks
//  - Visit count tracking
//  - Ultra-fast BFS Floodfill
//  - Robust reachable-set function
//  - Zero memory waste
// ---------------------------------------------------------
// Works perfectly with Hybrid Trémaux B2 and DFS optimizer.
// =========================================================

#define MAX_W 26
#define MAX_H 26

#define INF_DIST 1000000000

// Global maze dimensions (set by main code)
extern int gMazeW;
extern int gMazeH;

extern const int START_X;
extern const int START_Y;
extern const int START_ORI;

// Direction vectors
static const int DX[4] = {0, 1, 0, -1};
static const int DY[4] = {1, 0, -1, 0};

inline char dirChar(int d) {
    static const char c[4] = {'n','e','s','w'};
    return c[d & 3];
}

struct CellNode {
    int x, y, dir;
};

struct Path {
    CellNode cells[MAX_W * MAX_H];
    int len;
};

// =========================================================
//  MAZE MAP CLASS
// =========================================================
class MazeMap {
public:

    // TRUE wall map (symmetric)
    bool walls[MAX_W][MAX_H][4];

    // Trémaux edge marks: 0 → unvisited, 1 → visited once, 2 → visited twice
    uint8_t marks[MAX_W][MAX_H][4];

    // Cell visit count (non-Trémaux, but useful for exploration/DFS)
    uint8_t visitCount[MAX_W][MAX_H];
    
    // ROOT FIX: Track true nodes to ignore phantom holes in intermediate cells
    bool isNode[MAX_W][MAX_H];

    MazeMap() { clear(); }

    // -----------------------------------------------------
    // CLEAR ALL STRUCTURES
    // -----------------------------------------------------
    void clear() {
        for (int i = 0; i < MAX_W; i++)
            for (int j = 0; j < MAX_H; j++) {
                visitCount[i][j] = 0;
                isNode[i][j] = false;
                for (int d = 0; d < 4; d++) {
                    walls[i][j][d] = false;
                    marks[i][j][d] = 0;
                }
            }
    }

    // -----------------------------------------------------
    // BOUND CHECK
    // -----------------------------------------------------
    bool inBounds(int x, int y) const {
        return (x >= 0 && x < gMazeW && y >= 0 && y < gMazeH);
    }

    // -----------------------------------------------------
    // QUERY WALL
    // -----------------------------------------------------
    bool isBlocked(int x, int y, int d) const {
        if (!inBounds(x, y)) return true;
        return walls[x][y][d];
    }

    // -----------------------------------------------------
    // GET MARK
    // -----------------------------------------------------
    uint8_t edgeMark(int x, int y, int d) const {
        if (!inBounds(x, y)) return 2;
        return marks[x][y][d];
    }

    // -----------------------------------------------------
    // SET WALL (symmetric)
    // -----------------------------------------------------
    void setWall(int x, int y, int d) {
        if (!inBounds(x, y)) return;

        walls[x][y][d] = true;

        int nx = x + DX[d];
        int ny = y + DY[d];

        if (inBounds(nx, ny)) {
            int od = (d + 2) & 3;
            walls[nx][ny][od] = true;
        }
    }

    // -----------------------------------------------------
    // MARK TREMUX EDGE TRAVERSAL (symmetric)
    // -----------------------------------------------------
    void markEdge(int x, int y, int d) {
        if (!inBounds(x, y)) return;

        if (marks[x][y][d] < 2)
            marks[x][y][d]++;

        int nx = x + DX[d];
        int ny = y + DY[d];

        if (inBounds(nx, ny)) {
            int od = (d + 2) & 3;
            if (marks[nx][ny][od] < 2)
                marks[nx][ny][od]++;
        }
    }

    // -----------------------------------------------------
    // INCREMENT VISIT COUNT
    // -----------------------------------------------------
    uint8_t markVisit(int x, int y) {
        if (!inBounds(x, y)) return 0;
        visitCount[x][y]++;
        return visitCount[x][y];
    }

// =========================================================
//  ULTRA-FAST FLOODFILL BFS TO GOALS (FULL MAZE)
// =========================================================
void computeDistancesTo(const int goals[][2], int goalCount,
                        int dist[MAX_W][MAX_H]) const
{
    // initialize
    for (int i = 0; i < gMazeW; i++)
        for (int j = 0; j < gMazeH; j++)
            dist[i][j] = INF_DIST;

    // BFS queue (static to prevent stack overflow on large mazes)
    static int qx[MAX_W * MAX_H];
    static int qy[MAX_W * MAX_H];
    int qh = 0, qt = 0;

    // push all goals (multi-goal support)
    for (int g = 0; g < goalCount; g++) {
        int gx = goals[g][0];
        int gy = goals[g][1];
        if (inBounds(gx, gy)) {
            dist[gx][gy] = 0;
            qx[qh] = gx;
            qy[qh] = gy;
            qh++;
        }
    }

    // BFS
    while (qt < qh) {

        if ((qt & 15) == 0) delay(0);
        int x = qx[qt];
        int y = qy[qt];
        qt++;

        int base = dist[x][y];

        for (int d = 0; d < 4; d++) {
            if (isBlocked(x, y, d)) continue;

            int nx = x + DX[d];
            int ny = y + DY[d];

            if (!inBounds(nx, ny)) continue;

            if (dist[nx][ny] > base + 1) {
                dist[nx][ny] = base + 1;

                if (qh < MAX_W * MAX_H) {
                    qx[qh] = nx;
                    qy[qh] = ny;
                    qh++;
                }
            }
        }
    }
}

    // =========================================================
// FLOODFILL ON VISITED CELLS ONLY (FAST RUN SAFE)
// =========================================================
void computeDistancesOnVisitedTo(const int goals[][2], int goalCount,
                                 int dist[MAX_W][MAX_H]) const
{
    // initialize
    for (int i = 0; i < gMazeW; i++)
        for (int j = 0; j < gMazeH; j++)
            dist[i][j] = INF_DIST;

    static int qx[MAX_W * MAX_H];
    static int qy[MAX_W * MAX_H];
    int qh = 0, qt = 0;

    // push only goals that are VISITED
    for (int g = 0; g < goalCount; g++) {
        int gx = goals[g][0];
        int gy = goals[g][1];

        if (inBounds(gx, gy) && visitCount[gx][gy] > 0) {
            dist[gx][gy] = 0;
            qx[qh] = gx;
            qy[qh] = gy;
            qh++;
        }
    }

    // BFS
    while (qt < qh) {

        if ((qt & 15) == 0) delay(0);
        int x = qx[qt];
        int y = qy[qt];
        qt++;

        int base = dist[x][y];

        for (int d = 0; d < 4; d++) {
            if (isBlocked(x, y, d)) continue;

            int nx = x + DX[d];
            int ny = y + DY[d];

            if (!inBounds(nx, ny)) continue;

            // CRITICAL: do NOT enter unvisited cells
            if (visitCount[nx][ny] == 0) continue;

            if (dist[nx][ny] > base + 1) {
                dist[nx][ny] = base + 1;
                
                if (qh < MAX_W * MAX_H) {
                    qx[qh] = nx;
                    qy[qh] = ny;
                    qh++;
                }
            }
        }
    }
}


    // =========================================================
    // BFS REACHABLE SET — USED TO CHECK IF FINAL GOAL REACHABLE
    // =========================================================
    int reachableCountFrom(int sx, int sy, bool out[MAX_W][MAX_H]) const {

        for (int i = 0; i < gMazeW; i++)
            for (int j = 0; j < gMazeH; j++)
                out[i][j] = false;

        if (!inBounds(sx, sy)) return 0;

        static int qx[MAX_W * MAX_H];
        static int qy[MAX_W * MAX_H];
        int qh = 0, qt = 0;

        out[sx][sy] = true;
        qx[qh] = sx;
        qy[qh] = sy;
        qh++;

        while (qt < qh) {
            int x = qx[qt];
            int y = qy[qt];
            qt++;

            for (int d = 0; d < 4; d++) {
                if (isBlocked(x, y, d)) continue;

                int nx = x + DX[d];
                int ny = y + DY[d];

                if (inBounds(nx, ny) && !out[nx][ny]) {
                    out[nx][ny] = true;

                    if (qh < MAX_W * MAX_H) {
                        qx[qh] = nx;
                        qy[qh] = ny;
                        qh++;
                    }
                }
            }
        }

        int count = 0;
        for (int i = 0; i < gMazeW; i++)
            for (int j = 0; j < gMazeH; j++)
                if (out[i][j]) count++;

        return count;
    }
};
