// Waypoint table. Written by tools/lp_tool.py from camera measurements.
#pragma once
#include "follow.h"

// Camera angle (deg) -> heading (rad, counterclockwise from +x):
//   heading = ANGLE_SIGN * angle + ANGLE_OFFSET_DEG
// Fit on the straight sections, worst error 5.8 deg.
#define ANGLE_SIGN        1.0f
#define ANGLE_OFFSET_DEG  0.4f

// The turning center of the Zumo is this far in front of the marker (m).
// x and y below are turning-center positions, not marker positions.
#define MARKER_OFFSET_M   0.063f

static const waypoint_t WAYPOINTS[] = {
    //    x (m)    y (m)  angle  turn
    { -0.375f,  1.058f,   85.8f, true  },   // P1 turn entrance
    { -0.238f,  1.045f,  -87.9f, false },   // P2 turn exit
    { -0.236f,  0.328f,  -92.3f, true  },   // P3 turn entrance
    { -0.076f,  0.337f,   89.5f, false },   // P4 turn exit
    { -0.073f,  1.058f,   85.9f, true  },   // P5 turn entrance
    {  0.080f,  1.056f,  -87.0f, false },   // P6 turn exit
    {  0.074f,  0.336f,  -85.9f, true  },   // P7 turn entrance
    {  0.222f,  0.343f,   96.5f, false },   // P8 turn exit
    {  0.219f,  1.049f,   87.1f, true  },   // P9 turn entrance
    {  0.380f,  1.049f,  -88.8f, false },   // P10 turn exit
    {  0.376f,  0.139f,  -88.0f, true  },   // P11 turn entrance
    {  0.301f,  0.068f,  179.8f, false },   // P12 turn exit
    { -0.299f,  0.079f,  173.8f, true  },   // P13 turn entrance
    { -0.370f,  0.154f,   88.2f, false },   // P14 turn exit
};

// Waypoint order of one lap: start at P14, go around, stop at P14.
static const int ROUTE[] = { 13, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13 };
#define ROUTE_LEN ((int)(sizeof ROUTE / sizeof ROUTE[0]))
