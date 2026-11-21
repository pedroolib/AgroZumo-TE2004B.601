#pragma once

#include <stdbool.h>
#include <stdint.h>

// One waypoint measured with the local positioning camera.
typedef struct {
    float x, y;        // position in m, camera frame
    float angle_deg;   // raw camera angle recorded at the waypoint (kept for the report)
    bool turn;         // true: the section from this waypoint to the next one is a turn (arc)
} waypoint_t;

// Path and controller tuning. Speeds are motor units (-6000..6000).
#define PATH_STEP_M     0.01f   // distance between two path points
#define PATH_MAX_PTS    2000    // 20 m of path
#define ROUTE_MAX       32
#define V_STRAIGHT      3500    // speed on the straights
#define V_TURN          1700    // speed on the turns
#define V_START         900     // speed at the first path point
#define ACCEL_PER_PT    60      // speed gain per cm of path
#define DECEL_PER_PT    35      // speed loss per cm of path before a turn
#define V_PIVOT         900     // spin speed when the target is far to one side
#define PIVOT_DEG       70.0f   // spin in place above this heading error
#define LOOKAHEAD_PTS   9       // pure pursuit look-ahead, in path points (cm)
#define SEARCH_PTS      25      // how far ahead to look for the closest path point
#define FINISH_PTS      2       // stop this many points before the final waypoint
#define LOST_M          0.10f   // stop the lap if the tractor is this far from the path
#define HALF_TRACK_M    0.045f  // half the distance between the Zumo tracks
#define K_TURN          1.15f   // > 1 turns harder, to make up for track slip
#define LATENCY_S       0.15f   // camera pose age when it reaches the Zumo (frame_age_ms + CAN)
#define M_PER_UNIT      (0.5f / 6000.0f)   // track speed per motor unit, measure it (README)

typedef struct {
    int32_t left, right;   // motor commands
    int section;           // route index where the current section starts
    int reached;           // route index of the waypoint reached at this check, or -1
    bool done;             // true when the tractor is at the final waypoint
    bool lost;             // true when the tractor is more than LOST_M from the path
    float cross_track_m;   // distance from the tractor to the path
} follow_out_t;

// Builds the path through wp[route[0]], wp[route[1]], ... and resets the progress.
// Straight sections are lines. Turn sections are circle arcs that start tangent to
// the previous section, so the tractor stays on the track. The first section must be straight.
// Returns the number of path points, or -1 if the table is not valid.
int follow_init(const waypoint_t *wp, const int *route, int route_len);

// One check: camera pose in, motor commands out. heading is in rad, counterclockwise from +x.
void follow_step(float x, float y, float heading, follow_out_t *out);

// Path points, for the host test.
int follow_path(const float **x, const float **y);
