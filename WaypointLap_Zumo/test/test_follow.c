// Host test: simulates one lap with camera delay, noise and track slip.
// Build and run on the PC:
//   gcc -O2 -I.. test_follow.c ../follow.c -lm -o test_follow && ./test_follow
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "follow.h"
#include "waypoints.h"

#define DT          0.005    // simulation step, s
#define CAM_PERIOD  0.2      // the camera sends about 5 poses per second
#define CAM_DELAY   0.15     // pose age when it reaches the Zumo
#define SIM_M_PER_UNIT  (0.5 / 6000.0)   // assumed: full motor command = 0.5 m/s
#define HIST        ((int)(CAM_DELAY / DT) + 1)

static double noise(double amp) { return amp * (2.0 * rand() / RAND_MAX - 1.0); }

int main(void) {
    int n = follow_init(WAYPOINTS, ROUTE, ROUTE_LEN);
    assert(n > 0);
    const float *px, *py;
    follow_path(&px, &py);

    // Path check: no gap between neighbor points, so every line and arc joins the next one.
    for (int i = 1; i < n; i++) assert(hypot(px[i] - px[i - 1], py[i] - py[i - 1]) < 2.5 * PATH_STEP_M);

    // Lost check: a pose 20 cm from the path stops the lap.
    follow_out_t lost;
    follow_step(WAYPOINTS[ROUTE[0]].x + 0.2f, WAYPOINTS[ROUTE[0]].y, 0.0f, &lost);
    assert(lost.lost && lost.left == 0 && lost.right == 0);
    follow_init(WAYPOINTS, ROUTE, ROUTE_LEN);

    // Start 1 cm off the start waypoint, 8 degrees off the track direction.
    double x = WAYPOINTS[ROUTE[0]].x + 0.01, y = WAYPOINTS[ROUTE[0]].y, th = (90.0 + 8.0) * M_PI / 180.0;
    double hx[HIST], hy[HIST], hth[HIST];
    for (int i = 0; i < HIST; i++) { hx[i] = x; hy[i] = y; hth[i] = th; }

    int left = 0, right = 0, next = 1, h = 0;
    double max_err = 0.0, next_cam = 0.0;
    for (double t = 0.0; t < 120.0; t += DT) {
        if (t >= next_cam) {
            next_cam += CAM_PERIOD;
            int old = (h + 1) % HIST;   // oldest pose in the history = CAM_DELAY ago
            // The CAN frame carries whole mm and whole degrees.
            float mx = roundf((hx[old] + noise(0.003)) * 1000.0) / 1000.0f;
            float my = roundf((hy[old] + noise(0.003)) * 1000.0) / 1000.0f;
            // The camera angle follows the convention that waypoints.h describes.
            float cam = roundf((hth[old] * 180.0 / M_PI - ANGLE_OFFSET_DEG) / ANGLE_SIGN + noise(2.0));
            float heading = (ANGLE_SIGN * cam + ANGLE_OFFSET_DEG) * (float)M_PI / 180.0f;
            follow_out_t o;
            follow_step(mx, my, heading, &o);
            if (o.reached >= 0) {
                assert(o.reached == next);   // no waypoint skipped
                next++;
            }
            assert(!o.lost);
            if (o.done) {
                printf("Lap done in %.2f s (simulated), max cross-track error %.1f mm, %d waypoints\n",
                       t, max_err * 1000.0, next);
                assert(next == ROUTE_LEN);
                assert(max_err < 0.035);
                return 0;
            }
            left = o.left;
            right = o.right;
        }
        double vl = 0.95 * left * SIM_M_PER_UNIT, vr = right * SIM_M_PER_UNIT;   // left track slips 5 %
        x += (vl + vr) / 2.0 * cos(th) * DT;
        y += (vl + vr) / 2.0 * sin(th) * DT;
        th += (vr - vl) / (2.0 * HALF_TRACK_M) * DT;
        h = (h + 1) % HIST;
        hx[h] = x; hy[h] = y; hth[h] = th;

        double err = 1e9;
        for (int i = 0; i < n; i++) err = fmin(err, hypot(px[i] - x, py[i] - y));
        if (err > max_err) max_err = err;
    }
    printf("Lap not finished, max error %.1f mm, reached %d\n", max_err * 1000.0, next);
    return 1;
}
