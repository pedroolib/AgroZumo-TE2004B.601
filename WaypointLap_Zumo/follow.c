#include <math.h>
#include "follow.h"

#define PI_F 3.14159265f

static float px[PATH_MAX_PTS], py[PATH_MAX_PTS];
static int16_t pv[PATH_MAX_PTS];     // speed profile
static uint8_t psec[PATH_MAX_PTS];   // route index of the section that holds the point
static int wp_pt[ROUTE_MAX];         // path index of each route waypoint
static int n_pts, n_route, near_pt, next_wp;
static float last_l, last_r;        // last motor command, for the delay compensation

static float wrap(float a) {
    while (a > PI_F) a -= 2.0f * PI_F;
    while (a < -PI_F) a += 2.0f * PI_F;
    return a;
}

static bool add_pt(float x, float y, int sec, int16_t v) {
    if (n_pts >= PATH_MAX_PTS) return false;
    px[n_pts] = x;
    py[n_pts] = y;
    psec[n_pts] = (uint8_t)sec;
    pv[n_pts] = v;
    n_pts++;
    return true;
}

int follow_init(const waypoint_t *wp, const int *route, int route_len) {
    if (route_len < 2 || route_len > ROUTE_MAX || wp[route[0]].turn) return -1;
    n_pts = 0;
    n_route = route_len;
    near_pt = 0;
    next_wp = 1;
    last_l = last_r = 0.0f;

    float tx = 0.0f, ty = 0.0f;   // unit tangent at the end of the last section
    for (int k = 0; k < route_len - 1; k++) {
        const waypoint_t *a = &wp[route[k]];
        const waypoint_t *b = &wp[route[k + 1]];
        float dx = b->x - a->x, dy = b->y - a->y;
        float len = hypotf(dx, dy);
        if (len < PATH_STEP_M) return -1;
        wp_pt[k] = n_pts;

        // Turn: the circle through A and B that is tangent to the previous section at A.
        // Center C = A + r*n, n = unit normal toward the turn side, r = |AB|^2 / (2 AB.n).
        float side = (tx * dy - ty * dx) > 0.0f ? 1.0f : -1.0f;   // +1 left turn, -1 right turn
        float nx = -ty * side, ny = tx * side;
        float ab_n = dx * nx + dy * ny;
        if (a->turn && ab_n > 0.005f) {
            float r = (dx * dx + dy * dy) / (2.0f * ab_n);
            float cx = a->x + r * nx, cy = a->y + r * ny;
            float a0 = atan2f(a->y - cy, a->x - cx);
            float sweep = atan2f(b->y - cy, b->x - cx) - a0;
            if (side > 0) { while (sweep <= 0.0f) sweep += 2.0f * PI_F; }
            else          { while (sweep >= 0.0f) sweep -= 2.0f * PI_F; }
            int n = (int)(fabsf(sweep) * r / PATH_STEP_M);
            for (int i = 0; i < n; i++) {
                float t = a0 + sweep * (float)i / (float)n;
                if (!add_pt(cx + r * cosf(t), cy + r * sinf(t), k, V_TURN)) return -1;
            }
            float ux = (b->x - cx) / r, uy = (b->y - cy) / r;   // radial unit vector at B
            tx = -uy * side;
            ty = ux * side;
        } else {
            tx = dx / len;
            ty = dy / len;
            int n = (int)(len / PATH_STEP_M);
            for (int i = 0; i < n; i++) {
                float s = (float)i * PATH_STEP_M;
                if (!add_pt(a->x + tx * s, a->y + ty * s, k, V_STRAIGHT)) return -1;
            }
        }
    }
    const waypoint_t *last = &wp[route[route_len - 1]];
    if (!add_pt(last->x, last->y, route_len - 2, V_TURN)) return -1;
    wp_pt[route_len - 1] = n_pts - 1 - FINISH_PTS;   // the stop point counts as the final waypoint

    // Speed profile: brake before each turn and before the end, accelerate after.
    for (int i = n_pts - 2; i >= 0; i--)
        if (pv[i] > pv[i + 1] + DECEL_PER_PT) pv[i] = (int16_t)(pv[i + 1] + DECEL_PER_PT);
    pv[0] = V_START;
    for (int i = 1; i < n_pts; i++)
        if (pv[i] > pv[i - 1] + ACCEL_PER_PT) pv[i] = (int16_t)(pv[i - 1] + ACCEL_PER_PT);
    return n_pts;
}

void follow_step(float x, float y, float heading, follow_out_t *o) {
    // The pose is LATENCY_S old. Move it forward with the last motor command.
    float v = (last_l + last_r) / 2.0f * M_PER_UNIT;
    float w = (last_r - last_l) / (2.0f * HALF_TRACK_M) * M_PER_UNIT;
    x += v * LATENCY_S * cosf(heading + w * LATENCY_S / 2.0f);
    y += v * LATENCY_S * sinf(heading + w * LATENCY_S / 2.0f);
    heading += w * LATENCY_S;

    // Locate the tractor on the path. Search only a short window ahead, so the
    // progress never goes back and never jumps to a parallel lane.
    int end = near_pt + SEARCH_PTS;
    if (end > n_pts) end = n_pts;
    float best = 1e9f;
    for (int i = near_pt; i < end; i++) {
        float d = (px[i] - x) * (px[i] - x) + (py[i] - y) * (py[i] - y);
        if (d < best) { best = d; near_pt = i; }
    }
    o->cross_track_m = sqrtf(best);
    o->section = psec[near_pt];
    o->reached = -1;
    if (next_wp < n_route && near_pt >= wp_pt[next_wp]) o->reached = next_wp++;

    // Also stop once the tractor passes the final point, so a fast tractor does not overshoot it.
    float fx = px[n_pts - 1], fy = py[n_pts - 1];
    float ux = fx - px[n_pts - 2], uy = fy - py[n_pts - 2];   // direction of the path at the end
    bool passed = near_pt > n_pts - 1 - LOOKAHEAD_PTS && (fx - x) * ux + (fy - y) * uy <= 0.0f;
    o->done = near_pt >= n_pts - 1 - FINISH_PTS || passed;
    // Far from the path, the progress search can run ahead along the route, so stop instead.
    o->lost = o->cross_track_m > LOST_M;
    if (o->done || o->lost) {
        o->left = o->right = 0;
        last_l = last_r = 0.0f;
        return;
    }

    // Target ahead in the stored order, then pure pursuit: the arc through the
    // tractor and the target has curvature k = 2 sin(alpha) / distance.
    // Near the end, aim past the final point along the path, so the turn stays small.
    int t = near_pt + LOOKAHEAD_PTS;
    float tx = t < n_pts ? px[t] : fx + ux * (float)(t - n_pts + 1);
    float ty = t < n_pts ? py[t] : fy + uy * (float)(t - n_pts + 1);
    float dx = tx - x, dy = ty - y;
    float alpha = wrap(atan2f(dy, dx) - heading);   // > 0: target on the left
    float dist = fmaxf(hypotf(dx, dy), 0.02f);

    float l, r;
    if (fabsf(alpha) > PIVOT_DEG * PI_F / 180.0f) {
        float s = alpha > 0.0f ? 1.0f : -1.0f;
        l = -s * V_PIVOT;
        r = s * V_PIVOT;
    } else {
        float k = 2.0f * sinf(alpha) / dist;
        l = pv[near_pt] * (1.0f - k * HALF_TRACK_M * K_TURN);
        r = pv[near_pt] * (1.0f + k * HALF_TRACK_M * K_TURN);
    }
    // Keep the ratio between the tracks if one command is out of range.
    float m = fmaxf(fabsf(l), fabsf(r));
    if (m > 6000.0f) { l *= 6000.0f / m; r *= 6000.0f / m; }
    last_l = l;
    last_r = r;
    o->left = (int32_t)l;
    o->right = (int32_t)r;
}

int follow_path(const float **x, const float **y) {
    *x = px;
    *y = py;
    return n_pts;
}
