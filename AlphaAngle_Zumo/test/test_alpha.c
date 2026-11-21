// Host test with the example of the challenge handout.
//   gcc -I.. test_alpha.c ../alpha.c -lm -o test_alpha && ./test_alpha
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "alpha.h"

int main(void) {
    // Handout figure: five radii, expected alpha = 30.0 deg.
    const float r[5] = { 0.1082f, 0.1132f, 0.1200f, 0.1282f, 0.1375f };
    float a = alpha_from_radii(r, 0.015f);
    printf("handout radii -> alpha = %.2f deg (expected 30.0)\n", a);
    assert(fabsf(a - 30.0f) < 0.3f);

    // Mirror: S1 farther than S5 gives the negative angle.
    const float m[5] = { r[4], r[3], r[2], r[1], r[0] };
    assert(fabsf(alpha_from_radii(m, 0.015f) + 30.0f) < 0.3f);

    // Sweep: build exact radii for many angles and get the angle back.
    for (int deg = -80; deg <= 80; deg += 10) {
        float s_a = sinf(deg * 3.14159265f / 180.0f), rr[5];
        for (int i = 0; i < 5; i++) {
            float s = (i - 2) * 0.015f;
            rr[i] = sqrtf(0.12f * 0.12f + 2.0f * s * 0.12f * s_a + s * s);
        }
        assert(fabsf(alpha_from_radii(rr, 0.015f) - deg) < 0.1f);
    }

    const gray_cal_t cal[] = { { 100, 0.05f }, { 300, 0.15f } };
    assert(fabsf(gray_to_radius(200, cal, 2) - 0.10f) < 1e-6f);
    assert(gray_to_radius(50, cal, 2) == 0.05f && gray_to_radius(900, cal, 2) == 0.15f);
    printf("all alpha checks passed\n");
    return 0;
}
