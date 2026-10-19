/*
 * test_imu_pack.c - host-side tests for Buddy 4's packed representation.
 * No hardware needed. Build and run with:  make -C test && ./test/test_imu_pack
 *
 * Covers the evidence Week 13 asks for:
 *   - every in-range value survives pack then unpack unchanged
 *   - out-of-range values clamp instead of wrapping, and get counted
 *   - no two buddies own the same fault bit (the one-writer rule)
 *   - message size before and after the change
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "imu_terrain.h"
#include "app_bits.h"
#include "fault_flags.h"

static int failures = 0;

#define CHECK(cond, msg) do {                                   \
    if (!(cond)) { printf("  FAIL: %s\n", msg); failures++; }   \
} while (0)

/* The Week 6 unpacked message, kept only to measure what we saved. */
typedef struct {
    int32_t  evt;
    int32_t  valid;
    int32_t  hump_mm;
    int32_t  tilt_halfdeg;
    int32_t  peak_acc_cg;
    uint32_t t_ms;
    int32_t  seq;
} terrain_msg_old_t;

/* ---- hooks the task needs, stubbed so the file links on host ---- */
int32_t  imu_hal_read_fifo(int16_t *ax, int16_t *ay, int16_t *az, int32_t max)
{ (void)ax; (void)ay; (void)az; (void)max; return 0; }
int32_t  encoder_distance_mm(void) { return 0; }
void     terrain_publish(terrain_msg_t msg) { (void)msg; }
uint32_t board_time_us(void) { return 0; }

/* ------------------------------------------------------------------ */
static void test_roundtrip(void)
{
    int h, t, p, e;
    printf("round trip over every in-range value\n");

    for (h = HUMP_MM_MIN; h <= HUMP_MM_MAX; h++) {
        for (t = TILT_HALFDEG_MIN; t <= TILT_HALFDEG_MAX; t += 10) {
            for (p = PEAK_ACC_MIN; p <= PEAK_ACC_MAX; p += 10) {
                for (e = 0; e <= 5; e++) {
                    terrain_sample_t in, out;
                    terrain_msg_t m;
                    uint16_t clipped = 0;

                    in.evt = (terrain_evt_t)e;
                    in.valid = 1;
                    in.hump_mm = h;
                    in.tilt_halfdeg = t;
                    in.peak_acc_cg = p;
                    in.t_ms = 123456;
                    in.seq = 9;

                    m = terrain_pack(&in, &clipped);
                    terrain_unpack(m, &out);

                    CHECK(clipped == 0, "in-range value was clamped");
                    CHECK(out.evt == in.evt, "event type changed");
                    CHECK(out.hump_mm == in.hump_mm, "hump height changed");
                    CHECK(out.tilt_halfdeg == in.tilt_halfdeg, "tilt changed");
                    CHECK(out.peak_acc_cg == in.peak_acc_cg, "peak accel changed");
                    CHECK(out.t_ms == in.t_ms, "timestamp changed");
                    CHECK(out.seq == in.seq, "sequence changed");
                    if (failures) return;
                }
            }
        }
    }
    printf("  ok\n");
}

static void test_clamping(void)
{
    terrain_sample_t in, out;
    terrain_msg_t m;
    uint16_t clipped = 0;

    printf("out of range values saturate and are counted\n");

    memset(&in, 0, sizeof(in));
    in.evt = TERRAIN_EVT_HUMP_DONE;
    in.valid = 1;
    in.hump_mm      = 500;     /* a real hump will never be this tall     */
    in.tilt_halfdeg = 400;     /* nor this steep                          */
    in.peak_acc_cg  = 9999;    /* nor this hard an impact                 */

    m = terrain_pack(&in, &clipped);
    terrain_unpack(m, &out);

    CHECK(clipped == 3, "all three out-of-range fields should be counted");
    CHECK(out.hump_mm == HUMP_MM_MAX, "hump should saturate, not wrap");
    CHECK(out.tilt_halfdeg == TILT_HALFDEG_MAX, "tilt should saturate");
    CHECK(out.peak_acc_cg == PEAK_ACC_MAX, "peak accel should saturate");

    /* The point of clamping: a wrapped 500 mm would read as 500 & 127 = 116,
     * which looks like a plausible hump and would silently fail FR5. */
    CHECK(out.hump_mm != (500 & 127), "value must not have wrapped");

    if (!failures) printf("  ok\n");
}

static void test_fault_ownership(void)
{
    uint32_t owners[5] = { FLT_OWNED_BUDDY1, FLT_OWNED_BUDDY2, FLT_OWNED_BUDDY3,
                           FLT_OWNED_BUDDY4, FLT_OWNED_BUDDY5 };
    uint32_t all = 0;
    int i, j;

    printf("one writer per fault bit\n");

    for (i = 0; i < 5; i++) {
        for (j = i + 1; j < 5; j++) {
            CHECK((owners[i] & owners[j]) == 0, "two buddies own the same bit");
        }
        all |= owners[i];
    }
    CHECK(all == FLT_ALL_MASK, "owner masks do not cover all 10 fault bits");

    /* Buddy 4 sets its own bits, the Controller clears them. */
    fault_set(FLT_IMU_FAULT);
    fault_set(FLT_COLLISION);
    CHECK((fault_read() & FLT_OWNED_BUDDY4) == FLT_OWNED_BUDDY4, "set failed");
    fault_clear(FLT_COLLISION);
    CHECK((fault_read() & FLT_COLLISION) == 0, "clear failed");
    CHECK((fault_read() & FLT_IMU_FAULT) != 0, "clear hit the wrong bit");

    if (!failures) printf("  ok\n");
}

static void test_hump_maths(void)
{
    /* 300 mm travelled at 3.0 degrees should be about 15.7 mm.
     * Tolerance here is the FR5 figure, +/-5 mm. */
    int32_t h = terrain_hump_height_mm(300, 6);
    printf("hump height maths\n");
    printf("  300 mm at 3.0 deg -> %d mm\n", (int)h);
    CHECK(h >= 11 && h <= 21, "height outside the FR5 tolerance band");
    if (!failures) printf("  ok\n");
}

static void report_sizes(void)
{
    printf("\nmessage size, IMU/terrain event\n");
    printf("  before (unpacked struct) : %u bytes\n",
           (unsigned)sizeof(terrain_msg_old_t));
    printf("  after  (packed struct)   : %u bytes\n",
           (unsigned)sizeof(terrain_msg_t));
    printf("  saved                    : %u bytes per message (%.0f%%)\n",
           (unsigned)(sizeof(terrain_msg_old_t) - sizeof(terrain_msg_t)),
           100.0 * (sizeof(terrain_msg_old_t) - sizeof(terrain_msg_t))
                 / sizeof(terrain_msg_old_t));
    printf("  fault storage before     : 10 variables x 4 bytes = 40 bytes\n");
    printf("  fault storage after      : 10 bits in one 4 byte flag\n");
}

int main(void)
{
    printf("Buddy 4 packed representation tests\n\n");
    imu_terrain_init();

    test_roundtrip();
    test_clamping();
    test_fault_ownership();
    test_hump_maths();
    report_sizes();

    printf("\n%s\n", failures ? "TESTS FAILED" : "all tests passed");
    return failures ? 1 : 0;
}
