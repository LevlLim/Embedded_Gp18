/*
 * app_bits.h - minimum-bit packing helpers (Change Requirement 1)
 * Buddy 4 (2500853) - IMU / terrain subsystem
 *
 * Rules this header exists to enforce:
 *   1. A field uses only the bits its valid range needs, at the resolution
 *      the requirement asks for (not the smallest number that "looks" ok).
 *   2. Values are clamped before packing. A value outside its range must
 *      saturate and be counted, never wrap (Risk R3).
 *   3. Packing happens at the boundary only. Working maths stays 32-bit.
 */
#ifndef APP_BITS_H
#define APP_BITS_H

#include <stdint.h>

/* Build a mask of n bits. n must be 1..32. */
#define BITMASK(n)              ((n) >= 32 ? 0xFFFFFFFFu : ((1u << (n)) - 1u))

/* Place/extract an unsigned field of width w at bit position p. */
#define BITS_PACK(val, p, w)    (((uint32_t)(val) & BITMASK(w)) << (p))
#define BITS_UNPACK(word, p, w) (((uint32_t)(word) >> (p)) & BITMASK(w))

/* Compile-time guard: a field must fit in its declared width. */
#define STATIC_ASSERT_FITS(max_val, width) \
    typedef char fits_##width##_##max_val[((max_val) <= BITMASK(width)) ? 1 : -1]

/*
 * Saturating conversion. Returns the clamped value and increments *clipped
 * when the input was out of range, so the test plan can report how often a
 * packed field would have overflowed.
 */
static inline uint32_t bits_clamp_u(int32_t v, int32_t lo, int32_t hi,
                                    uint16_t *clipped)
{
    if (v < lo) { if (clipped) (*clipped)++; return (uint32_t)lo; }
    if (v > hi) { if (clipped) (*clipped)++; return (uint32_t)hi; }
    return (uint32_t)v;
}

#endif /* APP_BITS_H */
