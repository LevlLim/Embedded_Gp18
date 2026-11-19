/*
 * barcode_edge.h
 *
 * Subsystem 3 -- barcode edge detector (dark / light from the raw IR
 * reading).  Its own header so subsystem3.h, which holds each team's tuned
 * settings, needs no change.  Pure: host-tested on blurred sheets.
 */

#ifndef BARCODE_EDGE_H
#define BARCODE_EDGE_H

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    bool     b_dark;
    uint16_t extreme;   /* highest while dark, lowest while light */
    uint16_t floor;     /* lowest reading of the last gap         */
    uint16_t min_dark;  /* a bar must rise above this (sensor and */
                        /* paper dependent: S3_BC_MIN_DARK)       */
    uint16_t prev1;     /* last two raw samples, for the average  */
    uint16_t prev2;
} s3_bc_edge_t;

/* One raw sample -> true while over black.  thr is used only when
   S3_BC_DROP_PCT is 0 (fixed threshold mode). */
bool s3_bc_edge_step(s3_bc_edge_t *p_edge, uint16_t value, uint16_t thr);

#endif /* BARCODE_EDGE_H */

/* end of file */