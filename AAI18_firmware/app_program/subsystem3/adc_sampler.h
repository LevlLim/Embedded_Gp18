/*
 * adc_sampler.h
 *
 * Group AAI18 -- sole owner of the RP2040 ADC and its input mux.
 *
 * OWNERSHIP: Buddy 3.  Design Review V2 section 3 and section 9.
 *
 * WHY THIS MODULE EXISTS
 * ----------------------
 * The RP2040 has ONE ADC behind a shared mux.  Two independent claimants
 * switching the mux corrupt each other's conversions, silently.  This
 * module owns the peripheral, free-runs it in round-robin across
 * ADC0/ADC1/ADC2, and publishes timestamped samples.  Line Sensor and
 * Barcode Decoder read those; neither touches an ADC register.
 *
 * SAMPLE RATE (why it is low)
 * ---------------------------
 * The FIFO is only 4 entries deep and is drained by polling from a 1 ms
 * cyclic handler.  The original 32 kS/s rate filled it in 125 us, so EVERY
 * poll found an overrun and threw the samples away -- no sample was ever
 * stored.  The rate is now 1.5 kS/s in total (500 S/s per channel), which
 * leaves 2.7 ms before the FIFO can overflow.  Per-channel resolution is
 * 2 ms, i.e. 0.6 mm of travel at 0.3 m/s -- far finer than any bar.  If
 * more is ever needed, the design's DMA ring (section 3) is the next step.
 *
 * MINIMUM-BIT STORAGE (CR1)
 * -------------------------
 * Each stored sample is ONE 32-bit word: the 12-bit conversion in bits
 * 11:0 and the low 20 bits of its microsecond timestamp in bits 31:12.
 * The 20-bit stamp wraps every 1.05 s, far longer than the 512 ms the ring
 * spans, and the full time is rebuilt on read.  This halves the ring from
 * 2048 to 1024 bytes -- the largest single RAM saving in the firmware --
 * and, because a sample is now a single aligned word, a reader on another
 * task can never see a value from one sample with the time of another.
 */

#ifndef ADC_SAMPLER_H
#define ADC_SAMPLER_H

#include "../app.h"
#include <stdint.h>
#include <stdbool.h>

#define ADC_CH_LINE_L       (0u)    /* GP26 */
#define ADC_CH_LINE_C       (1u)    /* GP27 */
#define ADC_CH_BARCODE      (2u)    /* GP28 */
#define ADC_CH_COUNT        (3u)

/* Total conversion rate across the three channels, and the resulting
   spacing between consecutive FIFO entries. */
#define ADC_TOTAL_RATE_SPS  (1500u)
#define ADC_SAMPLE_GAP_US   (1000000u / ADC_TOTAL_RATE_SPS)    /* 666 */

/* Ring of recent barcode samples: 256 x 2 ms = 512 ms of history, long
   enough for the widest symbol at the speeds assumption A1 allows. */
#define ADC_RING_LEN        (256u)

/* The FIFO is drained by a 1 ms cyclic handler created in init. */
#define ADC_POLL_PERIOD_MS  (1u)

/* Unpacked sample handed to consumers. */
typedef struct
{
    uint16_t value;         /* 12-bit right-aligned conversion result */
    uint32_t timestamp_us;  /* free-running microsecond timer        */
} adc_sample_t;

/* Configure the ADC, start free-running round-robin conversion, and start
   the 1 ms cyclic handler that drains the FIFO.  Call once from phase 1. */
ER adc_sampler_init(void);

/* Latest value for one channel.  Returns false if no sample yet. */
bool adc_sampler_read(uint8_t channel, adc_sample_t *p_out);

/* Copy up to max_out barcode samples newer than after_us into p_out,
   oldest first.  Returns how many were copied.  Called by the barcode
   task (the ring's single reader). */
uint8_t adc_sampler_since(uint32_t after_us, adc_sample_t *p_out,
                          uint8_t max_out);

/* Count of FIFO overrun events since init (target: zero). */
uint32_t adc_sampler_overruns(void);

/* Register dump and counters for bring-up (also printed when line
   calibration fails). */
void adc_sampler_selftest(void);

#endif /* ADC_SAMPLER_H */

/* end of file */
