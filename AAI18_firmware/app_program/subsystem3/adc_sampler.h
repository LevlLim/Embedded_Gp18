/**
 * @file adc_sampler.h
 * @brief Sole ADC owner: GP28 each 1 ms, GP26/27 each 5 ms.
 */
#ifndef ADC_SAMPLER_H
#define ADC_SAMPLER_H
#include "../app.h"
#include "buddy3_config.h"

#define ADC_CH_LINE_L       (0U)
#define ADC_CH_LINE_R       (1U)
/* Legacy name retained for existing telemetry and the team's shared pin map.
 * GP27 is now mounted on the right edge, not over the centre of the line. */
#define ADC_CH_LINE_C       ADC_CH_LINE_R
#define ADC_CH_BARCODE      (2U)
#define ADC_CH_COUNT        (3U)
#define ADC_POLL_PERIOD_MS  (1U)

typedef struct
{
    uint16_t value;
    uint32_t timestamp_us;
} adc_sample_t;

ER adc_sampler_init(void);
/** False for missing/failed/stale acquisition, including long stalls. */
bool adc_sampler_read(uint8_t channel, adc_sample_t * p_out);
uint8_t adc_sampler_since(uint32_t after_us, adc_sample_t * p_out,
                         uint8_t capacity);
uint32_t adc_sampler_now_us(void);
/** Counts bounded conversion failures; acquisition no longer uses FIFO. */
uint32_t adc_sampler_overruns(void);
uint16_t adc_sampler_max_us(void);
void adc_sampler_selftest(void);
#endif /* ADC_SAMPLER_H */
