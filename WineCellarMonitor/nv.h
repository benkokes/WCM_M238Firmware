#ifndef __nv_h_
#define __nv_h_

#include <stdint.h>

typedef struct {
    uint8_t slot;

    uint16_t power_cycles;
    uint32_t runtime;

    uint16_t sensor_period;
    uint16_t sample_time;
} nv_data_t;

extern nv_data_t nv;

void nv_load_all(void);
void nv_save_byte(const uint8_t *data);
void nv_save_word(const uint16_t *data);

#endif // __nv_h_

