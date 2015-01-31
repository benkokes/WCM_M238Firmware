//#include <config.h>

#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>

#include "nv.h"

nv_data_t nv;

const nv_data_t nv_eeprom __attribute__((section(".eeprom"))) = {
    .slot                   = 0,

    .power_cycles           = 0,
    .runtime                = 0,

    .sensor_period          = 20,
    .sample_time            = 1000

};

void nv_load_all(void) {
    eeprom_read_block(&nv, &nv_eeprom, sizeof(nv_data_t));
}

void nv_save_byte(const uint8_t *data) {
    uint8_t *address = data - (uint8_t*)&nv + (uint8_t*)&nv_eeprom;
    eeprom_write_byte(address, *data);
}

void nv_save_word(const uint16_t *data) {
    uint8_t *address = (uint8_t*)data - (uint8_t*)&nv + (uint8_t*)&nv_eeprom;
    eeprom_write_word((uint16_t*)address, *data);
}


