#include "Arduino.h"


#ifndef osc_h
#define osc_h

void over_serial_config_B5(config_B5_t * Cc, size_t Csize);

void print_raw(char * arr, size_t s);

uint16_t load_and_print(uint16_t start, uint16_t s); 

bool serialin_to_array(uint8_t * arr, uint16_t arr_size, uint16_t timeout);

void print_config_B5();

uint16_t load_array_of_eeprom(uint16_t start, uint8_t * arr, uint8_t s);

//uint16_t write_array_to_eeprom(uint16_t start, uint8_t * arr, uint8_t s);


#endif