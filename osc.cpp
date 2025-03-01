
#include "Arduino.h"
#include <EEPROM.h>
#include "RPICTlib.h"
#include "osc.h"

void(* resetFunc) (void) = 0;

void over_serial_config_B5(config_B5_t * Cc, size_t Csize) {

  uint8_t * C_arr = (uint8_t *) Cc;
  EEPROM.get(0, *Cc);
  uint16_t L_extended = 2 * Cc->N_signal_node + 4 * Cc->N_power_node + 12 * Cc->N_3p_node + 2 * Cc->N_freq_node + 3 * Cc->N_channels;

  // If running for the first time. We set some defaults.
  if (Cc->VERSION != 0xB5) {
    Serial.println("# first run");
    Cc->KEY[0] = KEY0;
    Cc->KEY[1] = KEY1;
    Cc->KEY[2] = KEY2;
    Cc->KEY[3] = KEY3;
    Cc->VERSION = 0xB5;

    Cc->NODEID = 11;
    Cc->OUTPUT_RATE = 5000;

    Cc->N_signal_node = 0;
    Cc->N_power_node = 0;
    Cc->N_3p_node = 0;
    Cc->N_freq_node = 0;
    Cc->N_channels = 0;
    Cc->debug = 0;
    Cc->model[0] = 0;
    Cc->model[1] = 0;

    EEPROM.put(0, *Cc);

    resetFunc();
  }


  // Flush the input buffer first
  while (Serial.available()) Serial.read();

  // Print the config data with dynamic part
  Serial.print(F("#"));
  print_raw(C_arr, Csize);
  load_and_print(Csize, L_extended);
  Serial.println(F("#"));


  // Waiting for a new config
  bool sin_ok = serialin_to_array(C_arr, Csize, 1000);
  L_extended = 2 * Cc->N_signal_node + 4 * Cc->N_power_node + 12 * Cc->N_3p_node + 2 * Cc->N_freq_node + 3 * Cc->N_channels;

  // A config has been received from serial port
  if (sin_ok) {
    Serial.println("> Serial config fully received.");
    if ((C_arr[0] == KEY0) && (C_arr[1] == KEY1) && (C_arr[2] == KEY2) && (C_arr[3] == KEY3)) {

      Serial.println("> key ok. Writing config to eeprom");
      EEPROM.put(0, *Cc);

      uint16_t ii = 0;
      uint32_t xtimer = millis();
      while (ii < L_extended) {
        if (Serial.available()) {
          xtimer = millis();
          uint8_t bytein = Serial.read();
          //Serial.print(bytein);Serial.print(" ");

          EEPROM.write(Csize + ii, bytein);
          ii++;
        }
        if ((millis() - xtimer) > 3000) {
          Serial.println("> Timeout Arr.");
          break;
        }
      }

      Serial.println("> Reset.");
      delay(50);
      resetFunc();
    }
  }

}


void print_raw(char * arr, size_t s) {
  for (uint16_t j = 0; j < s; j++) {
    Serial.print(arr[j]);
  }
}

uint16_t load_and_print(uint16_t start, uint16_t s) {
  uint16_t j;
  char eep;
  for (j = 0; j < s; j++) {
    eep = EEPROM.read(j + start);
    Serial.print(eep);

  }
  return j + start;
}

bool serialin_to_array(uint8_t * arr, uint16_t arr_size, uint16_t timeout) {
  uint32_t xtimer = millis();
  uint16_t i = 0;
  while (i < arr_size) {
    if (Serial.available()) {
      xtimer = millis();
      arr[i] = Serial.read();
      //Serial.print(arr[i]);Serial.print(" ");
      i++;
    }
    if ((millis() - xtimer) > timeout) {
      return false;
    }
  }
  return true;
}

uint16_t load_array_of_eeprom(uint16_t start, uint8_t * arr, uint8_t s) {
  uint16_t j;
  for (j = 0; j < s; j++) {
    arr[j] = EEPROM.read(j + start);
  }
  return j + start;
}



//uint16_t write_array_to_eeprom(uint16_t start, uint8_t * arr, uint8_t s) {
//  uint16_t j;
//  for (j = 0; j < s; j++) {
//    EEPROM.write(j + start, arr[j]);
//  }
//  return j + start;
//}


//void print_config_B5() {
//  Serial.print("KEY = ");
//  Serial.print(C.KEY[0], HEX); Serial.print(" ");
//  Serial.print(C.KEY[1], HEX); Serial.print(" ");
//  Serial.print(C.KEY[2], HEX); Serial.print(" ");
//  Serial.print(C.KEY[3], HEX); Serial.print(" ");
//  Serial.println();
//  Serial.print("VERSION = ");  Serial.print(C.VERSION, HEX);
//  Serial.println();
//  Serial.print("FORMAT = ");  Serial.print(C.FORMAT);
//  Serial.println();
//  Serial.print("NODEID = ");  Serial.print(C.NODEID);
//  Serial.println();
//  Serial.print("OUTPUT_RATE = ");  Serial.print(C.OUTPUT_RATE);
//  Serial.println();
//  Serial.print("PHASECAL = ");  Serial.print(C.PHASECAL);
//  Serial.println();
//  Serial.print("VEST = ");  Serial.print(C.VEST);
//  Serial.println();
//  Serial.print("xpFREQ = ");  Serial.print(C.xpFREQ);
//  Serial.println();
//  Serial.print("N_cycles = ");  Serial.print(C.N_cycles);
//  Serial.println();
//  Serial.print("N_signal_node = ");  Serial.print(C.N_signal_node);
//  Serial.println();
//  Serial.print("N_power_node = ");  Serial.print(C.N_power_node);
//  Serial.println();
//  Serial.print("N_3p_node = ");  Serial.print(C.N_3p_node);
//  Serial.println();
//  Serial.print("N_freq_node = ");  Serial.print(C.N_freq_node);
//  Serial.println();
//  Serial.print("N_channels = ");  Serial.print(C.N_channels);
//  Serial.println();
//  Serial.print("debug = ");  Serial.print(C.debug);
//  Serial.println();
//  Serial.print("model = ");
//  Serial.print(C.model[0]); Serial.print(" ");
//  Serial.print(C.model[1]); Serial.print(" ");
//  Serial.print(C.model[2]); Serial.print(" ");
//  Serial.print(C.model[3]); Serial.print(" ");
//  Serial.print(C.model[4]); Serial.print(" ");
//  Serial.print(C.model[5]); Serial.print(" ");
//  Serial.println();
//}