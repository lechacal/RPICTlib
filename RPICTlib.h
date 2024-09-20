// RPICTlib library
// Version 1.7.0
// March 2024
// LeChacal.com
//
// This is free and unencumbered software released into the public domain.
// 
// Anyone is free to copy, modify, publish, use, compile, sell, or
// distribute this software, either in source code form or as a compiled
// binary, for any purpose, commercial or non-commercial, and by any
// means.
// 
// In jurisdictions that recognize copyright laws, the author or authors
// of this software dedicate any and all copyright interest in the
// software to the public domain. We make this dedication for the benefit
// of the public at large and to the detriment of our heirs and
// successors. We intend this dedication to be an overt act of
// relinquishment in perpetuity of all present and future rights to this
// software under copyright law.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
// OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
// ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
// OTHER DEALINGS IN THE SOFTWARE.
// 
// For more information, please refer to <http://unlicense.org/>

#define RPICTlib_1_7_0

#ifndef RPICTlib_h
#define RPICTlib_h

#include "Arduino.h"
#include <SPI.h>



extern uint16_t  ADC_COUNTS;
extern uint16_t  ADC_REF;

#define PHASE_ARRAY_LENGTH 10
extern float offset_filter;


//const float filter_b0 = 0.9984316659167189;
//const float filter_b1 = -0.9984316659167189;
//const float filter_a1 = -0.9968633318334380;


#if defined __AVR_ATmega328P__
	#define MCP3208_10_ON PORTB &= ~_BV(2);
	#define MCP3208_10_OFF PORTB |= _BV(2);
	#define MCP3208_06_ON PORTD &= ~_BV(6);
	#define MCP3208_06_OFF PORTD |= _BV(6);
	#define MCP3208_07_ON PORTD &= ~_BV(7);
	#define MCP3208_07_OFF PORTD |= _BV(7);
	#define MCP3208_08_ON PORTB &= ~_BV(0);
	#define MCP3208_08_OFF PORTB |= _BV(0);
	#define MCP3208_09_ON PORTB &= ~_BV(1);
	#define MCP3208_09_OFF PORTB |= _BV(1);
	#define MASTER 10
	#define SLAVE1 6
	#define SLAVE2 7
	#define SLAVE3 8
	#define SLAVE4 9
	
	#define SS0_PIN PINB2
	#define SS1_PIN PIND6
	#define SS2_PIN PIND7
	#define SS3_PIN PINB0
	#define SS4_PIN PINB1

#else if defined __AVR_DB__
	#define MASTER_PIN 0
	#define SLAVE1 6
	#define SLAVE2 7
	#define SLAVE3 8
	#define SLAVE4 9
	
	#define SPI_PORT PORTC
	#define MOSI_BIT 0
	#define MISO_BIT 1
	#define SCK_BIT 2
	
	#define MOSI_PIN PIN_PC0
	#define MISO_PIN PIN_PC1
	#define SCK_PIN PIN_PC2
	
	#define SS_PORT PORTF
	#define SS1_BIT 2
	#define SS2_BIT 3
	#define SS3_BIT 4
	#define SS4_BIT 5
	#define SS1_PIN PIN_PF2
	#define SS2_PIN PIN_PF3
	#define SS3_PIN PIN_PF4
	#define SS4_PIN PIN_PF5

#endif



#define CYCLE_CLOCK SPI_PORT.OUT |= _BV(SCK_BIT); SPI_PORT.OUT &= ~_BV(SCK_BIT);


#define CH_TYPE_NONE 0
#define CH_TYPE_REALPOWER 1
#define CH_TYPE_APPARENTPOWER 2
#define CH_TYPE_VRMS 3
#define CH_TYPE_IRMS 4
#define CH_TYPE_PEST 5
#define CH_TYPE_POWERFACTOR 6
#define CH_TYPE_TEMPERATURE 7
#define CH_TYPE_FREQUENCY 8
#define CH_TYPE_RTD 9
#define CH_TYPE_REACTIVE 10
#define CH_TYPE_3P_REALPOWER 11
#define CH_TYPE_3P_APPARENTPOWER 12
#define CH_TYPE_3P_REACTIVEPOWER 13
#define CH_TYPE_3P_POWERFACTOR 14
#define CH_TYPE_3P_VRMS 15
#define CH_TYPE_3P_IRMS 16
#define CH_TYPE_ERROR 17

#define NODE_TYPE_SIGNAL 0
#define NODE_TYPE_POWER 1
#define NODE_TYPE_SIGNAL_MCP 2
#define NODE_TYPE_POWER_MCP 3
#define NODE_TYPE_FREQ 4
#define NODE_TYPE_3PHASE 5

#define FORMAT_CSV 0
#define FORMAT_SPA 3
#define FORMAT_BIN 4

#define CT1 7
#define CT2 6
#define CT3 5
#define CT4 4
#define CT5 3
#define CT6 2
#define CT7 1
#define CT8 0
#define V1 0
#define ZMPT1 2
#define ZMPT2 1
#define ZMPT3 0



#define KEY0 0x43 // 67
#define KEY1 0x2D // 45
#define KEY2 0x7C // 124
#define KEY3 0x19 // 25

#define N_MAX_BOARD 5
#define N_SENSORS 8
#define MAX_NODES 28
#define MAX_CHANNELS_ 64
struct config_B4_t
{
  char KEY[4];
  byte VERSION = 0xB4;
  byte FORMAT;
  byte NODEID;
  unsigned int POLLING;
  float KCAL[N_MAX_BOARD][N_SENSORS];
  int8_t PHASECAL;
  float VEST;
  uint8_t xpFREQ;
  uint8_t N_cycles;
  byte N_nodes;
  byte N_channels;
  byte HW_sct[MAX_NODES];
  byte HW_mcp_sct[MAX_NODES];
  byte HW_vol[MAX_NODES];
  byte HW_mcp_vol[MAX_NODES];
  byte CH_type[MAX_CHANNELS_];
  byte CH_id[MAX_CHANNELS_];
  uint8_t debug;
};


struct config_B5_t
{
  char KEY[4];
  uint8_t VERSION = 0xB5;
  uint8_t FORMAT;
  uint8_t NODEID;
  unsigned int OUTPUT_RATE;
  float KCAL[N_MAX_BOARD][N_SENSORS];
  int8_t PHASECAL;
  float VEST;
  uint8_t xpFREQ;
  uint8_t N_cycles;
  uint8_t N_signal_node;
  uint8_t N_power_node;
  uint8_t N_3p_node;
  uint8_t N_freq_node;
  uint8_t N_channels;
  uint8_t debug;
  uint8_t model[N_MAX_BOARD+1];
};

// read_MCP3208_atmega328
// Read ADC value from a MCP3208 chip
// Optimised for Atmega328 
//
// channel: Channel number from 0 to 7.
// _cs: Chip Select pin to use.
uint16_t read_MCP3208_atmega328(uint8_t channel, uint8_t _cs);

// definitions for the ADC. vref in mV.
void adc_definition(uint8_t bits, uint16_t vref);

// Linear interpolation function
float linterp(uint16_t y, uint32_t dx, uint16_t y0, uint16_t y1);

class AC_Sensor
{
  public:
  	float offset;
  	float Ratio;
  	uint8_t inPin;
  	uint8_t level;
  	
  	void begin(uint8_t _inPin, uint8_t _level, float _CAL);
  	void update_offset(uint16_t sample);
  	

};



// SignalNode
// Class to compute RMS of a single signal only
class SignalNode
{
  public:
      	// begin
	// _inPinI: Current channel.
	// _ICAL: Calibration Coefficient for the current waveform.
	//void begin(uint8_t _inPin, SensorArray * _Sarr);
	void begin(AC_Sensor * _Sensor);
	// calcIrms
	// Computes Irms only
	// NUMBER_OF_SAMPLES: How many samples are used.
	// sInterval: Sampling interval.
	// return: error number. 0 - No error. 1 - Can not cope with sInterval. 2 - Out of range reading.
	void calcRMS(uint16_t NUMBER_OF_SAMPLES, uint16_t sInterval);
	
	float RMS;
	uint8_t err;
	AC_Sensor * Sensor;
	
  //private:
  	//SensorArray * Sarr;
};

class PowerNode
{
  public:
    	// begin
	// _inPinI: Current channel.
	// _mcpI: Chip Select pin for the mcp3208 for current.
	// _inPinV: Voltage channel.
	// _mcpV: Chip Select pin for the mcp3208 for voltage.
	// _ICAL: Calibration Coefficient for the current waveform.
	// _VCAL: Calibration Coefficient for the voltage waveform.
	// _PHASECAL: Calibration Coefficient for the phase delay between waveform.
	//void begin(uint8_t _inPinI, uint8_t _inPinV, int8_t _PHASECAL, SensorArray * _Sarr);
	void begin(AC_Sensor * _SensorI, AC_Sensor * _SensorV, int8_t _PHASECAL);
	// calcVI
	// Computes Real Power and relative values
	// NUMBER_OF_SAMPLES: How many samples are used.
	// sInterval: Sampling interval.
	// return: error number. 0 - No error. 1 - Can not cope with sInterval. 2 - Out of range reading.
	void calcVI(uint16_t NUMBER_OF_SAMPLES, uint16_t sInterval);
	
	float Irms, Vrms, realPower;
	uint8_t err;
	int8_t PHASECAL;
	AC_Sensor * SensorI;
	AC_Sensor * SensorV;
  //private:
  	//SensorArray * Sarr;
};

class FrequencyNode
{
  public:
  	// begin
	// _inPinI: Current channel.
	// _mcpI: Chip Select pin for the mcp3208 for current.
	// _inPinV: Voltage channel.
	// _mcpV: Chip Select pin for the mcp3208 for voltage.
	// _ICAL: Calibration Coefficient for the current waveform.
	// _VCAL: Calibration Coefficient for the voltage waveform.
	// _PHASECAL: Calibration Coefficient for the phase delay between waveform.
	void begin(AC_Sensor * _Sensor);
	
	// calcFreq
	// Compute Frequency using Zero Crossing method
	// Expected Frequency must be provided for timeouts.
	// Frequency will be computed against Voltage port.
	// Only the period is given in variable T.
	void calcFreq( uint16_t xpT);
	uint8_t err;
	AC_Sensor * Sensor;
	float T;

  private:
  	

};

class ThreeWattMeter
{
  public:
    // begin
    // _inPinI: Current channel.
    // _mcpI: Chip Select pin for the mcp3208 for current.
    // _inPinV: Voltage channel.
    // _mcpV: Chip Select pin for the mcp3208 for voltage.
    // _ICAL: Calibration Coefficient for the current waveform.
    // _VCAL: Calibration Coefficient for the voltage waveform.
    void begin(AC_Sensor * _SensorI1, AC_Sensor * _SensorV1,\
				 AC_Sensor * _SensorI2, AC_Sensor * _SensorV2,\
				 AC_Sensor * _SensorI3, AC_Sensor * _SensorV3,\
				 int8_t _PHASECAL);
    // calcVI
    // Computes Real Power and relative values
    // NUMBER_OF_SAMPLES: How many samples are used.
    // sInterval: Sampling interval.
    // return: error number. 0 - No error. 1 - Can not cope with sInterval. 2 - Out of range reading.
    void calcVI(uint16_t NUMBER_OF_SAMPLES, uint16_t sInterval);
    AC_Sensor * SensorI1;
    AC_Sensor * SensorI2;
    AC_Sensor * SensorI3;
    AC_Sensor * SensorV1;
    AC_Sensor * SensorV2;
    AC_Sensor * SensorV3;

    float I1rms, V1rms, I2rms, V2rms, I3rms, V3rms, P1, P2, P3;
    uint8_t err;
    int8_t PHASECAL;
    //float offsetI1, offsetV1, offsetI2, offsetV2, offsetI3, offsetV3;
    //float I1_RATIO, V1_RATIO, I2_RATIO, V2_RATIO, I3_RATIO, V3_RATIO;

  	

};


uint16_t analog_sample_db32(uint8_t channel, uint8_t level);

#endif