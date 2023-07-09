// RPICTlib library
// Version 1.6.0
// January 2022
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

#define RPICTlib_1_6_0

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
#endif

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

#define MASTER 10
#define SLAVE1 6
#define SLAVE2 7
#define SLAVE3 8
#define SLAVE4 9

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

class SensorArray
{
  public:
  	float * Offset;
  	float * Ratio;
  	size_t size;
  	void addMaster(float k0, float k1, float k2, float k3, float k4, float k5, float k6, float k7);
  	void addSlave(float k0, float k1, float k2, float k3, float k4, float k5, float k6, float k7);

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
	void begin(uint8_t _inPin, float _CAL);
	// calcIrms
	// Computes Irms only
	// NUMBER_OF_SAMPLES: How many samples are used.
	// sInterval: Sampling interval.
	// return: error number. 0 - No error. 1 - Can not cope with sInterval. 2 - Out of range reading.
	void calcRMS(uint16_t NUMBER_OF_SAMPLES, uint16_t sInterval);
	
	float RMS;
	uint8_t err;
	int inPin;
	float offset;
	float RATIO;
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
	void begin(uint8_t _inPinI, uint8_t _inPinV, float _ICAL, float _VCAL, int8_t _PHASECAL);
	// calcVI
	// Computes Real Power and relative values
	// NUMBER_OF_SAMPLES: How many samples are used.
	// sInterval: Sampling interval.
	// return: error number. 0 - No error. 1 - Can not cope with sInterval. 2 - Out of range reading.
	void calcVI(uint16_t NUMBER_OF_SAMPLES, uint16_t sInterval);
	
	float Irms, Vrms, realPower;
	uint8_t err;
	int inPinI, inPinV;
	int8_t PHASECAL;
	float offsetI, offsetV;
	float I_RATIO, V_RATIO;
  //private:
  	//SensorArray * Sarr;
};

// CurrentNode_mcp328
// Class to compute Irms only from a MCP3208.
class SignalNode_mcp3208
{
  public:
    	// begin
	// _inPinI: Current channel.
	// _mcpI: Chip Select pin for the mcp3208 for current.
	// _ICAL: Calibration Coefficient for the current waveform.
	void begin(uint8_t _inPin, uint8_t _mcp, SensorArray * _Sarr);
	// calcIrms
	// Computes Irms only
	// NUMBER_OF_SAMPLES: How many samples are used.
	// sInterval: Sampling interval.
	// return: error number. 0 - No error. 1 - Can not cope with sInterval. 2 - Out of range reading.
	void calcRMS(uint16_t NUMBER_OF_SAMPLES, uint16_t sInterval);
	
	float RMS;
	uint8_t err;
	uint8_t inPin, mcp;
	int8_t PHASECAL;
  
  private:
  	SensorArray * Sarr;

};

// PowerNode_mcp328
// Class to compute Real Power and relative values from a MCP3208.

class PowerNode_mcp3208
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
	void begin(uint8_t _inPinI, uint8_t _mcpI, uint8_t _inPinV,  uint8_t _mcpV,  int8_t _PHASECAL, SensorArray * _Sarr);
	// calcVI
	// Computes Real Power and relative values
	// NUMBER_OF_SAMPLES: How many samples are used.
	// sInterval: Sampling interval.
	// return: error number. 0 - No error. 1 - Can not cope with sInterval. 2 - Out of range reading.
	void calcVI(uint16_t NUMBER_OF_SAMPLES, uint16_t sInterval);
	
	float Irms, Vrms, realPower;
	uint8_t err;
	uint8_t inPinI, mcpI, inPinV, mcpV;
	int8_t PHASECAL;


  private:
  	SensorArray * Sarr;
};

// FrequencyNode_mcp328
// Class to compute frequency value from a MCP3208.

class FrequencyNode_mcp3208
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
	void begin(uint8_t _inPinI, uint8_t _mcpI, SensorArray * Sarr);
	
	// calcFreq
	// Compute Frequency using Zero Crossing method
	// Expected Frequency must be provided for timeouts.
	// Frequency will be computed against Voltage port.
	// Only the period is given in variable T.
	void calcFreq( uint16_t xpT);
	uint8_t err;
	uint8_t inPin, mcp;
	float T;
  private:
  	SensorArray * Sarr;

};


class TwoWattMeter_mcp3208
{
  public:
    // begin
    // _inPinI: Current channel.
    // _mcpI: Chip Select pin for the mcp3208 for current.
    // _inPinV: Voltage channel.
    // _mcpV: Chip Select pin for the mcp3208 for voltage.
    // _ICAL: Calibration Coefficient for the current waveform.
    // _VCAL: Calibration Coefficient for the voltage waveform.
    void begin(uint8_t _inPinI1, uint8_t _mcpI1, uint8_t _inPinV1,  uint8_t _mcpV1,\
    		 uint8_t _inPinI2, uint8_t _mcpI2, uint8_t _inPinV2,  uint8_t _mcpV2,\
    		 int8_t _PHASECAL, SensorArray * _Sarr);
    // calcVI
    // Computes Real Power and relative values
    // NUMBER_OF_SAMPLES: How many samples are used.
    // sInterval: Sampling interval.
    // return: error number. 0 - No error. 1 - Can not cope with sInterval. 2 - Out of range reading.
    void calcVI(uint16_t NUMBER_OF_SAMPLES, uint16_t sInterval);


    float I1rms, V1rms, I2rms, V2rms, P1, P2;
    uint8_t err;
    uint8_t inPinI1, mcpI1, inPinV1, mcpV1;
    uint8_t inPinI2, mcpI2, inPinV2, mcpV2;
    int8_t PHASECAL;
    //float offsetI1, offsetV1, offsetI2, offsetV2, offsetI3, offsetV3;
    //float I1_RATIO, V1_RATIO, I2_RATIO, V2_RATIO, I3_RATIO, V3_RATIO;
   private:
  	SensorArray * Sarr;

};

class ThreeWattMeter_mcp3208
{
  public:
    // begin
    // _inPinI: Current channel.
    // _mcpI: Chip Select pin for the mcp3208 for current.
    // _inPinV: Voltage channel.
    // _mcpV: Chip Select pin for the mcp3208 for voltage.
    // _ICAL: Calibration Coefficient for the current waveform.
    // _VCAL: Calibration Coefficient for the voltage waveform.
    void begin(uint8_t _inPinI1, uint8_t _mcpI1, uint8_t _inPinV1,  uint8_t _mcpV1,\
    		 uint8_t _inPinI2, uint8_t _mcpI2, uint8_t _inPinV2,  uint8_t _mcpV2,\
    		 uint8_t _inPinI3, uint8_t _mcpI3, uint8_t _inPinV3,  uint8_t _mcpV3,\
    		 int8_t _PHASECAL, SensorArray * _Sarr);
    // calcVI
    // Computes Real Power and relative values
    // NUMBER_OF_SAMPLES: How many samples are used.
    // sInterval: Sampling interval.
    // return: error number. 0 - No error. 1 - Can not cope with sInterval. 2 - Out of range reading.
    void calcVI(uint16_t NUMBER_OF_SAMPLES, uint16_t sInterval);


    float I1rms, V1rms, I2rms, V2rms, I3rms, V3rms, P1, P2, P3;
    uint8_t err;
    uint8_t inPinI1, mcpI1, inPinV1, mcpV1;
    uint8_t inPinI2, mcpI2, inPinV2, mcpV2;
    uint8_t inPinI3, mcpI3, inPinV3, mcpV3;
    int8_t PHASECAL;
    //float offsetI1, offsetV1, offsetI2, offsetV2, offsetI3, offsetV3;
    //float I1_RATIO, V1_RATIO, I2_RATIO, V2_RATIO, I3_RATIO, V3_RATIO;
   private:
  	SensorArray * Sarr;

};





#endif