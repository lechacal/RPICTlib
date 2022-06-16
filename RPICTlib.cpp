// RPICTlib library
// Version 1.5.3
// October 2021
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

#include "Arduino.h"
#include "RPICTlib.h"

float offset_filter = 0.001;


uint16_t ADC_COUNTS = (1 << 12);
uint16_t ADC_REF = 4096;

void adc_definition(uint8_t bits, uint16_t vref)
{
	ADC_COUNTS = (1 << bits);
	ADC_REF = vref;
}

void SignalNode::begin(uint8_t _inPin, SensorArray * _Sarr)
{
  inPin = _inPin;
  //offset = ADC_COUNTS>>1; //divide by 2 
  //RATIO = _CAL * ((ADC_REF / 1000.0) / (ADC_COUNTS));
  Sarr = _Sarr;
}

void SignalNode::calcRMS(uint16_t NUMBER_OF_SAMPLES, uint16_t sInterval)
{

  float offset = Sarr->Offset[inPin];
  float RATIO = Sarr->Ratio[inPin];

  float sum = 0;
  float centred;
  uint16_t sample;
  uint32_t timer1 = micros();
  
  uint16_t i=0;
  
  uint8_t tc = 0; // time check boolean

  err = 0;
  while (i < NUMBER_OF_SAMPLES) {
    if ((micros() - timer1) > sInterval) {
      timer1 += sInterval;
      sample = analogRead(inPin);
      if (sample==ADC_COUNTS-1) {
        err=2; 
        //return;
      }

      // IIR Low pass filter to center waveform,
      // alfa = 1/1024 => fc ~ 0.2Hz
      offset = offset + (sample - offset) * offset_filter;
      centred = sample - offset;
      sum += centred * centred;
      i++;

       // Check we looped at least once before entering here. Return error otherwise.
      if (tc) {
        err = 1;
        return;
      }
      else tc = 1;
    } // if
    else tc = 0; // We need to come here at least once to validate timing.
  } //while

  RMS = RATIO * sqrt(sum / NUMBER_OF_SAMPLES);
  
  Sarr->Offset[inPin] = offset;
}



void PowerNode::begin(uint8_t _inPinI, uint8_t _inPinV, int8_t _PHASECAL, SensorArray * _Sarr)
{
  inPinI = _inPinI;
  inPinV = _inPinV;
  //offsetI = ADC_COUNTS>>1; //divide by 2 
  //offsetV = ADC_COUNTS>>1;
  PHASECAL = _PHASECAL;
  Sarr = _Sarr;
  //V_RATIO = _VCAL * ((ADC_REF / 1000.0) / (ADC_COUNTS));
  //I_RATIO = _ICAL * ((ADC_REF / 1000.0) / (ADC_COUNTS));

}

void PowerNode::calcVI(uint16_t NUMBER_OF_SAMPLES, uint16_t sInterval)
{

  float offsetI = Sarr->Offset[inPinI];
  float offsetV = Sarr->Offset[inPinV];
  float I_RATIO = Sarr->Ratio[inPinI];
  float V_RATIO = Sarr->Ratio[inPinV];

  float sumI = 0;
  float sumV = 0;
  float sumP = 0;
  float centredI, centredV;
  int sampleI, sampleV;

  unsigned long timer1 = micros();
  uint8_t tc = 0; // time check boolean
  //unsigned long dt;
  
   // Building the array holding the samples to delay
  uint8_t ABS_PHASECAL = abs(PHASECAL);
  uint16_t *PHarray = new uint16_t[ABS_PHASECAL];  
  
  uint16_t i = 0; // index of samples
  int8_t j = 0; // index of items in phase array
  
  // If PHASECAL is not null then we start storing V or I
  if (PHASECAL != 0){
    while (j < ABS_PHASECAL){
      if ((micros() - timer1) > sInterval) {
  	 timer1 += sInterval;
  	 
         if (PHASECAL>0){
           PHarray[j] = analogRead(inPinV);
         }
         else {
           PHarray[j] = analogRead(inPinI);
         }
  	 j++;
      } // if timer
    }// while
  } // if PHASECAL 

  err = 0;
  j = 0;
  while (i < NUMBER_OF_SAMPLES) {
    //for (unsigned int n = 0; n < NUMBER_OF_SAMPLES; n++)
    if ((micros() - timer1) > sInterval) {
      timer1 += sInterval;
      sampleI = analogRead(inPinI);
      sampleV = analogRead(inPinV);
      
      
      if ((sampleV==ADC_COUNTS-1) || (sampleI==ADC_COUNTS-1)) {
        err=2;
        //return ;
      }

      // IIR Low pass filter to center waveform,
      // alfa = 1/1024 => fc ~ 0.2Hz
      offsetV = offsetV + (sampleV - offsetV) * offset_filter;
      offsetI = offsetI + (sampleI - offsetI) * offset_filter;

      // We only substract the previously computed offset to avoid introducing noise.
      if (PHASECAL == 0) {
      	centredV = sampleV - offsetV;
      	centredI = sampleI - offsetI;
      }
      else if (PHASECAL > 0){
      	centredV = PHarray[j] - offsetV; // Use the old value in PHarray
      	centredI = sampleI - offsetI;
      	PHarray[j] = sampleV; // Replace the value in PHarray
      }
      else {
      	centredV = sampleV - offsetV;
      	centredI = PHarray[j] - offsetI;
      	PHarray[j] = sampleI;
      }
      
      i++;
      j++;
      if (j==ABS_PHASECAL) j=0;

      // Root-mean-square method 
      sumP += centredV * centredI;
      sumI += centredI * centredI;
      sumV += centredV * centredV;

       // Check we looped at least once before entering here. Return error otherwise.
      if (tc) {
        err = 1;
        return;
      }
      else tc = 1;
    } // if
    else tc = 0; // We need to come here at least once to validate timing.
  } //while
  
  delete [] PHarray;
  
  Vrms = V_RATIO * sqrt(sumV / NUMBER_OF_SAMPLES);
  Irms = I_RATIO * sqrt(sumI / NUMBER_OF_SAMPLES);
  realPower = V_RATIO * I_RATIO * sumP / NUMBER_OF_SAMPLES;
  
  Sarr->Offset[inPinI] = offsetI;
  Sarr->Offset[inPinV] = offsetV;
}


float linterp(uint16_t y, uint32_t dx, uint16_t y0, uint16_t y1) {
  // Linear interpolation
  // function edited and optimised specifically for our purpose here.
  // Does not apply to the general case
  // x0 is zero here.
  return (float) (dx * (y - y0)) / (y1 - y0);
}

#if defined __AVR_ATmega328P__
uint16_t read_MCP3208_atmega328(uint8_t channel, uint8_t _cs) {
  
  uint8_t commandbits = 0b01100000 | ((channel & 0b111) << 2);
  
  if (_cs == 10) {
    MCP3208_10_ON
  }
  else if (_cs == 6){
    MCP3208_06_ON
  }
   else if (_cs == 7){
    MCP3208_07_ON
  }
   else if (_cs == 8){
    MCP3208_08_ON
  }
   else if (_cs == 9){
    MCP3208_09_ON
  }

  (void) SPI.transfer(commandbits);
  uint8_t b1 = SPI.transfer(0);
  uint8_t b2 = SPI.transfer(0);
  
  
  if (_cs == 10) {
    MCP3208_10_OFF
  }
  else if (_cs == 6){
    MCP3208_06_OFF
  }
   else if (_cs == 7){
    MCP3208_07_OFF
  }
   else if (_cs == 8){
    MCP3208_08_OFF
  }
   else if (_cs == 9){
    MCP3208_09_OFF
  }
  
  return (b1 << 4) | (b2 >> 4);
}
#endif

void SensorArray::addMaster(float k0, float k1, float k2, float k3, float k4, float k5, float k6, float k7){

	size = 8;
	
	Offset = new float[size];
	Ratio = new float[size];
	for (uint8_t i=0; i<size; i++){
		Offset[i] = ADC_COUNTS>>1;
	}
	Ratio[0] = k0 * ((ADC_REF / 1000.0) / (ADC_COUNTS));
	Ratio[1] = k1 * ((ADC_REF / 1000.0) / (ADC_COUNTS));
	Ratio[2] = k2 * ((ADC_REF / 1000.0) / (ADC_COUNTS));
	Ratio[3] = k3 * ((ADC_REF / 1000.0) / (ADC_COUNTS));
	Ratio[4] = k4 * ((ADC_REF / 1000.0) / (ADC_COUNTS));
	Ratio[5] = k5 * ((ADC_REF / 1000.0) / (ADC_COUNTS));
	Ratio[6] = k6 * ((ADC_REF / 1000.0) / (ADC_COUNTS));
	Ratio[7] = k7 * ((ADC_REF / 1000.0) / (ADC_COUNTS));

}


void SensorArray::addSlave(float k0, float k1, float k2, float k3, float k4, float k5, float k6, float k7){

	size_t newsize = size + 8;
	float* newOffset = new float[newsize];
	float* newRatio = new float[newsize];
	
	memcpy( newOffset, Offset, size * sizeof(float) );
	memcpy( newRatio, Ratio, size * sizeof(float) );
	
	delete [] Offset;
	delete [] Ratio;
	
	Offset = newOffset;
	Ratio = newRatio;
	
	for (uint8_t i=size; i<newsize; i++){
		Offset[i] = ADC_COUNTS>>1;
	}
	Ratio[size+0] = k0 * ((ADC_REF / 1000.0) / (ADC_COUNTS));
	Ratio[size+1] = k1 * ((ADC_REF / 1000.0) / (ADC_COUNTS));
	Ratio[size+2] = k2 * ((ADC_REF / 1000.0) / (ADC_COUNTS));
	Ratio[size+3] = k3 * ((ADC_REF / 1000.0) / (ADC_COUNTS));
	Ratio[size+4] = k4 * ((ADC_REF / 1000.0) / (ADC_COUNTS));
	Ratio[size+5] = k5 * ((ADC_REF / 1000.0) / (ADC_COUNTS));
	Ratio[size+6] = k6 * ((ADC_REF / 1000.0) / (ADC_COUNTS));
	Ratio[size+7] = k7 * ((ADC_REF / 1000.0) / (ADC_COUNTS));
	
	size = newsize;

}

uint8_t mcp_to_index(uint8_t mcp){
	if (mcp==10) return 0;
	else return mcp-5;
}

void SignalNode_mcp3208::begin(uint8_t _inPin, uint8_t _mcp, SensorArray * _Sarr)
{
  inPin = _inPin;
  mcp   = _mcp;
  Sarr = _Sarr;

}

void SignalNode_mcp3208::calcRMS(uint16_t NUMBER_OF_SAMPLES, uint16_t sInterval)
{

  float offset = Sarr->Offset[inPin + mcp_to_index(mcp)*8];
  float RATIO = Sarr->Ratio[inPin + mcp_to_index(mcp)*8];
  
  float sum = 0;
  float centred;
  uint16_t sample;
  uint32_t timer1 = micros();
  uint8_t tc = 0; // time check boolean
  
  uint16_t i=0;
  
  err = 0;
  while (i < NUMBER_OF_SAMPLES) {
    if ((micros() - timer1) > sInterval) {
      timer1 += sInterval;
      sample = read_MCP3208_atmega328(inPin, mcp);
  
      if (sample==ADC_COUNTS-1) {
        err=2; 
        //return ;
      }

      // IIR Low pass filter to center waveform,
      // alfa = 1/1024 => fc ~ 0.2Hz
      offset = offset + (sample - offset) * offset_filter;
      centred = sample - offset;
      sum += centred * centred;
      i++;

      // Check we looped at least once before entering here. Return error otherwise.
      if (tc) {
        err = 1;
        return ;
      }
      else tc = 1;
    } // if
    else tc = 0; // We need to come here at least once to validate timing.
  } //while

  RMS = RATIO * sqrt(sum / NUMBER_OF_SAMPLES);
  
  Sarr->Offset[inPin + mcp_to_index(mcp)*8] = offset;
}

void PowerNode_mcp3208::begin(uint8_t _inPinI, uint8_t _mcpI, uint8_t _inPinV, uint8_t _mcpV, int8_t _PHASECAL, SensorArray * _Sarr)
{
  inPinI = _inPinI;
  mcpI   = _mcpI;
  inPinV = _inPinV;
  mcpV   = _mcpV;
  PHASECAL = _PHASECAL;
  Sarr = _Sarr;
  
  //Serial.print(inPinI);
  //Serial.print(" ");
  //Serial.print(mcpI);
  //Serial.print(" ");
  //Serial.print(inPinV);
  //Serial.print(" ");
  //Serial.println(mcpV);
 

}


void PowerNode_mcp3208::calcVI(uint16_t NUMBER_OF_SAMPLES, uint16_t sInterval)
{
  float offsetI = Sarr->Offset[inPinI + mcp_to_index(mcpI)*8];
  float offsetV = Sarr->Offset[inPinV + mcp_to_index(mcpV)*8];
  float I_RATIO = Sarr->Ratio[inPinI + mcp_to_index(mcpI)*8];
  float V_RATIO = Sarr->Ratio[inPinV + mcp_to_index(mcpV)*8];
  

  float sumI = 0;
  float sumV = 0;
  float sumP = 0;
  float centredI, centredV;
  uint16_t sampleI, sampleV;

  unsigned long timer1 = micros();
  uint8_t tc = 0; // time check boolean
  //unsigned long dt;
  
  // Building the array holding the samples to delay
  uint8_t ABS_PHASECAL = abs(PHASECAL);
  uint16_t *PHarray = new uint16_t[ABS_PHASECAL];
  
  uint16_t i = 0; // index of samples
  int8_t j = 0; // index of items in phase array
  
  
  // If PHASECAL is not null then we start storing V or I
  if (PHASECAL != 0){
    while (j < ABS_PHASECAL){
      if ((micros() - timer1) > sInterval) {
  	 timer1 += sInterval;
  	 
         if (PHASECAL>0){
           PHarray[j] = read_MCP3208_atmega328(inPinV, mcpV);
         }
         else {
           PHarray[j] = read_MCP3208_atmega328(inPinI, mcpI);
         }
  	 j++;
      } // if timer
    }// while
  } // if PHASECAL 

  err = 0;
  j = 0;
  while (i < NUMBER_OF_SAMPLES) {
    //for (unsigned int n = 0; n < NUMBER_OF_SAMPLES; n++)
    if ((micros() - timer1) > sInterval) {
      timer1 += sInterval;
      
      sampleI = read_MCP3208_atmega328(inPinI, mcpI);
      sampleV = read_MCP3208_atmega328(inPinV, mcpV);

      
      if ((sampleV==ADC_COUNTS-1) || (sampleI==ADC_COUNTS-1)) {
        err=2;
        //return;
      }

      // IIR Low pass filter to center waveform,
      // alfa = 1/1024 => fc ~ 0.2Hz
      offsetV = offsetV + (sampleV - offsetV) * offset_filter;
      offsetI = offsetI + (sampleI - offsetI) * offset_filter;
      
      //offsetV = sampleV*offset_filter + offsetV*(1-offset_filter);
      //offsetI = sampleI*offset_filter + offsetI*(1-offset_filter);

      // We only substract the previously computed offset to avoid introducing noise.
      if (PHASECAL == 0) {
      	centredV = sampleV - offsetV;
      	centredI = sampleI - offsetI;
      }
      else if (PHASECAL > 0){
      	centredV = PHarray[j] - offsetV; // Use the old value in PHarray
      	centredI = sampleI - offsetI;
      	PHarray[j] = sampleV; // Replace the value in PHarray
      }
      else {
      	centredV = sampleV - offsetV;
      	centredI = PHarray[j] - offsetI;
      	PHarray[j] = sampleI;
      }
      
      i++;
      j++;
      if (j==ABS_PHASECAL) j=0;

      // Root-mean-square method 
      sumP += centredV * centredI;
      sumI += centredI * centredI;
      sumV += centredV * centredV;
      

      // Check we looped at least once before entering here. Return error otherwise.
      if (tc) {
        err = 1;
        return;
      }
      else tc = 1;
    } // if
    else tc = 0; // We need to come here at least once to validate timing.
  } //while
  
  delete [] PHarray;
  
  Vrms = V_RATIO * sqrt(sumV / NUMBER_OF_SAMPLES);
  Irms = I_RATIO * sqrt(sumI / NUMBER_OF_SAMPLES);
  realPower = V_RATIO * I_RATIO * sumP / NUMBER_OF_SAMPLES;
  
  Sarr->Offset[inPinI + mcp_to_index(mcpI)*8] = offsetI;
  Sarr->Offset[inPinV + mcp_to_index(mcpV)*8] = offsetV;
  
  //Serial.print(sampleI);
  //Serial.print(" ");
  //Serial.println(sampleV);
  

}

void FrequencyNode_mcp3208::begin(uint8_t _inPin, uint8_t _mcp, SensorArray * _Sarr)
{
  inPin = _inPin;
  mcp   = _mcp;
  Sarr = _Sarr;
}

void FrequencyNode_mcp3208::calcFreq( uint16_t xpT)
{

  float offset = Sarr->Offset[inPin + mcp_to_index(mcp)*8];

  uint32_t timeR1, timeR2, ta, tb;
  float ta_delta, tb_delta;
  uint16_t sampleR1, sampleR2;
  uint8_t sign_1, sign_2;
  const uint16_t Threshold = ADC_COUNTS / 20;

  //uint16_t YY[6];
  //uint32_t TT[6];

  uint32_t timer1 = micros();
  // ------------
  // First loop. Looking for a distinct polarity
  // ------------
  while (1) {
    sampleR1 = read_MCP3208_atmega328(inPin, mcp);
    timeR1 = micros();
    if (sampleR1 > (offset + Threshold) or sampleR1 < (offset - Threshold)) {
      sign_1 = (sampleR1 > offset);
      break;
    }

    if ((timeR1 - timer1) > xpT) { // Exit if we've been waiting more than a period.
      err = 3;
      return;
    }
  }
  timer1 = micros();
  //TT[0] = timeR1;
  //YY[0] = sampleR1;

  // ------------
  // Second loop. Looking for the first Zero Crossing.
  // ------------
  while (1) {
    sampleR2 = read_MCP3208_atmega328(inPin, mcp);
    timeR2 = micros();
    sign_2 = (sampleR2 > offset);
    if (sign_1 != sign_2) {
      ta = timeR1;
      ta_delta = linterp( offset, (timeR2 - timeR1), sampleR1, sampleR2);

      break;
    }
    else {
      timeR1 = timeR2;
      sampleR1 = sampleR2;
    }

    if ((timeR2 - timer1) > xpT) { // Exit if we've been waiting more than a period.
      err = 4;
      return;
 
    }

  }
  timer1 = ta + (uint32_t) ta_delta;
  //TT[1] = timeR1;
  //YY[1] = sampleR1;
  //TT[2] = timeR2;
  //YY[2] = sampleR2;

  // ------------
  // Third loop. Waiting for the polarity change. Mid wave.
  // ------------
  uint8_t cnt = 3; // we need at least 3 samples to confirm solidly
  while (1) {
    sampleR1 = read_MCP3208_atmega328(inPin, mcp);
    timeR1 = micros();
    sign_1 = (sampleR1 > offset);
    if (sign_1 != sign_2) {
      cnt --;
      if (cnt == 0)break;
    }

    if ((timeR1 - timer1) > xpT) { // Exit if we've been waiting more than a period.
      err = 5;
      return;
    }
  }
  //TT[3] = timeR1;
  //YY[3] = sampleR1;

  // ------------
  // forth loop. Looking for the second zero crossing.
  // ------------
  while (1) {
    sampleR2 = read_MCP3208_atmega328(inPin, mcp);
    timeR2 = micros();
    sign_2 = (sampleR2 > offset);
    if (sign_1 != sign_2) {
      tb = timeR1;
      tb_delta = linterp( offset, (timeR2 - timeR1), sampleR1, sampleR2);

      T = tb + tb_delta - ta - ta_delta;
      break;
    }
    else {
      timeR1 = timeR2;
      sampleR1 = sampleR2;
    }

    if ((timeR2 - timer1) > (xpT + xpT / 2)) { // Exit if we've been waiting more than 1.5 * period.
      err = 6;
      return;

    }

  } // while
   
  err = 0;
  
  //TT[4] = timeR1;
  //YY[4] = sampleR1;
  //TT[5] = timeR2;
  //YY[5] = sampleR2;

  //  for (uint8_t i=0; i<6; i++){
  //    Serial.print(TT[i]);
  //    Serial.print('\t');
  //    Serial.println(YY[i]);
  //  }
  
  // Not needed. we do not update the offset with this function.
  // Sarr->Offset[inPin + mcp_to_index(mcp)*8] = offset;
 
}





void TwoWattMeter_mcp3208::begin(uint8_t _inPinI1, uint8_t _mcpI1, uint8_t _inPinV1,  uint8_t _mcpV1,\
				 uint8_t _inPinI2, uint8_t _mcpI2, uint8_t _inPinV2,  uint8_t _mcpV2,\
				  int8_t _PHASECAL, SensorArray * _Sarr)
{
  
  inPinI1 = _inPinI1;
  mcpI1   = _mcpI1;
  inPinV1 = _inPinV1;
  mcpV1   = _mcpV1;
  inPinI2 = _inPinI2;
  mcpI2   = _mcpI2;
  inPinV2 = _inPinV2;
  mcpV2   = _mcpV2;
  
  PHASECAL = _PHASECAL;
  
  Sarr = _Sarr;
  
  
}

void TwoWattMeter_mcp3208::calcVI(uint16_t NUMBER_OF_SAMPLES, uint16_t sInterval)
{

 float offsetI1 = Sarr->Offset[inPinI1 + mcp_to_index(mcpI1)*8];
  float offsetV1 = Sarr->Offset[inPinV1 + mcp_to_index(mcpV1)*8];
  float I1_RATIO = Sarr->Ratio[inPinI1 + mcp_to_index(mcpI1)*8];
  float V1_RATIO = Sarr->Ratio[inPinV1 + mcp_to_index(mcpV1)*8];
  
  float offsetI2 = Sarr->Offset[inPinI2 + mcp_to_index(mcpI2)*8];
  float offsetV2 = Sarr->Offset[inPinV2 + mcp_to_index(mcpV2)*8];
  float I2_RATIO = Sarr->Ratio[inPinI2 + mcp_to_index(mcpI2)*8];
  float V2_RATIO = Sarr->Ratio[inPinV2 + mcp_to_index(mcpV2)*8];

  float sumI1 = 0;
  float sumV1 = 0;
  float sumP1 = 0;
  float sumI2 = 0;
  float sumV2 = 0;
  float sumP2 = 0;

  float centredI1, centredV1;
  uint16_t sampleI1, sampleV1;
  float centredI2, centredV2;
  uint16_t sampleI2, sampleV2;


  unsigned long timer1 = micros();
  uint8_t tc = 0; // time check boolean
  //unsigned long dt;
  
   // Building the array holding the samples to delay
  uint8_t ABS_PHASECAL = abs(PHASECAL);
  uint16_t *PHarray_1 = new uint16_t[ABS_PHASECAL];
  uint16_t *PHarray_2 = new uint16_t[ABS_PHASECAL];
  
  uint16_t i = 0; // index of samples
  int8_t j = 0; // index of items in phase array

  // If PHASECAL is not null then we start storing V or I
  if (PHASECAL != 0){
    while (j < ABS_PHASECAL){
      if ((micros() - timer1) > sInterval) {
  	 timer1 += sInterval;
  	 
         if (PHASECAL>0){
           PHarray_1[j] = read_MCP3208_atmega328(inPinV1, mcpV1);
           PHarray_2[j] = read_MCP3208_atmega328(inPinV2, mcpV2);
         }
         else {
           PHarray_1[j] = read_MCP3208_atmega328(inPinI1, mcpI1);
           PHarray_2[j] = read_MCP3208_atmega328(inPinI2, mcpI2);
         }
  	 j++;
      } // if timer
    }// while
  } // if PHASECAL 

  err = 0;
  j = 0;
  while (i < NUMBER_OF_SAMPLES) {
    //for (unsigned int n = 0; n < NUMBER_OF_SAMPLES; n++)
    if ((micros() - timer1) > sInterval) {
      timer1 += sInterval;

      sampleI1 = read_MCP3208_atmega328(inPinI1, mcpI1);
      sampleV1 = read_MCP3208_atmega328(inPinV1, mcpV1);
      sampleI2 = read_MCP3208_atmega328(inPinI2, mcpI2);
      sampleV2 = read_MCP3208_atmega328(inPinV2, mcpV2);

      if ((sampleV1 == ADC_COUNTS - 1) || (sampleI1 == ADC_COUNTS - 1)) {
        err = 2;
        //return;
      }
      else if ((sampleV2 == ADC_COUNTS - 1) || (sampleI2 == ADC_COUNTS - 1)) {
        err = 2;
        //return;
      }
      

      // IIR Low pass filter to center waveform,
      // alfa = 1/1024 => fc ~ 0.2Hz
      offsetV1 = offsetV1 + (sampleV1 - offsetV1) * offset_filter;
      offsetI1 = offsetI1 + (sampleI1 - offsetI1) * offset_filter;
      offsetV2 = offsetV2 + (sampleV2 - offsetV2) * offset_filter;
      offsetI2 = offsetI2 + (sampleI2 - offsetI2) * offset_filter;
      
	// We only substract the previously computed offset to avoid introducing noise.
      if (PHASECAL == 0) {
      	centredV1 = sampleV1 - offsetV1;
      	centredI1 = sampleI1 - offsetI1;
      	centredV2 = sampleV2 - offsetV2;
      	centredI2 = sampleI2 - offsetI2;
      	
      }
      else if (PHASECAL > 0){
      	centredV1 = PHarray_1[j] - offsetV1; // Use the old value in PHarray
      	centredI1 = sampleI1 - offsetI1;
      	centredV2 = PHarray_2[j] - offsetV2; // Use the old value in PHarray
      	centredI2 = sampleI2 - offsetI2;
      	
      	PHarray_1[j] = sampleV1; // Replace the value in PHarray
      	PHarray_2[j] = sampleV2;
      	
      }
      else {
      	centredV1 = sampleV1 - offsetV1;
      	centredI1 = PHarray_1[j] - offsetI1;
      	centredV2 = sampleV2 - offsetV2;
      	centredI2 = PHarray_2[j] - offsetI2;
      	
      	PHarray_1[j] = sampleI1;
      	PHarray_2[j] = sampleI2;
      	
      }
      
      i++;
      j++;
      if (j==ABS_PHASECAL) j=0;
      

      // Root-mean-square method 
      sumP1 += centredV1 * centredI1;
      sumI1 += centredI1 * centredI1;
      sumV1 += centredV1 * centredV1;
      sumP2 += centredV2 * centredI2;
      sumI2 += centredI2 * centredI2;
      sumV2 += centredV2 * centredV2;



      // Check we looped at least once before entering here. Return error otherwise.
      if (tc) {
        err = 1;
        return;
      }
      else tc = 1;
    } // if
    else tc = 0; // We need to come here at least once to validate timing.
  } //while
  
  delete [] PHarray_1;
  delete [] PHarray_2;

  V1rms = V1_RATIO * sqrt(sumV1 / NUMBER_OF_SAMPLES);
  I1rms = I1_RATIO * sqrt(sumI1 / NUMBER_OF_SAMPLES);
  P1 = V1_RATIO * I1_RATIO * sumP1 / NUMBER_OF_SAMPLES;

  V2rms = V2_RATIO * sqrt(sumV2 / NUMBER_OF_SAMPLES);
  I2rms = I2_RATIO * sqrt(sumI2 / NUMBER_OF_SAMPLES);
  P2 = V2_RATIO * I2_RATIO * sumP2 / NUMBER_OF_SAMPLES;

  
  Sarr->Offset[inPinI1 + mcp_to_index(mcpI1)*8] = offsetI1;
  Sarr->Offset[inPinV1 + mcp_to_index(mcpV1)*8] = offsetV1;
  //Sarr->Ratio[inPinI1 + mcp_to_index(mcpI1)*8] = I1_RATIO;
  //Sarr->Ratio[inPinV1 + mcp_to_index(mcpV1)*8] = V1_RATIO;
  
  Sarr->Offset[inPinI2 + mcp_to_index(mcpI2)*8] = offsetI2;
  Sarr->Offset[inPinV2 + mcp_to_index(mcpV2)*8] = offsetV2;
  //Sarr->Ratio[inPinI2 + mcp_to_index(mcpI2)*8] = I2_RATIO;
  //Sarr->Ratio[inPinV2 + mcp_to_index(mcpV2)*8] = V2_RATIO;
  
  
}


void ThreeWattMeter_mcp3208::begin(uint8_t _inPinI1, uint8_t _mcpI1, uint8_t _inPinV1,  uint8_t _mcpV1,\
				 uint8_t _inPinI2, uint8_t _mcpI2, uint8_t _inPinV2,  uint8_t _mcpV2,\
				 uint8_t _inPinI3, uint8_t _mcpI3, uint8_t _inPinV3,  uint8_t _mcpV3,\
				 int8_t _PHASECAL, SensorArray * _Sarr)
{
  inPinI1 = _inPinI1;
  mcpI1   = _mcpI1;
  inPinV1 = _inPinV1;
  mcpV1   = _mcpV1;
  inPinI2 = _inPinI2;
  mcpI2   = _mcpI2;
  inPinV2 = _inPinV2;
  mcpV2   = _mcpV2;
  inPinI3 = _inPinI3;
  mcpI3   = _mcpI3;
  inPinV3 = _inPinV3;
  mcpV3   = _mcpV3;
  
  PHASECAL = _PHASECAL;
  
  Sarr = _Sarr;
  
  //offsetI1 = ADC_COUNTS >> 1; //divide by 2
  //offsetV1 = ADC_COUNTS >> 1;
  //offsetI2 = ADC_COUNTS >> 1; //divide by 2
  //offsetV2 = ADC_COUNTS >> 1;
  //offsetI3 = ADC_COUNTS >> 1; //divide by 2
  //offsetV3 = ADC_COUNTS >> 1;

  //V1_RATIO = _VCAL1 * ((ADC_REF / 1000.0) / (ADC_COUNTS));
  //I1_RATIO = _ICAL1 * ((ADC_REF / 1000.0) / (ADC_COUNTS));
  //V2_RATIO = _VCAL2 * ((ADC_REF / 1000.0) / (ADC_COUNTS));
  //I2_RATIO = _ICAL2 * ((ADC_REF / 1000.0) / (ADC_COUNTS));
  //V3_RATIO = _VCAL3 * ((ADC_REF / 1000.0) / (ADC_COUNTS));
  //I3_RATIO = _ICAL3 * ((ADC_REF / 1000.0) / (ADC_COUNTS));
}

void ThreeWattMeter_mcp3208::calcVI(uint16_t NUMBER_OF_SAMPLES, uint16_t sInterval)
{

  float offsetI1 = Sarr->Offset[inPinI1 + mcp_to_index(mcpI1)*8];
  float offsetV1 = Sarr->Offset[inPinV1 + mcp_to_index(mcpV1)*8];
  float I1_RATIO = Sarr->Ratio[inPinI1 + mcp_to_index(mcpI1)*8];
  float V1_RATIO = Sarr->Ratio[inPinV1 + mcp_to_index(mcpV1)*8];
  
  float offsetI2 = Sarr->Offset[inPinI2 + mcp_to_index(mcpI2)*8];
  float offsetV2 = Sarr->Offset[inPinV2 + mcp_to_index(mcpV2)*8];
  float I2_RATIO = Sarr->Ratio[inPinI2 + mcp_to_index(mcpI2)*8];
  float V2_RATIO = Sarr->Ratio[inPinV2 + mcp_to_index(mcpV2)*8];
  
  float offsetI3 = Sarr->Offset[inPinI3 + mcp_to_index(mcpI3)*8];
  float offsetV3 = Sarr->Offset[inPinV3 + mcp_to_index(mcpV3)*8];
  float I3_RATIO = Sarr->Ratio[inPinI3 + mcp_to_index(mcpI3)*8];
  float V3_RATIO = Sarr->Ratio[inPinV3 + mcp_to_index(mcpV3)*8];

  float sumI1 = 0;
  float sumV1 = 0;
  float sumP1 = 0;
  float sumI2 = 0;
  float sumV2 = 0;
  float sumP2 = 0;
  float sumI3 = 0;
  float sumV3 = 0;
  float sumP3 = 0;
  float centredI1, centredV1;
  uint16_t sampleI1, sampleV1;
  float centredI2, centredV2;
  uint16_t sampleI2, sampleV2;
  float centredI3, centredV3;
  uint16_t sampleI3, sampleV3;

  unsigned long timer1 = micros();
  uint8_t tc = 0; // time check boolean
  //unsigned long dt;
  
   // Building the array holding the samples to delay
  uint8_t ABS_PHASECAL = abs(PHASECAL);
  uint16_t *PHarray_1 = new uint16_t[ABS_PHASECAL];
  uint16_t *PHarray_2 = new uint16_t[ABS_PHASECAL];
  uint16_t *PHarray_3 = new uint16_t[ABS_PHASECAL];
  
  uint16_t i = 0; // index of samples
  int8_t j = 0; // index of items in phase array

  // If PHASECAL is not null then we start storing V or I
  if (PHASECAL != 0){
    while (j < ABS_PHASECAL){
      if ((micros() - timer1) > sInterval) {
  	 timer1 += sInterval;
  	 
         if (PHASECAL>0){
           PHarray_1[j] = read_MCP3208_atmega328(inPinV1, mcpV1);
           PHarray_2[j] = read_MCP3208_atmega328(inPinV2, mcpV2);
           PHarray_3[j] = read_MCP3208_atmega328(inPinV3, mcpV3);
         }
         else {
           PHarray_1[j] = read_MCP3208_atmega328(inPinI1, mcpI1);
           PHarray_2[j] = read_MCP3208_atmega328(inPinI2, mcpI2);
           PHarray_3[j] = read_MCP3208_atmega328(inPinI3, mcpI3);
         }
  	 j++;
      } // if timer
    }// while
  } // if PHASECAL 

  err = 0;
  j = 0;
  while (i < NUMBER_OF_SAMPLES) {
    //for (unsigned int n = 0; n < NUMBER_OF_SAMPLES; n++)
    if ((micros() - timer1) > sInterval) {
      timer1 += sInterval;

      sampleI1 = read_MCP3208_atmega328(inPinI1, mcpI1);
      sampleV1 = read_MCP3208_atmega328(inPinV1, mcpV1);
      sampleI2 = read_MCP3208_atmega328(inPinI2, mcpI2);
      sampleV2 = read_MCP3208_atmega328(inPinV2, mcpV2);
      sampleI3 = read_MCP3208_atmega328(inPinI3, mcpI3);
      sampleV3 = read_MCP3208_atmega328(inPinV3, mcpV3);

      if ((sampleV1 == ADC_COUNTS - 1) || (sampleI1 == ADC_COUNTS - 1)) {
        err = 2;
        //return;
      }
      else if ((sampleV2 == ADC_COUNTS - 1) || (sampleI2 == ADC_COUNTS - 1)) {
        err = 2;
        //return;
      }
      else if ((sampleV3 == ADC_COUNTS - 1) || (sampleI3 == ADC_COUNTS - 1)) {
        err = 2;
        //return;
      }

      // IIR Low pass filter to center waveform,
      // alfa = 1/1024 => fc ~ 0.2Hz
      offsetV1 = offsetV1 + (sampleV1 - offsetV1) * offset_filter;
      offsetI1 = offsetI1 + (sampleI1 - offsetI1) * offset_filter;
      offsetV2 = offsetV2 + (sampleV2 - offsetV2) * offset_filter;
      offsetI2 = offsetI2 + (sampleI2 - offsetI2) * offset_filter;
      offsetV3 = offsetV3 + (sampleV3 - offsetV3) * offset_filter;
      offsetI3 = offsetI3 + (sampleI3 - offsetI3) * offset_filter;

	// We only substract the previously computed offset to avoid introducing noise.
      if (PHASECAL == 0) {
      	centredV1 = sampleV1 - offsetV1;
      	centredI1 = sampleI1 - offsetI1;
      	centredV2 = sampleV2 - offsetV2;
      	centredI2 = sampleI2 - offsetI2;
      	centredV3 = sampleV3 - offsetV3;
      	centredI3 = sampleI3 - offsetI3;
      }
      else if (PHASECAL > 0){
      	centredV1 = PHarray_1[j] - offsetV1; // Use the old value in PHarray
      	centredI1 = sampleI1 - offsetI1;
      	centredV2 = PHarray_2[j] - offsetV2; // Use the old value in PHarray
      	centredI2 = sampleI2 - offsetI2;
      	centredV3 = PHarray_3[j] - offsetV3; // Use the old value in PHarray
      	centredI3 = sampleI3 - offsetI3;
      	PHarray_1[j] = sampleV1; // Replace the value in PHarray
      	PHarray_2[j] = sampleV2;
      	PHarray_3[j] = sampleV3;
      }
      else {
      	centredV1 = sampleV1 - offsetV1;
      	centredI1 = PHarray_1[j] - offsetI1;
      	centredV2 = sampleV2 - offsetV2;
      	centredI2 = PHarray_2[j] - offsetI2;
      	centredV3 = sampleV3 - offsetV3;
      	centredI3 = PHarray_3[j] - offsetI3;
      	PHarray_1[j] = sampleI1;
      	PHarray_2[j] = sampleI2;
      	PHarray_3[j] = sampleI3;
      }
      
      i++;
      j++;
      if (j==ABS_PHASECAL) j=0;
      

      // Root-mean-square method 
      sumP1 += centredV1 * centredI1;
      sumI1 += centredI1 * centredI1;
      sumV1 += centredV1 * centredV1;
      sumP2 += centredV2 * centredI2;
      sumI2 += centredI2 * centredI2;
      sumV2 += centredV2 * centredV2;
      sumP3 += centredV3 * centredI3;
      sumI3 += centredI3 * centredI3;
      sumV3 += centredV3 * centredV3;



      // Check we looped at least once before entering here. Return error otherwise.
      if (tc) {
        err = 1;
        return;
      }
      else tc = 1;
    } // if
    else tc = 0; // We need to come here at least once to validate timing.
  } //while
  
  delete [] PHarray_1;
  delete [] PHarray_2;
  delete [] PHarray_3;

  V1rms = V1_RATIO * sqrt(sumV1 / NUMBER_OF_SAMPLES);
  I1rms = I1_RATIO * sqrt(sumI1 / NUMBER_OF_SAMPLES);
  P1 = V1_RATIO * I1_RATIO * sumP1 / NUMBER_OF_SAMPLES;

  V2rms = V2_RATIO * sqrt(sumV2 / NUMBER_OF_SAMPLES);
  I2rms = I2_RATIO * sqrt(sumI2 / NUMBER_OF_SAMPLES);
  P2 = V2_RATIO * I2_RATIO * sumP2 / NUMBER_OF_SAMPLES;
  
  V3rms = V3_RATIO * sqrt(sumV3 / NUMBER_OF_SAMPLES);
  I3rms = I3_RATIO * sqrt(sumI3 / NUMBER_OF_SAMPLES);
  P3 = V3_RATIO * I3_RATIO * sumP3 / NUMBER_OF_SAMPLES;

  
  Sarr->Offset[inPinI1 + mcp_to_index(mcpI1)*8] = offsetI1;
  Sarr->Offset[inPinV1 + mcp_to_index(mcpV1)*8] = offsetV1;
  Sarr->Ratio[inPinI1 + mcp_to_index(mcpI1)*8] = I1_RATIO;
  Sarr->Ratio[inPinV1 + mcp_to_index(mcpV1)*8] = V1_RATIO;
  
  Sarr->Offset[inPinI2 + mcp_to_index(mcpI2)*8] = offsetI2;
  Sarr->Offset[inPinV2 + mcp_to_index(mcpV2)*8] = offsetV2;
  Sarr->Ratio[inPinI2 + mcp_to_index(mcpI2)*8] = I2_RATIO;
  Sarr->Ratio[inPinV2 + mcp_to_index(mcpV2)*8] = V2_RATIO;
  
  Sarr->Offset[inPinI3 + mcp_to_index(mcpI3)*8] = offsetI3;
  Sarr->Offset[inPinV3 + mcp_to_index(mcpV3)*8] = offsetV3;
  Sarr->Ratio[inPinI3 + mcp_to_index(mcpI3)*8] = I3_RATIO;
  Sarr->Ratio[inPinV3 + mcp_to_index(mcpV3)*8] = V3_RATIO;
}
