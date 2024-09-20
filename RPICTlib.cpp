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

void AC_Sensor::begin(uint8_t _inPin, uint8_t _level, float _CAL){

#if defined __AVR_DB__
#if _AVR_PINCOUNT == 32
	if (_level==0){
		if (_inPin==7){
			inPin = 21; // CT1
		}
		else if (_inPin==6){
			inPin = 20; // CT2
		}
		else {
			inPin = _inPin + 13;
		}
	}
	else {
		inPin = _inPin;
		
	}
#else
	inPin = _inPin;
#endif	

#else if defined __AVR_ATmega328P__
	inPin = _inPin;
#endif

	level = _level;

	Ratio = _CAL * ((ADC_REF / 1000.0) / ADC_COUNTS);
	offset = ADC_COUNTS>>1;

}

void AC_Sensor::update_offset(uint16_t sample){
      // IIR Low pass filter to center waveform,
      // alfa = 1/1024 => fc ~ 0.2Hz
	offset = offset + (sample - offset) * offset_filter;

}

void SignalNode::begin(AC_Sensor * _Sensor)
{
  Sensor = _Sensor;
}

void SignalNode::calcRMS(uint16_t NUMBER_OF_SAMPLES, uint16_t sInterval)
{

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
      sample = analog_sample_db32(Sensor->inPin, Sensor->level);
      if (sample==ADC_COUNTS-1) {
        err=2; 
      }

      Sensor->update_offset(sample);
      centred = sample - Sensor->offset;
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

  RMS = Sensor->Ratio * sqrt(sum / NUMBER_OF_SAMPLES);
  
}


void PowerNode::begin(AC_Sensor * _SensorI, AC_Sensor * _SensorV, int8_t _PHASECAL)
{
  SensorI = _SensorI;
  SensorV = _SensorV;
  PHASECAL = _PHASECAL;

}

void PowerNode::calcVI(uint16_t NUMBER_OF_SAMPLES, uint16_t sInterval)
{

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
           PHarray[j] = analog_sample_db32(SensorV->inPin, SensorV->level);
         }
         else {
           PHarray[j] = analog_sample_db32(SensorI->inPin, SensorI->level);
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
      sampleV = analog_sample_db32(SensorV->inPin, SensorV->level);
      sampleI = analog_sample_db32(SensorI->inPin, SensorI->level);
      
      
      
      if ((sampleV==ADC_COUNTS-1) || (sampleI==ADC_COUNTS-1)) {
        err=2;
        //return ;
      }

      SensorV->update_offset(sampleV);
      SensorI->update_offset(sampleI);

      // We only substract the previously computed offset to avoid introducing noise.
      if (PHASECAL == 0) {
      	centredV = sampleV - SensorV->offset;
      	centredI = sampleI - SensorI->offset;
      }
      else if (PHASECAL > 0){
      	centredV = PHarray[j] - SensorV->offset; // Use the old value in PHarray
      	centredI = sampleI - SensorI->offset;
      	PHarray[j] = sampleV; // Replace the value in PHarray
      }
      else {
      	centredV = sampleV - SensorV->offset;
      	centredI = PHarray[j] - SensorI->offset;
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
  
  Vrms = SensorV->Ratio * sqrt(sumV / NUMBER_OF_SAMPLES);
  Irms = SensorI->Ratio * sqrt(sumI / NUMBER_OF_SAMPLES);
  realPower = SensorV->Ratio * SensorI->Ratio * sumP / NUMBER_OF_SAMPLES;
  
  //Sarr->Offset[inPinI] = offsetI;
  //Sarr->Offset[inPinV] = offsetV;
}

float linterp(uint16_t y, uint32_t dx, uint16_t y0, uint16_t y1) {
  // Linear interpolation
  // function edited and optimised specifically for our purpose here.
  // Does not apply to the general case
  // x0 is zero here.
  return (float) dx * ((float)y - (float)y0) / ((float)y1 - (float)y0);
}


void FrequencyNode::begin(AC_Sensor * _Sensor)
{
  Sensor = _Sensor;

}

void FrequencyNode::calcFreq( uint16_t xpT)
{

  //offset = ADC_COUNTS >> 1;

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
    sampleR1 = analog_sample_db32(Sensor->inPin, Sensor->level);
    timeR1 = micros();
    if (sampleR1 > (Sensor->offset + Threshold) or sampleR1 < (Sensor->offset - Threshold)) {
      sign_1 = (sampleR1 > Sensor->offset);
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
    sampleR2 = analog_sample_db32(Sensor->inPin, Sensor->level);
    timeR2 = micros();
    sign_2 = (sampleR2 > Sensor->offset);
    if (sign_1 != sign_2) {
      ta = timeR1;
      ta_delta = linterp( Sensor->offset, (timeR2 - timeR1), sampleR1, sampleR2);

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
    sampleR1 = analog_sample_db32(Sensor->inPin, Sensor->level);
    timeR1 = micros();
    sign_1 = (sampleR1 > Sensor->offset);
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
    sampleR2 = analog_sample_db32(Sensor->inPin, Sensor->level);
    timeR2 = micros();
    sign_2 = (sampleR2 > Sensor->offset);
    if (sign_1 != sign_2) {
      tb = timeR1;
      tb_delta = linterp( Sensor->offset, (timeR2 - timeR1), sampleR1, sampleR2);

      T = (float)(tb - ta) + tb_delta - ta_delta;
      break;
    }
    else {
      timeR1 = timeR2;
      sampleR1 = sampleR2;
    }

    if ((timeR2 - timer1) > (xpT*1.5)) { // Exit if we've been waiting more than 1.5 * period.
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
  

 
}

void ThreeWattMeter::begin(AC_Sensor * _SensorI1, AC_Sensor * _SensorV1,\
				 AC_Sensor * _SensorI2, AC_Sensor * _SensorV2,\
				 AC_Sensor * _SensorI3, AC_Sensor * _SensorV3,\
				 int8_t _PHASECAL)
{
  SensorI1 = _SensorI1;
  SensorV1 = _SensorV1;
  SensorI2 = _SensorI2;
  SensorV2 = _SensorV2;
  SensorI3 = _SensorI3;
  SensorV3 = _SensorV3;
  
  PHASECAL = _PHASECAL;
  
}

void ThreeWattMeter::calcVI(uint16_t NUMBER_OF_SAMPLES, uint16_t sInterval)
{

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
           PHarray_1[j] = analog_sample_db32(SensorV1->inPin, SensorV1->level);;
           PHarray_2[j] = analog_sample_db32(SensorV2->inPin, SensorV2->level);
           PHarray_3[j] = analog_sample_db32(SensorV3->inPin, SensorV3->level);
         }
         else {
           PHarray_1[j] = analog_sample_db32(SensorI1->inPin, SensorI1->level);
           PHarray_2[j] = analog_sample_db32(SensorI2->inPin, SensorI2->level);
           PHarray_3[j] = analog_sample_db32(SensorI3->inPin, SensorI3->level);
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

      sampleI1 = analog_sample_db32(SensorI1->inPin, SensorI1->level);
      sampleV1 = analog_sample_db32(SensorV1->inPin, SensorV1->level);
      sampleI2 = analog_sample_db32(SensorI2->inPin, SensorI2->level);
      sampleV2 = analog_sample_db32(SensorV2->inPin, SensorV2->level);
      sampleI3 = analog_sample_db32(SensorI3->inPin, SensorI3->level);
      sampleV3 = analog_sample_db32(SensorV3->inPin, SensorV3->level);

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

      SensorV1->update_offset(sampleV1);
      SensorI1->update_offset(sampleI1);
      SensorV2->update_offset(sampleV2);
      SensorI2->update_offset(sampleI2);
      SensorV3->update_offset(sampleV3);
      SensorI3->update_offset(sampleI3);

	// We only substract the previously computed offset to avoid introducing noise.
      if (PHASECAL == 0) {
      	centredV1 = sampleV1 - SensorV1->offset;
      	centredI1 = sampleI1 - SensorI1->offset;
      	centredV2 = sampleV2 - SensorV2->offset;
      	centredI2 = sampleI2 - SensorI2->offset;
      	centredV3 = sampleV3 - SensorV3->offset;
      	centredI3 = sampleI3 - SensorI3->offset;
      }
      else if (PHASECAL > 0){
      	centredV1 = PHarray_1[j] - SensorV1->offset; // Use the old value in PHarray
      	centredI1 = sampleI1 - SensorI1->offset;
      	centredV2 = PHarray_2[j] - SensorV2->offset; // Use the old value in PHarray
      	centredI2 = sampleI2 - SensorI2->offset;
      	centredV3 = PHarray_3[j] - SensorV3->offset; // Use the old value in PHarray
      	centredI3 = sampleI3 - SensorI3->offset;
      	PHarray_1[j] = sampleV1; // Replace the value in PHarray
      	PHarray_2[j] = sampleV2;
      	PHarray_3[j] = sampleV3;
      }
      else {
      	centredV1 = sampleV1 - SensorV1->offset;
      	centredI1 = PHarray_1[j] - SensorI1->offset;
      	centredV2 = sampleV2 - SensorV2->offset;
      	centredI2 = PHarray_2[j] - SensorI2->offset;
      	centredV3 = sampleV3 - SensorV3->offset;
      	centredI3 = PHarray_3[j] - SensorI3->offset;
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

  V1rms = SensorV1->Ratio * sqrt(sumV1 / NUMBER_OF_SAMPLES);
  I1rms = SensorI1->Ratio * sqrt(sumI1 / NUMBER_OF_SAMPLES);
  P1 = SensorV1->Ratio * SensorI1->Ratio * sumP1 / NUMBER_OF_SAMPLES;

  V2rms = SensorV2->Ratio * sqrt(sumV2 / NUMBER_OF_SAMPLES);
  I2rms = SensorI2->Ratio * sqrt(sumI2 / NUMBER_OF_SAMPLES);
  P2 = SensorV2->Ratio * SensorI2->Ratio * sumP2 / NUMBER_OF_SAMPLES;
  
  V3rms = SensorV3->Ratio * sqrt(sumV3 / NUMBER_OF_SAMPLES);
  I3rms = SensorI3->Ratio * sqrt(sumI3 / NUMBER_OF_SAMPLES);
  P3 = SensorV3->Ratio * SensorI3->Ratio * sumP3 / NUMBER_OF_SAMPLES;

  
 
}


uint16_t analog_sample_db32(uint8_t channel, uint8_t level) {
#if defined __AVR_DB__
	uint16_t adc_val;

	if (level==0){
		return analogRead(channel);
	}
	else {
  
	    //MCP3208_SS1_ON;
		SS_PORT.OUT &= ~_BV(level+1);

		SPI_PORT.OUT |= _BV(MOSI_BIT);
		CYCLE_CLOCK
		CYCLE_CLOCK

		// setup bits to be written
		for (int i = 2; i >= 0; i--) {
			if (channel & _BV(i)) {
				SPI_PORT.OUT |= _BV(MOSI_BIT);
			} else {
				SPI_PORT.OUT &= ~_BV(MOSI_BIT);
			}
			CYCLE_CLOCK
		}
		CYCLE_CLOCK
		CYCLE_CLOCK


		adc_val = 0;
		//read bits from adc
		for (int i = 11; i >= 0; i--) {
			if (SPI_PORT.IN & _BV(MISO_BIT)) {
				adc_val += _BV(i);
			}
			CYCLE_CLOCK
		}

		SS_PORT.OUT |= _BV(level+1);
	  
		return adc_val;
	  }

#else if defined __AVR_ATmega328P__
	uint8_t commandbits = 0b01100000 | ((channel & 0b111) << 2);
  
	if (level == 10) {
		MCP3208_10_ON
	}
	else if (level == 6){
		MCP3208_06_ON
	}
	else if (level == 7){
		MCP3208_07_ON
	}
	else if (level == 8){
		MCP3208_08_ON
	}
	else if (level == 9){
		MCP3208_09_ON
	}

	(void) SPI.transfer(commandbits);
	uint8_t b1 = SPI.transfer(0);
	uint8_t b2 = SPI.transfer(0);
  
  
	if (level == 10) {
		MCP3208_10_OFF
	}
	else if (level == 6){
		MCP3208_06_OFF
	}
	else if (level == 7){
		MCP3208_07_OFF
	}
	else if (level == 8){
		MCP3208_08_OFF
	}
	else if (level == 9){
		MCP3208_09_OFF
	}
  
	return (b1 << 4) | (b2 >> 4);
#endif
}










