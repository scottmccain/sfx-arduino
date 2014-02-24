//////////////////////////////////////////////////////////////////////////////
//
// This file is part of funkGenie.
// 
//  Copyright 2013 Mark T
//  Copyright 2014 Kari Brown
// 
//  funkGenie is free software: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License as published
//  by the Free Software Foundation, either version 3 of the License, or (at
//  your option) any later version.
//
//  funkGenie is distributed in the hope that it will be useful, but
//  WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
//  or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
//  License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  along with funkGenie.
//  If not, see <http://www.gnu.org/licenses/>.
//
//////////////////////////////////////////////////////////////////////////////

#include "stdlib.h"
#include "math.h"

// AD/DA Globals
const unsigned int sampleRate =  48000;
const unsigned int sampleBits =  12;

// Function generator modes
const unsigned int SINEWAVE =    0;
const unsigned int SQUARE =      1;
const unsigned int TRIANGLE =    2;
const unsigned int SAWTOOTH =    3;

// Global variables
unsigned int tableSize, tableSize2, tableSize3, tableSize4; // Table size and multiples
unsigned int idxMask, maxVal;                               // Index mask and maximum value of DAC output
float degPerSample, timePerSample;                          // Degrees per sample + time per sample
volatile float dac_hz, dac_t, dac_degreesPerStep;           // Frequency, cycle time + degrees per sample
volatile unsigned int dac_mode, samplesPerCycle, halfCycle; // DAC mode, samples per cycle and samples per half cycle
volatile int cycle_count = 0;                               // Cycle counter
unsigned int *iTable = NULL;                                // Wavetable storage

// Circular buffer, power of two.
#define BUFSIZE 0x400
#define BUFMASK 0x3FF
volatile int samples[BUFSIZE];
volatile int sptr = 0;

// Interpolates between a and b based on x (0.0... 1.0)
inline float linterp(float a, float b, float x)
{
  return  a * (1.0f - x) + b * x;
}

// Initialises global variables and allocates memory for wavetable
int initGlobals()
{
  tableSize = pow(2, sampleBits - 1);
  tableSize2 = 2 * tableSize;
  tableSize3 = 3 * tableSize;
  tableSize4 = 4 * tableSize;
  idxMask = 4 * tableSize - 1;
  maxVal = tableSize2 - 1;
  degPerSample = (float)tableSize / 90.0f;
  timePerSample = 1.0f / (float)sampleRate;
 
  if(iTable)
    free(iTable);

  iTable = (unsigned int*)malloc(tableSize * sizeof(unsigned int));

  setFrequency(197.3f);
  
  return 0;
}

// Sets the frequency of the function generator and updates relevan globals
int setFrequency(float f)
{
  dac_hz = f;
  dac_t = 1.0f / dac_hz;
  dac_degreesPerStep = 360.0f / (dac_t / timePerSample);
  samplesPerCycle = dac_t / timePerSample;
  halfCycle = samplesPerCycle / 2;
  
  return 0;
}

// Initialises the wavetable with quarter of sine wave
// Square, triangle and sawtooth waves are not wavetable based in this example
int initTable(unsigned int mode)
{
  dac_mode = mode;
  if(dac_mode == SQUARE || dac_mode == SAWTOOTH)
    return 0;

  TcChannel *t = &(TC0->TC_CHANNEL)[0];                // pointer to TC0 registers for its channel 0
  t->TC_CCR = TC_CCR_CLKDIS;                           // disable internal clocking

  for(unsigned int x = 0; x < tableSize; x++)
  {
    switch(dac_mode)
    {
      case SINEWAVE:
        iTable[x] = floor(((sin(((float)x / (float)tableSize) * HALF_PI) + 1.0f) * tableSize));
        break;
      default:
        iTable[x] = tableSize;
        return 1;
    }
  }
  t->TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;             // re-enable local clocking and switch to hardware trigger source.
  return 0;
}

// Returns a sample from wavetable flipping and reversing as necessary
inline unsigned int getSample(unsigned int idx)
{
  idx = idx & idxMask;
  
  if(idx < tableSize)
    return iTable[idx];
  if(idx < tableSize2)
    return iTable[tableSize - (idx - tableSize + 1)];
  if(idx < tableSize3)
    return tableSize2 - iTable[idx - tableSize2];
  else
    return tableSize2 - iTable[tableSize - (idx - tableSize3 + 1)];
}

// Returns interpolated sample based on angle
inline unsigned int iFunc(float deg)
{
  float fIdx1 = deg * degPerSample;
  float fIdx2 = (deg + dac_degreesPerStep) * degPerSample;
  unsigned int idx1 = fIdx1;
  unsigned int idx2 = fIdx2;

  if(idx1 == idx2)
    return getSample(idx1);

  unsigned int s1 = getSample(idx1);
  unsigned int s2 = getSample(idx2);
  float frac = fIdx1 - floor(fIdx1);

  return linterp(s1, s2, frac);
}

// Power up initialisation
void setup()
{
  initGlobals();
  initTable(SINEWAVE);
  
  adc_setup();                                         // setup ADC
  
  pmc_enable_periph_clk(TC_INTERFACE_ID + 0*3+0);      // clock the TC0 channel 0

  TcChannel *t = &(TC0->TC_CHANNEL)[0];                // pointer to TC0 registers for its channel 0
  t->TC_CCR = TC_CCR_CLKDIS;                           // disable internal clocking while setup regs
  t->TC_IDR = 0xFFFFFFFF;                              // disable interrupts
  t->TC_SR;                                            // read int status reg to clear pending
  t->TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1 |             // use TCLK1 (prescale by 2, = 42MHz)
              TC_CMR_WAVE |                            // waveform mode
              TC_CMR_WAVSEL_UP_RC |                    // count-up PWM using RC as threshold
              TC_CMR_EEVT_XC0 |                        // Set external events from XC0 (this setup TIOB as output)
              TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_CLEAR |
              TC_CMR_BCPB_CLEAR | TC_CMR_BCPC_CLEAR;
  
  t->TC_RC =  (1.0 / (double)sampleRate) / (1.0 / 42000000.0); // counter resets on RC, so sets period in terms of 42MHz clock
  t->TC_RA =  t->TC_RC / 2;                            // roughly square wave
  t->TC_CMR = (t->TC_CMR & 0xFFF0FFFF) |
              TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_SET;     // set clear and set from RA and RC compares
  
  t->TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;             // re-enable local clocking and switch to hardware trigger source.

  setup_pio_TIOA0();                                   // drive Arduino pin 2 at 48kHz to bring clock out

  dac_setup();                                         // setup up DAC auto-triggered at 48kHz
}

void setup_pio_TIOA0()                                 // Configure Ard pin 2 as output from TC0 channel A (copy of trigger event)
{
  PIOB->PIO_PDR = PIO_PB25B_TIOA0;                     // disable PIO control
  PIOB->PIO_IDR = PIO_PB25B_TIOA0;                     // disable PIO interrupts
  PIOB->PIO_ABSR |= PIO_PB25B_TIOA0;                   // switch to B peripheral
}

// Setup DAC. This example uses DAC1. Uncomment the commented lines below for
// DAC0 and comment out the DAC1 related lines below them for DAC0 usage instead.
void dac_setup()
{
  pmc_enable_periph_clk(DACC_INTERFACE_ID);            // start clocking DAC
  DACC->DACC_CR = DACC_CR_SWRST;                       // reset DAC
 
  DACC->DACC_MR = 
    DACC_MR_TRGEN_EN | DACC_MR_TRGSEL(1) |             // trigger 1 = TIO output of TC0
    //(0 << DACC_MR_USER_SEL_Pos) |                      // select channel 0
    (1 << DACC_MR_USER_SEL_Pos) |                      // select channel 1
    DACC_MR_REFRESH (0x0F) |                           // bit of a guess... I'm assuming refresh not needed at 48kHz
    (24 << DACC_MR_STARTUP_Pos);                       // 24 = 1536 cycles which I think is in range 23..45us since DAC clock = 42MHz

  DACC->DACC_IDR = 0xFFFFFFFF;                         // no interrupts
  //DACC->DACC_CHER = DACC_CHER_CH0 << 0;                // enable chan0
  DACC->DACC_CHER = DACC_CHER_CH1 << 0;                // enable chan1
}

// Write out DAC value
inline void dac_write (int val)
{
  DACC->DACC_CDR = val & 0xFFF;
}

// AD converter setup.
void adc_setup()
{
  NVIC_EnableIRQ (ADC_IRQn);                           // enable ADC interrupt vector
  ADC->ADC_IDR = 0xFFFFFFFF;                           // disable interrupts
  ADC->ADC_IER = 0x80;                                 // enable AD7 End-Of-Conv interrupt (Arduino pin A0)
  ADC->ADC_CHDR = 0xFFFF;                              // disable all channels
  ADC->ADC_CHER = 0x80;                                // enable just A0
  ADC->ADC_CGR = 0x15555555;                           // All gains set to x1
  ADC->ADC_COR = 0x00000000;                           // All offsets off
  
  ADC->ADC_MR = (ADC->ADC_MR & 0xFFFFFFF0) |
                (1 << 1) | ADC_MR_TRGEN;               // 1 = trig source TIO from TC0
}

#ifdef __cplusplus
extern "C" 
{
#endif

void ADC_Handler(void)
{
  // Store AD conversion result in the circular buffer.
  if(ADC->ADC_ISR & ADC_ISR_EOC7)                      // ensure there was an End-of-Conversion and we read the ISR reg
  {
    int val = *(ADC->ADC_CDR+7);                       // get conversion result
    samples[sptr] = val;                               // stick in circular buffer
    sptr = (sptr + 1) & BUFMASK;                       // move pointer
  }

  if(dac_mode == SINEWAVE)
  {
      float deg = (float)cycle_count * dac_degreesPerStep;
      dac_write(iFunc(deg));
  }
  else if(dac_mode == SQUARE)
  {
      if(cycle_count <= halfCycle)
        dac_write(maxVal);
      else
        dac_write(0);
  }
  else if(dac_mode == TRIANGLE)
  {
    unsigned int ramp = cycle_count * (tableSize2 / halfCycle);
    if(cycle_count < halfCycle)
      dac_write(ramp)
    else
      dac_write(tableSize4 - ramp);
  }
  else if(dac_mode == SAWTOOTH)
  {
      dac_write(cycle_count * (tableSize2 / samplesPerCycle));
  }
  
  // Update cycle_count
  if(cycle_count < samplesPerCycle)
    cycle_count++;
  else
    cycle_count = 0;
}

#ifdef __cplusplus
}
#endif



// Main loop.
// Iterate through the wave shapes and ramp from 50Hz to 5000Hz and back again
unsigned int wave_shape = 0;
void loop()
{
  initTable(wave_shape);
  for(float f = 50.0f; f < 5000.0f; f += 5.0f)
  {
    setFrequency(f);
    delay(10);
  }
  for(float f = 5000.0f; f > 50.0f; f -= 5.0f)
  {
    setFrequency(f);
    delay(10);
  }

  wave_shape++;
  if(wave_shape > 3)
    wave_shape = 0;

}

