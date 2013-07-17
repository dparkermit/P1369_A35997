#ifndef __ETM_DSP_FUNCTIONS_H
#define __ETM_DSP_FUNCTIONS_H

unsigned int AverageADC128(unsigned int* array_to_avg);
/*
  This function takes a pointer to an array of 12 bit adc values and averages.
  The result is a 16 bit number.
  Effectivly Sum[0->127] / 8
  If the array is not at least 128 unsigned integers long, bad things will happen
*/

unsigned int AverageADC16(unsigned int* array_to_avg);
/*
  This function takes a pointer to an array of 16 bit adc values and averages.
  The result is a 16 bit number.
  Effectivly Sum[0->16] / 16
  If the array is not at least 16 unsigned integers long, bad things will happen
*/


unsigned int Scale13Q3(unsigned int value, unsigned int scale);

unsigned int MakeScale13Q3(unsigned int num, unsigned int den);

unsigned int Scale16Bit(unsigned int value, unsigned int num, unsigned int den);  // DPARKER - this needs more descriptive name


unsigned int ETMScale16Bit(unsigned int value, unsigned int fractional_multiplier, unsigned int shift_left_bits);
/*
  This is a fast 16 bit scale function - 9 cycles plus call cycles
  This scales "value" by "fractional_multiplier" and then shift the results left "shift_left_bits"
  The total scale factor = fractional_multiplier * 2^shift_left_bits.
  fractional_multiplier has range 0-.999985 so to get a gain of exactly 1 use fractional_multiplier = .5(0x8000) and shift_left_bits = 1;
*/

unsigned int RCFilterNTau(unsigned int previous_value, unsigned int reading, unsigned int FILTER_TAU_BITS);
/*
  This performs a fast "almost RC" filter where Tau is 2^FILTER_TAU_BITS samples.
  The max value of FILTER_TAU_BITS is 15

  If "previous_value" is zero, this is a special case where the output will be set to reading.  
  This is so that when this is first called (previous value = zero), the filtered value will not start at zero (which could take a long time to reach correct value)
*/
#define RC_FILTER_2_TAU   1
#define RC_FILTER_4_TAU   2
#define RC_FILTER_8_TAU   3
#define RC_FILTER_16_TAU  4
#define RC_FILTER_32_TAU  5
#define RC_FILTER_64_TAU  6
#define RC_FILTER_128_TAU 7
#define RC_FILTER_256_TAU 8
#define RC_FILTER_512_TAU 9


extern unsigned int saturation_scale13Q3_count;
extern unsigned int saturation_makescale13Q3_count;
extern unsigned int saturation_scale16_count;
extern unsigned int etm_scale16bit_saturation_count;
#endif
