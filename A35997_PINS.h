#ifndef __A35997_PINS_H
#define __A35997_PINS_H

#include <p30f6014a.h>


#define TRIS_OUTPUT_MODE 0
#define TRIS_INPUT_MODE  1


// This is fixing an error on the schematic.  These pins MUST BE INPUTS
#define TRIS_MAKE_ME_INPUT_1                 _TRISD0
#define TRIS_MAKE_ME_INPUT_2                 _TRISD11



// Test Point Output Pins
#define PIN_TEST_POINT_24_CLK_OUT            _LATC15
#define TRIS_PIN_TEST_POINT_24_CLK_OUT       _TRISC15

#define PIN_TEST_POINT_25                    _LATA12
#define TRIS_PIN_TEST_POINT_25               _TRISA12

#define PIN_TEST_POINT_26                    _LATA13
#define TRIS_PIN_TEST_POINT_26               _TRISA13

#define PIN_TEST_POINT_27                    _LATB12
#define TRIS_PIN_TEST_POINT_27               _TRISB12

#define PIN_TEST_POINT_28                    _LATB13
#define TRIS_PIN_TEST_POINT_28               _TRISB13

#define PIN_TEST_POINT_29                    _LATG2
#define TRIS_PIN_TEST_POINT_29               _TRISG2
#define OLL_TP29_FOLDBACK_ON                 0

#define PIN_TEST_POINT_30                    _LATG3
#define TRIS_PIN_TEST_POINT_30               _TRISG3
#define OLL_TP_30_MAX_ATTENUATION            1


// Test LED Pins Output Pins
#define TEST_LED_ON                          0
#define TEST_LED_OFF                         1

#define PIN_TEST_LED_1                       _LATC14
#define TRIS_PIN_TEST_LED_1                  _TRISC14

#define PIN_TEST_LED_2                       _LATC13
#define TRIS_PIN_TEST_LED_2                  _TRISC13

#define PIN_TEST_LED_3                       _LATG8
#define TRIS_PIN_TEST_LED_3                  _TRISG8

#define PIN_TEST_LED_4                       _LATG9
#define TRIS_PIN_TEST_LED_4                  _TRISG9


// Front Panel LED Output Pins
#define OLL_FRONT_PANEL_LED_ON               1
#define OLL_FRONT_PANEL_LED_OFF              0

#define PIN_FRONT_PANEL_LED_GREEN            _LATD10
#define TRIS_PIN_FRONT_PANEL_LED_GREEN       _TRISD10

#define PIN_FRONT_PANEL_LED_RED              _LATD9
#define TRIS_PIN_FRONT_PANEL_LED_RED         _TRISD9

#define PIN_FRONT_PANEL_LED_BLUE             _LATD8
#define TRIS_PIN_FRONT_PANEL_LED_BLUE        _TRISD8


// Digital Outputs
#define PIN_SUM_FLT                          _LATG15
#define TRIS_PIN_SUM_FLT                     _TRISG15
#define OLL_PIN_SUM_FAULT_FAULTED            0                        

#define PIN_RS422_DE                         _LATC1
#define TRIS_PIN_RS422_DE                    _TRISC1
#define OLL_PIN_RS422_DE_ENABLE_TRANSMIT     1

#define PIN_ENABLE_RF_AMP                    _LATA15
#define TRIS_PIN_ENABLE_RF_AMP               _TRISA15
#define OLL_PIN_ENABLE_RF_AMP_ENABLED        1

// High Speed ADC Digital Control Outputs
#define PIN_AD7686_CONVERT                   _LATC2
#define TRIS_PIN_AD7686_CONVERT              _TRISC2
#define OLL_AD7686_START_CONVERSION          1
             
#define PIN_AD7686_1_CS                      _LATC4
#define TRIS_PIN_AD7686_1_CS                 _TRISC4
#define OLL_AD7686_SELECT_DEVICE             0

#define PIN_AD7686_2_CS                      _LATC3
#define TRIS_PIN_AD7686_2_CS                 _TRISC3


// Digital Inputs
#define PIN_PS_ONE_OK                        _RF4
#define TRIS_PIN_PS_ONE_OK                   _TRISF4
#define ILL_PS_OK                            1

#define PIN_PS_TWO_OK                        _RF5
#define TRIS_PIN_PS_TWO_OK                   _TRISF5

#define PIN_RF_ENABLE                        _RA14       // This is also INT3
#define TRIS_PIN_RF_ENABLE                   _TRISA14
#define ILL_PIN_RF_ENABLE_ENABLED            1




// DAC - These pins are managed by the LTC2656 Module and are provided for reference
/*
  DAC_CLR_NOT - B15
  DAC_LDAC_NOT - D14
  DAC_CS_NOT - D15
*/



// Analog Pins - These are managed by the ADC module but provided here for reference
/*
  AN2 - Detector #4 (Reverse 2)  - 0
  AN3 - Detector #3 (Reverse 1)  - 1 
  AN4 - Detector #2 (Forward 2)  - Read By External ADC
  AN5 - Detector #1 (Forward 1)  - Read By External ADC
  AN6 - Amplifier Temperature    - 2
  AN7 - Spare                    - 3
  AN8 - Detector #1 Temperature  - 4
  AN9 - Detector #2 Temperature  - 5
  AN10 - Detector #3 Temperature - 6
  AN11 - Detector #4 Temperature - 7
  AN14 - Power Level From Cust   - 8 
*/


#endif
