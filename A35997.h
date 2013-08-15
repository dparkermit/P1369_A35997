/*
  -------------------------------------------
  This file contains configuration data specific to the A35997-000
  
  Dan Parker
  2013-07-03


  Summary of control Loop

  MAIN LOOP
    Check for Serial Commands
    Every 10ms (TMR2) Check for faults (need to run this on 10ms Clock so that faults can be accurately timed)
    Manages the state
    Sets all the LEDs


  EXTERNAL ADCs
    These are also continuously sampling
    Not sure exactly how to manage and filter this
    SPI interrupt driven, Highest Priority with shadow registers

    (*) SPI Interrupt Saves SPIBUF to Data 2, CS2->High, Convert->High. Convert->Low, Exits = 16 bit delay + execution time  
    (*) SPI Interrupt CS1->Low and starts 16 bit transfer, exits = 16 bit delay + execution time
    (*) SPI Interrupt Saves SPIBUF to Data 1, CS1->high, CS2->Low, exits = 16 bit delay + execution time
    (*) Repeat 

    The speed is 48 spi clock cycles + instruction cycles (lets guess 100) 
    Assuming 10MHz SPI and 30MHz Clock then maybe 123 KHz on a good day
  

  PID UPDATE INTERRUPT
    This uses TMR1 to generate the 3rd highest priority interrupt   
    The PID update runs at 10KHz (this may be adjusted at a later date)
    If PIN_RF_ENABLE is high & Software_OK_FOR_RF, set PIN_ENABLE_RF_AMP and set the PID target to the power level from customer (filtered of course)
    If Foldback bit is set, limit target to 250 Watts
    If PIN_RF_ENABLE is low, clear PIN_ENABLE_RF_AMP and set the PID target to zero, zero the output
    Calculate the new output.
    It then writes the output to the DAC over the SPI bus (should this be done inside the interrupt or outside the interrupt???)
    It may be necessary to make writing to the DAC asynchronous (ugh . . . new module)
    


    
  SERIAL INTERRUPTS
    Automatically read serial data in/out of the input/output buffers
    Commands however are executed in the main loop


  STATES
    STARTUP FROM RESET (Setup the Processor)
    Startup LED FLASH (Flash ALL LEDs)
    OFF (Green LED)
    RF ACTIVE (BLUE LED)
    FOLDBACK (FLASHING BLUE LED)
    FAULT - OVER TEMP (FLASHING RED LED)
    FAULT - ALL OTHER (RED LED)

  --------------------------------------------
*/

#ifndef __A35997_H
#define __A35997_H


#include <p30F6014a.h>
#include <dsp.h>
#include <spi.h>
#include <uart.h>
#include <timer.h>
#include <adc12.h>
#include "A35997_PINS.h"
#include "LTC2656.h"
#include "ETM_BUFFER_BYTE_64.h"
#include "ETM_DSP_FUNCTIONS.h"
#include "serial_A35997.h"
#include "faults_A35997.h"
#include "A35997_CONFIG.h"
#include "version_A35997.h"

/* 
   -------------- Resource Summary  -------------------
   TMR1 - is used to time the PID Interrupt (100us)
   TMR2 - is used to generate 10mSTicToc (fault timing)
   TMR3 - UNUSED
   TMR4 - UNUSED
   TMR5 - UNUSED



   INT1 - UNUSED
   INT2 - UNUSED
   INT3 - UNUSED
   INT4 - UNUSED

   Interupts (From Highest to Lowest Priority)
   * SPI2 (External ADC)
   * Internal ADC
   * TMR1 - PID Interrupt
   * U1RX
   * U1TX
   
   
   
   ------------- END Resource Summary ------------------------- 
*/ 




/* ------------------------------ CLOCK AND TIMING CONFIGURATION ------------------------- */
#define FCY_CLK                    29495000      // 29.495 MHz
#define FCY_CLK_MHZ                29.495        // 29.495 MHz

#define UART1_BAUDRATE             122000        // U1 Baud Rate




// -------------------------------------------- INTERNAL MODULE CONFIGURATION --------------------------------------------------//

/* 
   --- SPI1 Port ---
   This SPI port is used to connect to the octal DAC
*/
#define A35997_SPI1CON_VALUE  (FRAME_ENABLE_OFF & ENABLE_SDO_PIN & SPI_MODE16_ON & SPI_SMP_ON & SPI_CKE_ON & SLAVE_ENABLE_OFF & CLK_POL_ACTIVE_HIGH & MASTER_ENABLE_ON)
#define A35997_SPI1STAT_VALUE (SPI_ENABLE & SPI_IDLE_CON & SPI_RX_OVFLOW_CLR)   

/*
  --- SPI2 Port --- 
  This SPI port is used to connect with Isolated 16 Bit DACs
*/
#define A35997_SPI2CON_VALUE  (FRAME_ENABLE_OFF & DISABLE_SDO_PIN & SPI_MODE16_ON & SPI_SMP_ON & SPI_CKE_ON & SLAVE_ENABLE_OFF & CLK_POL_ACTIVE_HIGH & MASTER_ENABLE_ON)
#define A35997_SPI2STAT_VALUE (SPI_ENABLE & SPI_IDLE_CON & SPI_RX_OVFLOW_CLR)   


#if FCY_CLK == 29495000
// SPI1 Clock (divide by 4) 7.37MHz
// SPI2 Clock (divide by 4) 7.37MHz
#define A35997_SPI1CON_CLOCK (SEC_PRESCAL_1_1 & PRI_PRESCAL_4_1)
#define A35997_SPI2CON_CLOCK (SEC_PRESCAL_1_1 & PRI_PRESCAL_4_1)

#else
// FCY_CLK = 10000000
// SPI1 Clock (divide by 1) 10MHz
// SPI2 Clock (divide by 1) 10MHz
#define A35997_SPI1CON_CLOCK (SEC_PRESCAL_1_1 & PRI_PRESCAL_1_1)
#define A35997_SPI2CON_CLOCK (SEC_PRESCAL_1_1 & PRI_PRESCAL_1_1)

#endif



/* 
   --- UART 1 setup ---
   See uart.h and Microchip documentation for more information about the condfiguration

*/
#define A35997_U1MODE_VALUE        (UART_EN & UART_IDLE_STOP & UART_DIS_WAKE & UART_DIS_LOOPBACK & UART_DIS_ABAUD & UART_NO_PAR_8BIT & UART_1STOPBIT)
#define A35997_U1STA_VALUE         (UART_INT_TX & UART_TX_PIN_NORMAL & UART_TX_ENABLE & UART_INT_RX_CHAR & UART_ADR_DETECT_DIS)
#define A35997_U1BRG_VALUE         (((FCY_CLK/UART1_BAUDRATE)/16)-1)


/*
  --- Timer1 Setup ---
  Period of 100uS
*/
#define A35997_T1CON_VALUE             (T1_OFF & T1_IDLE_CON & T1_GATE_OFF & T1_PS_1_1 & T1_SOURCE_INT)
#define A35997_TMR1_PERIOD_US          100
#define A35997_PR1_VALUE               (FCY_CLK_MHZ*A35997_TMR1_PERIOD_US)



/*
  --- Timer2 Setup ---
  Period of 10mS
*/
#define A35997_T2CON_VALUE             (T2_OFF & T2_IDLE_CON & T2_GATE_OFF & T2_PS_1_8 & T2_32BIT_MODE_OFF & T2_SOURCE_INT)
#define A35997_TMR2_PERIOD_US          10000
#define A35997_PR2_VALUE               (FCY_CLK_MHZ*A35997_TMR2_PERIOD_US/8)



/* 
   --- 12-BIT ADC Configuration ---
   Goal, when setup, the system will scan through the selected Analog channels and sample
   
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
   AN14 - Power Level From Cust   - 8 - This needs to be read with every other sample


   The internal ADC is set up to sample
   (BUF0) AN2  - Detector #4 (Reverse B) - 0
   (BUF1) AN14 - Power Level From Customer
   (BUF2) AN3 - Detector #3 (Reverse A)  - 1 
   (BUF3) AN14 - Power Level From Customer
   (BUF4) AN6 - Amplifier Temperature    - 2
   (BUF5) AN14 - Power Level From Customer
   (BUF6) AN8 - Detector #1 (Forward A) Temperature  - 4
   (BUF7) AN14 - Power Level From Customer
   (BUF8) AN9 - Detector #2 (Forward B) Temperature  - 5
   (BUF9) AN14 - Power Level From Customer
   (BUFA) AN10 - Detector #3 (Reverse A) Temperature - 6
   (BUFB) AN14 - Power Level From Customer
   (BUFC) AN11 - Detector #4 (Reverse B) Temperature - 7
   (BUFD) AN14 - Power Level From Customer
   (BUFE) AN14 - Power Level From Customer
   (BUFF) UNUSED


*/



#define A35997_ADCON1_VALUE (ADC_MODULE_OFF & ADC_IDLE_STOP & ADC_FORMAT_INTG & ADC_CLK_AUTO & ADC_AUTO_SAMPLING_ON)
#define A35997_ADCON2_VALUE (ADC_VREF_EXT_EXT & ADC_SCAN_ON & ADC_SAMPLES_PER_INT_15 & ADC_ALT_BUF_OFF & ADC_ALT_INPUT_ON)
#define A35997_ADCHS_VALUE  (ADC_CH0_POS_SAMPLEA_AN2 & ADC_CH0_NEG_SAMPLEA_VREFN & ADC_CH0_POS_SAMPLEB_AN14 & ADC_CH0_NEG_SAMPLEB_VREFN)
#define A35997_ADPCFG_VALUE (ENABLE_AN2_ANA & ENABLE_AN3_ANA & ENABLE_AN4_ANA & ENABLE_AN5_ANA & ENABLE_AN6_ANA & ENABLE_AN7_ANA & ENABLE_AN8_ANA & ENABLE_AN9_ANA & ENABLE_AN10_ANA & ENABLE_AN11_ANA & ENABLE_AN14_ANA)
#define A35997_ADCSSL_VALUE (SKIP_SCAN_AN0 & SKIP_SCAN_AN1 & SKIP_SCAN_AN4 & SKIP_SCAN_AN5 & SKIP_SCAN_AN7 & SKIP_SCAN_AN12 & SKIP_SCAN_AN13 & SKIP_SCAN_AN15 )

// These settings generate a sample rate of 180 KHz
// With 15 samples per interrupt this yields a sample rate on the individual signals of 12 KHz (96KHz on the program signal)
#if FCY_CLK ==  29495000
// Experiments find that we need a sample time of at least 2 conversion clocks for the reading to settle
#define A35997_ADCON3_VALUE (ADC_SAMPLE_TIME_2 & ADC_CONV_CLK_SYSTEM & ADC_CONV_CLK_10Tcy)

#else
// FCY_CLK = 10000000
#define A35997_ADCON3_VALUE (ADC_SAMPLE_TIME_2 & ADC_CONV_CLK_SYSTEM & ADC_CONV_CLK_7Tcy2)

#endif




// ----------- Data Structures ------------ //

typedef struct {
  unsigned int adc_reading_calibrated;                  // RAM - This is the adc_reading after digital filtering and adc calibration
  unsigned int adc_temperature;               // This is the temperature of the control board used adjust the adc reading. 
  unsigned int adc_cal_gain;                            // EEPROM CONSTANT - Used to calibrate the board for gain errors (Resistor tolerances, Gain stage erros, dac gain, ect)
                                                        // The constant is a 16 bit range from 0->2 (or 2 minus (1 16 bit LSB))
  signed int adc_cal_gain_thermal_adjust;               // EEPROM_CONSTANT - Used to calibrate the previous value for thermal changes from 50*C
  signed int adc_cal_offset;                            // EEPROM CONSTANT - Used to calibrate the board for DC offsets. (adc offset, op-amp offsets)
  signed int adc_cal_offset_thermal_adjust;             // EEPROM_CONSTANT - Used to calibrate the previous value for thermal changes from 50*C

  unsigned int detector_temperature;                    // RAM - This is the current temperature of the detector (in K) as determined from the ADC reading.
                                                        // This adc reading is not calibrated
  unsigned int detector_level_calibrated;               // RAM - calibrats the adc reading for the detector device and temperature characteristics
                                                        // DPARKER, not sure how this is going to be implemented yet.  Probably a table with offset/ramp from different power ranges
                                                        // The reverse detectors do not bother to calibrate the level

  unsigned int power_reading_centi_watts;               // RAM - This is detector_level_calibrated converted to watts.

  unsigned int max_power;                               // CONSTANT - Loaded at initialization
  unsigned int over_max_power_count;                    // RAM - This counts how long it has been over powered for
  unsigned int over_power_trip_time;                    // CONSTANT - this is how long (in 10ms units) an over power condition must exist to cause a fault


  unsigned int detector_scale_factor;
  long detector_intercept_milli_dB;

  unsigned int pad_attenuation_milli_dB;
  unsigned int coupler_attenuation_milli_dB;

  unsigned int serial_number;
  unsigned int calibration_date;
  


} RF_DETECTOR;




/*
  --- Public Functions ---
*/

void DoA35997StateMachine(void);
/*
  This is called to update the FSM
  The state is stored as a global variable so this function has no argument and returns no value
*/




/*
  --- Gobal Variales ---
*/
extern RF_DETECTOR forward_power_detector_A;
extern RF_DETECTOR forward_power_detector_B;
extern RF_DETECTOR reverse_power_detector_A;
extern RF_DETECTOR reverse_power_detector_B;
extern RF_DETECTOR program_power_level;
extern tPID pid_forward_power;
extern fractional pid_forward_power_kCoeffs[];

extern unsigned int control_state;

extern volatile unsigned int total_forward_power_centi_watts;
extern volatile unsigned int total_reverse_power_centi_watts;
extern volatile unsigned int rf_amplifier_dac_output;
extern volatile unsigned int last_valid_detector_A_adc_reading;
extern volatile unsigned int last_valid_detector_B_adc_reading;

extern unsigned int software_foldback_mode_enable;
extern unsigned int LTC2656_write_error_count;
extern unsigned int serial_link_power_target;


extern unsigned int gui_debug_value_1;
extern unsigned int gui_debug_value_2;
extern unsigned int gui_debug_value_3;
extern unsigned int gui_debug_value_4;


/*
  --- STATE DEFINITIONS ---
*/

#define STATE_START_UP                       0x01
#define STATE_FLASH_LEDS_AT_STARTUP          0x05

#define STATE_RF_OFF                         0x11

#define STATE_RF_ON                          0x21
#define STATE_RF_ON_FOLDBACK                 0x25

#define STATE_FAULT_GENERAL_FAULT            0x31
#define STATE_FAULT_OVER_TEMP                0x35
#define STATE_STARTUP_FAILURE                0x37



// Fixed Scale factors
#define PROGRAM_LEVEL_TO_CENTI_WATTS_SCALE_FACTOR           54272           // 53 Watts per volt 
#define DETECTOR_TEMPERATURE_ADC_READING_TO_DECI_DEGREES_K  10240           // .15625


// Front Panel LED States
#define SOLID_GREEN                1
#define SOLID_BLUE                 2
#define FLASH_BLUE                 3
#define SOLID_RED                  4
#define FLASH_RED                  5
#define FLASH_ALL_SERIES           6

#define _ISRFASTNOPSV __attribute__((interrupt, shadow, no_auto_psv)) 
#define _ISRNOPSV __attribute__((interrupt, no_auto_psv)) 




#endif
