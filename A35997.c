#include "A35997.h"
#include "faults_A35997.h"
#include "A35997_DETECTOR_LOOK_UP_TABLE.h"

#ifdef _DO_TIME_COMP
unsigned int cycles_running;
#endif

#ifdef _DO_THERMAL_COMP
#define TEMPERATURE_TIME_CONSTANT 62914
//const unsigned int temperature_coeff_mult[64] = {32768,33423,34079,34734,34636,34505,34406,34308,34472,34636,34800,35095,34832,34570,34308,34013,34013,34013,34013,34013,34013,34013,34013,34013,34013,34013,34013,34013,34013,34013,34013,34013,34013,34013,34013,34013,34013,34013,34013,34013,34013,34013,34013,34013,34013,34013,34013,34013,34013,34013,34013,34013,34013,34013,34013,34013,34013,34013,34013,34013,34013,34013,34013,34013};

const unsigned int temperature_coeff_mult[64] = {33686,33531,33377,33224,33071,33071,33071,33071,33071,32919,32768,32768,32768,32768,32768,32768,32768,32768,32768,32768,32768,32768,32768,32768,32768,32768,32768,32768,32768,32768,32768,32768,32768,32768,32768,32768,32768,32768,32768,32768,32768,32768,32768,32768,32768,32768,32768,32768,32768,32768,32768,32768,32768,32768,32768,32768,32768,32768,32768,32768,32768,32768,32768,32768};
unsigned int coupler_temperature_scale_unknown;
unsigned int coupler_multiplier;

#endif


int GetCombinerTemperatureDelta(unsigned int temperature);
long ConverterADCReadingToMillidB(RF_DETECTOR* ptr_rf_det);


volatile unsigned int detector_A_min_value;
volatile unsigned int detector_B_min_value;
volatile unsigned int detector_A_max_value;
volatile unsigned int detector_B_max_value;

//--------- Local Function Prototypes ---------//
unsigned int ConvertAmplifierTemperatureADCtoDeciDegreesK(unsigned int adc_reading); 
unsigned int ConvertDetectorTemperatureADCtoDeciDegreesK(unsigned int adc_reading);
unsigned int ConvertProgramLevelToPowerCentiWatts(unsigned int program_level);
void DoFrontPanelLED(void);
void DoA35997StartUp(void);
void CalibrateADCReading(RF_DETECTOR* ptr_rf_det, unsigned int adc_reading);
void CalibrateDetectorLevel(RF_DETECTOR* ptr_rf_det);
void ConvertDetectorLevelToPowerCentiWatts(RF_DETECTOR* ptr_rf_det);
void Do10msTicToc(void);
void FilterADCs(void);


// --------- Constant Tables in Stored in Program Memory ---------------//
const unsigned int detector_voltage_to_power_look_up_table[4096] = {DETECTOR_LOOK_UP_TABLE_VALUES};



// Debugging Information
// DPARKER - WHERE ARE THESE USED????
unsigned int gui_debug_value_1;
unsigned int gui_debug_value_2;
unsigned int gui_debug_value_3;
unsigned int gui_debug_value_4;



// ---- Global Structures ------//
RF_DETECTOR forward_power_detector_A;
RF_DETECTOR forward_power_detector_B;
RF_DETECTOR reverse_power_detector_A;
RF_DETECTOR reverse_power_detector_B;
RF_DETECTOR program_power_level;
tPID pid_forward_power;
fractional pid_forward_power_kCoeffs[] = {0,0,0};

// ------- Local Structures ---------//
LTC2656 U6_LTC2656;
fractional pid_forward_power_controlHistory[3] __attribute__ ((section (".ybss, bss, ymemory")));
fractional pid_forward_power_abcCoefficient[3] __attribute__ ((section (".xbss, bss, xmemory")));




//---------- Global Variables --------- //
unsigned int control_state;
volatile unsigned int total_forward_power_centi_watts;
volatile unsigned int total_reverse_power_centi_watts;
volatile unsigned int rf_amplifier_dac_output;
unsigned int software_foldback_mode_enable = 0;
unsigned int LTC2656_write_error_count = 0;
unsigned int serial_link_power_target = 0;
volatile unsigned int last_valid_detector_A_adc_reading = 0xFFFF;
volatile unsigned int last_valid_detector_B_adc_reading = 0xFFFF;



//---------- Local  Variables ----------//
unsigned int power_ramp_centi_watts = 0;
unsigned int software_rf_disable = 1;
unsigned int minimum_power_to_operate = MINIMUM_POWER_TARGET;

unsigned int led_pulse_count;
unsigned int front_panel_led_startup_flash_couter;
unsigned int front_panel_led_pulse_count;
unsigned int front_panel_led_state;
unsigned int start_reset_process;


//------------- ADC Data Storage Circular Buffers ------------//
unsigned int reverse_detector_a_array[128];
unsigned int reverse_detector_b_array[128];
unsigned int rf_amplifier_temperature_array[128];
unsigned int fwd_rf_det_a_temperature_array[128];
unsigned int fwd_rf_det_b_temperature_array[128];
unsigned int rev_rf_det_a_temperature_array[128];
unsigned int rev_rf_det_b_temperature_array[128];

volatile unsigned int adc_result_index;


unsigned int forward_detector_a_array[16];
unsigned int forward_detector_b_array[16];

volatile unsigned int spi2_state = 0;
volatile unsigned int ad7686_result_index;

unsigned int *fwd_det_a_pointer = forward_detector_a_array;
unsigned int *fwd_det_b_pointer = forward_detector_b_array;



void DoA35997StateMachine(void) {
  
  switch(control_state) {
    
  case STATE_START_UP:
    DoA35997StartUp();
    control_state = STATE_FLASH_LEDS_AT_STARTUP;
    PIN_TEST_POINT_29 = !OLL_TP29_FOLDBACK_ON;
    break;

  case STATE_FLASH_LEDS_AT_STARTUP:
    PIN_TEST_POINT_29 = !OLL_TP29_FOLDBACK_ON;
    front_panel_led_startup_flash_couter = 0;
    front_panel_led_state = FLASH_ALL_SERIES;
    front_panel_led_pulse_count = 0;
    PIN_SUM_FLT = !OLL_PIN_SUM_FAULT_FAULTED;
    while (control_state == STATE_FLASH_LEDS_AT_STARTUP) {
      DoSerialCommand();
      Do10msTicToc();
      if (front_panel_led_startup_flash_couter > FRONT_PANEL_LED_NUMBER_OF_FLASHES_AT_STARTUP) {
	control_state = STATE_RF_OFF;
	ResetAllFaults();
      }
    }
    break;    
    
  case STATE_RF_OFF:
    PIN_SUM_FLT = !OLL_PIN_SUM_FAULT_FAULTED;
    PIN_TEST_POINT_29 = !OLL_TP29_FOLDBACK_ON;
    software_foldback_mode_enable = 0;
    software_rf_disable = 0; 
    front_panel_led_state = SOLID_GREEN;
    front_panel_led_pulse_count = 0;
    // We don't want to wait for the software loop to start the amplifier so this must be pre-set to the on condition.
    while (control_state == STATE_RF_OFF) {
      PIN_ENABLE_RF_AMP = !OLL_PIN_ENABLE_RF_AMP_ENABLED;
      DoSerialCommand();
      Do10msTicToc();
      if (FaultCheckOverTemp()) {
	control_state = STATE_FAULT_OVER_TEMP;
      } else if (FaultCheckGeneralFault()) {
	control_state = STATE_FAULT_GENERAL_FAULT;
      } else if (PIN_RF_ENABLE == ILL_PIN_RF_ENABLE_ENABLED) {
	control_state = STATE_RF_ON;
      }
    }
    break;
    

  case STATE_RF_ON:
    PIN_SUM_FLT = !OLL_PIN_SUM_FAULT_FAULTED;
    PIN_TEST_POINT_29 = !OLL_TP29_FOLDBACK_ON;
    PIN_ENABLE_RF_AMP = OLL_PIN_ENABLE_RF_AMP_ENABLED;
    software_foldback_mode_enable = 0;
    software_rf_disable = 0;
    front_panel_led_state = SOLID_BLUE;
    front_panel_led_pulse_count = 0;
    while (control_state == STATE_RF_ON) {
      DoSerialCommand();
      Do10msTicToc();
      if (FaultCheckOverTemp()) {
	control_state = STATE_FAULT_OVER_TEMP;
      } else if (FaultCheckGeneralFault()) {
	control_state = STATE_FAULT_GENERAL_FAULT;
      } else if (PIN_RF_ENABLE != ILL_PIN_RF_ENABLE_ENABLED) {
	control_state = STATE_RF_OFF;
      } else if (CheckReflectedPowerFault()) {
	control_state = STATE_RF_ON_FOLDBACK;
      }
    }
    break;


    
  case STATE_RF_ON_FOLDBACK:
    PIN_SUM_FLT = !OLL_PIN_SUM_FAULT_FAULTED;
    PIN_TEST_POINT_29 = OLL_TP29_FOLDBACK_ON;
    software_foldback_mode_enable = 1;
    front_panel_led_state = FLASH_BLUE;
    front_panel_led_pulse_count = 0;
    while (control_state == STATE_RF_ON_FOLDBACK) {
      DoSerialCommand();
      Do10msTicToc();
      if (FaultCheckOverTemp()) {
	control_state = STATE_FAULT_OVER_TEMP;
      } else if (FaultCheckGeneralFault()) {
	control_state = STATE_FAULT_GENERAL_FAULT;
      } else if (PIN_RF_ENABLE != ILL_PIN_RF_ENABLE_ENABLED) {
	control_state = STATE_RF_OFF;
      } else if (!CheckReflectedPowerFault()) {
	control_state = STATE_RF_ON;
      }
    }
    break;
    
    
    
  case STATE_FAULT_OVER_TEMP:
    PIN_SUM_FLT = OLL_PIN_SUM_FAULT_FAULTED;
    PIN_TEST_POINT_29 = !OLL_TP29_FOLDBACK_ON;
    software_rf_disable = 1;
    front_panel_led_state = FLASH_RED;
    front_panel_led_pulse_count = 0;
    while (control_state == STATE_FAULT_OVER_TEMP) {
      PIN_ENABLE_RF_AMP = !OLL_PIN_ENABLE_RF_AMP_ENABLED;
      DoSerialCommand();
      Do10msTicToc();
      if (FaultCheckGeneralFault()) {
	control_state = STATE_FAULT_GENERAL_FAULT;
      } else if (!FaultCheckGeneralFault() && !FaultCheckOverTemp()) {
	control_state = STATE_RF_OFF;
      }
    }
    break;
    
  case STATE_FAULT_GENERAL_FAULT:
    PIN_SUM_FLT = OLL_PIN_SUM_FAULT_FAULTED;
    PIN_TEST_POINT_29 = !OLL_TP29_FOLDBACK_ON;
    software_rf_disable = 1;
    front_panel_led_state = SOLID_RED;
    front_panel_led_pulse_count = 0;
    while (control_state == STATE_FAULT_GENERAL_FAULT) {
      PIN_ENABLE_RF_AMP = !OLL_PIN_ENABLE_RF_AMP_ENABLED;
      DoSerialCommand();
      Do10msTicToc();
      if (!FaultCheckGeneralFault() && !FaultCheckOverTemp()) {
	control_state = STATE_RF_OFF;
      }
    }
    break;
  }
}


void DoA35997StartUp(void) {

  // Initialize Detectors
  // PROGRAM POWER LEVEL
  program_power_level.adc_cal_gain = 0x8000;  // dparker read all this cal constant from EEPROM
  program_power_level.adc_cal_gain_thermal_adjust = 0;
  program_power_level.adc_cal_offset = 0x0000;
  program_power_level.adc_cal_offset_thermal_adjust = 0;
  
  // Forward Detector A
  forward_power_detector_A.adc_cal_gain = 0x8000;  // dparker read all this cal constant from EEPROM
  forward_power_detector_A.adc_cal_gain_thermal_adjust = 0;
  forward_power_detector_A.adc_cal_offset = 0x0000;
  forward_power_detector_A.adc_cal_offset_thermal_adjust = 0;
  forward_power_detector_A.max_power = FORWARD_DETECTOR_MAX_POWER;
  forward_power_detector_A.over_power_trip_time = FORWARD_OVER_POWER_TRIP_TIME_10MS_UNITS;

  // Forward Detector B
  forward_power_detector_B.adc_cal_gain = 0x8000;  // dparker read all this cal constant from EEPROM
  forward_power_detector_B.adc_cal_gain_thermal_adjust = 0;
  forward_power_detector_B.adc_cal_offset = 0x0000;
  forward_power_detector_B.adc_cal_offset_thermal_adjust = 0;
  forward_power_detector_B.max_power = FORWARD_DETECTOR_MAX_POWER;
  forward_power_detector_B.over_power_trip_time = FORWARD_OVER_POWER_TRIP_TIME_10MS_UNITS;
  
  // Reverse Detector B
  reverse_power_detector_A.adc_cal_gain = 0x8000;  // dparker read all this cal constant from EEPROM
  reverse_power_detector_A.adc_cal_gain_thermal_adjust = 0;
  reverse_power_detector_A.adc_cal_offset = 0x0000;
  reverse_power_detector_A.adc_cal_offset_thermal_adjust = 0;
  reverse_power_detector_A.max_power = REVERSE_DETECTOR_MAX_POWER;
  reverse_power_detector_A.over_power_trip_time = REVERSE_OVER_POWER_TRIP_TIME_10MS_UNITS;

  // Reverse Detector B
  reverse_power_detector_B.adc_cal_gain = 0x8000;  // dparker read all this cal constant from EEPROM
  reverse_power_detector_B.adc_cal_gain_thermal_adjust = 0;
  reverse_power_detector_B.adc_cal_offset = 0x8000;
  reverse_power_detector_B.adc_cal_offset_thermal_adjust = 0;
  reverse_power_detector_B.max_power = REVERSE_DETECTOR_MAX_POWER;
  reverse_power_detector_B.over_power_trip_time = REVERSE_OVER_POWER_TRIP_TIME_10MS_UNITS;
    

  // --------- BEGIN IO PIN CONFIGURATION ------------------
  
  // Initialize the output latches before setting TRIS
  PIN_TEST_POINT_24_CLK_OUT = 0;
  PIN_TEST_POINT_25 = 0;
  PIN_TEST_POINT_26 = 0;
  PIN_TEST_POINT_27 = 0;
  PIN_TEST_POINT_28 = 0;
  PIN_TEST_POINT_29 = !OLL_TP29_FOLDBACK_ON;
  PIN_TEST_POINT_30 = 0;

  PIN_TEST_LED_1 = TEST_LED_OFF;
  PIN_TEST_LED_2 = TEST_LED_OFF;
  PIN_TEST_LED_3 = TEST_LED_OFF;
  PIN_TEST_LED_4 = TEST_LED_OFF;

  PIN_FRONT_PANEL_LED_GREEN = OLL_FRONT_PANEL_LED_OFF;
  PIN_FRONT_PANEL_LED_BLUE = OLL_FRONT_PANEL_LED_OFF;
  PIN_FRONT_PANEL_LED_RED = OLL_FRONT_PANEL_LED_OFF;

  PIN_SUM_FLT = !OLL_PIN_SUM_FAULT_FAULTED;
  PIN_RS422_DE = OLL_PIN_RS422_DE_ENABLE_TRANSMIT;
  PIN_ENABLE_RF_AMP = !OLL_PIN_ENABLE_RF_AMP_ENABLED;
  
  PIN_AD7686_CONVERT = !OLL_AD7686_START_CONVERSION;
  PIN_AD7686_1_CS = !OLL_AD7686_SELECT_DEVICE;
  PIN_AD7686_2_CS = !OLL_AD7686_SELECT_DEVICE;

  // Set all TRIS Values

  TRIS_PIN_TEST_POINT_24_CLK_OUT = TRIS_OUTPUT_MODE;
  TRIS_PIN_TEST_POINT_25 = TRIS_OUTPUT_MODE;
  TRIS_PIN_TEST_POINT_26 = TRIS_OUTPUT_MODE;
  TRIS_PIN_TEST_POINT_27 = TRIS_OUTPUT_MODE;
  TRIS_PIN_TEST_POINT_28 = TRIS_OUTPUT_MODE;
  TRIS_PIN_TEST_POINT_29 = TRIS_OUTPUT_MODE;
  TRIS_PIN_TEST_POINT_30 = TRIS_OUTPUT_MODE;

  TRIS_PIN_TEST_LED_1 = TRIS_OUTPUT_MODE;
  TRIS_PIN_TEST_LED_2 = TRIS_OUTPUT_MODE;
  TRIS_PIN_TEST_LED_3 = TRIS_OUTPUT_MODE;
  TRIS_PIN_TEST_LED_4 = TRIS_OUTPUT_MODE;

  TRIS_PIN_FRONT_PANEL_LED_GREEN = TRIS_OUTPUT_MODE;
  TRIS_PIN_FRONT_PANEL_LED_BLUE = TRIS_OUTPUT_MODE;
  TRIS_PIN_FRONT_PANEL_LED_RED = TRIS_OUTPUT_MODE;

  TRIS_PIN_SUM_FLT = TRIS_OUTPUT_MODE;
  TRIS_PIN_RS422_DE = TRIS_OUTPUT_MODE;
  TRIS_PIN_ENABLE_RF_AMP = TRIS_OUTPUT_MODE;
  
  TRIS_PIN_AD7686_CONVERT = TRIS_OUTPUT_MODE;
  TRIS_PIN_AD7686_1_CS = TRIS_OUTPUT_MODE;
  TRIS_PIN_AD7686_2_CS = TRIS_OUTPUT_MODE;

  TRIS_PIN_PS_ONE_OK = TRIS_INPUT_MODE;
  TRIS_PIN_PS_TWO_OK = TRIS_INPUT_MODE;
  TRIS_PIN_RF_ENABLE = TRIS_INPUT_MODE;


  // ----------- Configure Interrupts ---------- //
  
  // Configure SPI2 Interrupt
  _SPI2IE = 0; // Disable Interrupt
  _SPI2IF = 0; // Clear Interrupt Flag
  _SPI2IP = 6; // Priority Level 6
  
  
  // Configure ADC Interrupt
  _ADIE = 0;  // Disable Interrupt
  _ADIF = 0;  // Clear Interrupt Flag
  _ADIP = 5;  // Priority Level 5 

  // Configure T1 Interrupt
  _T1IE = 0;  // Disable Interrupt
  _T1IF = 0;  // Clear Interrupt Flag  
  _T1IP = 4;  // Priority Level 4

  // Configure UART Interrupts
  _U1RXIE = 0; // Disable RX Interrupt
  _U1RXIP = 3; // Priority Level 3
  
  _U1TXIE = 0; // Disable TX Interrupt
  _U1RXIP = 3; // Priority Level 3


  // --------- CONFIGURATION FOR THE SPI BUSSES ---------------- //
  OpenSPI2((A35997_SPI2CON_VALUE & A35997_SPI2CON_CLOCK), A35997_SPI2STAT_VALUE);  // Configure SPI bus 2 based on H file parameters
  
  // ------ CONFIGURE the CAN Modules to be OFF -------------- //
  C1CTRL = 0b0000000100000000;
  C2CTRL = 0b0000000100000000;
  
  
  // ----------------- UART #1 Setup and Data Buffer -------------------------//
  // Setup the UART input and output buffers
  BufferByte64Initialize(&uart1_input_buffer);
  BufferByte64Initialize(&uart1_output_buffer);
  
  U1MODE = A35997_U1MODE_VALUE;
  U1BRG = A35997_U1BRG_VALUE;
  U1STA = A35997_U1STA_VALUE;
  

  // ---------- Configure Timers ----------------- //
  
  // Configure TMR1
  T1CON = A35997_T1CON_VALUE;
  PR1 = A35997_PR1_VALUE;
  TMR1 = 0;
  
  // Configure TMR2
  T2CON = A35997_T2CON_VALUE;
  PR2 = A35997_PR2_VALUE;  
  TMR2 = 0;
  _T2IF = 0;

  // Initialize the PID structure
  pid_forward_power.abcCoefficients = &pid_forward_power_abcCoefficient[0];    /*Set up pointer to derived coefficients */
  pid_forward_power.controlHistory = &pid_forward_power_controlHistory[0];     /*Set up pointer to controller history samples */

  PIDInit(&pid_forward_power);                                                    /*Clear the controler history and the controller output */
  pid_forward_power_kCoeffs[0] = Q15(POWER_PID_P_COMPONENT);
  pid_forward_power_kCoeffs[1] = Q15(POWER_PID_I_COMPONENT);
  pid_forward_power_kCoeffs[2] = Q15(POWER_PID_D_COMPONENT);
  PIDCoeffCalc(pid_forward_power_kCoeffs, &pid_forward_power);             /*Derive the a,b, & c coefficients from the Kp, Ki & Kd */

  // --------------- Initialize U6 - LTC2656 ------------------------- //
  U6_LTC2656.pin_cable_select_not = _PIN_RD15;
  U6_LTC2656.pin_dac_clear_not = _PIN_RB15;
  U6_LTC2656.pin_load_dac_not = _PIN_RD14;
  U6_LTC2656.pin_por_select = _PIN_NOT_CONNECTED;
  U6_LTC2656.por_select_value = 0;
  U6_LTC2656.spi_port = ETM_SPI_PORT_1;
  U6_LTC2656.spi_con1_value = A35997_SPI1CON_VALUE;
  U6_LTC2656.spi_con2_value = 0x0000;
  U6_LTC2656.spi_stat_value = A35997_SPI1STAT_VALUE;
  U6_LTC2656.spi_bit_rate = 7500000;
  U6_LTC2656.fcy_clk = FCY_CLK;
  // DPARKER most of these values for CON1/CON2/STAT should be fixed for the LTC2656 so they don't need to be stored in ram or worried about by the user, just set them as constants in the module

  OpenSPI1((A35997_SPI1CON_VALUE & A35997_SPI1CON_CLOCK), A35997_SPI1STAT_VALUE);  // Configure SPI bus 1 based on H file parameters

  SetupLTC2656(&U6_LTC2656);
  
  // ---- Configure the dsPIC ADC Module ------------ //
  ADCON1 = A35997_ADCON1_VALUE;             // Configure the high speed ADC module based on H file parameters
  ADCON2 = A35997_ADCON2_VALUE;             // Configure the high speed ADC module based on H file parameters
  ADCON3 = A35997_ADCON3_VALUE;             // Configure the high speed ADC module based on H file parameters
  ADCHS  = A35997_ADCHS_VALUE;              // Configure the high speed ADC module based on H file parameters
  
  ADPCFG = A35997_ADPCFG_VALUE;             // Set which pins are analog and which are digital I/O
  ADCSSL = A35997_ADCSSL_VALUE;             // Set which analog pins are scanned


  // Begin External ADC operation (this uses SPI2)
  _SPI2IF = 0;
  _SPI2IE = 1;
  SPI2BUF = 0xAAAA;
  
  // Begin Internal ADC operation
  _ADIF = 0;
  _ADIE = 1;  
  ADCON1bits.ADON = 1;
  
  // Begin T1 (100us) Timer and Interrupt Operation
  _T1IF = 0;
  _T1IE = 1;
  T1CONbits.TON = 1;

  // Begin UART operation
  PIN_RS422_DE = OLL_PIN_RS422_DE_ENABLE_TRANSMIT;   // Enable the RS422 Driver output (The reciever is always enabled)
  command_string.data_state = COMMAND_BUFFER_EMPTY;  // The command buffer is empty

  _U1TXIF = 0;	// Clear the Transmit Interrupt Flag
  _U1RXIF = 0;	// Clear the Recieve Interrupt Flag
  _U1TXIE = 1;	// Enable Transmit Interrupts
  _U1RXIE = 1;	// Enable Recieve Interrupts  
  U1MODEbits.UARTEN = 1;	// And turn the peripheral on


  //Startup TMR2 (used for 10ms polling)
  TMR2 = 0;
  T2CONbits.TON = 1;

}


/*---------------- RF Detector Functions ------------------ */

/* 
   The data calibration process

   (1) Data is read by ADC (internal or external and stored in a circular buffer
   (2) The circular buffer is averaged (by AverageADC128 or AverageADC16 deppending upon the buffer type)
   (3) (ONLY SLOW SIGNALS) - The averaged result is low pass filtered
   (4) The ADCReading is Calibrated for offset & gain (offset & gain are temperature compensated) -> Stored as RF_DETECTOR.adc_reading_calibrated
   (5) This value has the RF detector calibration applied (This must be done per device over temperature) -> Stored as RF_DETECTOR.detector_level_calibrated
   (6) This value is converted into centi_watts via a 12 bit lookup table -> Stored as power_reading_centi_watts

*/

void CalibrateADCReading(RF_DETECTOR* ptr_rf_det, unsigned int adc_reading) {
  unsigned int cal_reading;
  unsigned int offset;
  unsigned int cal_gain;
  cal_gain = ptr_rf_det->adc_cal_gain;
  cal_gain += ptr_rf_det->adc_cal_gain_thermal_adjust * (ptr_rf_det->adc_temperature - 323);
  cal_reading = ETMScale16Bit(adc_reading, cal_gain, 1);
  offset = ptr_rf_det->adc_cal_offset;
  offset += ptr_rf_det->adc_cal_offset_thermal_adjust * (ptr_rf_det->adc_temperature - 323);
  if (offset >= 0) {
    if ((0xFFFF - offset) <= cal_reading) {
      // adding these together would cause an overflow
      cal_reading = 0xFFFF;
    } else {
      cal_reading += offset;
    }
  } else {
    if ((0x0000 - offset) >= cal_reading) {
      // adding this together would cause a negative overflow
      cal_reading = 0x0000;
    } else {
      cal_reading += offset;
    }
  } 
  ptr_rf_det->adc_reading_calibrated = cal_reading;
}

void CalibrateDetectorLevel(RF_DETECTOR* ptr_rf_det) {
  ptr_rf_det->detector_level_calibrated = ptr_rf_det->adc_reading_calibrated;
  // DPARKER - Need to do calibration . . . Also need to figure out how to do calibration
}

void ConvertDetectorLevelToPowerCentiWatts(RF_DETECTOR* ptr_rf_det) {
  unsigned int remainder;
  unsigned int location;
  unsigned int value1;
  unsigned int value2;
  unsigned int power;
  
  remainder = (ptr_rf_det->detector_level_calibrated & 0x000F);
  location = ptr_rf_det->detector_level_calibrated >> 4;
  value1 = detector_voltage_to_power_look_up_table[location];
  value2 = detector_voltage_to_power_look_up_table[location+1];
  
  if (value2 >= value1) {
    power = value2-value1;
    power *= remainder;
    power >>= 4;
    power += value1;
  } else {
    power = value1-value2;
    power *= (16 - remainder);
    power >>= 4;
    power += value2;
  }

  power >>= 1;

  //power = ETMScale16Bit(power, 55705, 0);  // Multiply by .85

  ptr_rf_det->power_reading_centi_watts = power;
}

unsigned int ConvertProgramLevelToPowerCentiWatts(unsigned int program_level) {
  // 10V program = 530 Watts
  // Input scalling = V_in / 5
  // 2V = 530 Watts
  // 0xFFFF = 542.72 Watts = 54272 Centi Watts
  // Need to multiply by 54272 and shift right 16 bits
  return ETMScale16Bit(program_level, PROGRAM_LEVEL_TO_CENTI_WATTS_SCALE_FACTOR, 0);
}

unsigned int ConvertAmplifierTemperatureADCtoDeciDegreesK(unsigned int adc_reading){
  /*
    25 Deg V = 750mV
    +/- 10mV per deg C
    0x0000 = 0V = -50 C = 223 K

    Deg K = 223 + V_adc/320
    Deci Deg K = (223 + V_adc/320)*10
    = 2230 + V_adc/32
  */
  return (2230 + (adc_reading>>5));
}

unsigned int ConvertDetectorTemperatureADCtoDeciDegreesK(unsigned int adc_reading) {
  /*
    The detectors put out 2mv per Deg K
    0xFFFF = 2.048 volts
    To converter to deg K, just divide by 32
    Then multiply by 10 to get deci Deg K
  */
  return ETMScale16Bit(adc_reading, DETECTOR_TEMPERATURE_ADC_READING_TO_DECI_DEGREES_K, 0);
}


void Do10msTicToc(void) {
  PIN_TEST_POINT_28 = 1;
  ClrWdt();
  if (_T2IF) {
    _T2IF = 0;
    //10ms roll has occured
    
    // Flash LED 1 on the control board
    led_pulse_count += 1;
    led_pulse_count &= 0b00011111;
    if (led_pulse_count & 0b00010000) {
      PIN_TEST_LED_1 = TEST_LED_ON;
    } else  {
      PIN_TEST_LED_1 = TEST_LED_OFF;
    }
    
    // Manage the front panel LEDs
    DoFrontPanelLED();


    // ------- Generate Fault Reset on Positive RF_ENABLE Transition--------- //
    if ((PIN_RF_ENABLE == ILL_PIN_RF_ENABLE_ENABLED) && (start_reset_process)) {
      ResetAllFaults();
    }
    if (PIN_RF_ENABLE == !ILL_PIN_RF_ENABLE_ENABLED) {
      // Start the Reset Process
      start_reset_process = 1;
    } else {
      start_reset_process = 0;
    }

    FilterADCs();           
    UpdateFaults();


#ifdef _DO_THERMAL_COMP
    
    coupler_temperature_scale_unknown += (total_forward_power_centi_watts >> 6);
    coupler_temperature_scale_unknown = ETMScale16Bit(coupler_temperature_scale_unknown, TEMPERATURE_TIME_CONSTANT, 0);
    /*
      500 Watts will stabilize at 19531 , 19
      400 Watts will stabilize at 15624 , 15
      300 Watts will stabilize at 11718 , 11
      200 Watts will stabilize at 7812  , 7
      100 Watts will stabilize at 3906  , 3
    */
    
    coupler_multiplier = temperature_coeff_mult[(coupler_temperature_scale_unknown >> 10)];
#endif

  }
  PIN_TEST_POINT_28 = 0;
}


void DoFrontPanelLED(void) {
    
  // Handle to front panel LED state
  front_panel_led_pulse_count += 1;
  front_panel_led_pulse_count &= 0b01111111;
  // this will roll over every 10ms * 2^7 = 1.28 seconds, this is the LED period
  switch(front_panel_led_state) {
    
  case SOLID_GREEN:
    PIN_FRONT_PANEL_LED_GREEN = OLL_FRONT_PANEL_LED_ON;
    PIN_FRONT_PANEL_LED_BLUE = OLL_FRONT_PANEL_LED_OFF;
    PIN_FRONT_PANEL_LED_RED = OLL_FRONT_PANEL_LED_OFF;
    break;
    
  case SOLID_BLUE:
    PIN_FRONT_PANEL_LED_GREEN = OLL_FRONT_PANEL_LED_OFF;
    PIN_FRONT_PANEL_LED_BLUE = OLL_FRONT_PANEL_LED_ON;
    PIN_FRONT_PANEL_LED_RED = OLL_FRONT_PANEL_LED_OFF;
    break;
    
  case FLASH_BLUE:
    PIN_FRONT_PANEL_LED_GREEN = OLL_FRONT_PANEL_LED_OFF;
    PIN_FRONT_PANEL_LED_RED = OLL_FRONT_PANEL_LED_OFF;

    if ((front_panel_led_pulse_count & 0b01000000) == 0) {
      PIN_FRONT_PANEL_LED_BLUE = OLL_FRONT_PANEL_LED_ON;
    } else {
      PIN_FRONT_PANEL_LED_BLUE = OLL_FRONT_PANEL_LED_OFF;
    }
    break;

  case SOLID_RED:
    PIN_FRONT_PANEL_LED_GREEN = OLL_FRONT_PANEL_LED_OFF;
    PIN_FRONT_PANEL_LED_BLUE = OLL_FRONT_PANEL_LED_OFF;
    PIN_FRONT_PANEL_LED_RED = OLL_FRONT_PANEL_LED_ON;
    break;
      
  case FLASH_RED:
    PIN_FRONT_PANEL_LED_GREEN = OLL_FRONT_PANEL_LED_OFF;
    PIN_FRONT_PANEL_LED_BLUE = OLL_FRONT_PANEL_LED_OFF;

    if ((front_panel_led_pulse_count & 0b01000000) == 0) {
      PIN_FRONT_PANEL_LED_RED = OLL_FRONT_PANEL_LED_ON;
    } else {
      PIN_FRONT_PANEL_LED_RED = OLL_FRONT_PANEL_LED_OFF;
    }
    break;
      
  case FLASH_ALL_SERIES:
    if (front_panel_led_pulse_count == 1) {
      front_panel_led_startup_flash_couter++;
    }
    if (front_panel_led_pulse_count <= 42) {
      PIN_FRONT_PANEL_LED_GREEN = OLL_FRONT_PANEL_LED_ON;
      PIN_FRONT_PANEL_LED_BLUE = OLL_FRONT_PANEL_LED_OFF;
      PIN_FRONT_PANEL_LED_RED = OLL_FRONT_PANEL_LED_OFF;
    } else if (front_panel_led_pulse_count <= 84) {
      PIN_FRONT_PANEL_LED_GREEN = OLL_FRONT_PANEL_LED_OFF;
      PIN_FRONT_PANEL_LED_BLUE = OLL_FRONT_PANEL_LED_ON;
      PIN_FRONT_PANEL_LED_RED = OLL_FRONT_PANEL_LED_OFF;
    } else {
      PIN_FRONT_PANEL_LED_GREEN = OLL_FRONT_PANEL_LED_OFF;
      PIN_FRONT_PANEL_LED_BLUE = OLL_FRONT_PANEL_LED_OFF;
      PIN_FRONT_PANEL_LED_RED = OLL_FRONT_PANEL_LED_ON;
    }
  }
}





long ConverterADCReadingToMillidB(RF_DETECTOR* ptr_rf_det) {
  long delta;
  long detector_milli_dB;
  unsigned int temperature_combiner;

  /* 
     Step 1, use the slope and intercept values to generate the dB level
     
     milli_dB = Intercept_milli_dB - Delta 
     
     Delta = slope(dB/V)*(2.048 V / 2^16 bits) * (1000 millidB / dB) * ADC_Reading(bits)


     To keep the 16 bit math simple -   
     scale_factor = ((Slope(dB/V) * 1000) * 1.024) (This is a constant for a detector and stored in EEPROM)
     Delta = (scale_factor * ADC_Reading) >> 15
  */
  
  delta = ptr_rf_det->detector_scale_factor * ptr_rf_det->adc_reading_calibrated;
  delta >>= 15;
  
  detector_milli_dB = ptr_rf_det->detector_intercept_milli_dB - delta;
  
  
  /*
    Now we need to add in all the system offsets
    
    Constant - Pad Attenuation
    Constant - Coupler Attenuation
    
  */

  detector_milli_dB += ptr_rf_det->pad_attenuation_milli_dB;

  detector_milli_dB += ptr_rf_det->coupler_attenuation_milli_dB;

  /*
    Temperature calibration
    We need to calibrate our readings based on the temperature of the combiner/coupler.

    Assumptions.
    Thermal drift is a function of the temperature of the Combiner
    We are going to use two time constant
    Temperature = Base Plate temperature (measured) + Slow Temperature Constant (bulk material around Combiner) + Fast Temperature Constant (Combiner Material)
    
    Once every 10mS execute the following
    temperature_fast = temperature_fast * time_constant_fast + forward_power * fast_multiplier

    Once every 640ms seconds (64 10ms Invervals) - execute the following 
    temperature_slow = temperature_slow * time_constant_slow + forward_power * slow_multiplier

    temperature_combiner = temperature_fast + temperature_slow + base_plate_temperature

    This temperature is not normalized to deg C!!!

  */


  detector_milli_dB += GetCombinerTemperatureDelta(temperature_combiner);


  return detector_milli_dB;
}

int GetCombinerTemperatureDelta(unsigned int temperature) {
  return 0;
}


void FilterADCs(void) {
  /*
    Forward Detector A and B are read by the external ADC and filtered by the SPI Interrupt because their updated values are needed immediately
    Power Level Program from the customer is read by the internal ADC and filtered by the internal ADC because their updated values are needed immediately
  
    Reverse Detector A and B - Do these need to be managed in real life or is 10 Hz (10ms x8 tau) fast enough?
  
    rf amplifier temperature & all 4 detector temperatures
    These are averaged over 128 samples then low pass filtered
    The reverse power detector data is averaged and low pass filtered (at a higher frequency)
  */
  unsigned int averaged_adc_reading;
  unsigned int temperature_reading;
  unsigned long power_long;

  averaged_adc_reading = AverageADC128(rf_amplifier_temperature_array);
  temperature_reading = ConvertAmplifierTemperatureADCtoDeciDegreesK(averaged_adc_reading);
  //rf_amplifier_temp = RCFilter64Tau(rf_amplifier_temp, temperature_reading);
  program_power_level.detector_temperature = RCFilterNTau(program_power_level.detector_temperature, temperature_reading,RC_FILTER_64_TAU);

  averaged_adc_reading = AverageADC128(fwd_rf_det_a_temperature_array);
  temperature_reading = ConvertDetectorTemperatureADCtoDeciDegreesK(averaged_adc_reading);
  //forward_power_detector_A.detector_temperature = RCFilter64Tau(forward_power_detector_A.detector_temperature, temperature_reading);
  forward_power_detector_A.detector_temperature = RCFilterNTau(forward_power_detector_A.detector_temperature, temperature_reading,RC_FILTER_64_TAU);
  
  averaged_adc_reading = AverageADC128(fwd_rf_det_b_temperature_array);
  temperature_reading = ConvertDetectorTemperatureADCtoDeciDegreesK(averaged_adc_reading);
  //forward_power_detector_B.detector_temperature = RCFilter64Tau(forward_power_detector_B.detector_temperature, temperature_reading);
  forward_power_detector_B.detector_temperature = RCFilterNTau(forward_power_detector_B.detector_temperature, temperature_reading,RC_FILTER_64_TAU);

  averaged_adc_reading = AverageADC128(rev_rf_det_a_temperature_array);
  temperature_reading = ConvertDetectorTemperatureADCtoDeciDegreesK(averaged_adc_reading);
  //reverse_power_detector_A.detector_temperature = RCFilter64Tau(reverse_power_detector_A.detector_temperature, temperature_reading);
  reverse_power_detector_A.detector_temperature = RCFilterNTau(reverse_power_detector_A.detector_temperature, temperature_reading,RC_FILTER_64_TAU);
  
  averaged_adc_reading = AverageADC128(rev_rf_det_b_temperature_array);
  temperature_reading = ConvertDetectorTemperatureADCtoDeciDegreesK(averaged_adc_reading);
  //reverse_power_detector_B.detector_temperature = RCFilterNTau(reverse_power_detector_B.detector_temperature, temperature_reading,RC_FILTER_64_TAU);
  reverse_power_detector_B.detector_temperature = reverse_power_detector_A.detector_temperature;

  averaged_adc_reading = AverageADC128(reverse_detector_a_array);
  // This would be the place to add detector level calibration if it was needed which I do not think it is for the reverse detectors
  //reverse_power_detector_A.detector_level_calibrated = RCFilter8Tau(reverse_power_detector_A.detector_level_calibrated, averaged_adc_reading);
  reverse_power_detector_A.detector_level_calibrated = RCFilterNTau(reverse_power_detector_A.detector_level_calibrated, averaged_adc_reading,RC_FILTER_2_TAU);
  ConvertDetectorLevelToPowerCentiWatts(&reverse_power_detector_A);
  
  averaged_adc_reading = AverageADC128(reverse_detector_b_array);
  // This would be the place to add detector level calibration if it was needed which I do not think it is for the reverse detectors
  reverse_power_detector_B.detector_level_calibrated = RCFilterNTau(reverse_power_detector_B.detector_level_calibrated, averaged_adc_reading,RC_FILTER_2_TAU);
  reverse_power_detector_B.detector_level_calibrated = reverse_power_detector_A.detector_level_calibrated;
  ConvertDetectorLevelToPowerCentiWatts(&reverse_power_detector_B);

  power_long = reverse_power_detector_A.power_reading_centi_watts;
  power_long += reverse_power_detector_B.power_reading_centi_watts;
  if (power_long >= 0xFFFF) {
    total_reverse_power_centi_watts = 0xFFFF;
  } else {
    total_reverse_power_centi_watts = (power_long & 0xFFFF);
  }
}


//----------------------- ISRs ------------------------- //


// SPI2 Interrupt - External ADC
// SPI2 Interrupt is moved to assembly in A35997_assemblt_functions.s
/*
  void _ISRFASTNOPSV _SPI2Interrupt(void) {
*/


// ADC Interrupt
void _ISRNOPSV _ADCInterrupt(void) {
//void __attribute__((interrupt(__save__(CORCON,SR)),no_auto_psv)) _ADCInterrupt(void) {
  // At a sample rate of 180KHz, it takes 83uS to perform the 15 conversions.
  // The ADC conversion process is started by the 100uS interrupt and this data process will complete before the next T1 interrupt

#ifdef _DEBUG_MODE
  PIN_TEST_POINT_26 = 1; // TIME this interrupt
#endif

  _ASAM = 0;
  _ADIF = 0;

#ifdef _DEBUG_MODE
  PIN_TEST_POINT_26 = 0;
#endif
}


// PID Interrupt on TMR1 (100us) 
//void _ISRNOPSV _T1Interrupt(void) {
void __attribute__((interrupt(__save__(ACCA,CORCON,SR)),no_auto_psv)) _T1Interrupt(void) {
  unsigned int power_level_average;
  unsigned long power_long;
  unsigned int power_target_centi_watts;
  unsigned long power_average;
  unsigned int power_ramp_rate;

  int pid_p;
  int pid_i;
  int pid_d;
  int delta;

  /*
    The pid functions provided by dsp.h use Q15 fractional data.
    This represents fractional inputs/outputs from -1 to 1.
    Our inputs to the PID come from an unipolar ADC.
    The control output of the PID goes directly to a DAC.
    In order for the PID control to work with our unsigned 16 bit data two changed must be made.
    (1) Measured output must be converted from 16 bit data to Q(15).  This is easily done by shifting one bit to the right.
        From to PIDs point of view we are just decreasing our resolution by 1 bit, not a problem here.
    (2) Control output must be convereted so that it can iterface to our DAC.
        The control output ranges from -1 to 1.  so perform the following logic.
	If (control_output < 0) then (control_output = 0).
	Multiply control_output by 2 (shift left 1 bit) to use the full range of the DAC.
	NOTE: You must perform the math like this we because the PID will set the output to Zero at start.
	If you shift the entire 16 bits to unsinged interger, 0 will become 0x7FFF and your control_output will start at 50%, not zero!!!!
  */
  PIN_TEST_POINT_25 = 1; // Time this interrupt

  // Copy Data From ADC Buffer to RAM
  reverse_detector_b_array[adc_result_index] = ADCBUF0;  
  power_level_average = ADCBUF1;
  reverse_detector_a_array[adc_result_index] = ADCBUF2;
  power_level_average += ADCBUF3;
  rf_amplifier_temperature_array[adc_result_index] = ADCBUF4;
  power_level_average += ADCBUF5;
  fwd_rf_det_a_temperature_array[adc_result_index] = ADCBUF6;
  power_level_average += ADCBUF7;
  fwd_rf_det_b_temperature_array[adc_result_index] = ADCBUF8;
  power_level_average += ADCBUF9;
  rev_rf_det_a_temperature_array[adc_result_index] = ADCBUFA;
  power_level_average += ADCBUFB;
  rev_rf_det_b_temperature_array[adc_result_index] = ADCBUFC;
  power_level_average += ADCBUFD;
  power_level_average += ADCBUFE;

  adc_result_index++;
  adc_result_index &= 0b01111111;
  
  power_level_average <<= 1; // Scale power level average up to 16 bits

  _ASAM = 1; // Internal ADC conversion is synced to this interrupt.  
  _T1IF = 0;



  // -- begin very ugly section -- // 

  /*
    For reasons beyond Dan's understanding AverageADC16 is returning bogus values at RF amplifier startup
    I have no clue why this happens when the RF is ramping up and not at any other time.
   */


  /*
  last_valid_detector_A_adc_reading = AverageADC16(forward_detector_a_array);
  last_valid_detector_B_adc_reading = AverageADC16(forward_detector_b_array);

  if (last_valid_detector_A_adc_reading < detector_A_min_value) {
    last_valid_detector_A_adc_reading = detector_A_min_value;
  }
  if (last_valid_detector_B_adc_reading < detector_B_min_value) {
    last_valid_detector_B_adc_reading = detector_B_min_value;
  }


  if (last_valid_detector_A_adc_reading > 40000) {
    detector_A_min_value = 37000;
    detector_A_max_value = 0xFFFF;
  } else {
    detector_A_max_value = last_valid_detector_A_adc_reading + 2000;
    if (last_valid_detector_A_adc_reading < 12000) {
      detector_A_min_value = 10000;
    } else {
      detector_A_min_value = last_valid_detector_A_adc_reading - 2000;
    }
  }

  if (last_valid_detector_B_adc_reading > 40000) {
    detector_B_min_value = 37000;
    detector_B_max_value = 0xFFFF;
  } else {
    detector_B_max_value = last_valid_detector_B_adc_reading + 2000;
    if (last_valid_detector_B_adc_reading < 12000) {
      detector_B_min_value = 10000;
    } else {
      detector_B_min_value = last_valid_detector_B_adc_reading - 2000;
    }
  }
  */

  power_average = 0;
  power_average += forward_detector_a_array[0];

  power_average += forward_detector_a_array[1];
  power_average += forward_detector_a_array[2];
  power_average += forward_detector_a_array[3];
  power_average += forward_detector_a_array[4];
  power_average += forward_detector_a_array[5];
  power_average += forward_detector_a_array[6];
  power_average += forward_detector_a_array[7];
  power_average += forward_detector_a_array[8];
  power_average += forward_detector_a_array[9];
  power_average += forward_detector_a_array[10];
  power_average += forward_detector_a_array[11];
  power_average += forward_detector_a_array[12];
  power_average += forward_detector_a_array[13];
  power_average += forward_detector_a_array[14];
  power_average += forward_detector_a_array[15];
  power_average >>= 4;

  last_valid_detector_A_adc_reading = power_average;

  power_average = 0;
  power_average += forward_detector_b_array[0];
  power_average += forward_detector_b_array[1];
  power_average += forward_detector_b_array[2];
  power_average += forward_detector_b_array[3];
  power_average += forward_detector_b_array[4];
  power_average += forward_detector_b_array[5];
  power_average += forward_detector_b_array[6];
  power_average += forward_detector_b_array[7];
  power_average += forward_detector_b_array[8];
  power_average += forward_detector_b_array[9];
  power_average += forward_detector_b_array[10];
  power_average += forward_detector_b_array[11];
  power_average += forward_detector_b_array[12];
  power_average += forward_detector_b_array[13];
  power_average += forward_detector_b_array[14];
  power_average += forward_detector_b_array[15];
  power_average >>= 4;

  last_valid_detector_B_adc_reading = power_average;


  gui_debug_value_1 = last_valid_detector_A_adc_reading;
  gui_debug_value_2 = last_valid_detector_B_adc_reading;

  // -- end very ugly section -- // 



  // ---------------- Calculate the Forward Power --------------- //
  CalibrateADCReading(&forward_power_detector_A, last_valid_detector_A_adc_reading);
  CalibrateDetectorLevel(&forward_power_detector_A);
  ConvertDetectorLevelToPowerCentiWatts(&forward_power_detector_A);
  
  
  CalibrateADCReading(&forward_power_detector_B, last_valid_detector_B_adc_reading);
  CalibrateDetectorLevel(&forward_power_detector_B);
  ConvertDetectorLevelToPowerCentiWatts(&forward_power_detector_B);


#ifdef _DO_THERMAL_COMP
  forward_power_detector_B.power_reading_centi_watts = ETMScale16Bit(forward_power_detector_B.power_reading_centi_watts, coupler_multiplier, 1);
  forward_power_detector_A.power_reading_centi_watts = ETMScale16Bit(forward_power_detector_A.power_reading_centi_watts, coupler_multiplier, 1); 
#endif




#ifdef _DO_TIME_COMP
#define STARTUP_FOLDBACK 35111
#define STARTUP_FOLDBACK_POINT_2 34312
#define STARTUP_FOLDBACK_POINT_1 33512
  if (cycles_running <= 13) {
    cycles_running++;
    forward_power_detector_A.power_reading_centi_watts = ETMScale16Bit(forward_power_detector_A.power_reading_centi_watts, STARTUP_FOLDBACK, 1);
    forward_power_detector_B.power_reading_centi_watts = ETMScale16Bit(forward_power_detector_B.power_reading_centi_watts, STARTUP_FOLDBACK, 1);
  } else if (cycles_running == 14) {
    forward_power_detector_A.power_reading_centi_watts = ETMScale16Bit(forward_power_detector_A.power_reading_centi_watts, STARTUP_FOLDBACK_POINT_2, 1);
    forward_power_detector_B.power_reading_centi_watts = ETMScale16Bit(forward_power_detector_B.power_reading_centi_watts, STARTUP_FOLDBACK_POINT_2, 1);
  } else if (cycles_running == 15) {
    forward_power_detector_A.power_reading_centi_watts = ETMScale16Bit(forward_power_detector_A.power_reading_centi_watts, STARTUP_FOLDBACK_POINT_1, 1);
    forward_power_detector_B.power_reading_centi_watts = ETMScale16Bit(forward_power_detector_B.power_reading_centi_watts, STARTUP_FOLDBACK_POINT_1, 1);
  }


  if (PIN_RF_ENABLE == !ILL_PIN_RF_ENABLE_ENABLED) {
    cycles_running = 0;
  }
#endif


  
  power_long = forward_power_detector_B.power_reading_centi_watts;
  power_long += forward_power_detector_A.power_reading_centi_watts;

  if (power_long >= 0xFFFF) {
    total_forward_power_centi_watts = 0xFFFF;
  } else {
    total_forward_power_centi_watts = (power_long & 0xFFFF);
  }

  pid_forward_power.measuredOutput = (total_forward_power_centi_watts >> 1);
  

  // --------- Calculate the target power ----------------- //
  // program_power_level.power_reading_centi_watts is the "programed power", this is not limited by range or state and is only used by the GUI
  // pid_forward_power.controlReference is the working "target power", this is limited by min/max values and state
  
  CalibrateADCReading(&program_power_level, power_level_average);
  power_target_centi_watts = ConvertProgramLevelToPowerCentiWatts(program_power_level.adc_reading_calibrated);
  
#ifdef _DEBUG_MODE
  power_target_centi_watts = serial_link_power_target;
#endif
  
  program_power_level.power_reading_centi_watts = power_target_centi_watts;  // Store this value for use by the GUI


  // First check that target is less than the Max power, if the target is greater than max power, set it to max power
  if (power_target_centi_watts >= MAX_POWER_TARGET) {
    power_target_centi_watts = MAX_POWER_TARGET;
  } 
  
  // If we are in foldback mode, limit the power to the max foldback power
  if (software_foldback_mode_enable) {
    if (power_target_centi_watts > FOLDBACK_POWER_PROGRAM) {
      power_target_centi_watts = FOLDBACK_POWER_PROGRAM;
    }
  }

#ifdef _POWER_RAMP
  power_ramp_rate = ETMScale16Bit(power_target_centi_watts, RAMP_RATE_SCALE_FACTOR, 0);
  power_ramp_centi_watts = power_ramp_centi_watts + power_ramp_rate;
  if (power_ramp_centi_watts >= power_target_centi_watts) {
    power_ramp_centi_watts = power_target_centi_watts;
  }
#else
  power_ramp_centi_watts = power_target_centi_watts
#endif




#ifdef _POWER_BASED_PID_MODE
  
  if (power_ramp_centi_watts < 5000) {
    // 0 -> 50 Watts
    delta = power_ramp_centi_watts;
    delta >>= 7;
    pid_p = PID_P_0_50_SLOPE;
    pid_p *= delta;
    pid_p += PID_P_0_WATT;
    
    pid_i = PID_I_0_50_SLOPE;
    pid_i *= delta;
    pid_i += PID_I_0_WATT;
    
    pid_d = PID_D_0_50_SLOPE;
    pid_d *= delta;
    pid_d += PID_D_0_WATT;
  } else if (power_ramp_centi_watts < 10000) {
    // 50 -> 100 Watts
    delta = power_ramp_centi_watts - 5000;
    delta >>= 7;
    pid_p = PID_P_50_100_SLOPE;
    pid_p *= delta;
    pid_p += PID_P_50_WATT;
    
    pid_i = PID_I_50_100_SLOPE;
    pid_i *= delta;
    pid_i += PID_I_50_WATT;
    
    pid_d = PID_D_50_100_SLOPE;
    pid_d *= delta;
    pid_d += PID_D_50_WATT;
  } else if (power_ramp_centi_watts < 15000) {
    // 100 -> 150 Watts
    delta = power_ramp_centi_watts - 10000;
    delta >>= 7;
    pid_p = PID_P_100_150_SLOPE;
    pid_p *= delta;
    pid_p += PID_P_100_WATT;
    
    pid_i = PID_I_100_150_SLOPE;
    pid_i *= delta;
    pid_i += PID_I_100_WATT;
    
    pid_d = PID_D_100_150_SLOPE;
    pid_d *= delta;
    pid_d += PID_D_100_WATT;
  } else if (power_ramp_centi_watts < 20000) {
    // 150 -> 200 Watts
    delta = power_ramp_centi_watts - 15000;
    delta >>= 7;
    pid_p = PID_P_150_200_SLOPE;
    pid_p *= delta;
    pid_p += PID_P_150_WATT;
    
    pid_i = PID_I_150_200_SLOPE;
    pid_i *= delta;
    pid_i += PID_I_150_WATT;
    
    pid_d = PID_D_150_200_SLOPE;
    pid_d *= delta;
    pid_d += PID_D_150_WATT;
  } else if (power_ramp_centi_watts < 25000) {
    // 200 -> 250 Watts
    delta = power_ramp_centi_watts - 20000;
    delta >>= 7;
    pid_p = PID_P_200_250_SLOPE;
    pid_p *= delta;
    pid_p += PID_P_200_WATT;
    
    pid_i = PID_I_200_250_SLOPE;
    pid_i *= delta;
    pid_i += PID_I_200_WATT;
    
    pid_d = PID_D_200_250_SLOPE;
    pid_d *= delta;
    pid_d += PID_D_200_WATT;
  } else if (power_ramp_centi_watts < 30000) {
    // 250 -> 300 Watts
    delta = power_ramp_centi_watts - 25000;
    delta >>= 7;
    pid_p = PID_P_250_300_SLOPE;
    pid_p *= delta;
    pid_p += PID_P_250_WATT;
    
    pid_i = PID_I_250_300_SLOPE;
    pid_i *= delta;
    pid_i += PID_I_250_WATT;
    
    pid_d = PID_D_250_300_SLOPE;
    pid_d *= delta;
    pid_d += PID_D_250_WATT;
  } else if (power_ramp_centi_watts < 35000) {
    // 300 -> 350 Watts
    delta = power_ramp_centi_watts - 30000;
    delta >>= 7;
    pid_p = PID_P_300_350_SLOPE;
    pid_p *= delta;
    pid_p += PID_P_300_WATT;
    
    pid_i = PID_I_300_350_SLOPE;
    pid_i *= delta;
    pid_i += PID_I_300_WATT;
    
    pid_d = PID_D_300_350_SLOPE;
    pid_d *= delta;
    pid_d += PID_D_300_WATT;
  } else if (power_ramp_centi_watts < 40000) {
    // 350 -> 400 Watts
    delta = power_ramp_centi_watts - 35000;
    delta >>= 7;
    pid_p = PID_P_350_400_SLOPE;
    pid_p *= delta;
    pid_p += PID_P_350_WATT;
    
    pid_i = PID_I_350_400_SLOPE;
    pid_i *= delta;
    pid_i += PID_I_350_WATT;
    
    pid_d = PID_D_350_400_SLOPE;
    pid_d *= delta;
    pid_d += PID_D_350_WATT;
  } else if (power_ramp_centi_watts < 45000) {
    // 400 -> 450 Watts
    delta = power_ramp_centi_watts - 40000;
    delta >>= 7;
    pid_p = PID_P_400_450_SLOPE;
    pid_p *= delta;
    pid_p += PID_P_400_WATT;
    
    pid_i = PID_I_400_450_SLOPE;
    pid_i *= delta;
    pid_i += PID_I_400_WATT;
    
    pid_d = PID_D_400_450_SLOPE;
    pid_d *= delta;
    pid_d += PID_D_400_WATT;
  } else {
    // 450+ Watts
    delta = power_ramp_centi_watts - 45000;
    delta >>= 7;
    pid_p = PID_P_450_500_SLOPE;
    pid_p *= delta;
    pid_p += PID_P_450_WATT;
    
    pid_i = PID_I_450_500_SLOPE;
    pid_i *= delta;
    pid_i += PID_I_450_WATT;
    
    pid_d = PID_D_450_500_SLOPE;
    pid_d *= delta;
    pid_d += PID_D_450_WATT;
  }

  pid_forward_power_kCoeffs[0] = pid_p;
  pid_forward_power_kCoeffs[1] = pid_i;
  pid_forward_power_kCoeffs[2] = pid_d;
  
  PIDCoeffCalc(pid_forward_power_kCoeffs, &pid_forward_power);             // Derive the a,b, & c coefficients from the Kp, Ki & Kd

#endif
  
  if ((PIN_RF_ENABLE == ILL_PIN_RF_ENABLE_ENABLED) && (software_rf_disable == 0) && (power_target_centi_watts > minimum_power_to_operate)) {
    //if ((PIN_RF_ENABLE == ILL_PIN_RF_ENABLE_ENABLED) && (software_rf_disable == 0)) {
    // The RF output should be enabled
    minimum_power_to_operate = MINIMUM_POWER_TARGET - MINIMUM_POWER_TARGET_HYSTERESIS;
    PIN_TEST_POINT_30 = OLL_TP_30_MAX_ATTENUATION;
    PIN_ENABLE_RF_AMP = OLL_PIN_ENABLE_RF_AMP_ENABLED;
    pid_forward_power.controlReference = power_ramp_centi_watts >> 1;
    PID(&pid_forward_power);
  } else {
    power_ramp_centi_watts = 0;
    minimum_power_to_operate = MINIMUM_POWER_TARGET;
    // The RF Output should be disabled
    PIN_TEST_POINT_30 = !OLL_TP_30_MAX_ATTENUATION;
    pid_forward_power.controlReference = 0;
    pid_forward_power.controlOutput = 0;
  }
  
  // Do not let the PID loop saturate at negative 1, make it saturate at zero
  // This will make recovery from negative saturation much faster
  if (pid_forward_power.controlOutput <= 0) {
    pid_forward_power_controlHistory[0] = 0;
    pid_forward_power_controlHistory[1] = 0;
    pid_forward_power_controlHistory[2] = 0;
    pid_forward_power.controlOutput = 0;
  }
  
  
  rf_amplifier_dac_output = pid_forward_power.controlOutput;
  if (rf_amplifier_dac_output & 0x8000) {
    rf_amplifier_dac_output = 0x0000;
  }
  rf_amplifier_dac_output = rf_amplifier_dac_output << 1;
  rf_amplifier_dac_output = 0xFFFF - rf_amplifier_dac_output;  // Invert the slope of the output

#ifdef _OPEN_LOOP_MODE
  //rf_amplifier_dac_output = program_power_level.adc_reading_calibrated;
#endif


/*
#ifdef _DEBUG_MODE
  if (WriteLTC2656TwoChannels(&U6_LTC2656, LTC2656_WRITE_AND_UPDATE_DAC_B, rf_amplifier_dac_output, LTC2656_WRITE_AND_UPDATE_DAC_A, total_forward_power_centi_watts)) {
    LTC2656_write_error_count++;
    gui_debug_value_4 = LTC2656_write_error_count;
  }
#else
*/
  if (WriteLTC2656(&U6_LTC2656, LTC2656_WRITE_AND_UPDATE_DAC_B, rf_amplifier_dac_output)) {
    LTC2656_write_error_count++;
  }
//#endif

  PIN_TEST_POINT_25 = 0;
}

// THIS FUNCTION FOR DEBUGGING ONLY
void _ISRNOPSV _AddressError(void) {
  Nop();
  Nop();
  __asm__ ("Reset");
}


// THIS FUNCTION FOR DEBUGGING ONLY
void _ISRNOPSV _DefaultInterrupt(void) {
  Nop();
  Nop();
  __asm__ ("Reset");
}


