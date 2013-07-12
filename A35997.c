#include "A35997.h"
#include "faults_A35997.h"
#include "A35997_DETECTOR_LOOK_UP_TABLE.h"



#define SPI2_STATE_READ_B_AND_START_CONVERSION    0
#define SPI2_STATE_START_A_TRANSFER               1
#define SPI2_STATE_READ_A_AND_START_B_TRANSFER    2


const unsigned int detector_voltage_to_power_look_up_table[4096] = {DETECTOR_LOOK_UP_TABLE_VALUES};


// Control structers for the thyratron heater PID loops
tPID pid_forward_power;
fractional pid_forward_power_controlHistory[3] __attribute__ ((section (".ybss, bss, ymemory")));
fractional pid_forward_power_abcCoefficient[3] __attribute__ ((section (".xbss, bss, xmemory")));
fractional pid_forward_power_kCoeffs[] = {0,0,0};



// Debugging Information

unsigned int sp1_bus_error_count;
unsigned int sp2_bus_error_count;
unsigned int LTC2656_write_error_count;


LTC2656 U6_LTC2656;


RF_DETECTOR forward_power_detector_A;
RF_DETECTOR forward_power_detector_B;

RF_DETECTOR reverse_power_detector_A;
RF_DETECTOR reverse_power_detector_B;

RF_DETECTOR program_power_level;


unsigned int reverse_detector_a_level = 1;
unsigned int reverse_detector_b_level = 1;

unsigned int rf_amplifier_temp = 1;
unsigned int fwd_rf_det_a_temp = 1;
unsigned int fwd_rf_det_b_temp = 1;
unsigned int rev_rf_det_a_temp = 1;
unsigned int rev_rf_det_b_temp = 1;

volatile unsigned int power_program = 1;
volatile unsigned char adc_result_index;

unsigned int reverse_detector_a_array[128];
unsigned int reverse_detector_b_array[128];
unsigned int rf_amplifier_temperature_array[128];
unsigned int fwd_rf_det_a_temperature_array[128];
unsigned int fwd_rf_det_b_temperature_array[128];
unsigned int rev_rf_det_a_temperature_array[128];
unsigned int rev_rf_det_b_temperature_array[128];



volatile unsigned int spi2_state;
unsigned int forward_detector_a_array[16];
unsigned int forward_detector_b_array[16];
volatile unsigned char AD7686_result_index;

volatile unsigned int measured_power_a;
volatile unsigned int measured_power_b;
volatile unsigned int forward_power;

volatile unsigned int power_target_centi_watts;
volatile unsigned int total_forward_power_centi_watts;
volatile unsigned int rf_amplifier_dac_output;




// global variables
unsigned char control_state;
unsigned char software_foldback_mode_enable = 0;
unsigned char software_rf_disable = 1;




unsigned char led_pulse_count;
unsigned char start_reset_process;


void DoA35997StartUp(void);
void CalibrateADCReading(RF_DETECTOR* ptr_rf_det, unsigned int adc_reading);
void CalibrateDetectorLevel(RF_DETECTOR* ptr_rf_det);
void ConvertDetectorLevelToPowerCentiWatts(RF_DETECTOR* ptr_rf_det);
void Do10msTicToc(void);
void FilterADCs(void);


unsigned int ConvertProgramLevelToPowerCentiWatts(unsigned int program_level);

void SetFrontPanelLedState(unsigned char led_state);
#define SOLID_GREEN                1
#define SOLID_BLUE                 2
#define FLASH_BLUE                 3
#define SOLID_RED                  4
#define FLASH_RED                  5



#define DPARKER_LED_FLASH_COMPLETE 1

void DoA35997StateMachine(void) {
  
  switch(control_state) {
    
  case STATE_START_UP:
    DoA35997StartUp();
    control_state = STATE_FLASH_LEDS_AT_STARTUP;
    break;

  case STATE_FLASH_LEDS_AT_STARTUP:
    PIN_SUM_FLT = !OLL_PIN_SUM_FAULT_FAULTED;
    while (control_state == STATE_FLASH_LEDS_AT_STARTUP) {
      DoSerialCommand();
      Do10msTicToc();
      if (FaultCheckOverTemp()) {
	control_state = STATE_FAULT_OVER_TEMP;
      } else if (FaultCheckGeneralFault()) {
	control_state = STATE_FAULT_GENERAL_FAULT;
      } else if (DPARKER_LED_FLASH_COMPLETE) {
	control_state = STATE_RF_OFF; 
      }
    }
    break;    
    
  case STATE_RF_OFF:
    PIN_SUM_FLT = !OLL_PIN_SUM_FAULT_FAULTED;
    software_foldback_mode_enable = 0;
    software_rf_disable = 0; 
    SetFrontPanelLedState(SOLID_GREEN);
    // We don't want to wait for the software loop to start the amplifier so this must be pre-set to the on condition.
    while (control_state == STATE_RF_OFF) {
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
    software_foldback_mode_enable = 0;
    software_rf_disable = 0;
    SetFrontPanelLedState(SOLID_BLUE);
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
    software_foldback_mode_enable = 1;
    SetFrontPanelLedState(FLASH_BLUE);
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
    software_rf_disable = 1;
    SetFrontPanelLedState(FLASH_RED);
    while (control_state == STATE_FAULT_OVER_TEMP) {
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
    software_rf_disable = 1;
    SetFrontPanelLedState(SOLID_RED);
    while (control_state == STATE_FAULT_GENERAL_FAULT) {
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
  
  // DPARKER Initialize PID
  


  // --------- BEGIN IO PIN CONFIGURATION ------------------
  
  // Initialize the output latches before setting TRIS
  PIN_TEST_POINT_24_CLK_OUT = 0;
  PIN_TEST_POINT_25 = 0;
  PIN_TEST_POINT_26 = 0;
  PIN_TEST_POINT_27 = 0;
  PIN_TEST_POINT_28 = 0;
  PIN_TEST_POINT_29 = 0;
  PIN_TEST_POINT_30 = 0;

  PIN_TEST_LED_1 = TEST_LED_OFF;
  PIN_TEST_LED_2 = TEST_LED_OFF;
  PIN_TEST_LED_3 = TEST_LED_OFF;
  PIN_TEST_LED_4 = TEST_LED_OFF;

  PIN_FRONT_PANEL_LED_GREEN = FRONT_PANEL_LED_OFF;
  PIN_FRONT_PANEL_LED_BLUE = FRONT_PANEL_LED_OFF;
  PIN_FRONT_PANEL_LED_RED = FRONT_PANEL_LED_OFF;

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
  uart1_input_buffer.write_location = 0;  
  uart1_input_buffer.read_location = 0;
  uart1_output_buffer.write_location = 0;
  uart1_output_buffer.read_location = 0;
  
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
  // DPARKER move parameters to H file
  pid_forward_power.abcCoefficients = &pid_forward_power_abcCoefficient[0];    /*Set up pointer to derived coefficients */
  pid_forward_power.controlHistory = &pid_forward_power_controlHistory[0];     /*Set up pointer to controller history samples */

  PIDInit(&pid_forward_power);                                                    /*Clear the controler history and the controller output */
  pid_forward_power_kCoeffs[0] = Q15(0.02);
  pid_forward_power_kCoeffs[1] = Q15(0.08);
  pid_forward_power_kCoeffs[2] = Q15(0.08);
  PIDCoeffCalc(&pid_forward_power_kCoeffs[0], &pid_forward_power);             /*Derive the a,b, & c coefficients from the Kp, Ki & Kd */





  ResetAllFaults();
  
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
  spi2_state = SPI2_STATE_READ_B_AND_START_CONVERSION;
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



/*  RF Detector Functions */

/* 
   The data calibration process

   (1) Data is read by ADC (internal or external and stored in a circular buffer
   (2) The circular buffer is averaged (by AverageADC128 or AverageADC16 deppending upon the buffer type)
   (3 - ONLY SLOW SIGNALS) - The averaged result is low pass filtered
   (4) The ADCReading is Calibrated for offset & gain (offset & gain are temperature compensated) -> Stored as RF_DETECTOR.adc_reading_calibrated
   (5) This value has the RF detector calibration applied (This must be done per device over temperature) -> Stored as RF_DETECTOR.detector_level_calibrated
   (6) This value is converted into centi_watts via a 12 bit lookup table -> Stored as power_reading_centi_watts

*/

void CalibrateADCReading(RF_DETECTOR* ptr_rf_det, unsigned int adc_reading) {
  ptr_rf_det->adc_reading_calibrated = adc_reading;
  // DPARKER - Need to do calibration
}

void CalibrateDetectorLevel(RF_DETECTOR* ptr_rf_det) {
  ptr_rf_det->detector_level_calibrated = ptr_rf_det->adc_reading_calibrated;
  // DPARKER - Need to do calibration
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
  ptr_rf_det->power_reading_centi_watts = power;
}


unsigned int ConvertProgramLevelToPowerCentiWatts(unsigned int program_level) {
  unsigned long data;
  unsigned int return_data;


  // 10V program = 530 Watts
  // Input scalling = V_in / 5
  // 2V = 530 Watts
  // 0xFFFF = 542.72 Watts = 54272 Centi Watts
  // Need to multiply by 54272 and shit right 16 bits

  data = program_level;
  data *= 54272;
  data >>= 16;
  return_data = data;
  return return_data;
}

void Do10msTicToc(void) {

  ClrWdt();
  if (_T2IF) {
    _T2IF = 0;
    //10ms roll has occured

    // Flash LED 1 on the control board
    led_pulse_count = ((led_pulse_count + 1) & 0b00001111);
    if (led_pulse_count == 0) {
      // 10ms * 16 counter has ocurred
      if (PIN_TEST_LED_1) {
	PIN_TEST_LED_1 = 0;
      } else {
	PIN_TEST_LED_1 = 1;
      }  
    }
    

    // ----------- Fault Reset Logic --------- //
    // Faults are reset on a positive transition of the PIN_RF_ENABLE Signal
    if ((PIN_RF_ENABLE == ILL_PIN_RF_ENABLE_ENABLED) && (start_reset_process)) {
      ResetAllFaults();
    }
    if (PIN_RF_ENABLE == !ILL_PIN_RF_ENABLE_ENABLED) {
      // Start the Reset Process
      start_reset_process = 1;
    } else {
      start_reset_process = 0;
    }
    // ResetSPI1(); // ResetSPI(2) // DPARKER - This may be requried to fix bit errors in the SPI bus
    
    

    FilterADCs();  // Read Data from the DAC arrays, average, and filter
    
    
    UpdateFaults();  // Update all the fault registers.  Note this must only happen once every 10ms because some faults are timed
  
    RollOffCalculation();  // DPARKER FIGURE THIS OUT

  } 
}
 
 
 
void FilterADCs(void) {
  unsigned int averaged_adc_reading;
  // Forward Detector A and B are read by the external ADC and filtered by the SPI Interrupt because their updated values are needed immediately
  // Power Level Program from the customer is read by the internal ADC and filtered by the internal ADC because their updated values are needed immediately
  
  // Reverse Detector A and B - Do these need to be managed in real life or is 10 Hz (10ms x8 tau) fast enough?
  
  // rf amplifier temperature & all 4 detector temperatures
  // These are averaged over 128 samples then low pass filtered
  // The reverse power detector data is averaged and low pass filtered (at a higher frequency)
  averaged_adc_reading = AverageADC128(rf_amplifier_temperature_array);
  rf_amplifier_temp = RCFilterNTau(rf_amplifier_temp, averaged_adc_reading, RC_FILTER_64_TAU);

  averaged_adc_reading = AverageADC128(fwd_rf_det_a_temperature_array);
  fwd_rf_det_a_temp = RCFilterNTau(fwd_rf_det_a_temp, averaged_adc_reading, RC_FILTER_64_TAU);

  averaged_adc_reading = AverageADC128(fwd_rf_det_b_temperature_array);
  fwd_rf_det_b_temp = RCFilterNTau(fwd_rf_det_b_temp, averaged_adc_reading, RC_FILTER_64_TAU);
  
  averaged_adc_reading = AverageADC128(rev_rf_det_a_temperature_array);
  rev_rf_det_a_temp = RCFilterNTau(rev_rf_det_a_temp, averaged_adc_reading, RC_FILTER_64_TAU);

  averaged_adc_reading = AverageADC128(rev_rf_det_b_temperature_array);
  rev_rf_det_b_temp = RCFilterNTau(rev_rf_det_b_temp, averaged_adc_reading, RC_FILTER_64_TAU);
  
  averaged_adc_reading = AverageADC128(reverse_detector_a_array);
  reverse_detector_a_level = RCFilterNTau(reverse_detector_a_level, averaged_adc_reading, RC_FILTER_8_TAU);

  averaged_adc_reading = AverageADC128(reverse_detector_b_array);
  reverse_detector_b_level = RCFilterNTau(reverse_detector_b_level, averaged_adc_reading, RC_FILTER_8_TAU);

}


void SetFrontPanelLedState(unsigned char led_state) {
  // DPARKER WRITE THIS SUB
}


// ------------ ISRs ---------------- //




// SPI2 Interrupt - External ADC

void _ISRFASTNOPSV _SPI2Interrupt(void) {
  // DPARKER - between 1/3 and 1/2 of the processor time is spent processing this ISR, consider how it can be re-written to be faster and move it to assembly

  // With 29.5 MHz Clock this samples each signal at about 100 KHz
  unsigned int trash;
  PIN_TEST_POINT_27 = 1;  // Time this interrupt
  _SPI2IF = 0;
  switch(spi2_state) {
  
  case SPI2_STATE_READ_B_AND_START_CONVERSION:
    // (*) SPI Interrupt Saves SPIBUF to Data 2, CS2->High, Convert->High. Convert->Low, Exits = 16 bit delay + execution time  
    PIN_AD7686_2_CS = !OLL_AD7686_SELECT_DEVICE; 
    PIN_AD7686_CONVERT = !OLL_AD7686_START_CONVERSION;
    forward_detector_b_array[AD7686_result_index] = SPI2BUF;
    PIN_AD7686_CONVERT = OLL_AD7686_START_CONVERSION;
    SPI2BUF = 0xAAAA;
    spi2_state = SPI2_STATE_START_A_TRANSFER;
    AD7686_result_index++;
    AD7686_result_index &= 0b00001111; // 16 element circular buffer
    break;

  case SPI2_STATE_START_A_TRANSFER:
    // (*) SPI Interrupt CS1->Low and starts 16 bit transfer, exits = 16 bit delay + execution time
    trash = SPI2BUF;
    PIN_AD7686_1_CS = OLL_AD7686_SELECT_DEVICE;
    SPI2BUF = 0xAAAA;
    spi2_state = SPI2_STATE_READ_A_AND_START_B_TRANSFER;
    break;
    
  case SPI2_STATE_READ_A_AND_START_B_TRANSFER:
    PIN_AD7686_1_CS = !OLL_AD7686_SELECT_DEVICE;
    forward_detector_a_array[AD7686_result_index] = SPI2BUF;
    PIN_AD7686_2_CS = OLL_AD7686_SELECT_DEVICE;
    SPI2BUF = 0xAAAA;
    spi2_state = SPI2_STATE_READ_B_AND_START_CONVERSION;
    break;
  }
  PIN_TEST_POINT_27 = 0;
}



// ADC Interrupt
void _ISRNOPSV _ADCInterrupt(void) {
  unsigned int power_level_average;
  _ASAM = 0;
  PIN_TEST_POINT_26 = 1; // TIME this interrupt
  // _ASAM = 0; // Stop Auto Sampling
  _ADIF = 0;

  // Copy Data From Buffer to RAM
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
  power_program = RCFilterNTau(power_program, power_level_average, RC_FILTER_2_TAU);

  // DPARKER it may be better to store the power_program in a circular buffer and average/filter it in the T1 interrupt
  // DPARKER filter the power level from the customer without require an external function call.   That will make this interrupt much faster.

  //_ASAM = 1; // Start Auto Sampling
  PIN_TEST_POINT_26 = 0;
  _ASAM = 1;
}


#define MINIMUM_POWER_TARGET            100           // 1 Watt
#define FOLDBACK_POWER_PROGRAM          25000         // 250 Watts

// PID Interrupt on TMR1 (100us) 
void _ISRNOPSV _T1Interrupt(void) {
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
  // Time this interrupt
  PIN_TEST_POINT_25 = 1; // Time this interrupt
  _T1IF = 0;

  CalibrateADCReading(&program_power_level, power_program);
  power_target_centi_watts = ConvertProgramLevelToPowerCentiWatts(program_power_level.adc_reading_calibrated);
  
  if ((PIN_RF_ENABLE == ILL_PIN_RF_ENABLE_ENABLED) && (software_rf_disable == 0) && (power_target_centi_watts > MINIMUM_POWER_TARGET)) {
    // The RF output is enabled
    PIN_ENABLE_RF_AMP = OLL_PIN_ENABLE_RF_AMP_ENABLED;
    pid_forward_power.controlReference = power_target_centi_watts;
    if (software_foldback_mode_enable) {
      if (pid_forward_power.controlReference > FOLDBACK_POWER_PROGRAM) {
        pid_forward_power.controlReference = FOLDBACK_POWER_PROGRAM;
      }
    }
  } else {
    // The RF Output should be disabled
    PIN_ENABLE_RF_AMP = !OLL_PIN_ENABLE_RF_AMP_ENABLED;
    pid_forward_power.controlReference = 0;
  }
  CalibrateADCReading(&forward_power_detector_A, (AverageADC16(forward_detector_a_array)));
  CalibrateDetectorLevel(&forward_power_detector_A);
  ConvertDetectorLevelToPowerCentiWatts(&forward_power_detector_A);
  // DPARKER, there is no reason why these three functions shouldn't be a single step
  
  CalibrateADCReading(&forward_power_detector_B, AverageADC16(forward_detector_b_array));
  CalibrateDetectorLevel(&forward_power_detector_B);
  ConvertDetectorLevelToPowerCentiWatts(&forward_power_detector_B);
  // DPARKER, there is no reason why these three functions shouldn't be a single step
  
  total_forward_power_centi_watts = forward_power_detector_B.power_reading_centi_watts + forward_power_detector_A.power_reading_centi_watts;

  pid_forward_power.controlReference = (power_target_centi_watts >> 1);
  pid_forward_power.measuredOutput = (total_forward_power_centi_watts >> 1);
  PID(&pid_forward_power);
  
  rf_amplifier_dac_output = pid_forward_power.controlOutput;
  if (rf_amplifier_dac_output & 0x8000) {
    rf_amplifier_dac_output = 0x0000;
  }
  rf_amplifier_dac_output = rf_amplifier_dac_output << 1;

  //WriteLTC2656(&U6_LTC2656, LTC2656_WRITE_AND_UPDATE_DAC_B, rf_amplifier_dac_output);
  WriteLTC2656(&U6_LTC2656, LTC2656_WRITE_AND_UPDATE_DAC_B, power_target_centi_watts);  // DPARKER TESTING
  // This Write LTC2656 command takes a very long time, make this an interrupt driven process???
  
  PIN_TEST_POINT_25 = 0;
}


void _ISRNOPSV _AddressError(void) {
  Nop();
  Nop();
  __asm__ ("Reset");
}


// DPARKER THIS FUNCTION FOR DEBUGGING ONLY
void _ISRNOPSV _DefaultInterrupt(void) {
  Nop();
  Nop();
  //__asm__ ("Reset");
}
