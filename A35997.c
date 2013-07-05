#include "faults_A35997.h"

// Debugging Information

unsigned int sp1_bus_error_count;
unsigned int sp2_bus_error_count;
unsigned int LTC2656_write_error_count;


LTC2656 U6_LTC2656;


// global variables
unsigned char control_state;


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
    SetFrontPanelLedState(GREEN);
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
    SetFrontPanelLedState(BLUE);
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
	control_state = STATE_RF_ON_FOLDBACK:
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
  // Debug Counter Initializations
  unsigned int sp1_bus_error_count;
  unsigned int sp2_bus_error_count;
  unsigned int LTC2656_write_error_count;


  
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
  OpenSPI1((A35997_SPI1CON_VALUE & A35997_SPI1CON_CLOCK), A35997_SPI1STAT_VALUE);  // Configure SPI bus 1 based on H file parameters
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

  ResetAllFaults();
  
  // --------------- Initialize U6 - LTC2656 ------------------------- //
  U6_LTC2656.pin_cable_select = _PIN_RD15;
  U6_LTC2656.pin_dac_clear = _PIN_RB15;
  U6_LTC2656.pin_load_dac = _PIN_RD14;
  U6_LTC2656.pin_por_select = _PIN_NOT_CONNECTED;
  U6_LTC2656.por_select_value = 0;
  U6_LTC2656.spi_port = SPI_PORT_1;

  SetupLTC2656(&U6_LTC2656);
  

  
  // ---- Configure the dsPIC ADC Module ------------ //
  ADCON1 = A34760_ADCON1_VALUE;             // Configure the high speed ADC module based on H file parameters
  ADCON2 = A34760_ADCON2_VALUE;             // Configure the high speed ADC module based on H file parameters
  ADCON3 = A34760_ADCON3_VALUE;             // Configure the high speed ADC module based on H file parameters
  ADCHS  = A34760_ADCHS_VALUE;              // Configure the high speed ADC module based on H file parameters
  
  ADPCFG = A34760_ADPCFG_VALUE;             // Set which pins are analog and which are digital I/O
  ADCSSL = A34760_ADCSSL_VALUE;             // Set which analog pins are scanned


  // Begin External ADC operation (this uses SPI2)
  spi2_state = SPI2_STATE_READ_B_AND_START_CONVERSION;
  _SPI2IF = 0;
  _SPI2IE = 1;
  
  // Begin Internal ADC operation
  _ADIF = 0;
  _ADIE = 1;  
  ADCON1bits.ADON = 1;
  
  // Begin T1 (100us) Timer and Interrupt Operation
  _T1IF = 0;
  _T1IE = 1;
  T1CONbits.TON = 1;

  // Begin UART operation
  PIN_RS422_DE = OLL_RS422_DE_ENABLE_RS422_DRIVER  // Enable the RS422 Driver output (The reciever is always enabled)
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
  ptr_rf_det.adc_reading_calibrated = adc_reading;
  // DPARKER - Need to do calibration
}

void CalibrateDetectorLevel(RF_DETECTOR* ptr_rf_det) {
  ptr_rf_det->detector_level_calibrated = ptr_rf_det.adc_reading_calibrated;
  // DPARKER - Need to do calibration
}

void ConvertDetectorLevelToPowerCentiWatts(RF_DETECTOR* prt_rf_det) {
  // NEED TO MAKE A 12 bit lookuptable and store in Flash
  // Convert to 12 bit number
  // Read from lookup table [n]
  // Read from loopup table [n+1]
  // Linear extrapolate between the two

  // Result = X[n];
  // Result += ( (X[n+1] - X[n]) * 4_LSB ) >> 4;
  // 

  return 0;
  // DPARKER need to make conversion
}

unsigned char led_pulse_count;
unsigned char start_reset_process;

void Do10msTicToc() {

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
  
    RolloffCalculation();  // DPARKER FIGURE THIS OUT
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
  reverse_detector_a_level = RCFilterNTau(reverse_detector_A_level, averaged_adc_reading, RC_FILTER_8_TAU);

  averaged_adc_reading = AverageADC128(reverse_detector_b_array);
  reverse_detector_b_level = RCFilterNTau(reverse_detector_b_level, averaged_adc_reading, RC_FILTER_8_TAU);

}




// ------------ ISRs ---------------- //
unsigned int reverse_detector_a_level = 1;
unsigned int reverse_detector_b_level = 1;

unsigned int rf_amplifier_temp = 1;
unsigned int fwd_rf_det_a_temp = 1;
unsigned int fwd_rf_det_b_temp = 1;
unsigned int rev_rf_det_a_temp = 1;
unsigned int rev_rf_det_b_temp = 1;

volatile unsigned int power_program = 1;
volatile unsigned char adc_result_index;

volatile unsigned int reverse_detector_a_array[128];
volatile unsigned int reverse_detector_b_array[128];
volatile unsigned int rf_amplifier_temperature_array[128];
volatile unsigned int fwd_rf_det_a_temperature_array[128];
volatile unsigned int fwd_rf_det_b_temperature_array[128];
volatile unsigned int rev_rf_det_a_temperature_array[128];
volatile unsigned int rev_rf_det_b_temperature_array[128];



volatile unsigned int spi2_state;
volatile unsigned int forward_detector_a_array[16];
volatile unsigned int forward_detector_b_array[16];
volatile unsigned char AD7686_result_index;

volatile unsigned int measured_power_a;
volatile unsigned int measured_power_b;
volatile unsigned int forward_power;




// SPI2 Interrupt - External ADC

#define SPI2_STATE_READ_B_AND_START_CONVERSION    0
#define SPI2_STATE_START_A_TRANSFER               1
#define SPI2_STATE_READ_A_AND_START_B_TRANSFER    2

void _ISRFASTNOPSV _SPI2Interrupt(void) {
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
    throw_away_data = SPI2BUF;
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
}



// ADC Interrupt
void _ISRNOPSV _ADCInterrupt(void) {
  unsigned int power_level_average;

  // _ASAM = 0; // Stop Auto Sampling
  _ADIF = 0;

  // Copy Data From Buffer to RAM
  if (_BUFS) {
    // read from 0x0 -> 0x7
    power_level_average = ADCBUF0 + ADCBUF2 + ADCBUF4 + ADCBUF6;
    reverse_detector_B_array[adc_result_index] = ADCBUF1;
    reverse_detector_A_array[adc_result_index] = ADCBUF3;
    rf_amplifier_temperature_array[adc_result_index] = ADCBUF5;
  } else {
    power_level_average = ADCBUF8 + ADCBUFA + ADCBUFC + ADCBUFE;
    fwd_rf_det_a_temperature_array[adc_result_index] = ADCBUF9;
    fwd_rf_det_b_temperature_array[adc_result_index] = ADCBUFB;
    rev_rf_det_a_temperature_array[adc_result_index] = ADCBUFD;
    rev_rf_det_b_temperature_array[adc_result_index] = ADCBUFF;

    adc_result_index++;
    adc_result_index &= 0b01111111;
  }
  power_level_average <= 2; // Scale power level average up to 16 bits
  power_program = RCFilterNTau(power_program, power_level_average, RC_FILTER_8_TAU);

  // DPARKER it may be better to store the power_program in a circular buffer and average/filter it in the T1 interrupt
  // DPARKER filter the power level from the customer withour require an external function call.   That will make this interrupt much faster.

  //_ASAM = 1; // Start Auto Sampling
}

volatile unsigned int power_target_centi_watts;
volatile unsigned int total_forward_power_centi_watts;

// PID Interrupt on TMR1 (100us)
void _ISRNOPSV _T1Interrupt(void) {
  power_target_centi_watts = ConvertProgramLevelToPowerCentiWatts(CalirbrateADCReading(adc_average_power_program));
  
  if ((PIN_RF_ENABLE == ILL_PIN_RF_ENABLE_ENABLED) && (software_rf_disable == 0) && (power_target_centi_watts > MINIMUM_POWER_TARGET)) {
    // The RF output is enabled
    PIN_ENABLE_RF_AMP = OLL_PIN_ENABLE_RF_AMP_ENABLED;
    forward_power_pid.target = power_target_centi_watts;
    if (software_foldback_mode_enable) {
      if (forward_power_pid.target > FOLDBACK_POWER_PROGRAM) {
        forward_power_pid.target = FOLDBACK_POWER_PROGRAM;
      }
    }
  } else {
    // The RF Output should be disabled
    PIN_ENABLE_RF_AMP = !OLL_PIN_ENABLE_RF_AMP_ENABLED;
    forward_power_pid.target = 0;
  }

  CalibrateADCReading(&forward_power_detector_A, AverageADC16(forward_detector_a_array));
  CalibrateDetectorLevel(&forward_detector_A);
  ConvertDetectorLevelToPowerCentiWatts(&forward_detector_A);
  // DPARKER, there is no reason why these three functions shouldn't be a single step
  
  CalibrateADCReading(&forward_power_detector_B, AverageADC16(forward_detector_b_array));
  CalibrateDetectorLevel(&forward_detector_B);
  ConvertDetectorLevelToPowerCentiWatts(&forward_detector_B);

  total_forward_power_centi_watts = forward_detector_B.power_reading_centi_watts + forward_detector_A.power_reading_centi_watts

  
  forward_power_pid.target = pid_power_target;
  forward_power_pid.measure_output = forward_power;

  DoETMPid(&forward_power_pid); // DPARKER need to write all the PID functions
}
