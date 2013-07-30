#include "faults_A35997.h"

unsigned int fault_status_register;               // Register that contains the current state of faults, this is not latched
unsigned int fault_latched_register;              // This latches faults.
// DPARKER, need to update fault handeling a bit, need to determine faults/status info what does what and how to store

unsigned int over_refected_power_count = 0;

unsigned int CheckDetectorTemperatureOutOfRange(RF_DETECTOR* ptr_rf_det);
unsigned int CheckDetectorOverPower(RF_DETECTOR* ptr_rf_det);
unsigned int CheckReverseDetectorMismatch(void);
unsigned int CheckForwardDetectorMismatch(void);


void UpdateUnderPowerCounter(void);
void UpdateOverPowerCounter(void);

unsigned int over_power_counter = 0;
unsigned int under_power_counter = 0;
unsigned int forward_detector_mismatch_count = 0;
unsigned int reverse_detector_mismatch_count = 0;


void ResetAllFaults(void) {
  fault_latched_register = 0x0000;
  over_power_counter = 0;
  under_power_counter = 0;
  forward_detector_mismatch_count = 0;
  reverse_detector_mismatch_count = 0;
  forward_power_detector_A.over_max_power_count = 0;
  forward_power_detector_B.over_max_power_count = 0;
  reverse_power_detector_A.over_max_power_count = 0;
  reverse_power_detector_B.over_max_power_count = 0;
}

void UpdateFaults(void) {
  unsigned int max_reflected_power;
  // Check that power supply 1 is working
  if (PIN_PS_ONE_OK != ILL_PS_OK) {
    // Set the status register and the fault register
    fault_status_register |= FAULT_POWER_SUPPLY_1_FAILURE;
    fault_latched_register |= FAULT_POWER_SUPPLY_1_FAILURE;
  } else {
    // Clear the status register
    fault_status_register &= ~FAULT_POWER_SUPPLY_1_FAILURE;
  }
  
  // Check that power supply 2 is working
  if (PIN_PS_TWO_OK != ILL_PS_OK) {
    // Set the status register and the fault register
    fault_status_register |= FAULT_POWER_SUPPLY_2_FAILURE;
    fault_latched_register |= FAULT_POWER_SUPPLY_2_FAILURE;
  } else {
    // Clear the status register
    fault_status_register &= ~FAULT_POWER_SUPPLY_2_FAILURE;
  }

  // Check that the RF Amplifier is operating within acceptable temperature range
  if ((program_power_level.detector_temperature < RF_AMPLIFIER_TEMPERATURE_MIN) || (program_power_level.detector_temperature > RF_AMPLIFIER_TEMPERATURE_MAX)) {
    // rf amplifier temperature is out of range
    fault_status_register |= FAULT_RF_AMPLIFIER_TEMP_OOR;
    fault_latched_register |= FAULT_RF_AMPLIFIER_TEMP_OOR;
  } else {
    fault_status_register &= ~FAULT_RF_AMPLIFIER_TEMP_OOR;
  }

  
  // ---------- Check for over power condition --------- //
  // DPARKER chage UpdateOverPowerCounter to CheckOverPower add comparison to the sub
  UpdateOverPowerCounter();
  if (over_power_counter >= FAULT_TIME_OVER_POWER) {
    fault_status_register |= FAULT_OUTPUT_OVER_POWER;
    fault_latched_register |= FAULT_OUTPUT_OVER_POWER; 
  } else {
    fault_status_register &= ~FAULT_OUTPUT_OVER_POWER;
  }
  
  // --------- Check for under power condition -------- //
  // DPARKER chage UpdateUnderPowerCounter to CheckOverPower add comparison to the sub
  UpdateUnderPowerCounter();
  if (under_power_counter >= FAULT_TIME_UNDER_POWER) {
    fault_status_register |= FAULT_OUTPUT_POWER_LOW;
    fault_latched_register |= FAULT_OUTPUT_POWER_LOW; 
  } else {
    fault_status_register &= ~FAULT_OUTPUT_POWER_LOW;
  }


  // DPARKER - UNRECOVERABLE ERROR



  // Forward Power mismatch
  if (CheckForwardDetectorMismatch()){
    fault_status_register |= FAULT_FWD_PWR_MISMATCH;
    fault_latched_register |= FAULT_FWD_PWR_MISMATCH; 
  } else {
    fault_status_register &= ~FAULT_FWD_PWR_MISMATCH;
  }
  
  // Reverse Power mismatch
  if (CheckReverseDetectorMismatch()){
    fault_status_register |= FAULT_REV_PWR_MISMATCH;
    fault_latched_register |= FAULT_REV_PWR_MISMATCH; 
  } else {
    fault_status_register &= ~FAULT_REV_PWR_MISMATCH;
  }

  

  if (CheckDetectorOverPower(&forward_power_detector_A)) {
    // rf amplifier temperature is out of range
    fault_status_register |= FAULT_FWD_PWR_DET_A_ERROR;
    fault_latched_register |= FAULT_FWD_PWR_DET_A_ERROR;
  } else {
    fault_status_register &= ~FAULT_FWD_PWR_DET_A_ERROR;
  }

  if (CheckDetectorOverPower(&forward_power_detector_B)) {
    // rf amplifier temperature is out of range
    fault_status_register |= FAULT_FWD_PWR_DET_B_ERROR;
    fault_latched_register |= FAULT_FWD_PWR_DET_B_ERROR;
  } else {
    fault_status_register &= ~FAULT_FWD_PWR_DET_B_ERROR;
  }

  if (CheckDetectorOverPower(&reverse_power_detector_A)) {
    // rf amplifier temperature is out of range
    fault_status_register |= FAULT_REV_PWR_DET_A_ERROR;
    fault_latched_register |= FAULT_REV_PWR_DET_A_ERROR;
  } else {
    fault_status_register &= ~FAULT_REV_PWR_DET_A_ERROR;
  }

  if (CheckDetectorOverPower(&reverse_power_detector_B)) {
    // rf amplifier temperature is out of range
    fault_status_register |= FAULT_REV_PWR_DET_B_ERROR;
    fault_latched_register |= FAULT_REV_PWR_DET_B_ERROR;
  } else {
    fault_status_register &= ~FAULT_REV_PWR_DET_B_ERROR;
  }

  if (CheckDetectorTemperatureOutOfRange(&forward_power_detector_A)) {
    fault_status_register |= FAULT_FWD_PWR_DET_A_TEMP_OOR;
    fault_latched_register |= FAULT_FWD_PWR_DET_A_TEMP_OOR;
  } else {
    fault_status_register &= ~FAULT_FWD_PWR_DET_A_TEMP_OOR;
  } 

  if (CheckDetectorTemperatureOutOfRange(&forward_power_detector_B)) {
    fault_status_register |= FAULT_FWD_PWR_DET_B_TEMP_OOR;
    fault_latched_register |= FAULT_FWD_PWR_DET_B_TEMP_OOR;
  } else {
    fault_status_register &= ~FAULT_FWD_PWR_DET_B_TEMP_OOR;
  } 

  if (CheckDetectorTemperatureOutOfRange(&reverse_power_detector_A)) {
    fault_status_register |= FAULT_REV_PWR_DET_A_TEMP_OOR;
    fault_latched_register |= FAULT_REV_PWR_DET_A_TEMP_OOR;
  } else {
    fault_status_register &= ~FAULT_REV_PWR_DET_A_TEMP_OOR;
  } 

  if (CheckDetectorTemperatureOutOfRange(&reverse_power_detector_B)) {
    fault_status_register |= FAULT_REV_PWR_DET_B_TEMP_OOR;
    fault_latched_register |= FAULT_REV_PWR_DET_B_TEMP_OOR;
  } else {
    fault_status_register &= ~FAULT_REV_PWR_DET_B_TEMP_OOR;
  } 
  
  // Integrate how much time reflected power has been over the max over reflected value

  if (software_foldback_mode_enable) {
    max_reflected_power = MAX_FOLDBACK_REFLECTED_POWER;
  } else {
    max_reflected_power = MAX_STEADY_STATE_REFLECTED_POWER;
  }
  if (total_reverse_power_centi_watts > max_reflected_power) {
    over_refected_power_count++;
    if (over_refected_power_count > (MAX_OVER_REFLECTED_TIME + MAX_OVER_REFLECTED_TIME_HYSTERESIS)) {
      over_refected_power_count = MAX_OVER_REFLECTED_TIME + MAX_OVER_REFLECTED_TIME_HYSTERESIS;
    }
  } else {
    if (over_refected_power_count > 0) {
      over_refected_power_count--;
    }
  }

  fault_latched_register &= FAULT_MASK;
}







unsigned int CheckDetectorTemperatureOutOfRange(RF_DETECTOR* ptr_rf_det) {
  if ((ptr_rf_det->detector_temperature < RF_DETECTOR_TEMPERATURE_MIN) || (ptr_rf_det->detector_temperature > RF_DETECTOR_TEMPERATURE_MAX)) {
    return 1;
  } else {
    return 0;
  }
}

unsigned int CheckForwardDetectorMismatch(void) {
  unsigned int difference;
  unsigned int max_difference;
  unsigned int a_power;
  unsigned int b_power;

  a_power = forward_power_detector_A.power_reading_centi_watts;
  b_power = forward_power_detector_B.power_reading_centi_watts;
  if (a_power > b_power) {
    difference = a_power - b_power;
    max_difference = ETMScale16Bit(a_power, FORWARD_DETECTOR_MISMATCH_SCALE, 0);
  } else {
    difference = b_power - a_power;
    max_difference = ETMScale16Bit(b_power, FORWARD_DETECTOR_MISMATCH_SCALE, 0);
  }
  if (max_difference < FORWARD_DETECTOR_MISMATCH_MINIMUM_FOR_FAULT) {
    max_difference = FORWARD_DETECTOR_MISMATCH_MINIMUM_FOR_FAULT;
  }
  
  if (difference > max_difference) {
    forward_detector_mismatch_count++;
    if (forward_detector_mismatch_count >= FORWARD_DETECTOR_MISMATCH_TIME) {
      forward_detector_mismatch_count = FORWARD_DETECTOR_MISMATCH_TIME;
    }
  } else {
    if (forward_detector_mismatch_count > 0) {
      forward_detector_mismatch_count--;
    }
  }
  gui_debug_value_1 = forward_detector_mismatch_count;
  gui_debug_value_2 = max_difference;



  if (forward_detector_mismatch_count >= FORWARD_DETECTOR_MISMATCH_TIME) {
    return 1;
  } else {
    return 0;
  }
} 

unsigned int CheckReverseDetectorMismatch(void) {
  unsigned int difference;
  unsigned int max_difference;
  unsigned int a_power;
  unsigned int b_power;

  a_power = reverse_power_detector_A.power_reading_centi_watts;
  b_power = reverse_power_detector_B.power_reading_centi_watts;
  if (a_power > b_power) {
    difference = a_power - b_power;
    max_difference = ETMScale16Bit(a_power, REVERSE_DETECTOR_MISMATCH_SCALE, 0);
  } else {
    difference = b_power - a_power;
    max_difference = ETMScale16Bit(b_power, REVERSE_DETECTOR_MISMATCH_SCALE, 0);
  }
  if (max_difference < REVERSE_DETECTOR_MISMATCH_MINIMUM_FOR_FAULT) {
    max_difference = REVERSE_DETECTOR_MISMATCH_MINIMUM_FOR_FAULT;
  }
  
  if (difference > max_difference) {
    reverse_detector_mismatch_count++;
    if (reverse_detector_mismatch_count >= REVERSE_DETECTOR_MISMATCH_TIME) {
      reverse_detector_mismatch_count = REVERSE_DETECTOR_MISMATCH_TIME;
    }
  } else {
    if (reverse_detector_mismatch_count > 0) {
      reverse_detector_mismatch_count--;
    }
  }

  gui_debug_value_3 = reverse_detector_mismatch_count;
  gui_debug_value_4 = max_difference;

  if (reverse_detector_mismatch_count >= REVERSE_DETECTOR_MISMATCH_TIME) {
    return 1;
  } else {
    return 0;
  }
} 




unsigned int CheckDetectorOverPower(RF_DETECTOR* ptr_rf_det) {
  if (ptr_rf_det->power_reading_centi_watts > ptr_rf_det->max_power) {
    ptr_rf_det->over_max_power_count++;
  } else {
    if (ptr_rf_det->over_max_power_count > 0) {
      ptr_rf_det->over_max_power_count--;
    }
  }
  
  if (ptr_rf_det->over_max_power_count > ptr_rf_det->over_power_trip_time) {
    ptr_rf_det->over_max_power_count--;
    return 1;
  } else {
    return 0;
  }
}

unsigned int FaultCheckOverTemp(void) {
  return (fault_latched_register & FAULT_MASK_TEMPERATURE_FAULT);
}

unsigned int FaultCheckGeneralFault(void) {
  return (fault_latched_register & FAULT_MASK_GENERAL_FAULT);
}

unsigned int CheckReflectedPowerFault(void) {
  if (over_refected_power_count >= MAX_OVER_REFLECTED_TIME) {
    return 1;
  } else {
    return 0;
  }
}

void UpdateOverPowerCounter(void) {
  unsigned int max_power_relative_to_prgram;
  unsigned int target_power;

  target_power = pid_forward_power.controlReference << 1;
  max_power_relative_to_prgram = ETMScale16Bit(target_power, FAULT_OVER_POWER_MULT, 1);
  if (max_power_relative_to_prgram < (target_power + FAULT_MINIMUM_OVER_POWER)) {
    max_power_relative_to_prgram = target_power + FAULT_MINIMUM_OVER_POWER;
  }
  if (total_forward_power_centi_watts >= max_power_relative_to_prgram) {
    over_power_counter++;
    if (over_power_counter >= FAULT_TIME_OVER_POWER) {
      over_power_counter = FAULT_TIME_OVER_POWER;
    }
  } else {
    if (over_power_counter > 0) {
      over_power_counter--;
    }
  }
}



void UpdateUnderPowerCounter(void) {
  unsigned int min_power_relative_to_prgram;
  unsigned int target_power;

  target_power = pid_forward_power.controlReference << 1;
  if (target_power <= FAULT_MINIMUM_OVER_POWER) {
    // We can not get an under power no watter what the power reading because our program level is so low
    min_power_relative_to_prgram = 0;
    if (under_power_counter > 0) {
      under_power_counter--;
    }
  } else {
    min_power_relative_to_prgram = ETMScale16Bit(target_power, FAULT_UNDER_POWER_MULT, 0);
    if (min_power_relative_to_prgram > (target_power - FAULT_MINIMUM_UNDER_POWER)) {
      min_power_relative_to_prgram = target_power - FAULT_MINIMUM_UNDER_POWER;
    }
    if (total_forward_power_centi_watts <= min_power_relative_to_prgram) {
      under_power_counter++;
      if (under_power_counter >= FAULT_TIME_UNDER_POWER) {
	under_power_counter = FAULT_TIME_UNDER_POWER;
      }
    } else {
      if (under_power_counter > 0) {
	under_power_counter--;
      }
    }
  }
}
