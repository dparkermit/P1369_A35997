#include "faults_A35997.h"

unsigned int fault_status_register;               // Register that contains the current state of faults, this is not latched
unsigned int fault_latched_register;              // Register that latches faults.

unsigned int over_refected_power_count = 0;

unsigned char CheckDetectorTemperatureOutOfRange(RF_DETECTOR* ptr_rf_det);

unsigned char CheckDetectorMismatch(RF_DETECTOR* ptr_rf_det_a, RF_DETECTOR* ptr_rf_det_b);

unsigned char CheckDetectorOverPower(RF_DETECTOR* ptr_rf_det);

void ResetAllFaults(void) {
  fault_latched_register = 0x0000;
}

void RollOffCalculation(void) {
  // DPARKER WRITE THIS
}

void UpdateFaults(void) {
  unsigned int max_power_relative_to_prgram;
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
  
  max_power_relative_to_prgram = ETMScale16Bit(total_forward_power_centi_watts, FAULT_OVER_POWER_MULT, 1);
  if (total_forward_power_centi_watts >= max_power_relative_to_prgram) {

  }

  // DPARKER - OUTPUT POWER LOW
  // DPARKER - OUTPUT OVER POWER
  // DPARKER - UNRECOVERABLE ERROR
  // DPARKER - FWD_PWR_MISMATCH
  // DPARKER - REV_PWR_MISMATCH



  if (CheckDetectorMismatch(&forward_power_detector_A, &forward_power_detector_B)) {
    
  }

  if (CheckDetectorMismatch(&reverse_power_detector_A, &reverse_power_detector_B)) {
    
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
}

unsigned char CheckDetectorTemperatureOutOfRange(RF_DETECTOR* ptr_rf_det) {
  if ((ptr_rf_det->detector_temperature < RF_DETECTOR_TEMPERATURE_MIN) || (ptr_rf_det->detector_temperature > RF_DETECTOR_TEMPERATURE_MAX)) {
    return 1;
  } else {
    return 0;
  }
}

unsigned char CheckDetectorMismatch(RF_DETECTOR* ptr_rf_det_a, RF_DETECTOR* ptr_rf_det_b) {
  return 0;  // DPARKER WRITE THIS
}

unsigned char CheckDetectorOverPower(RF_DETECTOR* ptr_rf_det) {
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
