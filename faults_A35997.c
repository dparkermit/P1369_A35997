#include "faults_A35997.h"

unsigned int fault_status_register;               // Register that contains the current state of faults, this is not latched
unsigned int fault_latched_register;              // Register that latches faults.



void ResetAllFaults(void) {
  // DPARKER WRITE THIS
}

void RollOffCalculation(void) {
  // DPARKER WRITE THIS
}

void UpdateFaults(void) {

  // Check that power supply 1 is working
  if (PIN_PS_ONE_OK != ILL_PS_OK) {
    // Set the status register and the fault register
    fault_status_register |= FAULT_POWER_SUPPLY_1_FAILURE;
    fault_latched_register |= FAULT_POWER_SUPPLY_1_FAILURE;
  } else {
    // Clear the status register
    fault_status_register &= !FAULT_POWER_SUPPLY_1_FAILURE;
  }

  // Check that power supply 2 is working
  if (PIN_PS_TWO_OK != ILL_PS_OK) {
    // Set the status register and the fault register
    fault_status_register |= FAULT_POWER_SUPPLY_2_FAILURE;
    fault_latched_register |= FAULT_POWER_SUPPLY_2_FAILURE;
  } else {
    // Clear the status register
    fault_status_register &= !FAULT_POWER_SUPPLY_2_FAILURE;
  }

  // Check that the RF Amplifier is operating within acceptable temperature range
  if (1) {
   
    
  } else {
    
  }


  // Detector Temperature scaling
  // = 600 mV @ 25 deg C +/- 2mV per deg C
  // = 470mV at -40 Deg C
  // = 720mV at 85 deg C


  
}

unsigned int FaultCheckOverTemp(void) {
  return (fault_latched_register & FAULT_MASK_TEMPERATURE_FAULT);
}

unsigned int FaultCheckGeneralFault(void) {
  return (fault_latched_register & FAULT_MASK_GENERAL_FAULT);
}


unsigned int CheckReflectedPowerFault(void) {
  return 0;
  // DPARKER NEED TO WRITE THIS
}
