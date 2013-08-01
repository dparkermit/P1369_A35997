#ifndef __FAULTS_A35997_H
#define __FAULTS_A35997_H  

#include "A35997.h"

// Fault Registers
extern unsigned int fault_status_register;               // Register that contains the current state of faults, this is not latched
extern unsigned int fault_latched_register;              // Register that latches faults.

#define FAULT_POWER_SUPPLY_1_FAILURE              0x0001 // Power Supply 1 Failure
#define FAULT_POWER_SUPPLY_2_FAILURE              0x0002 // Power Supply 2 Failure
#define FAULT_RF_AMPLIFIER_TEMP_OOR               0x0004 // RF Amplifier Temp out of Range (-40 -> 75 Deg C)
#define FAULT_OUTPUT_POWER_LOW                    0x0008 // Amplifier is unable to generate the requested power for more than X milliseconds
#define FAULT_OUTPUT_OVER_POWER                   0x0010 // Overpower - Output Power exceeds the max power for more than X milliseconds
#define FAULT_UNRECOVERABLE_SPI_ERROR             0x0020 // Unrecoverable Processor fault (SPI / EEPROM / Whatever)
#define FAULT_FWD_PWR_MISMATCH                    0x0040 // The difference between the two readbacks is more than N1 Watts for more than X1 milliseconds
#define FAULT_REV_PWR_MISMATCH                    0x0080 // The difference between the two readbacks is more than N2 Watts for more than X2 milliseconds



#define FAULT_FWD_PWR_DET_A_ERROR                 0x0100 // Forward Power Detector A Fault
#define FAULT_FWD_PWR_DET_B_ERROR                 0x0200 // Forward Power Detector B Fault
#define FAULT_REV_PWR_DET_A_ERROR                 0x0400 // Reverse Power Detector A Fault
#define FAULT_REV_PWR_DET_B_ERROR                 0x0800 // Reverse Power Detector B Fault
#define FAULT_FWD_PWR_DET_A_TEMP_OOR              0x1000 // Forward Power Detector A Temp out of Range (-40 -> 85 Deg C)
#define FAULT_FWD_PWR_DET_B_TEMP_OOR              0x2000 // Forward Power Detector B Temp out of Range (-40 -> 85 Deg C)
#define FAULT_REV_PWR_DET_A_TEMP_OOR              0x4000 // Reverse Power Detector A Temp out of Range (-40 -> 85 Deg C)
#define FAULT_REV_PWR_DET_B_TEMP_OOR              0x8000 // Reverse Power Detector B Temp out of Range (-40 -> 85 Deg C)



/* 
   Forward Power Detector Power Fault 
   The point of this fault is to identify if a detector has failed.
   (1) The detector reading is outside the valid range for this device.  In our case this would be an over power reading for X milliseconds
   (2) The two detectors readings are not within the greater of 30% or 10 watts of each other for X milliseconds
*/


#define FAULT_MASK_TEMPERATURE_FAULT              (FAULT_RF_AMPLIFIER_TEMP_OOR | FAULT_FWD_PWR_DET_A_TEMP_OOR | FAULT_FWD_PWR_DET_B_TEMP_OOR | FAULT_REV_PWR_DET_A_TEMP_OOR | FAULT_REV_PWR_DET_B_TEMP_OOR)


#define FAULT_MASK_GENERAL_FAULT                  (FAULT_POWER_SUPPLY_1_FAILURE | FAULT_POWER_SUPPLY_2_FAILURE | FAULT_OUTPUT_POWER_LOW | FAULT_OUTPUT_OVER_POWER | FAULT_UNRECOVERABLE_SPI_ERROR | FAULT_FWD_PWR_DET_A_ERROR | FAULT_FWD_PWR_DET_B_ERROR | FAULT_REV_PWR_DET_A_ERROR | FAULT_REV_PWR_DET_B_ERROR | FAULT_FWD_PWR_MISMATCH | FAULT_REV_PWR_MISMATCH)



//#define ACTIVE_FAULT_MASK      (FAULT_MASK_TEMPERATURE_FAULT | FAULT_MASK_GENERAL_FAULT)
#define ACTIVE_FAULT_MASK        0x0000 // Ignore all faults


extern unsigned int over_refected_power_count;

unsigned int FaultCheckOverTemp(void);

unsigned int FaultCheckGeneralFault(void);

unsigned int CheckReflectedPowerFault(void);

void ResetAllFaults(void);

void UpdateFaults(void);



#endif
