#ifndef __FAULTS_A35997_H
#define __FAULTS_A35997_H  

// Fault Registers
extern unsigned int fault_status_register;               // Register that contains the current state of faults, this is not latched
extern unsigned int fault_latched_register;              // Register that latches faults.

#define FAULT_POWER_SUPPLY_1_FAILURE              0x0001 // Power Supply 1 Failure
#define FAULT_POWER_SUPPLY_2_FAILURE              0x0002 // Power Supply 2 Failure
#define FAULT_RF_AMPLIFIER_TEMP_OOR               0x0004 // RF Amplifier Temp out of Range (-40 -> 75 Deg C)
#define FAULT_OUTPUT_POWER_LOW                    0x0008 // Amplifier is unable to generate the requested power.
#define FAULT_OUTPUT_OVER_POWER                   0x0010 // Overpower - Output Power exceeds 550 Watts for more than 1 second
#define FAULT_UNRECOVERABLE_SPI_ERROR             0x0020 // Unrecoverable SPI ERROR
// UNUSED #define FAULT_FWD_PWR_MISMATCH          0x0040 // The difference between the two readbacks is more than 100 Watts for more than 100mS
// UNUSED #define FAULT_REV_PWR_MISMATCH          0x0080 // The difference between the two readbacks is more than 100 Watts for more than 100mS



#define FAULT_FWD_PWR_DET_A_ERROR                 0x0100 // Forward Power Detector A - Power Fault
#define FAULT_FWD_PWR_DET_B_ERROR                 0x0200 // Forward Power Detector A - Power Fault
#define FAULT_REV_PWR_DET_A_ERROR                 0x0400 // Forward Power Detector A - Power Fault
#define FAULT_REV_PWR_DET_B_ERROR                 0x0800 // Forward Power Detector A - Power Fault
#define FAULT_FWD_PWR_DET_A_TEMP_OOR              0x1000 // Forward Power Detector A Temp out of Range (-40 -> 85 Deg C)
#define FAULT_FWD_PWR_DET_B_TEMP_OOR              0x2000 // Forward Power Detector B Temp out of Range (-40 -> 85 Deg C)
#define FAULT_REV_PWR_DET_A_TEMP_OOR              0x4000 // Forward Power Detector A Temp out of Range (-40 -> 85 Deg C)
#define FAULT_REV_PWR_DET_B_TEMP_OOR              0x8000 // Forward Power Detector B Temp out of Range (-40 -> 85 Deg C)



/* 
   Forward Power Detector Power Fault 
   Several Conditions can cause a detector power fault.
   (1) - Power reading exceeds what the device can produce for more than 100mS
   (2) - This channel is delivering less than 25% of the total output power (assuming the output power is more than 10 watts)


   Reverse Power Detector Power Faults
   (1) - Power reading exceeds the greater of (total foward power or 50 Watts) for more than 100mS
   (2) - Power reading of this channel is not withing 100 watts of the other channel

   
*/


#define FAULT_MASK_TEMPERATURE_FAULT              (FAULT_RF_AMPLIFIER_TEMP_OOR | FAULT_FWD_PWR_DET_A_TEMP_OOR | FAULT_FWD_PWR_DET_B_TEMP_OOR | FAULT_REV_PWR_DET_A_TEMP_OOR | FAULT_REV_PWR_DET_B_TEMP_OOR)
#define FAULT_MASK_GENERAL_FAULT                  (FAULT_POWER_SUPPLY_1_FAILURE | FAULT_POWER_SUPPLY_2_FAILURE | FAULT_OUTPUT_OVER_POWER | FAULT_FWD_PWR_DET_A_DATA_ERROR | FAULT_FWD_PWR_DET_B_DATA_ERROR | FAULT_REV_PWR_DET_A_DATA_ERROR | FAULT_REV_PWR_DET_B_DATA_ERROR | FAULT_FWD_PWR_MISMATCH | FAULT_REV_PWR_MISMATCH | FAULT_UNRECOVERABLE_SPI_ERROR)


unsigned int FaultCheckOverTemp(void);
unsigned int FaultCheckGeneralFault(void);
unsigned int CheckReflectedPowerFault(void);
void ResetAllFaults(void);



#endif
