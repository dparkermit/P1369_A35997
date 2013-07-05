#ifndef __SERIAL_H
#define __SERIAL_H

// Command List

#define CMD_SET_MAGNETRON_CURRENT_REMOTE_MODE           0x10
#define CMD_SET_MAGNETRON_CURRENT_LOCAL_MODE            0x11

#define CMD_PAC_SET_MODE_A                              0x20
#define CMD_PAC_SET_MODE_B                              0x21

#define CMD_SET_MAGNETRON_MAGNET_CURRENT                0x2B
#define CMD_SET_LAMBDA_VOLTAGE_MODE_A                   0x2A
#define CMD_SET_LAMBDA_VOLTAGE_MODE_B                   0x2D
#define CMD_SET_MAGNETRON_FILAMENT_VOLTAGE              0x2C

#define CMD_READ_RAM_VALUE                              0x30

#define CMD_CLEAR_PROCESSOR_RESET_DATA                  0x40

#define CMD_FORCE_SOFTWARE_RESTART                      0xA0
#define CMD_SOFTWARE_SKIP_WARMUP                        0xA1

#define CMD_SET_MAGNET_PS_CAL_DATA                      0xD0
#define CMD_READ_MAGNET_PS_CAL_DATA                     0xD1
#define CMD_SAVE_MAGNET_PS_CAL_DATA_TO_EEPROM           0xD2

#define CMD_SET_FILAMENT_PS_CAL_DATA                    0xD3
#define CMD_READ_FILAMENT_PS_CAL_DATA                   0xD4
#define CMD_SAVE_FILAMENT_PS_CAL_DATA_TO_EEPROM         0xD5

#define CMD_SET_THYR_CATHODE_PS_CAL_DATA                0xD6
#define CMD_READ_THYR_CATHODE_PS_CAL_DATA               0xD7
#define CMD_SAVE_THYR_CATHODE_PS_CAL_DATA_TO_EEPROM     0xD8

#define CMD_SET_THYR_RESERVOIR_PS_CAL_DATA              0xD9
#define CMD_READ_THYR_RESERVOIR_PS_CAL_DATA             0xDA
#define CMD_SAVE_THYR_RESERVOIR_PS_CAL_DATA_TO_EEPROM   0xDB

#define CMD_SET_HV_LAMBDA_MODE_A_CAL_DATA               0xDC
#define CMD_READ_HV_LAMBDA_MODE_A_CAL_DATA              0xDD
#define CMD_SAVE_HV_LAMBDA_MODE_A_CAL_DATA_TO_EEPROM    0xDE

#define CMD_SET_HV_LAMBDA_MODE_B_CAL_DATA               0xDF
#define CMD_READ_HV_LAMBDA_MODE_B_CAL_DATA              0xE0
#define CMD_SAVE_HV_LAMBDA_MODE_B_CAL_DATA_TO_EEPROM    0xE1               

#define CMD_SET_MAGNETRON_MODE_A_CAL_DATA               0xE2
#define CMD_READ_MAGNETRON_MODE_A_CAL_DATA              0xE3
#define CMD_SAVE_MAGNETRON_MODE_A_CAL_DATA_TO_EEPROM    0xE4

#define CMD_SET_MAGNETRON_MODE_B_CAL_DATA               0xE5
#define CMD_READ_MAGNETRON_MODE_B_CAL_DATA              0xE6
#define CMD_SAVE_MAGNETRON_MODE_B_CAL_DATA_TO_EEPROM    0xE7

#define CMD_SET_CNTRL_CAL_DATA                          0xE8
#define CMD_READ_CNTRL_CAL_DATA                         0xE9
#define CMD_SAVE_CNTRL_CAL_DATA_TO_EEPROM               0xEA








// RAM Locations
#define RAM_READ_STATE                                          0x01
#define RAM_READ_VERSION                                        0x02
#define RAM_READ_LOCAL_REMOTE_MAGNET_CURRENT_CONTROL            0x03

#define RAM_READ_THYR_CATH_HTR_VOTAGE_SET_POINT                 0x11
#define RAM_READ_THYR_CATH_HTR_VOTAGE_ADC                       0x10
#define RAM_READ_THYR_RESER_HTR_VOLTAGE_SET_POINT               0x13
#define RAM_READ_THYR_RESER_HTR_VOLTAGE_ADC                     0x12


#define RAM_READ_MAGNETRON_FILAMENT_VOLTAGE_SET_POINT           0x21
#define RAM_READ_MAGNETRON_FILAMENT_VOLTAGE_ADC                 0x20
#define RAM_READ_MAGNETRON_FILAMENT_CURRENT_ADC                 0x22


#define RAM_READ_MAGNETRON_MAGNET_CURRENT_SET_POINT             0x31
#define RAM_READ_MAGNETRON_MAGNET_CURRENT_ADC                   0x30
#define RAM_READ_MAGNETRON_MAGNET_VOLTAGE_ADC                   0x32


#define RAM_READ_HV_LAMBDA_SET_POINT_MODE_A                     0x40
#define RAM_READ_HV_LAMBDA_SET_POINT_MODE_B                     0x48


#define RAM_READ_PREVIOUS_PULSE_MAGNETRON_VOLTAGE_ADC           0x50
#define RAM_READ_PREVIOUS_PULSE_MAGNETRON_CURRENT_ADC           0x51
#define RAM_READ_ARC_COUNTER_PERSISTENT_HIGH_WORD               0x56
#define RAM_READ_ARC_COUNTER_PERSISTENT_LOW_WORD                0x57
#define RAM_READ_ARC_COUNTER_THIS_HV_ON                         0x58
#define RAM_READ_PULSE_COUNTER_PERSISTENT_WORD_3_MSB            0x59
#define RAM_READ_PULSE_COUNTER_PERSISTENT_WORD_2                0x5A
#define RAM_READ_PULSE_COUNTER_PERSISTENT_WORD_1                0x5B
#define RAM_READ_PULSE_COUNTER_PERSISTENT_WORD_0_LSB            0x5C
#define RAM_READ_PULSE_COUNTER_THIS_HV_ON_HIGH_WORD             0x5D
#define RAM_READ_PULSE_COUNTER_THIS_HV_ON_LOW_WORD              0x5E


#define RAM_READ_AVERAGE_PULSE_ENERGY                           0x60
#define RAM_READ_AVERAGE_PULSE_FREQUENCY                        0x61
#define RAM_READ_AVERAGE_MAGNETRON_INPUT_POWER                  0x62
#define RAM_READ_HV_LAMBDA_VPEAK_ADC                            0x63
#define RAM_READ_HV_LAMBDA_VMON_ADC                             0x64


#define RAM_READ_PULSE_MODE_A_FILTERED_CURRENT                  0x70
#define RAM_READ_PULSE_MODE_A_FILTERED_VOLTAGE                  0x71
#define RAM_READ_PULSE_MODE_A_MAX_CURRENT                       0x72
#define RAM_READ_PULSE_MODE_A_MIN_CURRENT                       0x73
#define RAM_READ_PULSE_MODE_A_MAX_VOLTAGE                       0x74
#define RAM_READ_PULSE_MODE_A_MIN_VOLTAGE                       0x75

#define RAM_READ_PULSE_MODE_B_FILTERED_CURRENT                  0x78
#define RAM_READ_PULSE_MODE_B_FILTERED_VOLTAGE                  0x79
#define RAM_READ_PULSE_MODE_B_MAX_CURRENT                       0x7A
#define RAM_READ_PULSE_MODE_B_MIN_CURRENT                       0x7B
#define RAM_READ_PULSE_MODE_B_MAX_VOLTAGE                       0x7C
#define RAM_READ_PULSE_MODE_B_MIN_VOLTAGE                       0x7D


#define RAM_READ_DEBUG_STATUS_REG                               0x90
#define RAM_READ_FAULT_MAGNETRON_FAULT_REG                      0x91
#define RAM_READ_FAULT_MAGNETRON_STATUS_REG                     0x92
#define RAM_READ_FAULT_MAGNETRON_WARNING_REG                    0x93
#define RAM_READ_FAULT_HIGH_VOLTAGE_FAULT_REG                   0x94
#define RAM_READ_FAULT_HIGH_VOLTAGE_STATUS_REG                  0x95
#define RAM_READ_FAULT_HIGH_VOLTAGE_WARNING_REG                 0x96
#define RAM_READ_FAULT_THYRATRON_FAULT_REG                      0x97
#define RAM_READ_FAULT_THYRATRON_STATUS_REG                     0x98
#define RAM_READ_FAULT_THYRATRON_WARNING_REG                    0x99
#define RAM_READ_FAULT_CONTROL_BOARD_FAULT_REG                  0x9A
#define RAM_READ_FAULT_CONTROL_BOARD_STATUS_REG                 0x9B
#define RAM_READ_FAULT_CONTROL_BOARD_WARNING_REG                0x9C


#define RAM_READ_COUNT_MAGNETRON_CURRENT_ADC_GLITCH             0xA0
#define RAM_READ_COUNT_MAGNETRON_VOLTAGE_ADC_GLITCH             0xA1
#define RAM_READ_COUNT_I2C_BUS_ERROR                            0xA2
#define RAM_READ_COUNT_SPI1_BUS_ERROR                           0xA3
#define RAM_READ_COUNT_SPI2_BUS_ERROR                           0xA4
#define RAM_READ_COUNT_EXTERNAL_ADC_FALSE_TRIGGER               0xA5
#define RAM_READ_COUNT_LTC2656_WRITE_ERROR                      0xA6
#define RAM_READ_COUNT_SETPOINT_NOT_VALID                       0xA7
#define RAM_READ_COUNT_SCALE16BIT_SATURATION                    0xA8
#define RAM_READ_COUNT_REVERSESCALE16BIT_SATURATION             0xA9
#define RAM_READ_COUNT_TIMING_ERROR_INT1                        0xAA
#define RAM_READ_COUNT_PROCESSOR_CRASH                          0xAB
#define RAM_READ_LAST_ACTION_BEFORE_CRASH                       0xAC
#define RAM_READ_COUNT_LVD_INTERRUPT                            0xAD
#define RAM_READ_LAST_OSCCON_BEFORE_CRASH                       0xAE

/*





#define CMD_SET_THYR_CATHODE_HTR              0x2D
#define CMD_SET_THYR_RESERVOIR_HTR            0x2F



#define CMD_READ_EEPROM_REGISTER              0x32



#define CMD_READ_RAM_HV_LAMBDA_A              0x40
#define CMD_READ_RAM_HV_LAMBDA_B              0x41
#define CMD_READ_RAM_FILAMENT                 0x42
#define CMD_READ_RAM_MAGNET                   0x43
#define CMD_READ_THYR_CATHODE_HTR             0x44
#define CMD_READ_THYR_RESERVOIR_HTR           0x45



#define CMD_ENABLE_SYSTEM                     0x50
#define CMD_DISABLE_SYSTEM                    0x51
#define CMD_HV_ON                             0x52
#define CMD_HV_OFF                            0x53
#define CMD_RESET_ALL_FAULTS                  0x54

#define CMD_DEBUG_1                           0x60


#define CMD_COULD_NOT_EXECUTE_NOW             0x90





*/



#define COMMAND_BUFFER_EMPTY  0x00
#define COMMAND_BUFFER_FULL   0x02

#define COMMAND_LENGTH        9
#define SYNC_BYTE_1           0xF1
#define SYNC_BYTE_2           0xF2
#define SYNC_BYTE_3_RECEIVE   0xF3
#define SYNC_BYTE_3_SEND      0xF4




struct CommandStringStruct {
  unsigned char command_byte;
  unsigned char register_byte;
  unsigned char data_high_byte;
  unsigned char data_low_byte;
  unsigned char data_state;
};

extern struct CommandStringStruct command_string;

void DoSerialCommand(void);

unsigned int GenerateMagnetVprog(unsigned int iprog);

#endif
