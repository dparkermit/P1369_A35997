#ifndef __SERIAL_A35997_H
#define __SERIAL_A35997_H

#include "A35997.h"

// Command List
#define CMD_READ_RAM_VALUE                              0x30

#define CMD_READ_EEPROM_REGISTER                        0x32

#define CMD_SET_TARGET_POWER                            0x34

#define CMD_RESET                                       0x36

#define CMD_SET_PID                                     0x38



// RAM Locations
#define RAM_READ_STATE                                          0x01
#define RAM_READ_VERSION                                        0x02

#define RAM_READ_FORWARD_POWER_DETECTOR_A_TEMPERATURE           0x10
#define RAM_READ_FORWARD_POWER_DETECTOR_A_POWER                 0x11

#define RAM_READ_FORWARD_POWER_DETECTOR_B_TEMPERATURE           0x18
#define RAM_READ_FORWARD_POWER_DETECTOR_B_POWER                 0x19

#define RAM_READ_REVERSE_POWER_DETECTOR_A_TEMPERATURE           0x20
#define RAM_READ_REVERSE_POWER_DETECTOR_A_POWER                 0x21

#define RAM_READ_REVERSE_POWER_DETECTOR_B_TEMPERATURE           0x28
#define RAM_READ_REVERSE_POWER_DETECTOR_B_POWER                 0x29

#define RAM_READ_PROGRAM_POWER_LEVEL                            0x30
#define RAM_READ_RF_AMPLIFIER_TEMPERATURE                       0x31
#define RAM_READ_TOTAL_FORWARD_POWER                            0x32
#define RAM_READ_PID_DAC_OUTPUT                                 0x33
#define RAM_READ_TOTAL_REVERSE_POWER                            0x34
#define RAM_READ_PID_POWER_TARGET                               0x35


#define RAM_READ_FAULT_REGISTER                                 0x40
#define RAM_READ_FAULT_STATUS_REGISTER                          0x41

#define RAM_READ_OVER_REVERSE_COUNT                             0x50
#define RAM_READ_SCALE_ERRORS                                   0x51
#define RAM_READ_LTC2656_ERRORS                                 0x52
#define RAM_READ_GUI_DEBUG_1                                    0x53
#define RAM_READ_GUI_DEBUG_2                                    0x54
#define RAM_READ_GUI_DEBUG_3                                    0x55
#define RAM_READ_GUI_DEBUG_4                                    0x56

#define RAM_READ_PID_P_COEF                                     0x60
#define RAM_READ_PID_I_COEF                                     0x61
#define RAM_READ_PID_D_COEF                                     0x62




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
extern BUFFERBYTE64 uart1_input_buffer;
extern BUFFERBYTE64 uart1_output_buffer;

void DoSerialCommand(void);


/*
  SIGNAL       USB-COM422-PLUS  <------------------->  Header          SIGNAL
  TXD-               1                                   5             B RXD-
  TXD+               2                                   2             A RXD+
  RXD+               3                                   4             Y TXD+
  RXD-               4                                   3             Z TXD-
  GND                5                                   6              GND
*/


#endif
