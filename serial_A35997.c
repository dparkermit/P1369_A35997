#include "Serial.h"
#include "A34760_PINS.h"
#include "Buffer64.h"
#include "A34760.h"
#include "config.h"
#include "faults.h"
#include "Version.h"
#include <libpic30.h>

/*
  Serial Commands

  A single command is stored in command_string
  If there is a valid command stored in command_string, then the command_string.data_state = COMMAND_BUFFER_FULL
  If there is NOT a volid command stored in command_string, then command_string.data_state = COMMAND_BUFFER_EMPTY
  
  
  When a byte is received on the UART it is transfered to the "uart1_input_buffer" by the UART receive interrupt - the input buffer is a circular buffer that is 64 Bytes deep
  (see buffer64.h for more infor on the buffer)
  
  Every time through the command loop (200us to 1ms) DoSerialCommand() will be called
  If the command_string is empty, then the input buffer is searched for a valid command (the oldest valid command will be moved to command_string)

  If a command was found OR the command_string was already full, then the command is executed.

  Assume an average execution cycle of 1mS and 9 bytes per command.  A command rate of 72 K Baund can be sustained. (57.6 K Baud Standard will work)
  
  Assume an average execution cycle of 500uS and 9 bytes per command, A command rate of 144 K Baud can be sustained (115.2 K Baud Standard should be safe) 

*/

void LookForCommand(void);
void ExecuteCommand(void);
unsigned char CheckCRC(unsigned int crc);
unsigned int MakeCRC(unsigned char command_byte, unsigned char register_byte, unsigned int data_word);
unsigned int ReadFromRam(unsigned int ram_location);
void SendCommand(unsigned char command_byte, unsigned char register_byte, unsigned int data_word);

struct CommandStringStruct command_string;


void DoSerialCommand(void) {
  /* 
     Look for a command and execute it.
  */
  
  if (command_string.data_state != COMMAND_BUFFER_FULL) {
    LookForCommand();
  }
  if (command_string.data_state == COMMAND_BUFFER_FULL) {
    ExecuteCommand();
  }
}



void LookForCommand(void) {
  unsigned char read_byte;
  unsigned int crc;
  /*
    If the state is "waitng for command" then it looks for a command in the buffer, if the state is "executing command" it does nothing
    
    To look for a command in the buffer.
    1) See if there are enough bytes in the buffer to contain a command.
    2) If there are look for command sync
       2b) If there are less bytes in the buffer than it takes to make a command, exit
    3) If command Syncs, check the checksum ^ increment the read_position as each byte is read
       3b) If command does not sync, increment the the read positon and return to step 1    
    4) If the checksum checks out, move the command data into the command data structure
    4b) If the checksum fails, return to step 1     
  */
  
  while ((command_string.data_state == COMMAND_BUFFER_EMPTY) && (Buffer64BytesInBuffer(&uart1_input_buffer) >= COMMAND_LENGTH)) {
    // Look for a command
    read_byte = Buffer64ReadByte(&uart1_input_buffer);
    if (read_byte == SYNC_BYTE_1) {
      read_byte = Buffer64ReadByte(&uart1_input_buffer);
      if (read_byte == SYNC_BYTE_2) {
	read_byte = Buffer64ReadByte(&uart1_input_buffer);
	if (read_byte == SYNC_BYTE_3_RECEIVE) {
	  // All of the sync bytes matched, this should be a valid command
	  command_string.command_byte   = Buffer64ReadByte(&uart1_input_buffer);
	  command_string.data_high_byte = Buffer64ReadByte(&uart1_input_buffer);
	  command_string.data_low_byte  = Buffer64ReadByte(&uart1_input_buffer);
	  command_string.register_byte  = Buffer64ReadByte(&uart1_input_buffer);
	  crc                           = Buffer64ReadByte(&uart1_input_buffer);
	  crc                           = (crc << 8) + Buffer64ReadByte(&uart1_input_buffer);
	  if (CheckCRC(crc)) {
	    command_string.data_state = COMMAND_BUFFER_FULL;
	  }
	}
      }
    }
  }
}


void SendCommand(unsigned char command_byte, unsigned char register_byte, unsigned int data_word) {
  unsigned int crc;
  crc = MakeCRC(command_byte, register_byte, data_word);
  Buffer64WriteByte(&uart1_output_buffer, SYNC_BYTE_1);
  Buffer64WriteByte(&uart1_output_buffer, SYNC_BYTE_2);
  Buffer64WriteByte(&uart1_output_buffer, SYNC_BYTE_3_SEND);
  Buffer64WriteByte(&uart1_output_buffer, command_byte);
  Buffer64WriteByte(&uart1_output_buffer, (data_word >> 8));
  Buffer64WriteByte(&uart1_output_buffer, (data_word & 0x00FF));
  Buffer64WriteByte(&uart1_output_buffer, register_byte);
  Buffer64WriteByte(&uart1_output_buffer, (crc >> 8));
  Buffer64WriteByte(&uart1_output_buffer, (crc & 0x00FF));

  if ((!U1STAbits.UTXBF) && (Buffer64IsNotEmpty(&uart1_output_buffer))) {
    /*
      There is at least one byte available for writing in the outputbuffer and the transmit buffer is not full.
      Move a byte from the output buffer into the transmit buffer
      All subsequent bytes will be moved from the output buffer to the transmit buffer by the U1 TX Interrupt
    */
    U1TXREG = Buffer64ReadByte(&uart1_output_buffer);
  }
}


void ExecuteCommand(void) {
  unsigned int itemp;
  unsigned int vtemp;
  unsigned int data_word;
  unsigned int return_data_word;
  unsigned int return_command_byte;
  
  data_word = command_string.data_high_byte;
  data_word = data_word << 8;
  data_word = data_word + command_string.data_low_byte;
  
  return_data_word = data_word;
  return_command_byte = command_string.command_byte;
  switch (command_string.command_byte) 
    {

    case CMD_READ_RAM_VALUE:
      return_data_word = ReadFromRam(command_string.register_byte);
      break;

    case CMD_SET_MAGNET_PS_CAL_DATA:
      ps_magnet_config_ram_copy[command_string.register_byte] = data_word;
      break;
      
    case CMD_READ_MAGNET_PS_CAL_DATA:
      return_data_word = ps_magnet_config_ram_copy[command_string.register_byte];
      break;
      
    case CMD_SAVE_MAGNET_PS_CAL_DATA_TO_EEPROM:
      _wait_eedata();
      _erase_eedata(EE_address_ps_magnet_config_in_EEPROM, _EE_ROW);
      _wait_eedata();
      _write_eedata_row(EE_address_ps_magnet_config_in_EEPROM, ps_magnet_config_ram_copy);
      break;

    case CMD_SET_MAGNETRON_FILAMENT_VOLTAGE:
      vtemp = data_word;
      itemp = GenerateFilamentIprog(vtemp);
      SetPowerSupplyTarget(&ps_filament, vtemp, itemp);
      ps_filament_config_ram_copy[EEPROM_V_SET_POINT] = ps_filament.v_command_set_point;
      ps_filament_config_ram_copy[EEPROM_I_SET_POINT] = ps_filament.i_command_set_point;
      _wait_eedata();
      _erase_eedata(EE_address_ps_filament_config_in_EEPROM, _EE_ROW);
      _wait_eedata();
      _write_eedata_row(EE_address_ps_filament_config_in_EEPROM, ps_filament_config_ram_copy);
      break;

    case CMD_CLEAR_PROCESSOR_RESET_DATA:
      // DPARKER using this command to reset "reset data"
      debug_status_register = 0;
      _POR = 0;
      _EXTR = 0;
      _SWR = 0;
      _BOR = 0;
      _TRAPR = 0;
      _WDTO = 0;
      _IOPUWR = 0;
      last_known_action = LAST_ACTION_CLEAR_LAST_ACTION;
      processor_crash_count = 0;
      break;

    }
  
  // Echo the command that was recieved back to the controller
  SendCommand(return_command_byte, command_string.register_byte, return_data_word);
  
  command_string.data_state = COMMAND_BUFFER_EMPTY;
}



unsigned int ReadFromRam(unsigned int ram_location) {
  unsigned long temp_long;
  unsigned int data_return;
  unsigned long long int temp_long_long;
  switch (ram_location) 
    {
      // Fault information
    case RAM_READ_DEBUG_STATUS_REG:
      data_return = debug_status_register;
      break;

    case RAM_READ_FAULT_MAGNETRON_FAULT_REG:
      data_return = faults_magnetron_fault_reg;
      break;

      // Read Status
    case RAM_READ_STATE:
      data_return = control_state;
      break;
      
    case RAM_READ_VERSION:
      data_return = VERSION_NUMBER;
      break;
      
      
      // Read Bedug Counters

    }  
  return data_return;
}


unsigned int MakeCRC(unsigned char command_byte, unsigned char register_byte, unsigned int data_word) {
  unsigned int crc;
  crc = SYNC_BYTE_1 + SYNC_BYTE_2 + SYNC_BYTE_3_SEND;
  crc += command_byte + register_byte;
  crc += (data_word >> 8);
  crc += (data_word & 0x00FF);
  
  return crc;
  // DPAKRER Make real CRC
}


unsigned char CheckCRC(unsigned int crc) {
  unsigned int crcCheck;
  // At the moment the CRC is just a checksum
  crcCheck = SYNC_BYTE_1 + SYNC_BYTE_2 + SYNC_BYTE_3_RECEIVE; 
  crcCheck += command_string.command_byte + command_string.register_byte;
  crcCheck += command_string.data_high_byte + command_string.data_low_byte;
  if (crcCheck == crc) {
    return 1;
  } else {
    return 0;
  }
  // DPARKER make Real CRC

}



void _ISRNOPSV _U1RXInterrupt(void) {
  _U1RXIF = 0;
  while (U1STAbits.URXDA) {
    Buffer64WriteByte(&uart1_input_buffer, U1RXREG);
  }
}



void _ISRNOPSV _U1TXInterrupt(void) {
  _U1TXIF = 0;
  while ((!U1STAbits.UTXBF) && (Buffer64BytesInBuffer(&uart1_output_buffer))) {
    /*
      There is at least one byte available for writing in the outputbuffer and the transmit buffer is not full.
      Move a byte from the output buffer into the transmit buffer
    */
    U1TXREG = Buffer64ReadByte(&uart1_output_buffer);
  }
}

