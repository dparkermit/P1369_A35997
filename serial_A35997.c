#include "serial_A35997.h"


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
g
*/

void LookForCommand(void);
void ExecuteCommand(void);
unsigned char CheckCRC(unsigned int crc);
unsigned int MakeCRC(unsigned char command_byte, unsigned char register_byte, unsigned int data_word);
unsigned int ReadFromRam(unsigned int ram_location);
void SendCommand(unsigned char command_byte, unsigned char register_byte, unsigned int data_word);

struct CommandStringStruct command_string;
BUFFERBYTE64 uart1_input_buffer;
BUFFERBYTE64 uart1_output_buffer;



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
  
  while ((command_string.data_state == COMMAND_BUFFER_EMPTY) && (BufferByte64BytesInBuffer(&uart1_input_buffer) >= COMMAND_LENGTH)) {
    // Look for a command
    read_byte = BufferByte64ReadByte(&uart1_input_buffer);
    if (read_byte == SYNC_BYTE_1) {
      read_byte = BufferByte64ReadByte(&uart1_input_buffer);
      if (read_byte == SYNC_BYTE_2) {
	read_byte = BufferByte64ReadByte(&uart1_input_buffer);
	if (read_byte == SYNC_BYTE_3_RECEIVE) {
	  // All of the sync bytes matched, this should be a valid command
	  command_string.command_byte   = BufferByte64ReadByte(&uart1_input_buffer);
	  command_string.data_high_byte = BufferByte64ReadByte(&uart1_input_buffer);
	  command_string.data_low_byte  = BufferByte64ReadByte(&uart1_input_buffer);
	  command_string.register_byte  = BufferByte64ReadByte(&uart1_input_buffer);
	  crc                           = BufferByte64ReadByte(&uart1_input_buffer);
	  crc                           = (crc << 8) + BufferByte64ReadByte(&uart1_input_buffer);
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
  BufferByte64WriteByte(&uart1_output_buffer, SYNC_BYTE_1);
  BufferByte64WriteByte(&uart1_output_buffer, SYNC_BYTE_2);
  BufferByte64WriteByte(&uart1_output_buffer, SYNC_BYTE_3_SEND);
  BufferByte64WriteByte(&uart1_output_buffer, command_byte);
  BufferByte64WriteByte(&uart1_output_buffer, (data_word >> 8));
  BufferByte64WriteByte(&uart1_output_buffer, (data_word & 0x00FF));
  BufferByte64WriteByte(&uart1_output_buffer, register_byte);
  BufferByte64WriteByte(&uart1_output_buffer, (crc >> 8));
  BufferByte64WriteByte(&uart1_output_buffer, (crc & 0x00FF));

  if ((!U1STAbits.UTXBF) && (BufferByte64IsNotEmpty(&uart1_output_buffer))) {
    /*
      There is at least one byte available for writing in the outputbuffer and the transmit buffer is not full.
      Move a byte from the output buffer into the transmit buffer
      All subsequent bytes will be moved from the output buffer to the transmit buffer by the U1 TX Interrupt
    */
    U1TXREG = BufferByte64ReadByte(&uart1_output_buffer);
  }
}


void ExecuteCommand(void) {
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

    case CMD_SET_TARGET_POWER:
      serial_link_power_target = data_word;
      break;

    case CMD_SET_PID:
      switch (command_string.register_byte)
	{
	case RAM_READ_PID_P_COEF:
	  pid_forward_power_kCoeffs[0] = data_word;
	  _T1IE = 0;
	  PIDCoeffCalc(pid_forward_power_kCoeffs, &pid_forward_power);
	  _T1IE = 1;
	  break;
	  
	case RAM_READ_PID_I_COEF:
	  pid_forward_power_kCoeffs[1] = data_word;
	  _T1IE = 0;
	  PIDCoeffCalc(pid_forward_power_kCoeffs, &pid_forward_power);
	  _T1IE = 1;
	  break;
	  
	case RAM_READ_PID_D_COEF:
	  pid_forward_power_kCoeffs[2] = data_word;
	  _T1IE = 0;
	  PIDCoeffCalc(pid_forward_power_kCoeffs, &pid_forward_power);
	  _T1IE = 1;
	  break;
	}
      break;
      
    case CMD_RESET:
#ifdef _DEBUG_MODE
      ResetAllFaults();
#endif
      break;
      
    }

  // Echo the command that was recieved back to the controller
  SendCommand(return_command_byte, command_string.register_byte, return_data_word);
  command_string.data_state = COMMAND_BUFFER_EMPTY;
}



unsigned int ReadFromRam(unsigned int ram_location) {
  unsigned int data_return;

  switch (ram_location) 
    {

    case RAM_READ_STATE:
      data_return = control_state;
      break;

    case RAM_READ_VERSION:
      data_return = A35997_SOFTWARE_VERSION;
      break;
    
    case RAM_READ_FORWARD_POWER_DETECTOR_A_TEMPERATURE:
      data_return = forward_power_detector_A.detector_temperature;
      break;

    case RAM_READ_FORWARD_POWER_DETECTOR_A_POWER:
      data_return = forward_power_detector_A.power_reading_centi_watts;
      break;

    case RAM_READ_FORWARD_POWER_DETECTOR_B_TEMPERATURE:
      data_return = forward_power_detector_B.detector_temperature;
      break;

    case RAM_READ_FORWARD_POWER_DETECTOR_B_POWER:
      data_return = forward_power_detector_B.power_reading_centi_watts;
      break;

    case RAM_READ_REVERSE_POWER_DETECTOR_A_TEMPERATURE:
      data_return = reverse_power_detector_A.detector_temperature;
      break;

    case RAM_READ_REVERSE_POWER_DETECTOR_A_POWER:
      data_return = reverse_power_detector_A.power_reading_centi_watts;
      break;

    case RAM_READ_REVERSE_POWER_DETECTOR_B_TEMPERATURE:
      data_return = reverse_power_detector_B.detector_temperature;
      break;

    case RAM_READ_REVERSE_POWER_DETECTOR_B_POWER:
      data_return = reverse_power_detector_B.power_reading_centi_watts;
      break;

    case RAM_READ_PID_POWER_TARGET:
      data_return = (pid_forward_power.controlReference << 1);
      break;

    case RAM_READ_PROGRAM_POWER_LEVEL:
      data_return = program_power_level.power_reading_centi_watts;
      break;

    case RAM_READ_RF_AMPLIFIER_TEMPERATURE:
      data_return = program_power_level.detector_temperature;
      break;

    case RAM_READ_TOTAL_FORWARD_POWER:
      data_return = total_forward_power_centi_watts;
      break;

    case RAM_READ_TOTAL_REVERSE_POWER:
      data_return = total_reverse_power_centi_watts;
      break;

    case RAM_READ_PID_DAC_OUTPUT:
      data_return = rf_amplifier_dac_output;
      break;

    case RAM_READ_FAULT_REGISTER:
      data_return = fault_latched_register;
      break;
      
    case RAM_READ_FAULT_STATUS_REGISTER:
      data_return = fault_status_register;
      break;
      
    case RAM_READ_OVER_REVERSE_COUNT:
      data_return = over_refected_power_count;
      break;

    case RAM_READ_SCALE_ERRORS:
      data_return = etm_scale16bit_saturation_count;
      break;

    case RAM_READ_LTC2656_ERRORS:
      data_return = LTC2656_write_error_count;
      break;

    case RAM_READ_GUI_DEBUG_1:
      data_return = gui_debug_value_1;
      break;

    case RAM_READ_GUI_DEBUG_2:
      data_return = gui_debug_value_2;
      break;

    case RAM_READ_GUI_DEBUG_3:
      data_return = gui_debug_value_3;
      break;

    case RAM_READ_GUI_DEBUG_4:
      data_return = gui_debug_value_4;
      break;

    case RAM_READ_PID_P_COEF:
      data_return = pid_forward_power_kCoeffs[0];
      break;

    case RAM_READ_PID_I_COEF:
      data_return = pid_forward_power_kCoeffs[1];
      break;

    case RAM_READ_PID_D_COEF:
      data_return = pid_forward_power_kCoeffs[2];
      break;

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



//void _ISRNOPSV _U1RXInterrupt(void) {
void __attribute__((interrupt(__save__(CORCON,SR)),no_auto_psv)) _U1RXInterrupt(void) {
  _U1RXIF = 0;
  while (U1STAbits.URXDA) {
    BufferByte64WriteByte(&uart1_input_buffer, U1RXREG);
  }
}



//void _ISRNOPSV _U1TXInterrupt(void) {
void __attribute__((interrupt(__save__(CORCON,SR)),no_auto_psv)) _U1TXInterrupt(void) {
  _U1TXIF = 0;
  while ((!U1STAbits.UTXBF) && (BufferByte64BytesInBuffer(&uart1_output_buffer))) {
    /*
      There is at least one byte available for writing in the outputbuffer and the transmit buffer is not full.
      Move a byte from the output buffer into the transmit buffer
    */
    U1TXREG = BufferByte64ReadByte(&uart1_output_buffer);
  }
}

