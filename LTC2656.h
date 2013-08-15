#ifndef __LTC2656_H
#define __LTC2656_H

#include "ETM_SPI.h"
#include "ETM_IO_PORTS.h"


/*
  This module is used to interface with a LTC2656 over the SPI bus.


  Module Requirements
  ETM_SPI
  ETM_IO_PORTS

  Dan Parker
  2013_04_17

*/

/*
  This module has been validated on A34760 (pic30F6014).  This board does not use all of the IO pins so it should be further validated on new designs
*/



typedef struct {
  unsigned long pin_cable_select_not;
  unsigned long pin_dac_clear_not;
  unsigned long pin_load_dac_not;
  unsigned long pin_por_select;
  unsigned char por_select_value;
  unsigned char spi_port;
  unsigned int  spi_con1_value;
  unsigned int  spi_con2_value;
  unsigned int  spi_stat_value;
  unsigned long spi_bit_rate;
  unsigned long fcy_clk;
} LTC2656;


#define LTC2656_SPI_CON_VALUE  0b0000010100111111 // This works for V1 and V2 because common bits are identical and there is no overlap on the non-identical bits
#define LTC2656_SPI_CON2_VALUE 0b0000000000000000 // This works for V1 and V2 because it is ignored in V1
#define LTC2656_SPI_STAT_VALUE 0b1010000000000000 // This works for V1 & V2 because they are identical
  
/* -------------- DAC Command Definitions ------------------- */
#define LTC2656_CMD_WRITE_TO_INPUT_REGISTER_N             0b00000000 // writes to input register N
#define LTC2656_CMD_UPDATE_DAC_REGISTER_N                 0b00010000 // updates output N
#define LTC2656_CMD_WRITE_TO_INPUT_REGISTER_N_UPDATE_ALL  0b00100000 // Writes to input register N and updates ALL outputs
#define LTC2656_CMD_WRITE_AND_UPDATE_N                    0b00110000 // Write to input register N and updates output N
#define LTC2656_CMD_POWER_DOWN_N                          0b01000000 // Powers down channel N
#define LTC2656_CMD_POWER_DOWN_CHIP                       0b01010000 // Powers down the chip and reference
#define LTC2656_CMD_SELECT_INTERNAL_REFERENCE             0b01100000 // Selects and Powers up the internal Reference
#define LTC2656_CMD_SELECT_EXTERNAL_REFERENCE             0b01110000 // Selects External Reference and powers down internal Reference
#define LTC2656_CMD_NO_OPERATION                          0b11110000 // No operation

#define LTC2656_ADR_A                                     0b00000000 // DAC A
#define LTC2656_ADR_B                                     0b00000001 // DAC B
#define LTC2656_ADR_C                                     0b00000010 // DAC C
#define LTC2656_ADR_D                                     0b00000011 // DAC D
#define LTC2656_ADR_E                                     0b00000100 // DAC E
#define LTC2656_ADR_F                                     0b00000101 // DAC F
#define LTC2656_ADR_G                                     0b00000110 // DAC G
#define LTC2656_ADR_H                                     0b00000111 // DAC H
#define LTC2656_ADR_ALL_DACS                              0b00001111 // ALL DACs

#define LTC2656_WRITE_AND_UPDATE_DAC_A                    (LTC2656_CMD_WRITE_AND_UPDATE_N | LTC2656_ADR_A)
#define LTC2656_WRITE_AND_UPDATE_DAC_B                    (LTC2656_CMD_WRITE_AND_UPDATE_N | LTC2656_ADR_B)
#define LTC2656_WRITE_AND_UPDATE_DAC_C                    (LTC2656_CMD_WRITE_AND_UPDATE_N | LTC2656_ADR_C)
#define LTC2656_WRITE_AND_UPDATE_DAC_D                    (LTC2656_CMD_WRITE_AND_UPDATE_N | LTC2656_ADR_D)
#define LTC2656_WRITE_AND_UPDATE_DAC_E                    (LTC2656_CMD_WRITE_AND_UPDATE_N | LTC2656_ADR_E)
#define LTC2656_WRITE_AND_UPDATE_DAC_F                    (LTC2656_CMD_WRITE_AND_UPDATE_N | LTC2656_ADR_F)
#define LTC2656_WRITE_AND_UPDATE_DAC_G                    (LTC2656_CMD_WRITE_AND_UPDATE_N | LTC2656_ADR_G)
#define LTC2656_WRITE_AND_UPDATE_DAC_H                    (LTC2656_CMD_WRITE_AND_UPDATE_N | LTC2656_ADR_H)

/* -------------- END DAC Command Definitions ------------------- */


void SetupLTC2656(LTC2656* ptr_LTC2656);
/*  
   Function Arguments:
   *ptr_LTC2656 : This is pointer to the structure that defines a particular LTC2656 chip

   This function should be called when somewhere during the startup of the processor.
   This will configure the SPI port (selected by spi_port) and the CS pin.
*/

void ClearOutputsLTC2656(LTC2656* ptr_LTC2656);
/*  
   Function Arguments:
   *ptr_LTC2656 : This is pointer to the structure that defines a particular LTC2656 chip
              
   This function will set all the outputs to their Power On Reset value (either 0 or .5 based on the POR pin)
*/



unsigned char WriteLTC2656(LTC2656* ptr_LTC2656, unsigned int command_word, unsigned int data_word);

/*  
   Function Arguments:
   *ptr_LTC2656     : This is pointer to the structure that defines a particular LTC2656 chip

   command_word     : This is the command word as defined above.  See the LTC2656 spec sheet for more information
                      You may wish to #define more commands using the "N commands and a Channel"
		      For Example if you wanted a command that powered down only CHN B
		      #define LTC2656_CMD_POWER_DOWN_CHIP             (LTC2656_CMD_POWER_DOWN_N | LTC2656_ADR_B)
   data_word        : This is the data (Voltage) to be sent along with the command
                      Voltage is scaled to the chip reference - Internal Reference is selected in this SetupLTC2656()
		      If using 12 bit version of the chip, only the 12 MSB are used. The 4 LSB are don't care.

   This will send a 32 bit command to to the LTC2656 and confirm the data sent to the LTC265
   If the data transmission is successful, the function will return 0
   If there is an error with the data transmission (error on SPI bus, or the data check failed), then the function will return the error code
   If there are any errors bad data may have been written to the DAC so the calling code should attempt to resend the data
*/

unsigned char WriteLTC2656TwoChannels(LTC2656* ptr_LTC2656, unsigned int command_word_one, unsigned int data_word_one, unsigned command_word_two, unsigned int data_word_two);


unsigned char WriteLTC2656AllDacChannels(LTC2656* ptr_LTC2656, unsigned int *dac_array);
/*  
   Function Arguments:
   *ptr_LTC2656     : This is pointer to the structure that defines a particular LTC2656 chip

   *ptr_data        : This is a pointer to the array (8 words) that contains the data for the 8 dac channels

   This will attempt to send out 8 commands (32 bits per command) and the SPI BUS
   The return data will be checked to ensure that the data was properly received
   If the entire data transfer is successful without errors, the function will return 0.
   If there are any errors on the spi bus or a data error is detected the function will exit immediately and return the error code
   If there are any errors bad data may have been written to the DAC so the calling code should attempt to resend the data
*/


extern unsigned int LTC2656_single_channel_error_count; // This global variable counts the number of times WriteLTC2656 failed to properly write data.
extern unsigned int LTC2656_all_channel_error_count; // This global variable counts the number of times WriteLTC2656AllDacChannels failed to properly write data.

#endif
