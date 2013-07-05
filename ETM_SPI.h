#ifndef __ETM_SPI_H
#define __ETM_SPI_H


/*
  This is an extension of the built in SPI functions that provides more robust operation and easier use.

  This has been written work on devices with 1 or 2 SPI ports.
  If you are using a device with more than 2 SPI ports those will need to be added to the source code

  Please NOTE!!!!  Many modules will require that we configure the SPIxCON SMP & CKE bits to get it to interface properly with the device
  The default values here work on some devices, but may not work on your device.  I suggest #defining the con settings in your own h file

  Dan Parker
  2013_04_02
*/

/*
  This module has only been validated on "V1" spi hardware.  This was validated on A34760 (pic30F6014A)
  DPARKER - This module needs to be validated on "V2" SPI hardware.
*/


void ConfigureSPI(unsigned char spi_port, unsigned int spi_con_value, unsigned int spi_con2_value, unsigned int spi_stat_value, unsigned long bit_rate, unsigned long fcy_clk);
/*
  This function configures the SPI port specified by "spi_port", with spi_con_value, spi_state_value and bit_rate.
  On some processors 
  The bit rate is determined by a set of dividers on the fcy_clk.
  The bit rate selected will be the maximum realisable bit rate than is less than or equal "bit_rate".
  The minimum bit rate is fcy_clk/512.  If the request bit rate is less than fcy_clk/512 the bit rate will be fcy_clk/512.  

  
  Some of the processers have one configuration register "spi_con_value", and some processors have two configuration registers "spi_con_value" & "spi_con2_value"
  If you are using a processor with only one configuration register spi_con2_value is ignored
*/

unsigned long SendAndReceiveSPI(unsigned int data_word, unsigned char spi_port);
/*
  This writes a word to the SPI bus and reads back a word from the SPI bus
  It sets up and maganges the TMR specified below to look for timeouts and looks for faults.
  If the SPI operation is successful, the function will return 0x0000dddd (dddd being the 16 bits read from the SPI port) 
  If the SPI operation fails, the function will return 0x11110000
*/


extern unsigned int etm_spi1_error_count; // This global variable counts the number of spi_errors.  It may be useful for code bebugging and validation
extern unsigned int etm_spi2_error_count; // This global variable counts the number of spi_errors.  It may be useful for code bebugging and validation


// #defines for the spi ports
#define ETM_SPI_PORT_1          1
#define ETM_SPI_PORT_2          2


#define ETM_DEFAULT_SPI_CON_VALUE  0b0000010100111111 // This works for V1 and V2 because common bits are identical and there is no overlap on the non-identical bits
#define ETM_DEFAULT_SPI_CON2_VALUE 0b0000000000000000 // This works for V1 and V2 because it is ignored in V1
#define ETM_DEFAULT_SPI_STAT_VALUE 0b1010000000000000 // This works for V1 & V2 because they are identical


/* 
   ----------- Example Code -----------
   temp = SendAndReceiveSPI(data_transmit_word, ETM_SPI_PORT_2);
   if (temp == 0x11110000) {
      // There was an error in the SPI process, execute corrective action
   } else {
      data_receive_word = temp & 0xFFFF;
   }
*/

#endif
