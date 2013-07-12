#include "ETM_SPI.h"

#if defined(__dsPIC30F__)
#include <p30fxxxx.h>
#elif defined(__dsPIC33F__)
#include <p33Fxxxx.h>
#elif defined(__PIC24H__)
#include <p24Hxxxx.h>
#endif


#if defined(__dsPIC33F__) || defined(__PIC24H__) || defined(__dsPIC33E__) || defined (__PIC24E__) || defined(__dsPIC30F1010__) || defined(__dsPIC30F2020__) || defined(__dsPIC30F2023__)
#define __SPI_V2
// DPARKER - Need to validate operation with _SPI_V2 - May need to completely re-write to get to work with _SPI_V2

#elif defined(__dsPIC30F__)
#define __SPI_V1

#endif

unsigned int etm_spi_loop_timeout; 

unsigned int etm_spi1_error_count = 0;
unsigned int etm_spi2_error_count = 0;

void ConfigureSPI(unsigned char spi_port, unsigned int spi_con_value, unsigned int spi_con2_value, unsigned int spi_stat_value, unsigned long bit_rate, unsigned long fcy_clk) {
  unsigned long spi_clock_divide;
  /*
    We need to set up the control registers to give a bit_rate as close to possible as the requested bit_rate without going over
    Spi_Clock = Fcy / (Primary_Prescaler * Secondary_Prescaler)
  */

  spi_con_value |= 0b0000000000011111; // Reset the Primary and Secondary Prescaler Bits to One, they will be adjusted below 

  spi_clock_divide = ((fcy_clk*64) / bit_rate);

  /*
    How many instruction cycles should it take the SPI bus to send out a command?
    It should be 16 bits * Primary Prescaler * Secondary Prescaler + however long it takes to update registers.
    Lets multiply this by 2 just to be safe.
  */


  if (spi_clock_divide <= 512) {
    // Primary Prescaler = 1
    etm_spi_loop_timeout = 1;
    spi_con_value &= 0xFFFF;
  } else {
    if ((spi_clock_divide / 4) <= 512) {
      // Primary Prescaler = 4
      etm_spi_loop_timeout = 4;
      spi_clock_divide /= 4;
      spi_con_value &= 0xFFFE;
    } else if ((spi_clock_divide / 16) <= 512) {
      // Primary Prescaler = 16
      etm_spi_loop_timeout = 16;
      spi_clock_divide /= 16;
      spi_con_value &= 0xFFFD;
    } else {
      // Primary Prescaler = 64
      etm_spi_loop_timeout = 64;
      spi_clock_divide /= 64;
      spi_con_value &= 0xFFFC;
    }
  }
    
  if (spi_clock_divide > 448) {
    // Secondary Prescaler = 8
    etm_spi_loop_timeout *= 8;
    spi_con_value &= 0xFFE3;
  } else if (spi_clock_divide > 384) {
    // Secondary Prescaler = 7
    etm_spi_loop_timeout *= 7;
    spi_con_value &= 0xFFE7;
  } else if (spi_clock_divide > 320) {
    // Secondary Prescaler = 6
    etm_spi_loop_timeout *= 6;
    spi_con_value &= 0xFFEB;
  } else if (spi_clock_divide > 256) {
    // Secondary Prescaler = 5
    etm_spi_loop_timeout *= 5;
    spi_con_value &= 0xFFEF;
  } else if (spi_clock_divide > 192) {
    // Secondary Prescaler = 4
    etm_spi_loop_timeout *= 4;
    spi_con_value &= 0xFFF3;
  } else if (spi_clock_divide > 128) {
    // Secondary Prescaler = 3
    etm_spi_loop_timeout *= 3;
    spi_con_value &= 0xFFF7;
  } else if (spi_clock_divide > 64) {
    // Secondary Prescaler = 2
    etm_spi_loop_timeout *= 2;
    spi_con_value &= 0xFFFB;
  } else {
    // Secondary Prescaler = 1
    spi_con_value &= 0xFFFF;
  }

  /*
    etm_spi_loop_timeout is now equal to Primary Prescaler * Secondary Prescaler.
    This is the number of CPU cycles it will take to clock out 1 bit.
    We need to clock out 16 bits, and provide some over head.
    However we are counting loops, not CPU cycles.  Each loop is at least 10 cycles long
    Therefore we should multiply by 16 and devide by 10.
    Because I want to supply a large range that does not false trigger, just multiply by 4.
   */
  etm_spi_loop_timeout *= 4;

#if defined(__SPI_V1)
  
#if defined(_SPI1IF)
  if (spi_port == ETM_SPI_PORT_1) {
    SPI1CON = spi_con_value;
    SPI1STAT = spi_stat_value;
  }
#endif //#if defined(_SPI1IF)
  
#if defined(_SPI2IF)
  if (spi_port == ETM_SPI_PORT_2) {
    SPI2CON = spi_con_value;
    SPI2STAT = spi_stat_value;
  }
#endif //#if defined(_SPI2IF)
  
  
#elif defined(__SPI_V2)  // #if defined(__SPI_V1)
  
#if defined(_SPI1IF)
  if (spi_port == ETM_SPI_PORT_1) {
    SPI1CON1 = spi_con_value;
    SPI1CON2 = spi_con2_value;
    SPI1STAT = spi_stat_value;
  }
#endif // #if defined(_SPI1IF)
  
#if defined(_SPI2IF)
  if (spi_port == ETM_SPI_PORT_2) {
    SPI2CON1 = spi_con_value;
    SPI2CON2 = spi_con2_value;
    SPI2STAT = spi_stat_value;
  }
#endif // #if defined(_SPI2IF)

#endif // #elif defined(__SPI_V2)
}


#define SPI_BUS_ACTIVE      0
#define SPI_DATA_SENT       1
#define SPI_DATA_RECEIVED   2 
#define SPI_BUS_TIMEOUT     3
#define SPI_BUS_OVERFLOW    4


unsigned long SendAndReceiveSPI(unsigned int data_word, unsigned char spi_port) {
  unsigned char spi_bus_status;
  unsigned long return_data;
  unsigned int loop_counter;
  
  spi_bus_status = SPI_BUS_ACTIVE;
  
#if defined(_SPI1IF)
  if (spi_port == ETM_SPI_PORT_1) {
    _SPI1IF = 0;
    SPI1BUF = data_word;
    loop_counter = 0;
    while (spi_bus_status == SPI_BUS_ACTIVE) {
      loop_counter++;
      if (loop_counter > etm_spi_loop_timeout) {
	// There was a timeout of the data read, this is effectively a fault
	spi_bus_status = SPI_BUS_TIMEOUT;
      } else if (SPI1STATbits.SPIROV) {
	// There was a data overflow in the buffer, this is a fault
	spi_bus_status = SPI_BUS_OVERFLOW;
      } else if (!SPI1STATbits.SPITBF) {
	spi_bus_status = SPI_DATA_SENT;
      }
    }
    while(spi_bus_status == SPI_DATA_SENT) {
      loop_counter++;
      if (loop_counter > etm_spi_loop_timeout) {
	// There was a timeout of the data read, this is effectively a fault
	spi_bus_status = SPI_BUS_TIMEOUT;
      } else if (SPI1STATbits.SPIROV) {
	// There was a data overflow in the buffer, this is a fault
	spi_bus_status = SPI_BUS_OVERFLOW;
      } else if (_SPI1IF) {
	// Data  been received in the buffer, read the data from the return buffer
	spi_bus_status = SPI_DATA_RECEIVED;
      } else if (SPI1STATbits.SPIRBF) {
	spi_bus_status = SPI_DATA_RECEIVED;
      }   
    }
    if (spi_bus_status == SPI_DATA_RECEIVED) {
      return_data = (0x0000FFFF & SPI1BUF);
    } else {
      return_data = 0x11110000;
      etm_spi1_error_count++;
    }
  }
#endif



#if defined(_SPI2IF)
  if (spi_port == ETM_SPI_PORT_2) {
    _SPI2IF = 0;
    SPI2BUF = data_word;
    loop_counter = 0;
    while (spi_bus_status == SPI_BUS_ACTIVE) {
      loop_counter++;
      if (loop_counter > etm_spi_loop_timeout) {
	// There was a timeout of the data read, this is effectively a fault
	spi_bus_status = SPI_BUS_TIMEOUT;
      } else if (SPI2STATbits.SPIROV) {
	// There was a data overflow in the buffer, this is a fault
	spi_bus_status = SPI_BUS_OVERFLOW;
      } else if (!SPI2STATbits.SPITBF) {
	spi_bus_status = SPI_DATA_SENT;
      }
    }
    while(spi_bus_status == SPI_DATA_SENT) {
      loop_counter++;
      if (loop_counter > etm_spi_loop_timeout) {
	// There was a timeout of the data read, this is effectively a fault
	spi_bus_status = SPI_BUS_TIMEOUT;
      } else if (SPI2STATbits.SPIROV) {
	// There was a data overflow in the buffer, this is a fault
	spi_bus_status = SPI_BUS_OVERFLOW;
      } else if (_SPI2IF) {
	// Data  been received in the buffer, read the data from the return buffer
	spi_bus_status = SPI_DATA_RECEIVED;
      }    
    }
    if (spi_bus_status == SPI_DATA_RECEIVED) {
      return_data = (0x0000FFFF & SPI2BUF);
    } else {
      return_data = 0x11110000;
      etm_spi2_error_count++;
    }
  }
#endif
  
  return return_data;
}
