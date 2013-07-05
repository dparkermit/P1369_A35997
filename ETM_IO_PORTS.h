#ifndef __ETM_IO_PORTS_H
#define __ETM_IO_PORTS_H



/*
  This module allows a programmatic way to access IO ports so that IO pins can be associated with an object.
  The Latch assignments are monatomic so it is not possible to have a problem with interrupts.

  It takes much longer than a direct assignement, but allows pins to be C objects.
  It is possible to address pins that are not on your processor.  If you do this the end result is that nothing will happen.

  This function takes 
  2 cycles to load the pin data into working registers 0,1
  2 cycles to "rcall" the function
  3 or 4 cycles to execute
  3 cycles to "Return"

  Adding New Processors
  In order to add new processors you need to edit the .s file to include to ports that acutally present on your processor
  Currently the following processors are supported
  P30F6014A
 

  WARNING!!!!!!
  In order to make these functions fast and monatomic the assembly is very tricky.
  The lower word of the Pin number tells the compiler how many instruction codes to jump to quickly get to the correct port.
  If you mess with the pin numbering or any of assembly this number will be off and you will at best affect the wrong pin or jump to a random execution spot.
  This will only show up when you call that pin so it might not be immediately apparent.

*/

/*
  This module has been validated as part of the LTC2656 module on A34760 (p30F6104A).  Validation on more projects should be done
  DPARKER - Add support for reading the value of a pin????
*/

void ETMSetPin(unsigned long pin);
/*
  This function will set the latch bit associate with pin to 1.
  If the pin is not an output this will have no effect
  10 cycles to execute including rcall,return,and memory operations
*/

void ETMClearPin(unsigned long pin);
/*
  This function will set the latch bit associate with pin to 1.
  If the pin is not an output this will have no effect
  11 cycles to execute including rcall,return,and memory operations
*/

void ETMPinTrisInput(unsigned long pin);
/*
  This function will set the tris bit associate with pin to input.
  10 cycles to execute including rcall,return,and memory operations
*/


void ETMPinTrisOutput(unsigned long pin);
/*
  This function will set the tris bit associate with pin to output.
  11 cycles to execute including rcall,return,and memory operations
*/





// In order to set a bit, the latch register is OR'd with the pin mask
// In order to clear a bit, the latch register is AND'd with the bitwise inverse of the pin mask
// In order to read a bit, the port register is AND'd with the pin mask


#define _PORT_PORTA 0x00000000
#define _PORT_PORTB 0x00020000
#define _PORT_PORTC 0x00040000
#define _PORT_PORTD 0x00060000
#define _PORT_PORTE 0x00080000
#define _PORT_PORTF 0x000A0000
#define _PORT_PORTG 0x000C0000
#define _PORT_NONE  0x000E0000


#define _PORT_PIN_0  0x00000001
#define _PORT_PIN_1  0x00000002
#define _PORT_PIN_2  0x00000004
#define _PORT_PIN_3  0x00000008
#define _PORT_PIN_4  0x00000010
#define _PORT_PIN_5  0x00000020
#define _PORT_PIN_6  0x00000040
#define _PORT_PIN_7  0x00000080
#define _PORT_PIN_8  0x00000100
#define _PORT_PIN_9  0x00000200
#define _PORT_PIN_A  0x00000400
#define _PORT_PIN_B  0x00000800
#define _PORT_PIN_C  0x00001000
#define _PORT_PIN_D  0x00002000
#define _PORT_PIN_E  0x00004000
#define _PORT_PIN_F  0x00008000


#define _PIN_RA0 _PORT_PORTA | _PORT_PIN_0
#define _PIN_RA1 _PORT_PORTA | _PORT_PIN_1
#define _PIN_RA2 _PORT_PORTA | _PORT_PIN_2
#define _PIN_RA3 _PORT_PORTA | _PORT_PIN_3
#define _PIN_RA4 _PORT_PORTA | _PORT_PIN_4
#define _PIN_RA5 _PORT_PORTA | _PORT_PIN_5
#define _PIN_RA6 _PORT_PORTA | _PORT_PIN_6
#define _PIN_RA7 _PORT_PORTA | _PORT_PIN_7
#define _PIN_RA8 _PORT_PORTA | _PORT_PIN_8
#define _PIN_RA9 _PORT_PORTA | _PORT_PIN_9
#define _PIN_RA10 _PORT_PORTA | _PORT_PIN_A
#define _PIN_RA11 _PORT_PORTA | _PORT_PIN_B
#define _PIN_RA12 _PORT_PORTA | _PORT_PIN_C
#define _PIN_RA13 _PORT_PORTA | _PORT_PIN_D
#define _PIN_RA14 _PORT_PORTA | _PORT_PIN_E
#define _PIN_RA15 _PORT_PORTA | _PORT_PIN_F


#define _PIN_RB0 _PORT_PORTB | _PORT_PIN_0
#define _PIN_RB1 _PORT_PORTB | _PORT_PIN_1
#define _PIN_RB2 _PORT_PORTB | _PORT_PIN_2
#define _PIN_RB3 _PORT_PORTB | _PORT_PIN_3
#define _PIN_RB4 _PORT_PORTB | _PORT_PIN_4
#define _PIN_RB5 _PORT_PORTB | _PORT_PIN_5
#define _PIN_RB6 _PORT_PORTB | _PORT_PIN_6
#define _PIN_RB7 _PORT_PORTB | _PORT_PIN_7
#define _PIN_RB8 _PORT_PORTB | _PORT_PIN_8
#define _PIN_RB9 _PORT_PORTB | _PORT_PIN_9
#define _PIN_RB10 _PORT_PORTB | _PORT_PIN_A
#define _PIN_RB11 _PORT_PORTB | _PORT_PIN_B
#define _PIN_RB12 _PORT_PORTB | _PORT_PIN_C
#define _PIN_RB13 _PORT_PORTB | _PORT_PIN_D
#define _PIN_RB14 _PORT_PORTB | _PORT_PIN_E
#define _PIN_RB15 _PORT_PORTB | _PORT_PIN_F


#define _PIN_RC0 _PORT_PORTC | _PORT_PIN_0
#define _PIN_RC1 _PORT_PORTC | _PORT_PIN_1
#define _PIN_RC2 _PORT_PORTC | _PORT_PIN_2
#define _PIN_RC3 _PORT_PORTC | _PORT_PIN_3
#define _PIN_RC4 _PORT_PORTC | _PORT_PIN_4
#define _PIN_RC5 _PORT_PORTC | _PORT_PIN_5
#define _PIN_RC6 _PORT_PORTC | _PORT_PIN_6
#define _PIN_RC7 _PORT_PORTC | _PORT_PIN_7
#define _PIN_RC8 _PORT_PORTC | _PORT_PIN_8
#define _PIN_RC9 _PORT_PORTC | _PORT_PIN_9
#define _PIN_RC10 _PORT_PORTC | _PORT_PIN_A
#define _PIN_RC11 _PORT_PORTC | _PORT_PIN_B
#define _PIN_RC12 _PORT_PORTC | _PORT_PIN_C
#define _PIN_RC13 _PORT_PORTC | _PORT_PIN_D
#define _PIN_RC14 _PORT_PORTC | _PORT_PIN_E
#define _PIN_RC15 _PORT_PORTC | _PORT_PIN_F

#define _PIN_RD0 _PORT_PORTD | _PORT_PIN_0
#define _PIN_RD1 _PORT_PORTD | _PORT_PIN_1
#define _PIN_RD2 _PORT_PORTD | _PORT_PIN_2
#define _PIN_RD3 _PORT_PORTD | _PORT_PIN_3
#define _PIN_RD4 _PORT_PORTD | _PORT_PIN_4
#define _PIN_RD5 _PORT_PORTD | _PORT_PIN_5
#define _PIN_RD6 _PORT_PORTD | _PORT_PIN_6
#define _PIN_RD7 _PORT_PORTD | _PORT_PIN_7
#define _PIN_RD8 _PORT_PORTD | _PORT_PIN_8
#define _PIN_RD9 _PORT_PORTD | _PORT_PIN_9
#define _PIN_RD10 _PORT_PORTD | _PORT_PIN_A
#define _PIN_RD11 _PORT_PORTD | _PORT_PIN_B
#define _PIN_RD12 _PORT_PORTD | _PORT_PIN_C
#define _PIN_RD13 _PORT_PORTD | _PORT_PIN_D
#define _PIN_RD14 _PORT_PORTD | _PORT_PIN_E
#define _PIN_RD15 _PORT_PORTD | _PORT_PIN_F

#define _PIN_RE0 _PORT_PORTE | _PORT_PIN_0
#define _PIN_RE1 _PORT_PORTE | _PORT_PIN_1
#define _PIN_RE2 _PORT_PORTE | _PORT_PIN_2
#define _PIN_RE3 _PORT_PORTE | _PORT_PIN_3
#define _PIN_RE4 _PORT_PORTE | _PORT_PIN_4
#define _PIN_RE5 _PORT_PORTE | _PORT_PIN_5
#define _PIN_RE6 _PORT_PORTE | _PORT_PIN_6
#define _PIN_RE7 _PORT_PORTE | _PORT_PIN_7
#define _PIN_RE8 _PORT_PORTE | _PORT_PIN_8
#define _PIN_RE9 _PORT_PORTE | _PORT_PIN_9
#define _PIN_RE10 _PORT_PORTE | _PORT_PIN_A
#define _PIN_RE11 _PORT_PORTE | _PORT_PIN_B
#define _PIN_RE12 _PORT_PORTE | _PORT_PIN_C
#define _PIN_RE13 _PORT_PORTE | _PORT_PIN_D
#define _PIN_RE14 _PORT_PORTE | _PORT_PIN_E
#define _PIN_RE15 _PORT_PORTE | _PORT_PIN_F


#define _PIN_RF0 _PORT_PORTF | _PORT_PIN_0
#define _PIN_RF1 _PORT_PORTF | _PORT_PIN_1
#define _PIN_RF2 _PORT_PORTF | _PORT_PIN_2
#define _PIN_RF3 _PORT_PORTF | _PORT_PIN_3
#define _PIN_RF4 _PORT_PORTF | _PORT_PIN_4
#define _PIN_RF5 _PORT_PORTF | _PORT_PIN_5
#define _PIN_RF6 _PORT_PORTF | _PORT_PIN_6
#define _PIN_RF7 _PORT_PORTF | _PORT_PIN_7
#define _PIN_RF8 _PORT_PORTF | _PORT_PIN_8
#define _PIN_RF9 _PORT_PORTF | _PORT_PIN_9
#define _PIN_RF10 _PORT_PORTF | _PORT_PIN_A
#define _PIN_RF11 _PORT_PORTF | _PORT_PIN_B
#define _PIN_RF12 _PORT_PORTF | _PORT_PIN_C
#define _PIN_RF13 _PORT_PORTF | _PORT_PIN_D
#define _PIN_RF14 _PORT_PORTF | _PORT_PIN_E
#define _PIN_RF15 _PORT_PORTF | _PORT_PIN_F



#define _PIN_RG0 _PORT_PORTG | _PORT_PIN_0
#define _PIN_RG1 _PORT_PORTG | _PORT_PIN_1
#define _PIN_RG2 _PORT_PORTG | _PORT_PIN_2
#define _PIN_RG3 _PORT_PORTG | _PORT_PIN_3
#define _PIN_RG4 _PORT_PORTG | _PORT_PIN_4
#define _PIN_RG5 _PORT_PORTG | _PORT_PIN_5
#define _PIN_RG6 _PORT_PORTG | _PORT_PIN_6
#define _PIN_RG7 _PORT_PORTG | _PORT_PIN_7
#define _PIN_RG8 _PORT_PORTG | _PORT_PIN_8
#define _PIN_RG9 _PORT_PORTG | _PORT_PIN_9
#define _PIN_RG10 _PORT_PORTG | _PORT_PIN_A
#define _PIN_RG11 _PORT_PORTG | _PORT_PIN_B
#define _PIN_RG12 _PORT_PORTG | _PORT_PIN_C
#define _PIN_RG13 _PORT_PORTG | _PORT_PIN_D
#define _PIN_RG14 _PORT_PORTG | _PORT_PIN_E
#define _PIN_RG15 _PORT_PORTG | _PORT_PIN_F

#define _PIN_NOT_CONNECTED _PORT_NONE



#endif
