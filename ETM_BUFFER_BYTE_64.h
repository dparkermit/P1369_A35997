/*
  This is a set a functions to support the use of Circular Buffer - 64 Byte Size Elements
  This is useful for Serial Communications or other slow data operations
  This should not be used for high speed array manipulation.
  Dan Parker - 2013_04_02
*/
  
/*
  DPARKER - This module needs to be validated
*/

#ifndef __ETM_BUFFER_BYTE_64
#define __ETM_BUFFER_BYTE_64

typedef struct {
  unsigned char data[64];
  unsigned char write_location;
  unsigned char read_location;
} BUFFERBYTE64;


void BufferByte64Initialize(BUFFERBYTE64* ptr);
/*
  This initializes the buffer.  All data in the buffer will be lost when this is called.
*/
  
void BufferByte64WriteByte(BUFFERBYTE64* ptr, unsigned char value);
/*
  Writes a byte to the buffer
  If the buffer is full the oldest byte will be overwritten
*/

unsigned char BufferByte64ReadByte(BUFFERBYTE64* ptr);
/*
  Reads a single byte from the buffer.
  If the buffer is empty zero will be returned and the write/read location will not be changed
  Before calling Buffer64ReadByte the buffer should be checked with Buffer64BytesInBuffer or Buffer64IsNotEmpty
*/

unsigned char BufferByte64BytesInBuffer(BUFFERBYTE64* ptr);
/*
  Returns the number of bytes stored in the buffer
*/

unsigned char BufferByte64IsNotEmpty(BUFFERBYTE64* ptr);
/*
  Returns zero if the buffer is Empty
  Returns one if the buffer is not empty
*/


/* 
   ------------  Example Code ---------------

   BUFFERBYTE64 etm_example_buffer;                                 // define a buffer

   BufferByte64Initialize(&etm_example_buffer);                     // Initialize the buffer

   BufferByte64WriteByte(&etm_example_buffer, data_to_write);       // Write a byte to the buffer

   if (BufferByte64IsNotEmpty(&etm_example_buffer) {
     data_from_buffer = BufferByte64ReadByte(&etm_example_buffer);  // Read a byte from the buffer
   }

   if (BufferByte64BytesInBuffer(&etm_example_buffer) >= SERIAL_COMMAND_LENGTH) {
     // Only look for a serial command if enough bytes are in the buffer
   }   

*/


#endif

