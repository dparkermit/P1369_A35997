#include "ETM_BUFFER_BYTE_64.h"


#define BUFFERBYTE64MASK 0b00111111


void BufferByte64Initialize(BUFFERBYTE64* ptr) {
  ptr->write_location = 0;
  ptr->read_location = 0;
}

void BufferByte64WriteByte(BUFFERBYTE64* ptr, unsigned char value) {
  ptr->data[ptr->write_location] = value;
  ptr->write_location += 1;
  ptr->write_location &= BUFFERBYTE64MASK;
  if (ptr->write_location == ptr->read_location) {
    ptr->read_location += 1;
    ptr->read_location &= BUFFERBYTE64MASK;
  }
}

unsigned char BufferByte64ReadByte(BUFFERBYTE64* ptr) {
  unsigned char local_read_location;
  unsigned char return_data;
						
  local_read_location = ptr->read_location;
  if (local_read_location != ptr->write_location) {
    // the buffer is not empty
    return_data = ptr->data[local_read_location];
    local_read_location += 1;
    local_read_location &= BUFFERBYTE64MASK; 
    ptr->read_location = local_read_location;
  } else {
    // the buffer was empty
    // return zero and do not increment the read_location
    return_data = 0;
  }
  return return_data;
}

unsigned char BufferByte64BytesInBuffer(BUFFERBYTE64* ptr) {
  return ((ptr->write_location - ptr->read_location) & BUFFERBYTE64MASK);
}

unsigned char BufferByte64IsNotEmpty(BUFFERBYTE64* ptr) {
  if (ptr->write_location == ptr->read_location) {
    return 0;
  } else {
    return 1;
  }
}



