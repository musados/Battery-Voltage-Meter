#include <SoftwareSerial.h>

class MySerial : public SoftwareSerial {

  public:
    MySerial(uint8_t receivePin, uint8_t transmitPin,
      bool inverse_logic = false) :
      SoftwareSerial(receivePin, transmitPin,inverse_logic) {}

    virtual size_t write(uint8_t byte) {
      return SoftwareSerial::write(byte);
    }

    int printf(char* fmt, ...) {
      char buff[256];
      va_list args;
      va_start(args, fmt);
      int return_status = vsnprintf(buff, sizeof(buff), fmt, args);
      va_end(args);
      uint8_t *s = (uint8_t *)&buff;
      while (*s) write(*s++);
      return return_status;
    }

};
