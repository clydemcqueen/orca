#ifndef MAESTRO_H
#define MAESTRO_H

#include <string>

namespace maestro {

// A very simple class to communicate w/ the Pololu Maestro board via USB.

class Maestro
{
private:
  int fd_;

  bool writeBytes(const unsigned char* bytes, size_t size);
  bool readBytes(unsigned char* bytes, size_t size);
  
public:
  Maestro();
  ~Maestro();

  bool connect(std::string port);
  void disconnect();
  bool ready();
  bool set_pwm(unsigned char channel, unsigned short value);
  bool get_pwm(unsigned char channel, unsigned short& value);
  bool get_analog(unsigned char channel, float& value);
  bool get_digital(unsigned char channel, bool& value);
};

} // namespace maestro

#endif // MAESTRO_H