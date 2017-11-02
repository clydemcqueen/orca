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
  bool getValue(unsigned char channel, unsigned short& value);
  
public:
  Maestro();
  ~Maestro();

  bool connect(std::string port);
  void disconnect();
  bool ready();
  bool setPWM(unsigned char channel, unsigned short value);
  bool getPWM(unsigned char channel, unsigned short& value);
  bool getAnalog(unsigned char channel, float& value);
  bool getDigital(unsigned char channel, bool& value);
};

} // namespace maestro

#endif // MAESTRO_H
