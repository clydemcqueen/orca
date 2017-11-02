#include "orca_driver/maestro.h"

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

namespace maestro {

Maestro::Maestro() : fd_{-1}
{
}

Maestro::~Maestro()
{
  if (ready())
  {
    disconnect();
  }
}

// Open the virtual serial port, return true if successful
bool Maestro::connect(std::string port)
{
  fd_ = open(port.c_str(), O_RDWR | O_NOCTTY);
  if (fd_ == -1)
  {
    // Likely causes of failure: (a) we're not root, (b) wrong port
    return false;
  }
  else
  {
    struct termios port_settings;
    tcgetattr(fd_, &port_settings);
    port_settings.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    port_settings.c_oflag &= ~(ONLCR | OCRNL);
    tcsetattr(fd_, TCSANOW, &port_settings);

    return true;
  }
}

// Close the virtual serial port
void Maestro::disconnect()
{
  close(fd_);
  fd_ = -1;  
}

// Return true if the port is open
bool Maestro::ready()
{
  return fd_ != -1;
}

// Set the servo / ESC PWM signal, value is in microseconds, return true if successful
bool Maestro::setPWM(unsigned char channel, unsigned short value)
{
  if (ready())
  {
    value *= 4; // Maestro units are 0.25us, e.g., 1500us becomes 6000qus
    unsigned char cmd[4] = {0x84, channel, static_cast<unsigned char>(value & 0x7F), static_cast<unsigned char>((value >> 7) & 0x7F)};
    return writeBytes(cmd, sizeof(cmd));
  }
  else
  {
    return false;
  }
}

// Get the value at a particular channel
bool Maestro::getValue(unsigned char channel, unsigned short& value)
{
  if (ready())
  {
    unsigned char cmd[2] = {0x90, channel};
    if (!writeBytes(cmd, sizeof(cmd)))
    {
      return false;
    }
    unsigned char response[2] = {0x00, 0x00};
    if (!readBytes(response, sizeof(response)))
    {
      return false;
    }
    value = response[0] + 256 * response[1];
    return true;    
  }
  else
  {
    return false;
  }
}

// Get the servo / ESC PWM signal, value is in microseconds, return true if successful
bool Maestro::getPWM(unsigned char channel, unsigned short& value)
{
  if (!getValue(channel, value))
  {
    return false;
  }

  // Maestro pwm measurements are in 0.25us
  value /= 4;
  return true;
}

// Get the value of an analog pin, 0-5.0V
bool Maestro::getAnalog(unsigned char channel, float& value)
{
  unsigned short temp;
  if (!getValue(channel, temp))
  {
    return false;
  }

  // Maestro analog measurements are 0-1023, mapped to 0-5.0V
  value = temp * 5.0 / 1023;
  return true;
}

// Get the value of a digital pin, true = high
bool Maestro::getDigital(unsigned char channel, bool& value)
{
  unsigned short temp;
  if (!getValue(channel, temp))
  {
    return false;
  }

  // Maestro digital measurements are 1023=high, everything else low
  value = (temp == 1023);
  return true;  
}

// Write bytes to the serial port, return true if successful
bool Maestro::writeBytes(const unsigned char* bytes, size_t size)
{
  if (ready())
  {
    return write(fd_, bytes, size) == size;
  }
  else
  {
    return false;
  }
}

// Read bytes from the serial port, return true if successful
bool Maestro::readBytes(unsigned char* bytes, size_t size)
{
  if (ready())
  {
    return read(fd_, bytes, size) == size;
  }
  else
  {
    return false;
  }  
}

} // namespace maestro
