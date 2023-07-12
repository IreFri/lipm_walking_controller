#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>
#include <iostream>

#include <boost/filesystem.hpp>

extern "C"
{
#include "usbio.h"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>
}

using namespace std::chrono_literals;

int main(int argc, char *argv[])
{
  std::string dev = argv[1];

  std::atomic<double> sensor_data_ {0}; ///< Data read from the sensor
  int baudrate_ = B115200;

  auto serial_port = boost::filesystem::canonical(dev).string();
  int fd = open(serial_port.c_str(), O_RDWR | O_NOCTTY);
  std::cout << "Is the serial port open?" << std::endl;
  if(fd < 0)
  {
    close(fd);
    std::cout << " No." << std::endl;
    return 0;
  }
  else
  {
    std::cout << " Yes" << std::endl;
  }

  fcntl(fd, F_SETFL, 0);
  struct termios tio;
  tcgetattr(fd, &tio);
  cfsetispeed(&tio, baudrate_);
  cfsetospeed(&tio, baudrate_);
  // non canonical, non echo back
  tio.c_lflag &= ~(ECHO | ICANON);
  // non blocking
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  tcsetattr(fd, TCSANOW, &tio);

  std::this_thread::sleep_for(100ms);
  // https://stackoverflow.com/questions/8888748/how-to-check-if-given-c-string-or-char-contains-only-digits
  auto is_digits = [](const std::string & str)
  {
    return std::all_of(str.begin(), str.end(), ::isdigit);
  };

  while(true)
  {
    char buf[255];
    int len = read(fd, buf, sizeof(buf));
    if(0 < len)
    {
      std::string str = "";
      // -2 to remove the \n\0 character
      for(int i = 0; i < len - 2; i++)
      {
        str = str + buf[i];
      }

      std::cout << "--------------------" << std::endl;
      std::cout << "readData: " << str << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  return 0;
}