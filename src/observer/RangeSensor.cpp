#include <lipm_walking/observer/RangeSensor.h>
#include <mc_observers/ObserverMacros.h>
#include <mc_rtc/version.h>
#include <mc_control/MCController.h>
#include <mc_mujoco/devices/RangeSensor.h>
#include <chrono>

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

namespace
{

int toBaudrate(const std::string& str)
{
  if(str == "B0")
  {
    return B0;
  }
  else if(str == "B50")
  {
    return B50;
  }
  else if(str == "B75")
  {
    return B75;
  }
  else if(str == "B110")
  {
    return B110;
  }
  else if(str == "B134")
  {
    return B134;
  }
  else if(str == "B150")
  {
    return B150;
  }
  else if(str == "B200")
  {
    return B200;
  }
  else if(str == "B300")
  {
    return B300;
  }
  else if(str == "B600")
  {
    return B600;
  }
  else if(str == "B1200")
  {
    return B1200;
  }
  else if(str == "B1800")
  {
    return B1800;
  }
  else if(str == "B2400")
  {
    return B2400;
  }
  else if(str == "B4800")
  {
    return B4800;
  }
  else if(str == "B9600")
  {
    return B9600;
  }
  else if(str == "B19200")
  {
    return B19200;
  }
  else if(str == "B38400")
  {
    return B38400;
  }
  else if(str == "B57600")
  {
    return B57600;
  }
  else if(str == "B115200")
  {
    return B115200;
  }
  else if(str == "B230400")
  {
    return B230400;
  }
  else if(str == "B460800")
  {
    return B460800;
  }

  return 0;
}

std::string fromBaudrate(int bd)
{
  if(bd == B0)
  {
    return "B0";
  }
  else if(bd == B50)
  {
    return "B50";
  }
  else if(bd == B75)
  {
    return "B75";
  }
  else if(bd == B110)
  {
    return "B110";
  }
  else if(bd == B134)
  {
    return "B134";
  }
  else if(bd == B150)
  {
    return "B150";
  }
  else if(bd == B200)
  {
    return "B200";
  }
  else if(bd == B300)
  {
    return "B300";
  }
  else if(bd == B600)
  {
    return "B600";
  }
  else if(bd == B1200)
  {
    return "B1200";
  }
  else if(bd == B1800)
  {
    return "B1800";
  }
  else if(bd == B2400)
  {
    return "B2400";
  }
  else if(bd == B4800)
  {
    return "B4800";
  }
  else if(bd == B9600)
  {
    return "B9600";
  }
  else if(bd == B19200)
  {
    return "B19200";
  }
  else if(bd == B38400)
  {
    return "B38400";
  }
  else if(bd == B57600)
  {
    return "B57600";
  }
  else if(bd == B115200)
  {
    return "B115200";
  }
  else if(bd == B230400)
  {
    return "B230400";
  }
  else if(bd == B460800)
  {
    return "B460800";
  }

  return "Undefined";
}

}

namespace lipm_walking
{

RangeSensor::RangeSensor(const std::string & type, double dt)
: mc_observers::Observer(type, dt)
{
}

void RangeSensor::configure(const mc_control::MCController & ctl, const mc_rtc::Configuration & config)
{
  robot_name_ = config("robot", ctl.robot().name());
  if(config.has("range_sensor"))
  {
    range_sensor_name_ = static_cast<std::string>(config("range_sensor"));
  }
  else
  {
    mc_rtc::log::error_and_throw<std::invalid_argument>("[RangeSensor::{}] 'range_sensor' is mandatory in the configuration.", name_);
  }

  if(config.has("serial_port"))
  {
    serial_port_name_ = static_cast<std::string>(config("serial_port"));
  }
  else
  {
    mc_rtc::log::error_and_throw<std::invalid_argument>("[RangeSensor::{}] 'serial_port' is mandatory in the configuration.", name_);
  }

  if(config.has("baudrate"))
  {
    const std::string str_baudrate = static_cast<std::string>(config("baudrate"));
    baudrate_ = toBaudrate(str_baudrate);
    if(baudrate_ == 0)
    {
      mc_rtc::log::error_and_throw<std::invalid_argument>("[RangeSensor::{}] 'baudrate' {} is not a correct value.", name_, str_baudrate);
    }
  }
  else
  {
    mc_rtc::log::error_and_throw<std::invalid_argument>("[RangeSensor::{}] 'baudrate' is mandatory in the configuration.", name_);
  }

  mc_rtc::log::info("[RangeSensor::{}] 'serial_port' is {}", name_, range_sensor_name_);
  mc_rtc::log::info("[RangeSensor::{}] 'range_sensor' is {}", name_, serial_port_name_);

  startReadingDevice();

  desc_ = fmt::format("{} (sensor={}, serial={})", name_, range_sensor_name_, serial_port_name_);
}

void RangeSensor::reset(const mc_control::MCController &)
{
  // Nothing to do here
}

bool RangeSensor::run(const mc_control::MCController &)
{
  // Nothing to do here
  return true;
}

void RangeSensor::update(mc_control::MCController & ctl)
{
  if(serial_port_is_open_)
  {
    ctl.robot(robot_name_).device<mc_mujoco::RangeSensor>(range_sensor_name_).update(sensor_data_);
  }
}

void RangeSensor::addToLogger(const mc_control::MCController & ctl,
                                   mc_rtc::Logger & logger,
                                   const std::string & category)
{
  logger.addLogEntry(category + "_range", [this, &ctl]() -> double {
    return ctl.robot(robot_name_).device<mc_mujoco::RangeSensor>(range_sensor_name_).data();
  });
  logger.addLogEntry(category + "_timestep", [this, &ctl]() -> double {
    return measured_sensor_time_.load();
  });
}

void RangeSensor::removeFromLogger(mc_rtc::Logger & logger, const std::string & category)
{
  logger.removeLogEntry(category + "_range");
  logger.removeLogEntry(category + "_timestep");
}

void RangeSensor::addToGUI(const mc_control::MCController & ctl,
                                mc_rtc::gui::StateBuilder & gui,
                                const std::vector<std::string> & category)
{
  gui.addElement(category,
    mc_rtc::gui::Label("Serial port", [this](){ return (serial_port_is_open_ ? "Open" : "Close"); }),
    mc_rtc::gui::ComboInput("Serial port selection",
      {"/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyUSB3", "/dev/ttyUSB4", "/dev/ttyUSB5", "/dev/ttyUSB6"},
      [this]() -> std::string
      {
        return serial_port_name_;
      },
      [this](const std::string & s)
      {
        serial_port_name_ = s;

        mc_rtc::log::info("[RangeSensor::{}] Changed serial port to {} ", name_, serial_port_name_);

        if(serial_port_is_open_)
        {
          is_reading_ = false;
          loop_sensor_.join();
          serial_port_is_open_ = false;
        }

        startReadingDevice();
      }),
    mc_rtc::gui::ComboInput("Baudrate selection",
      {"B0", "B50", "B75", "B110", "B134", "B150", "B200", "B300", "B600", "B1200", "B1800", "B2400", "B4800", "B9600", "B19200", "B38400", "B57600", "B115200", "B230400", "B460800"},
      [this]() -> std::string
      {
        return fromBaudrate(baudrate_);
      },
      [this](const std::string & s)
      {
        baudrate_ = toBaudrate(s);
        mc_rtc::log::info("[RangeSensor::{}] Changed baudrate to {} ", name_, s);

        if(serial_port_is_open_)
        {
          is_reading_ = false;
          loop_sensor_.join();
          serial_port_is_open_ = false;
        }
        startReadingDevice();
      }),
    mc_rtc::gui::Button("Open",
      [this]()
      {
        if(!serial_port_is_open_)
        {
          startReadingDevice();
        }
        else
        {
          mc_rtc::log::warning("[RangeSensor::{}] The device is already opened", name_);
        }
      }),
    mc_rtc::gui::Button("Close",
      [this]()
      {
        if(serial_port_is_open_)
        {
          is_reading_ = false;
          loop_sensor_.join();
          serial_port_is_open_ = false;
        }
        else
        {
          mc_rtc::log::warning("[RangeSensor::{}] The device is already closed", name_);
        }
      }),
    mc_rtc::gui::Checkbox("Debug output", [this]() { return debug_.load(); }, [this]() { debug_ = !debug_.load(); }),
    mc_rtc::gui::Label("Data",
      [this]()
      {
        return (serial_port_is_open_ ? std::to_string(sensor_data_) : "Sensor is not opened");
      }),
    mc_rtc::gui::Label("Elapsed time [ms]",
      [this]()
      {
        return measured_sensor_time_.load();
      })
  );

  gui.addPlot(
    fmt::format("RangeSensor::{}", name_),
    mc_rtc::gui::plot::X("t", [this, &ctl]() { static double t = 0.; return t += ctl.solver().dt(); }),
    mc_rtc::gui::plot::Y( "data", [this]() { return sensor_data_.load(); }, mc_rtc::gui::Color::Red)
  );
}

void RangeSensor::startReadingDevice()
{
  if(loop_sensor_.joinable())
  {
    loop_sensor_.join();
  }

  loop_sensor_ = std::thread(
    [this]()
    {
      mc_rtc::log::info("[RangeSensor::{}] Looking for {} ", name_, serial_port_name_);
      auto serial_port = std::string{};
      try
      {
        serial_port = boost::filesystem::canonical(serial_port_name_).string();
      }
      catch(...)
      {
        serial_port_is_open_ = false;
        mc_rtc::log::error("[RangeSensor::{}] Serial port {} is neither a device handle nor a symbolic link", name_, serial_port_name_);
        return;
      }

      // int fd = open(serial_port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
      int fd = open(serial_port.c_str(), O_RDWR | O_NOCTTY);
      if(fd < 0)
      {
        serial_port_is_open_ = false;
        close(fd);
        mc_rtc::log::error("[RangeSensor::{}] No {} found", name_, serial_port_name_);
        return;
      }
      else
      {
        mc_rtc::log::info("[RangeSensor::{}] Found {}", name_, serial_port_name_);
        serial_port_is_open_ = true;
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

      prev_time_ = clock::now();

      // https://stackoverflow.com/questions/8888748/how-to-check-if-given-c-string-or-char-contains-only-digits
      auto is_digits = [](const std::string & str)
      {
        return std::all_of(str.begin(), str.end(), ::isdigit);
      };

      is_reading_ = true;
      while(is_reading_)
      {
        char buf[255];
        int len = read(fd, buf, sizeof(buf));
        if(0 < len)
        {
          if(debug_.load())
          {
            mc_rtc::log::warning("[RangeSensor::{}] The buffer length is {}", name_, len);
          }

          prev_time_ = clock::now();
          std::string str = "";
          // -2 to remove the \n\0 character
          for(int i = 0; i < len - 2; i++)
          {
            str = str + buf[i];
          }

          if(debug_.load())
          {
            mc_rtc::log::warning("[RangeSensor::{}] The buffer data as a string is '{}'", name_, str);
          }

          if(!str.empty() && is_digits(str))
          {
            const double data = std::stod(str);
            if(data != 255.)
            {
              sensor_data_ = data * 0.001;
              if(debug_.load())
              {
                mc_rtc::log::warning("[RangeSensor::{}] The buffer data as a double is {}", name_, data);
              }
            }
          }
          else if(str == "Try to initialize VL6180x ...")
          {
            mc_rtc::log::error("[RangeSensor::{}] Try to initialize VL6180x ...'", name_, str);
          }
          else if(str == "[Re]Boot of the ArduinoNano")
          {
            mc_rtc::log::error("[RangeSensor::{}] The ArduinoNano connection (?) was reboot by itself?", name_);
          }

          {
            time_since_last_received_ = clock::now() - prev_time_;
            measured_sensor_time_ = time_since_last_received_.count();
            if(debug_.load())
            {
              mc_rtc::log::warning("[RangeSensor::{}] Elapsed time between 2 acqusitions {}", name_, measured_sensor_time_);
            }
          }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }

      close(fd);
      mc_rtc::log::warning("[RangeSensor::{}] Closing the loop to read data on device {}", name_, serial_port_name_);
    }
  );
}

} // namespace lipm_walking
EXPORT_OBSERVER_MODULE("RangeSensor", lipm_walking::RangeSensor)