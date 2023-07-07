#include <lipm_walking/observer/RangeSensor.h>
#include <mc_observers/ObserverMacros.h>
#include <mc_rtc/version.h>
#include <mc_control/MCController.h>
#include <mc_mujoco/devices/RangeSensor.h>
#include <chrono>

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

  mc_rtc::log::info("[RangeSensor::{}] 'serial_port' is {}", name_, range_sensor_name_);
  mc_rtc::log::info("[RangeSensor::{}] 'range_sensor' is {}", name_, serial_port_name_);

  // Configure the serial port
  serial_port_ = std::make_unique<mn::CppLinuxSerial::SerialPort>(serial_port_name_, mn::CppLinuxSerial::BaudRate::B_115200);

  loop_sensor_ = std::thread(
      [this]()
      {
        try
        {
          serial_port_->SetTimeout(100);
        }
        catch(const std::exception& e)
        {
          mc_rtc::log::error("[RangeSensor::{}] Could not set the timeout: {}", name_, e.what());
        }

        // Open the serial port
        try
        {
          serial_port_->Open();
          serial_port_is_open_ = true;
        }
        catch(const std::exception& e)
        {
          mc_rtc::log::error("[RangeSensor::{}] Could not open the serial port {}", name_, serial_port_name_);
        }

        try
        {
          serial_port_->SetTimeout(1);
        }
        catch(const std::exception& e)
        {
          mc_rtc::log::error("[RangeSensor::{}] Could not set the timeout: {}", name_, e.what());
        }

        auto start = std::chrono::high_resolution_clock::now();
        while(true)
        {
          if(serial_port_is_open_)
          {
            try
            {
              std::string data_str;
              serial_port_->Read(data_str);
              if(!data_str.empty())
              {
                const double data = std::stod(data_str);
                if(data != 255.)
                {
                  // const std::lock_guard<std::mutex> lock(sensor_mutex_);
                  const std::lock_guard<std::mutex> lock(sensor_mutex_);
                  std::cout << data << std::endl;
                  sensor_data_ = data * 0.001;
                }
              }
              print_reading_error_once_ = true;

            }
            catch(const std::exception& e)
            {
              if(!print_reading_error_once_)
              {
                mc_rtc::log::error("[RangeSensor::{}] Could not read from the serial port {}", name_, serial_port_name_);
                print_reading_error_once_ = false;
              }
            }

            {
              using namespace std::chrono_literals;
              std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(1.));
            }
            const auto now = std::chrono::high_resolution_clock::now();
            measured_sensor_time_ = std::chrono::duration<double, std::milli>(now - start).count();
            start = std::chrono::high_resolution_clock::now();
          }
        }

        if(serial_port_is_open_)
        {
          serial_port_->Close();
        }

      }
  );

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
    const std::lock_guard<std::mutex> lock(sensor_mutex_);
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
}

void RangeSensor::removeFromLogger(mc_rtc::Logger & logger, const std::string & category)
{
  logger.removeLogEntry(category + "_range");
}

void RangeSensor::addToGUI(const mc_control::MCController &,
                                mc_rtc::gui::StateBuilder & gui,
                                const std::vector<std::string> & category)
{
  gui.addElement(category,
    mc_rtc::gui::Label("Serial port", [this](){ return (serial_port_is_open_ ? "Open" : "Close"); }),
    mc_rtc::gui::Button("Try to open the serial port",
      [this]()
      {
        try
        {
          serial_port_->Open();
          serial_port_is_open_ = true;
        }
        catch(const std::exception& e)
        {
          mc_rtc::log::error("[RangeSensor::{}] Could not open the serial port {}", name_, serial_port_name_);
        }
      }),
    mc_rtc::gui::Button("Try to close the serial port",
      [this]()
      {
        try
        {
          serial_port_->Close();
          serial_port_is_open_ = false;
        }
        catch(const std::exception& e)
        {
          mc_rtc::log::error("[RangeSensor::{}] Could not close the serial port {}", name_, serial_port_name_);
        }
      }),
    mc_rtc::gui::Label("Data",
      [this]()
      {
        const std::lock_guard<std::mutex> lock(sensor_mutex_);
        return (serial_port_is_open_ ? std::to_string(sensor_data_) : "Sensor is not opened");
      }),
    mc_rtc::gui::Label("Elapsed time [ms]",
      [this]()
      {
        return measured_sensor_time_;
      })
  );
}

} // namespace lipm_walking
EXPORT_OBSERVER_MODULE("RangeSensor", lipm_walking::RangeSensor)