#ifndef POWER_EVO_COMM_MODULE_COMM_MODULE_INCLUDED
#define POWER_EVO_COMM_MODULE_COMM_MODULE_INCLUDED

// DUNE headers.
#include <DUNE/DUNE.hpp>
#include <DUNE/Utils/LineParser.hpp>
#include <vector>

const uint8_t c_max_allowed_channels = 5;

struct channel_info
{
  std::string name;
  bool default_state;
  uint64_t reset_delay;
  uint8_t reset_active;
  DUNE::Hardware::GPIO *reset_gpio;
  bool state;
  bool fault;
};

namespace Power
{
  namespace EvoCommModule
  {
    using DUNE_NAMESPACES;
    class CommModule
    {
    public:
      //! Command to get Status of the switches
      const std::string c_status_command = "STATUS_WORD?\r\n";
      //! Temperature Pressure Humidity
      double m_temperature , m_pressure , m_humidity;
      //! Power voltage read, 5V voltage read
      double m_vin_voltage , m_5v_voltage;
      //! epoch of last serial querry
      double m_last_valid_serial_update;
      //! Pointer to task
      Tasks::Task* m_task;
      //! Serial port handle.
      SerialPort* m_handle;
      //! Create channels array
      std::vector<channel_info>& m_channels;
      //! Line Parser
      DUNE::Utils::LineParser* m_parser;

      CommModule(Tasks::Task* task ,SerialPort* handle, std::vector<channel_info>& channels):
      m_task(task),
      m_handle(handle),
      m_channels(channels)
      {
        for (uint8_t i = 0 ; i < m_channels.size() ; i++)
        {
          m_task->inf("Channel Number %d , Name %s , resetActive %u\r\n" , i , m_channels[i].name.c_str() ,  m_channels[i].reset_active );

          if (m_channels[i].reset_gpio != NULL)
          {
            m_channels[i].reset_gpio->setDirection("output");
            m_channels[i].reset_gpio->setValue((m_channels[i].reset_active) ? false:true);
          }
          if (setChannel(i , m_channels[i].default_state))
          {
            m_task->err("Error in setting channel value");
          }
        }
        try
        {
          m_parser = new DUNE::Utils::LineParser("\r\n");
        }
        catch (...)
        {
          m_task->err("Invalid EOL for Line Parser");
        }
      }

      uint8_t
      setChannel(uint8_t number , bool state)
      {
        try
        {
          std::stringstream command;
          command << m_channels[number].name << "_SW=" << state << "\r\n";
          m_handle->writeString(command.str().c_str());
        }
        catch (...)
        {
          return 1;
        }
        return 0;
      }

      uint8_t
      pollStatus()
      {
        try
        {
          m_handle->writeString(c_status_command.c_str());
        }
        catch ( ... )
        {
          return 1;
        }
        return 0;
      }

      uint8_t
      pollSerialInput()
      {
        try
        {
          char data[1024];
          size_t ret = m_handle->read(data , sizeof(data));

          if (ret)
          {
            m_parser->append(data , ret);
          }
          processData();
        }
        catch ( ... )
        {
          return 1;
        }
        return 0;
      }

      void
      processData()
      {
        std::vector<std::string> lines;
        if (m_parser->parse(lines))
        {
          for (unsigned int i = 0 ; i < lines.size() ; i ++)
          {
            if (lines[i].find("* BME280:") != std::string::npos)
            {
              if (std::sscanf(lines[i].c_str() , "* BME280: T=%lf P=%lf H=%lf", &m_temperature , &m_pressure , &m_humidity ) == 3)
              {
                m_last_valid_serial_update = Clock::getSinceEpoch();
              }
            }
            //! * ADC: VIN_MON=16.503 5V_MON=4.728
            else if (lines[i].find("* ADC:") != std::string::npos)
            {
              if (std::sscanf(lines[i].c_str() , "* ADC: VIN_MON=%lf 5V_MON=%lf", &m_vin_voltage , &m_5v_voltage) == 2)
              {
                m_last_valid_serial_update = Clock::getSinceEpoch();
              }
            }
            //! * STATUS: WORD=0185
            else if (lines[i].find("* STATUS:") != std::string::npos)
            {
              unsigned int status_value;
              if (std::sscanf(lines[i].c_str() , "* STATUS: WORD=%x" , &status_value) == 1)
              {
                for (uint8_t j = 0 ; j < m_channels.size() ; j++)
                {
                  if (m_channels[j].name.find("WIFI") != std::string::npos)
                    m_channels[j].state = (status_value >> 8) & 0x01; //! WIFI
                  else if (m_channels[j].name.find("XBEE") != std::string::npos)
                    m_channels[j].state = status_value & 0x01;        //! XBEE
                  else if (m_channels[j].name.find("GPS") != std::string::npos)
                    m_channels[j].state = (status_value >> 5) & 0x01; //! GPS
                  else if (m_channels[j].name.find("ATM") != std::string::npos)
                    m_channels[j].state = (status_value >> 4) & 0x01; //! ATM_CLK
                  else if (m_channels[j].name.find("SAT") != std::string::npos)
                    m_channels[j].state = (status_value >> 6) & 0x01; //! SAT_GSM
                }
                m_last_valid_serial_update = Clock::getSinceEpoch();
              }
            }
            else if (lines[i].find("* FAULTS:") != std::string::npos)
            {
              unsigned int fault_value;
              if (std::sscanf(lines[i].c_str() , "* FAULTS: WORD=%x" , &fault_value) == 1)
              {
                for (uint8_t j = 0 ; j < m_channels.size() ; j++)
                {
                  if (m_channels[j].name.find("WIFI") != std::string::npos)
                    m_channels[j].fault = (fault_value >> 8) & 0x01;  //! WIFI
                  else if (m_channels[j].name.find("XBEE") != std::string::npos)
                    m_channels[j].fault = fault_value & 0x01;         //! XBEE
                  else if (m_channels[j].name.find("GPS") != std::string::npos)
                    m_channels[j].fault = (fault_value >> 5) & 0x01;  //! GPS
                  else if (m_channels[j].name.find("ATM") != std::string::npos)
                    m_channels[j].fault = (fault_value >> 4) & 0x01;  //! ATM_CLK
                  else if (m_channels[j].name.find("SAT") != std::string::npos)
                    m_channels[j].fault = (fault_value >> 6) & 0x01;  //! SAT_GSM
                }
                m_last_valid_serial_update = Clock::getSinceEpoch();
              }
            }
          }
        }
      }
    };
  }
}
#endif