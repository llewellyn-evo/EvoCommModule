#ifndef POWER_EVO_COMM_MODULE_COMM_MODULE_INCLUDED
#define POWER_EVO_COMM_MODULE_COMM_MODULE_INCLUDED

// DUNE headers.
#include <DUNE/DUNE.hpp>

#define MAX_CHANNELS	5	

typedef struct{
  std::string name;
  int reset_pin;
  bool default_state;
  uint64_t reset_delay;
  uint8_t reset_active;
  uint8_t state;
  uint8_t fault;
}Channels;

namespace Power
{
  namespace EvoCommModule
  {
    using DUNE_NAMESPACES;
    class CommModule :  public BasicModem
    {
      public: 
      //! Create channels array
      Channels* m_channels;
      

      CommModule(Tasks::Task* task ,SerialPort* handle, Channels* channels):
      m_task(task),
      m_handle(handle),
      m_channels(channels),
      BasicModem(task, handle)
      {
      	m_handle->flushInput();
        for (uint8_t i = 0 ; i < MAX_CHANNELS ; i++)
        {
          m_task->inf("Channel Number %d , Name %s , Reset Pin %d , resetActive %u\r\n" , i , m_channels[i].name.c_str() , m_channels[i].reset_pin , m_channels[i].reset_active );
          if (m_channels[i].reset_pin > -1)
          {

            m_gpio[i] = new DUNE::Hardware::GPIO(m_channels[i].reset_pin);
            m_gpio[i]->setDirection("output");
            m_gpio[i]->setValue((m_channels[i].reset_active) ? false:true);
          }
          setChannel(i , m_channels[i].default_state);
        }
        setReadMode(READ_MODE_LINE);
        start();
        setLineTrim(true);
        setLineTermIn("\r\n");
        setTimeout(0.1);
      }

      uint8_t 
      setChannel(uint8_t number , uint8_t state)
      {
        if (number < MAX_CHANNELS)
        {
          try
          {
            uint8_t bfr[20];
            int n = sprintf((char*) bfr , "%s=%d\r\n" ,COMMANDS[number], state);
            bfr[n] = '\0';
            m_handle->write(bfr , n);
          }
          catch (...)
          {
            return 1;
          } 
        }
        return 0;
      }
      
      void
      pollSerialInput()
      {
        if ((Clock::getSinceEpoch() - m_last_serial_querry) > 10.0)
        {
          m_handle->write("STATUS_WORD?\r\n" , 14);
          m_last_serial_querry = Clock::getSinceEpoch();
        }

        try
        {
          std::string line = readLine();
          //! * BME280: T=36.97 P=101133.94 H=15.18
          if (line.find("* BME280:") != std::string::npos)
          {
            if (std::sscanf(line.c_str() , "* BME280: T=%f P=%f H=%f", &m_temperature , &m_pressure , &m_humidity ) == 3)
            {
              IMC::RelativeHumidity hum;
              IMC::Temperature temp;
              IMC::Pressure pres;
              temp.value = m_temperature;
              m_task->dispatch(temp);
              pres.value = m_pressure / 100.0;
              m_task->dispatch(pres);
              hum.value = m_humidity;
              m_task->dispatch(hum);
            }
          }
          //! * ADC: VIN_MON=16.503 5V_MON=4.728
          else if (line.find("* ADC:") != std::string::npos)
          {
            if (std::sscanf(line.c_str() , "* ADC: VIN_MON=%f 5V_MON=%f", &m_vin_voltage , &m_5v_voltage) == 2)
            {
              IMC::Voltage volt;
              volt.value = m_vin_voltage;
              m_task->dispatch(volt);
            }
          }
          //! * STATUS: WORD=0185
          else if (line.find("* STATUS:") != std::string::npos)
          {
            uint32_t status_value;
            if (std::sscanf(line.c_str() , "* STATUS: WORD=%x" , &status_value) == 1)
            {
              decodeStatusWord(status_value);
            }
          }
          else if (line.find("* FAULTS:") != std::string::npos)
          {
            uint32_t fault_value;
            if (std::sscanf(line.c_str() , "* FAULTS: WORD=%x" , &status_value) == 1)
            {
              decodeFaultWord(status_value);
            }
          }
        }
        catch ( ... )
        {
          //! timeout in reading serial. 
        }
      }


      private:
      //! Channel 0 = Wifi
      //! Channel 1 = XBEE
      //! Channel 2 = GPS
      //! Channel 3 = ATMCLK
      //! Channel 4 = GSM/SAT
      const char *COMMANDS[MAX_CHANNELS] = { "WIFI_SW", "XBEE_SW", "GPS_SW", "ATM_CLK_SW" , "SAT_GSM_SW" };
      //! Pointer to task
      Tasks::Task* m_task;
      //! Serial port handle.
      SerialPort* m_handle;
      //! GPIO Channels 
      DUNE::Hardware::GPIO* m_gpio[MAX_CHANNELS];
      //! Temperature Pressure Humidity
      float m_temperature , m_pressure , m_humidity;
      //! Power voltage read, 5V voltage read
      float m_vin_voltage , m_5v_voltage;
      //! epoch of last serial update
      double m_last_serial_querry;


      void 
      decodeStatusWord(uint32_t val)
      {
        for (uint8_t i = 0 ; i < MAX_CHANNELS ; i++)
        {
          if (i == 0) //! WIFI
            m_channels[i].state = (val >> 8) & 0x01;
          else if (i == 1)  //! XBEE
            m_channels[i].state = val & 0x01;
          else if (i == 2) //! GPS
            m_channels[i].state = (val >> 5) & 0x01;
          else if (i == 3)//! ATM_CLK
            m_channels[i].state = (val >> 4) & 0x01;
          else if (i == 4)//! SAT_GSM
            m_channels[i].state = (val >> 6) & 0x01;
        }
      }

      void
      decodeFaultWord(uint8_t val)
      {
        for (uint8_t i = 0 ; i < MAX_CHANNELS ; i++)
        {
          if (i == 0) //! WIFI
            m_channels[i].fault = (val >> 8) & 0x01;
          else if (i == 1)  //! XBEE
            m_channels[i].fault = val & 0x01;
          else if (i == 2) //! GPS
            m_channels[i].fault = (val >> 5) & 0x01;
          else if (i == 3)//! ATM_CLK
            m_channels[i].fault = (val >> 4) & 0x01;
          else if (i == 4)//! SAT_GSM
            m_channels[i].fault = (val >> 6) & 0x01;
        }
      }
    };
  }
}
#endif