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
  bool state;
}Channels;

namespace Power
{
  namespace EvoCommModule
  {
    using DUNE_NAMESPACES;
    class CommModule
    {
      public:
      //! Needs to be changed when cortex firmware is changed
      //! For now 
      //! Channel 0 = Wifi
      //! Channel 1 = XBEE
      //! Channel 2 = GPS
      //! Channel 3 = ATMCLK
      //! Channel 4 = GSM/SAT
      uint8_t ONCOMMAND[MAX_CHANNELS] = {'W' , 'X' , 'G' , 'C' , 'S'};
      uint8_t OFFCOMMAND[MAX_CHANNELS] =  {'w' , 'x' , 'g' , 'c' , 's'};
      //! Pointer to task
      Tasks::Task* m_task;
      //! Serial port handle.
      SerialPort* m_handle;
      //! Create channels array
      Channels* m_channels;
      //! GPIO Channels 
      DUNE::Hardware::GPIO* m_gpio[MAX_CHANNELS];
      //! Temperature Pressure Humidity
      double m_temperature , m_pressure , m_humidity;

      CommModule(Tasks::Task* task ,SerialPort* handle, Channels* channels):
      m_task(task),
      m_handle(handle),
      m_channels(channels)
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
          m_channels[i].state = false;
          setChannel(i , m_channels[i].default_state);
        }
      }

      uint8_t 
      setChannel(uint8_t number , bool state)
      {
        if (number < MAX_CHANNELS)
        {
          try
          {
            uint8_t bfr[5];
            int n = sprintf((char*) bfr , "%c\r\n" , (state) ? ONCOMMAND[number]:OFFCOMMAND[number]);
            bfr[n] = '\0';
            m_handle->write(bfr , n);
            m_channels[number].state = state;
          }
          catch (...)
          {
            return 1;
          } 
        }
        return 0;
      }
      //! This needs to change once firmware for Cortex is finalised
      void
      pollSerialInput()
      {
      	static uint8_t bfr[256];
        static uint8_t index = 0;
        static bool found = false;
        uint8_t chr[1];
      	if (Poll::poll(*m_handle, 0.005))
        {
          m_handle->read(chr, 1);
          if (chr[0] == 'T' && !found)
          {
            index = 0;
            found = true;
          }

          if (found)
          {
            bfr[index++] = chr[0];
          }
          if (chr[0] == '\n' && found)
          {
            index = 0;
            found = false;
            if (!strncmp((char*)bfr , "TPH:", 4))
            {
              std::string s((char*)bfr);
              std::vector<std::string> tokens;
              std::istringstream ss(s);
              std::string token;
              while(std::getline(ss, token, ' '))
              {
                tokens.push_back(token);
              }
              if (tokens.size() > 3)
              {
                m_temperature = std::stod(tokens[1]);
                m_pressure = std::stod(tokens[2]);
                m_humidity = std::stod(tokens[3]);
              }
            }   
          }
        }
      }
    };
  }
}
#endif