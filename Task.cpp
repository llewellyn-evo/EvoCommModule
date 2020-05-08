//***************************************************************************
// Copyright 2007-2020 Universidade do Porto - Faculdade de Engenharia      *
// Laboratório de Sistemas e Tecnologia Subaquática (LSTS)                  *
//***************************************************************************
// This file is part of DUNE: Unified Navigation Environment.               *
//                                                                          *
// Commercial Licence Usage                                                 *
// Licencees holding valid commercial DUNE licences may use this file in    *
// accordance with the commercial licence agreement provided with the       *
// Software or, alternatively, in accordance with the terms contained in a  *
// written agreement between you and Faculdade de Engenharia da             *
// Universidade do Porto. For licensing terms, conditions, and further      *
// information contact lsts@fe.up.pt.                                       *
//                                                                          *
// Modified European Union Public Licence - EUPL v.1.1 Usage                *
// Alternatively, this file may be used under the terms of the Modified     *
// EUPL, Version 1.1 only (the "Licence"), appearing in the file LICENCE.md *
// included in the packaging of this file. You may not use this work        *
// except in compliance with the Licence. Unless required by applicable     *
// law or agreed to in writing, software distributed under the Licence is   *
// distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF     *
// ANY KIND, either express or implied. See the Licence for the specific    *
// language governing permissions and limitations at                        *
// https://github.com/LSTS/dune/blob/master/LICENCE.md and                  *
// http://ec.europa.eu/idabc/eupl.html.                                     *
//***************************************************************************
// Author: Llewellyn-Fernandes                                              *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>
#include "CommModule.hpp"

namespace Power
{
  //! Insert short task description here.
  //!
  //! Insert explanation on task behaviour here.
  //! @author Llewellyn-Fernandes
  namespace EvoCommModule
  {
    using DUNE_NAMESPACES;
    struct Arguments
    {
      std::string uart_dev;
      //! Serial port baud rate.
      unsigned uart_baud;
      //! Array of channels
      std::vector<channel_info> channels;
      //! Period for Status Querry
      double querry_period;
      //! Period to dispatch imc Messages
      double imc_message_period;
    };

    struct Task: public DUNE::Tasks::Task
    {
      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      CommModule* m_comm_module;
      //! Arguments for the task
      Arguments m_args;
      //! Serial port handle.
      SerialPort* m_handle;
      //! Timer for Status Querry
      DUNE::Time::Counter<double> m_status_querry_timer;
      //! Timer to Disapatch  IMC Messages
      DUNE::Time::Counter<double> m_imc_data_dispatch_timer;
      //! Max number of seconds to consider stale data
      const uint8_t c_max_seconds = 10;

      Task(const std::string& name, Tasks::Context& ctx):
      DUNE::Tasks::Task(name, ctx),
      m_comm_module(NULL),
      m_handle(NULL)
      {
        param("Serial Port - Device", m_args.uart_dev)
        .defaultValue("/dev/ttymxc6")
        .description("Serial port device used to communicate with the Power Switch");

        param("Serial Port - Baud Rate", m_args.uart_baud)
        .defaultValue("115200")
        .description("Serial port baud rate");

        param("Status Querry Period", m_args.querry_period)
        .defaultValue("10")
        .description("Period at which to querry status of Switches");

        param("IMC Message Disapatch Period", m_args.imc_message_period)
        .defaultValue("5")
        .description("Period at which to dispatch imc messages");

        for (unsigned i = 0; i < c_max_allowed_channels ; ++i)
        {
          channel_info channel;
          int reset = -1;

          param(String::str("Channel %u Name", i), channel.name)
          .defaultValue("channel")
          .description("Channel Name");

          param(String::str("Channel %u Reset Pin", i), reset)
          .defaultValue("-1")
          .description("Channel reset pin");

          param(String::str("Channel %u Default", i), channel.default_state)
          .defaultValue("false")
          .description("Channel default state");

          if (reset)
          {
            channel.reset_gpio = new DUNE::Hardware::GPIO(reset);

            param(String::str("Channel %u Reset Delay", i), channel.reset_delay)
            .defaultValue("1000")
            .description("Channel reset delay");

            param(String::str("Channel %u Reset Active", i), channel.reset_active)
            .defaultValue("1")
            .description("Channel reset active");
          }

          if (!channel.name.empty())
          {
            m_args.channels.push_back(channel);
          }
        }
        bind<IMC::PowerChannelControl>(this);
        bind<IMC::QueryPowerChannelState>(this);
      }

       //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
        if (m_comm_module)
        {
          if (paramChanged(m_args.uart_dev) || paramChanged(m_args.uart_baud))
          {
            throw RestartNeeded(DTR("restarting to change parameters"), 1);
          }
          else if (paramChanged(m_args.querry_period))
          {
            m_status_querry_timer.setTop(m_args.querry_period);
          }
          else if (paramChanged(m_args.imc_message_period))
          {
            m_imc_data_dispatch_timer.setTop(m_args.imc_message_period);
          }
        }
      }

      //! Acquire resources.
      void
      onResourceAcquisition(void)
      {
        try
        {
          if (!m_handle)
            m_handle = new SerialPort(m_args.uart_dev, m_args.uart_baud);
        }
        catch (...)
        {
          throw RestartNeeded(DTR(Status::getString(CODE_COM_ERROR)), 5);
        }

        if (!m_comm_module)
        {
          m_comm_module = new CommModule(this , m_handle , m_args.channels);
        }

        m_status_querry_timer.setTop(m_args.querry_period);
        m_imc_data_dispatch_timer.setTop(m_args.imc_message_period);
      }

      //! Initialize resources.
      void
      onResourceInitialization(void)
      {
      }

      //! Release resources.
      void
      onResourceRelease(void)
      {
      }


      void
      consume(const IMC::QueryPowerChannelState* msg)
      {
        (void)msg;
        for (uint8_t i = 0; i < m_comm_module->m_channels.size(); i++)
        {
          IMC::PowerChannelState resp;
          resp.name = m_comm_module->m_channels[i].name;
          resp.state = m_comm_module->m_channels[i].state;
          dispatch(resp);
        }
      }

      void
      checkTimers()
      {
        if (m_status_querry_timer.overflow())
        {
          if (m_comm_module->pollStatus())
          {
            err("Error in Getting status of Switches");
          }
          m_status_querry_timer.reset();
        }

        if (m_imc_data_dispatch_timer.overflow() && (Clock::getSinceEpoch() - m_comm_module->m_last_valid_serial_update) < c_max_seconds)
        {
          IMC::RelativeHumidity hum;
          IMC::Temperature temp;
          IMC::Pressure pres;
          IMC::Voltage volt;

          temp.value = m_comm_module->m_temperature;
          dispatch(temp);
          pres.value = m_comm_module->m_pressure / 100.0;
          dispatch(pres);
          hum.value = m_comm_module->m_humidity;
          dispatch(hum);
          volt.value = m_comm_module->m_vin_voltage;
          dispatch(volt);
          m_imc_data_dispatch_timer.reset();
        }
      }

      void
      consume(const IMC::PowerChannelControl* msg)
      {
        for (uint8_t i = 0; i < m_comm_module->m_channels.size() ; i++)
        {
          if (m_comm_module->m_channels[i].name == msg->name)
          {
            switch (msg->op)
            {
              case IMC::PowerChannelControl::PCC_OP_TURN_ON:
                //! Set Swtich ON
                if (m_comm_module->setChannel(i , 1))
                {
                  err("Error in setting channel value");
                }
              break;

              case IMC::PowerChannelControl::PCC_OP_TURN_OFF:
                //! Set Switch OFF
                if (m_comm_module->setChannel(i , 0))
                {
                  err("Error in setting channel value");
                }
              break;

              case IMC::PowerChannelControl::PCC_OP_SCHED_ON:
              case IMC::PowerChannelControl::PCC_OP_SCHED_OFF:
                err("Scheduled ON/OFF Operation not supported");
              break;

              default:
              break;
            }
          }
        }
      }

      //! Main loop.
      void
      onMain(void)
      {
        while (!stopping())
        {
          checkTimers();
          if (m_comm_module->pollSerialInput())
          {
            err("Error in reading data from serial port");
          }
          waitForMessages(0.5);
        }
      }
    };
  }
}
DUNE_TASK
