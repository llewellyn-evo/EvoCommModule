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
      Channels channel[MAX_CHANNELS];
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

        for (unsigned i = 0; i < MAX_CHANNELS; ++i)
        {
          param(String::str("Channel %u Name", i), m_args.channel[i].name)
          .defaultValue("channel")
          .description("Channel Name");

          param(String::str("Channel %u Reset Pin", i), m_args.channel[i].reset_pin)
          .defaultValue("-1")
          .description("Channel reset pin");

          param(String::str("Channel %u Reset Delay", i), m_args.channel[i].reset_delay)
          .defaultValue("1000")
          .description("Channel reset delay");

          param(String::str("Channel %u Reset Active", i), m_args.channel[i].reset_active)
          .defaultValue("1")
          .description("Channel reset active");

          param(String::str("Channel %u Default", i), m_args.channel[i].default_state)
          .defaultValue("false")
          .description("Channel default state");
        }
        bind<IMC::PowerChannelControl>(this);
        bind<IMC::QueryPowerChannelState>(this);
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
          m_comm_module = new CommModule(this , m_handle , m_args.channel);
        }
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
        for (uint8_t i = 0; i < MAX_CHANNELS; i++)
        {
          IMC::PowerChannelState resp;
          resp.name = m_comm_module->m_channels[i].name;
          resp.state = m_comm_module->m_channels[i].state;
          dispatch(resp);
        }
      }

      void
      consume(const IMC::PowerChannelControl* msg)
      {
        for (uint8_t i = 0; i < MAX_CHANNELS; i++)
        {
          if (m_args.channel[i].name == msg->name)
          {
            switch (msg->op)
            {
              case IMC::PowerChannelControl::PCC_OP_TURN_ON:
              case IMC::PowerChannelControl::PCC_OP_SCHED_ON:
                //! Set Swtich ON
                m_comm_module->setChannel(i , 1);
                break;

              case IMC::PowerChannelControl::PCC_OP_TURN_OFF:
              case IMC::PowerChannelControl::PCC_OP_SCHED_OFF:
                //! Set Switch OFF
                m_comm_module->setChannel(i , 0);
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
          m_comm_module->pollSerialInput();
          waitForMessages(0.5);
        }
      }
    };
  }
}
DUNE_TASK
