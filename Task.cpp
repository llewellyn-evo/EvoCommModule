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
      //! TCP listening port.
      unsigned tcp_port;
      //! timer for TCP client async data
      double tcp_data_timer;
    };

    struct Task: public DUNE::Tasks::Task
    {
      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      CommModule* m_comm_module = NULL;
      //! Arguments for the task
      Arguments m_args;
      //! Serial port handle.
      SerialPort* m_handle;
      // Socket handle.
      TCPSocket* m_sock;
      // I/O Multiplexer.
      Poll m_poll;
      // Clients.
      std::list<TCPSocket*> m_clients;
      //! Timer to send data to clients
      DUNE::Time::Counter<float> m_client_data_timer;

      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        m_comm_module(NULL),
        m_handle(NULL),
        m_sock(NULL)
      {
        param("Serial Port - Device", m_args.uart_dev)
        .defaultValue("/dev/ttymxc6")
        .description("Serial port device used to communicate with the Power Switch");

        param("Serial Port - Baud Rate", m_args.uart_baud)
        .defaultValue("115200")
        .description("Serial port baud rate");

        param("TCP - Port", m_args.tcp_port)
        .defaultValue("0")
        .description("TCP port to listen on");

        param("TCP Data Timer", m_args.tcp_data_timer)
        .defaultValue("2.0")
        .description("Time between async TCP data");

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

        if (m_args.tcp_port > 0)
        {
          try
          {
            m_sock = new TCPSocket;
          }
          catch (std::runtime_error& e)
          {
            throw RestartNeeded(e.what(), 30);
          }
        }
      }

      //! Initialize resources.
      void
      onResourceInitialization(void)
      {
        if (m_sock != NULL)
        {
          m_sock->bind(m_args.tcp_port);
          m_sock->listen(1024);
          m_sock->setNoDelay(true);
          m_poll.add(*m_sock);
          this->inf("Listening on 0.0.0.0:%d" , m_args.tcp_port);
          m_client_data_timer.setTop(2);
        }
      }

      //! Release resources.
      void
      onResourceRelease(void)
      {
        if (m_sock != NULL)
        {
          m_poll.remove(*m_sock);
          delete m_sock;
          m_sock = NULL;


          std::list<TCPSocket*>::iterator itr = m_clients.begin();
          for (; itr != m_clients.end(); ++itr)
          {
            m_poll.remove(*(*itr));
            delete *itr;
          }
          m_clients.clear();
        }
      }


      void
      consume(const IMC::QueryPowerChannelState* msg)
      {
        (void)msg;
        for (uint8_t i = 0; i < MAX_CHANNELS; i++)
        {
          IMC::PowerChannelState resp;
          resp.name = m_comm_module->m_channels[i].name;
          resp.state = (m_comm_module->m_channels[i].state) ? 1:0;
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
                m_comm_module->setChannel(i , true);
                break;

              case IMC::PowerChannelControl::PCC_OP_TURN_OFF:
              case IMC::PowerChannelControl::PCC_OP_SCHED_OFF:
                //! Set Switch OFF
                m_comm_module->setChannel(i , false);
                break;
              default:
                break;
            }
          }
        }
      }

      void
      checkMainSocket(void)
      {
        if (m_poll.wasTriggered(*m_sock))
        {
          this->inf(DTR("accepting connection request"));
          try
          {
            TCPSocket* nc = m_sock->accept();
            m_clients.push_back(nc);
            m_poll.add(*nc);
          }
          catch (std::runtime_error& e)
          {
            err("%s", e.what());
          }
        }
      }

      void
      checkClientSockets(void)
      {
        char bfr[512];
        char command_resp[10] = "ERROR\r\n";

        std::list<TCPSocket*>::iterator itr = m_clients.begin();
        while (itr != m_clients.end())
        {
          if (m_poll.wasTriggered(*(*itr)))
          {
            try
            {
              int rv = (*itr)->read(bfr, sizeof(bfr));
              if (rv > 0 && bfr[0] == '$')
              {
                std::string s(bfr);
                for(uint8_t i = 0 ; i < MAX_CHANNELS ; i++)
                {
                  if (s.find(m_args.channel[i].name) != std::string::npos)
                  {
                    std::vector<std::string> tokens;
                    std::istringstream ss(s);
                    std::string token;
                    while(std::getline(ss, token, ','))
                    {
                      tokens.push_back(token);
                    }
                    if (tokens.size() == 2)
                    {
                      int val = std::stoi(tokens[1]);
                      if ((val == 1 || val == 0) && (!m_comm_module->setChannel(i , val)))
                      {
                        memset(command_resp , 0 , sizeof(command_resp));
                        strcpy(command_resp , "OK\r\n");
                      }
                    }
                  }
                }
              }
              (*itr)->write(command_resp, (unsigned)strlen(command_resp));
              //! Return Value Here
            }
            catch (Network::ConnectionClosed& e)
            {
              (void)e;
              m_poll.remove(*(*itr));
              delete *itr;
              itr = m_clients.erase(itr);
              continue;
            }
            catch (std::runtime_error& e)
            {
              err("%s", e.what());
            }
          }
          ++itr;
        }
      }

      //! Dispatch data to all TCP clients
      void
      dispatchToClients(const char* bfr, unsigned bfr_len)
      {
        std::list<TCPSocket*>::iterator itr = m_clients.begin();
        while (itr != m_clients.end())
        {
          try
          {
            (*itr)->write(bfr, bfr_len);
            ++itr;
          }
          catch (std::runtime_error& e)
          {
            err("%s", e.what());
            m_poll.remove(*(*itr));
            delete *itr;
            itr = m_clients.erase(itr);
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
          if (m_args.tcp_port > 0 && m_poll.poll(0.005))
          {
            checkMainSocket();
            checkClientSockets();
          }
          else if (m_args.tcp_port > 0 && m_client_data_timer.overflow())
          {
            char data[100];
            int len = sprintf(data , "+TPH,%2.2f,%2.2f,%2.2f\r\n",m_comm_module->m_temperature , m_comm_module->m_pressure , m_comm_module->m_humidity); 
            dispatchToClients(data , len);
            m_client_data_timer.reset();
          }
          waitForMessages(0.005);
        }
      }
    };
  }
}

DUNE_TASK
