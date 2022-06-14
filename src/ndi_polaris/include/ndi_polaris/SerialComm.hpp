 ///////////////////////////////////////////////////////////////////////////////
//                                                                            //
// This file is part of eos, a lightweight C++ service based framework        //
//                                                                            //
// Copyright (C) 2010, 2011 Alexandru Duliu                                   //
//                                                                            //
// eos is free software; you can redistribute it and/or                       //
// modify it under the terms of the GNU Lesser General Public                 //
// License as published by the Free Software Foundation; either               //
// version 3 of the License, or (at your option) any later version.           //
//                                                                            //
// eos is distributed in the hope that it will be useful, but WITHOUT ANY     //
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS  //
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License or the //
// GNU General Public License for more details.                               //
//                                                                            //
// You should have received a copy of the GNU Lesser General Public           //
// License along with eos. If not, see <http://www.gnu.org/licenses/>.        //
//                                                                            //
///////////////////////////////////////////////////////////////////////////////

#pragma once

/*
 * SerialComm.hpp
 *
 *  Created on: Sept 4, 2011
 *      Author: duliu
 */

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/thread.hpp>

//namespace eos
//{

class SerialComm
{
public:

    SerialComm();
    ~SerialComm();

    void open(std::string port, unsigned int baudRate);
    void close();

    unsigned int sendMsg(void *tx, unsigned int ltx, void *rx, unsigned int lrx, unsigned int timeout);
    unsigned int MSBtoLSB(unsigned int i);
    unsigned int LSBtoMSB(unsigned int i);

protected:
    size_t transmit(const void *tx, size_t len);

    size_t receive(void *rx, size_t len,unsigned int timeout);
    size_t receive(void *rx, size_t len);

    bool isOpen( bool complain=false );

protected:
    static boost::shared_ptr<boost::asio::io_service> m_io_service;
    boost::asio::serial_port *m_serial_port;

    boost::mutex m_semaphore;
};

//}
