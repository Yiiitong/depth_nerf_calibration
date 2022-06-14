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
 * StepMotor.hpp
 *
 *  Created on: Sept 2, 2011
 *      Author: duliu
 */

#include "SerialComm.hpp"
//#include <Device.hpp>


//namespace eos
//{

class SerialDevice //: public Device
{
public:
    enum SerialDeviceType {
        DT_MICROCONTROLLER = 1,
        DT_RMOTOR = 2,
        DT_LSTAGE = 3
    };

    SerialDevice();
    ~SerialDevice();


protected:
    // some internal configs
    unsigned int m_address;
    SerialDevice::SerialDeviceType m_deviceType;

    // serial connection
    SerialComm m_comm;
    std::string m_port;
    unsigned int m_baudrate;
};

//}
