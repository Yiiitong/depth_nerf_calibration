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
 * NDIComm.hpp
 *
 *  Created on: Sep 7, 2011
 *      Author: duliu
 */

#include <map>
#include <string>
#include <vector>

#include "TrackerFrame.hpp"
#include "SerialComm.hpp"

//namespace eos
//{

class NDIComm : public SerialComm
{
public:
    enum Device
    {
        PolarisAccedo,
        Aurora,
        PolarisSpectra,
        PolarisVicra,
        Other
    };

    enum PortStatus
    {
        Occupied= 0x0001,
        Switch1= 0x0002,
        Switch2= 0x0004,
        Switch3= 0x0008,
        Initialized= 0x0010,
        Enabled= 0x0020,
        OutOfVolume= 0x0040,
        PartiallyOutOfVolume= 0x0080,
        AlgorithmLimitation= 0x0100,
        IRInterference= 0x0200,
        ProcessingException= 0x1000,
        FellBehindWhilePorcessing= 0x4000,
        DataBufferLimitation= 0x8000
    };

    enum TrackingStatus
    {
        OK= NDIComm::Initialized |
            NDIComm::Enabled,
        Partial= NDIComm::PartiallyOutOfVolume,
        Interference= NDIComm::IRInterference,
        NO= NDIComm::Occupied |
            NDIComm::OutOfVolume |
            NDIComm::AlgorithmLimitation |
            NDIComm::ProcessingException |
            NDIComm::FellBehindWhilePorcessing |
            NDIComm::DataBufferLimitation
    };

public:
    NDIComm();
    ~NDIComm();

    void registerTarget( const std::string &id, const std::string &filename );

    void initDevice();

    void startTracking();
    void stopTracking();

    void grabFrame( TrackerFrame &frame );

protected:
    // control functions
    void resetDevice();
    void getDeviceInfo();
    void initTargets();
    uint32_t grabTarget( char* src, uint32_t &handle, Eigen::Affine3f &trans, int &trackingStatus, float &error, uint32_t &frameNo );

    // NDI commands
    void COMM( unsigned int baudRate );
    void RESET();
    void INIT();
    void VER4();
    uint32_t SFLIST( const std::string &replyOption );
    void PHSR( const std::string &replyOption );
    void PHF( const std::string &handle );
    uint32_t PHRQ();
    void PVWR( uint32_t portHandle, uint32_t bytesCount, const char *buffer );
    void PINIT( uint32_t portHandle );
    void PENA( uint32_t portHandle );
    void TSTART();
    void TSTOP();
    void TX();

    // communication
    bool sendCommand( const std::string &command, bool addCRC );
    bool sendCommand( const std::string &command, bool addCRC, const std::string &methode );
    bool sendCommand( const std::string &command, bool addCRC, const std::string &methode, const std::string &expectedResponse );
    bool receiveCommand( unsigned int timeout=0 );
    bool checkResponse();
    bool checkResponse( const std::string &expectedResponse );
    bool checkResponse( const std::string &expectedResponse, const std::string &methode );
    bool checkError( std::string &error );
    bool checkWarning( std::string &warning );

    // CRC handling
    uint32_t getCRC( const std::string &str);
    bool checkCRC( const std::string &str);

protected:
    // some fixed values
    static const size_t s_bufferSize = 1024;
    static const char s_cr = 0xD; // carriage return

    // outgoing message handling
    std::string m_sendBuffer;

    // incoming message handling
    std::string m_receiveBuffer;

    // CRC
    uint32_t m_crcTable[256];

    // properties
    NDIComm::Device m_deviceType;
    bool m_initialized;
    bool m_trackingStarted;

    // information - polaris
    bool m_activeToolPortsAvailable;
    bool m_passiveToolPortsAvailable;
    bool m_multipleVolumeCharacterizationSupported;
    bool m_TIPSupportAvailable;
    bool m_activeWirelessToolsPortsAvailable;
    uint32_t m_activeToolPorts;
    uint32_t m_passiveToolPorts;
    uint32_t m_activeToolPortsTIP; // TIP - tool-in-port
    uint32_t m_activeWirelessToolPorts;

    // information - aurora
    // TODO

    // ROMs
    std::vector<std::string> m_romIDs;
    std::vector<std::string> m_romFilenames;
    std::vector<uint32_t> m_portHandles;
    std::map<size_t, uint32_t> m_portHandleIndices;

    friend class NDITrackingSystem;
};

//}
