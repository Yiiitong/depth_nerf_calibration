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

#include <cstdlib>
#include <fstream>
#include <iostream>

#include <boost/algorithm/string.hpp>
#include <boost/bind.hpp>

#include <ndi_polaris/NDIComm.hpp>


NDIComm::NDIComm() :
    SerialComm(),
    m_deviceType(NDIComm::Other),
    m_initialized(false),
    m_trackingStarted(false),
    m_activeToolPortsAvailable(false),
    m_passiveToolPortsAvailable(false),
    m_multipleVolumeCharacterizationSupported(false),
    m_TIPSupportAvailable(false),
    m_activeWirelessToolsPortsAvailable(false),
    m_activeToolPorts(0),
    m_passiveToolPorts(0),
    m_activeToolPortsTIP(0),
    m_activeWirelessToolPorts(0)
{
    // reserve a certain amount of space for the buffers
    m_sendBuffer.reserve( s_bufferSize );
    m_receiveBuffer.reserve( s_bufferSize );

    // generate the CRC lookup table
    for( uint32_t i=0; i<256; i++ )
    {
        uint64_t temp = i;
        for( uint32_t j=0; j<8; j++ )
            temp = ( temp >> 1 ) ^ (( temp & 1 ) ? 0xA001L : 0 );

        m_crcTable[i] = static_cast<uint32_t>(temp & 0xFFFF);
    }
}


NDIComm::~NDIComm()
{
    stopTracking();
}


void NDIComm::registerTarget( const std::string &id, const std::string &filename )
{
    m_romIDs.push_back( id );
    m_romFilenames.push_back( filename );
}


void NDIComm::initDevice()
{
    // check if the port is open
    isOpen( true );

    // save the old baud rate
    boost::asio::serial_port_base::baud_rate oldBaudRate;
    m_serial_port->get_option( oldBaudRate );

    // set serial connection to default values for NDI
    m_serial_port->set_option(boost::asio::serial_port_base::baud_rate(9600));
    m_serial_port->set_option(boost::asio::serial_port_base::character_size(8));
    m_serial_port->set_option(boost::asio::serial_port_base::parity( boost::asio::serial_port_base::parity::none ));
    m_serial_port->set_option(boost::asio::serial_port_base::stop_bits( boost::asio::serial_port_base::stop_bits::one ));
    m_serial_port->set_option(boost::asio::serial_port_base::flow_control( boost::asio::serial_port_base::flow_control::none ));

    // reset the connection to the device
    resetDevice();

    // set the communication to the already configured baud rate
    COMM( oldBaudRate.value() );

    // restore the previous baud rate
    m_serial_port->set_option( oldBaudRate );

    // init the device
    INIT();

    // get the device information
    getDeviceInfo();

    // init the ports
    initTargets();

    // made it so far, so all is well in the jungle
    m_initialized = true;
}


void NDIComm::startTracking()
{
    if( m_initialized )
    {
        TSTART();
        m_trackingStarted = true;
    }
    else
        throw std::runtime_error("NDIComm::startTracking: device not initialized");
}


void NDIComm::stopTracking()
{
    if( m_initialized && m_trackingStarted )
    {
        TSTOP();
        m_trackingStarted = false;
    }
}


void NDIComm::grabFrame( TrackerFrame &frame )
{
    // init stuff
    uint32_t count = 0;
    uint32_t handle = 0;
    uint32_t systemStatus = 0;
    Eigen::Affine3f trans;
    float err;
    int trackingStatus;
    uint32_t frameNo = 0;
    uint32_t offset=0;

    // get the frame
    TX();

    // check if there was an error
    if( checkResponse() )
    {
        // init the targets
        std::vector<TrackerTarget> targets;

        // get the number of handles
        sscanf(&m_receiveBuffer[0],"%02x",&count);
        offset += 2;

        // go over the handles and get the goods
//        std::cout << m_receiveBuffer << std::endl;
        for( uint32_t i=0; i<count; i++ )
        {
            // get the transformation and error
            offset += grabTarget( &m_receiveBuffer[offset], handle, trans, trackingStatus, err, frameNo );

            // check that we are still in sync
            if( '\n' == m_receiveBuffer[offset] )
                offset++;
            else
                std::cout << "NDIComm::grabFrame: sync lost at handle " << handle << m_romIDs[i] << ", offset " << offset << std::endl;

            // store
            targets.push_back( TrackerTarget(handle, trans, trackingStatus, err ) );
        }

        // store the frame
        frame.set( frameNo, targets );

        // get the systemStatus
        sscanf(&m_receiveBuffer[offset],"%04x",&systemStatus);
    }
}


void NDIComm::resetDevice()
{
    for( size_t i=0; i<3; i++ )
    {
        // send a serial break so that the device also changes its settings to default
        m_serial_port->send_break();

        // wait for the ack that it was reset
        if( receiveCommand() && checkResponse( "RESET" ) )
            return;
    }

    throw std::runtime_error( "NDIComm::hardwareReset: did not get confirmation of connection reset (serial break)." );
}


void NDIComm::getDeviceInfo()
{
    // get device properties
    VER4();

    // made it so far, now find out what device type this is
    if( m_receiveBuffer.find("Polaris") != std::string::npos ||
        m_receiveBuffer.find("polaris") != std::string::npos ||
        m_receiveBuffer.find("POLARIS") != std::string::npos )
    {
        // determine which type of polaris device this is
        if( m_receiveBuffer.find("Accedo") != std::string::npos ||
            m_receiveBuffer.find("accedo") != std::string::npos ||
            m_receiveBuffer.find("ACCEDO") != std::string::npos )
        {
            m_deviceType = NDIComm::PolarisAccedo;
        }
        else if( m_receiveBuffer.find("Spectra") != std::string::npos ||
                 m_receiveBuffer.find("spectra") != std::string::npos ||
                 m_receiveBuffer.find("SPECTRA") != std::string::npos )
        {
            m_deviceType = NDIComm::PolarisSpectra;
        }
        else if( m_receiveBuffer.find("Vicra") != std::string::npos ||
                 m_receiveBuffer.find("vicra") != std::string::npos ||
                 m_receiveBuffer.find("VICRA") != std::string::npos )
        {
            m_deviceType = NDIComm::PolarisVicra;
        }

        // query a list of supported features
        uint32_t features = SFLIST( "00" );

        // extract the features from the feature word
        m_activeToolPortsAvailable = static_cast<bool>( 0x01 & features );
        m_passiveToolPortsAvailable = static_cast<bool>( 0x02 & features );
        m_multipleVolumeCharacterizationSupported = static_cast<bool>( 0x04 & features );
        m_TIPSupportAvailable = static_cast<bool>( 0x08 & features );
        m_activeWirelessToolsPortsAvailable = static_cast<bool>( 0x10 & features );

        // query the number of active ports
        if( m_activeToolPortsAvailable )
            m_activeToolPorts = SFLIST( "01" );

        // query the number of passive ports
        if( m_passiveToolPortsAvailable )
            m_passiveToolPorts = SFLIST( "02" );

        // query the number of active tool ports supporting TIP detection
        if( m_TIPSupportAvailable )
            m_activeToolPortsTIP = SFLIST( "04" );

        // query the number of active wireless ports
        if( m_activeWirelessToolsPortsAvailable )
            m_activeWirelessToolPorts = SFLIST( "05" );
    }
    else if( m_receiveBuffer.find("Aurora") != std::string::npos ||
             m_receiveBuffer.find("aurora") != std::string::npos ||
             m_receiveBuffer.find("AURORA") != std::string::npos )
    {
        m_deviceType = NDIComm::Aurora;

        throw std::runtime_error("NDIComm::getDeviceInfo: Aurora device currently not supported.");

//        // query the number of active ports
//        uint32_t magneticPorts = SFLIST( "10" );

//        // query the number of passive ports
//        uint32_t FGs = SFLIST( "12" );
    }
    else
        throw std::runtime_error("NDIComm::getDeviceInfo: could not identify device.");
}


void NDIComm::initTargets()
{
    // query which ports shpuld be initialized
    PHSR("01");

    // get the number of available ports
    uint32_t handleCount = 0;
    sscanf(m_receiveBuffer.data(),"%02x",&handleCount);

    // run over all handles and release unused handles
    for( uint32_t i=0; i<handleCount; i++ )
        PHF( m_receiveBuffer.substr(i*5 + 2, 2) );

    // check if there were any tools specified
    if( m_romIDs.size() == 0 )
        throw std::runtime_error("NDIComm::initTargets: no ROM files specified.");

    // load rom files
    for( size_t i=0; i<m_romIDs.size(); i++ )
    {
        // open the ROM file
        std::ifstream file( m_romFilenames[i].c_str(), std::ios::in|std::ios::binary);
        if( !file.is_open() )
        {
            std::string err( "NDIComm::initTargets: could not open file: " );
            err.append( m_romFilenames[i] );
            throw std::runtime_error( err );
        }

        // get the length of the file
        file.seekg(0, std::ios::end);
        std::streamsize fileSize = file.tellg();
        file.seekg(0, std::ios::beg);
        uint32_t bytesCount = static_cast<uint32_t>(fileSize);

        // read the ROM file and close the file
        char buffer[1024];
        for( size_t j=0; j<1024; j++ )
            buffer[j] = 0;
        file.read(buffer, fileSize);
        file.close();

        // get a port handle
        uint32_t portHandle = PHRQ();

        // convet the hex bytes into ASCII
        char charBuffer[2049];
        for( size_t j=0; j<1024; j++ )
            sprintf( &charBuffer[j*2], "%02X", static_cast<uint8_t>(buffer[j]) );

        // transmit the rom file (PVWR breakes it into chuncks and sends them)
        PVWR( portHandle, bytesCount, charBuffer );

        // init the handle
        PINIT( portHandle );

        // enable the handle/target
        PENA( portHandle );

        // if made it so far than all is well and the handle is solid
        m_portHandles.push_back( portHandle );

        // update the port handle indices
        m_portHandleIndices.insert( std::pair<size_t,uint32_t>( i, portHandle ) );
    }
}


uint32_t NDIComm::grabTarget( char* src, uint32_t &handle, Eigen::Affine3f &trans, int &trackingStatus, float &error, uint32_t &frameNo )
{
    // init stuff
    float q[4];
    float t[3];
    uint32_t status = 0;
    uint32_t offset = 0;
    Eigen::Affine3f tempTrans;
    bool targetMissing = false;

    // get the port handle
    sscanf(&src[offset],"%02x",&handle);
    offset += 2;

    // check if this target is missing
    const char missing[] = "MISSING";
    if( strncmp( missing, &src[offset], 7 ) == 0 )
    {
        targetMissing = true;
        offset += 7;
    }
    else
    {
        // grab the quaernion
        for( uint32_t i=0; i<4; i++ )
        {
            sscanf(&src[offset],"%6f",&q[i]);
            q[i] *= 0.0001f;
            offset += 6;
        }

        // grab the translation
        for( uint32_t i=0; i<3; i++ )
        {
            sscanf(&src[offset],"%7f",&t[i]);
            t[i] *= 0.01f * 0.001f;
            offset += 7;
        }

        // set transformation
        tempTrans.setIdentity();
        tempTrans.translate( Eigen::Vector3f( t[0], t[1], t[2] ) );
        tempTrans.rotate( Eigen::Quaternionf( q[0], q[1], q[2], q[3] ) );
//        tempTrans.matrix()[12] = 0.0f;
//        tempTrans.matrix()[13] = 0.0f;
//        tempTrans.matrix()[14] = 0.0f;
        trans = tempTrans;

        // grab the error
        sscanf(&src[offset],"%6f",&error);
        error *= 0.0001f;
        offset += 6;
    }

    // grab the port status
    sscanf(&src[offset],"%8X",&status);
    offset += 8;

    // determine the tracking status (check if initialized and enables)
    trackingStatus = TrackerTarget::NO;
    if( !targetMissing )
    {
        if( status & NDIComm::OK )
            trackingStatus = TrackerTarget::OK;
        else
        {
            if( status & NDIComm::Partial )
                trackingStatus = TrackerTarget::Partial;
            else
            {
                if( status & NDIComm::Interference )
                    trackingStatus = TrackerTarget::Interference;
                else if( status & NDIComm::NO )
                    trackingStatus = TrackerTarget::NO;
            }
        }
    }

    // grab the frame number
    sscanf(&src[offset],"%8X",&frameNo);
    offset +=8;

    return offset;
}


void NDIComm::COMM(unsigned int baudRate)
{
    // init stuff
    std::string command( "COMM:" );

    // convert the standard baud rate into an indexed version known to the device
    switch( baudRate )
    {
        case 9600: command.append("0"); break;
        case 14400: command.append("1"); break;
        case 19200: command.append("2"); break;
        case 38400: command.append("3"); break;
        case 57600: command.append("4"); break;
        case 115200: command.append("5"); break;
        case 921600: command.append("6"); break;
        case 1228739: command.append("7"); break;
        default:
            std::stringstream err;
            err << "NDIComm::COMM: invalid baud rate: ";
            err << baudRate;
            throw std::runtime_error( err.str() );
    }

    // set the rest to 8 data bits, no parity, one stop bit and no HW handshaking
    command.append("0000");

    // send the command
    sendCommand( command, true, "NDI::COMM", "OKAY" );
}


void NDIComm::RESET()
{
    // init stuff
    std::string command( "RESET:" );

    // send the command
    sendCommand( command, true, "NDI::RESET", "RESET,SCUONLY" );
}


void NDIComm::INIT()
{
    // init stuff
    std::string command( "INIT:" );

    // send the command
    sendCommand( command, true, "NDI::INIT", "OKAY" );
}


void NDIComm::VER4()
{
    // init stuff
    std::string command("VER:4");

    // send the command
    sendCommand( command, true, "NDI::VER4" );
}


uint32_t NDIComm::SFLIST( const std::string &replyOption )
{
    // init stuff
    std::string command("SFLIST:");
    command.append(replyOption);

    // check if the reply option is supported
    if( replyOption.compare("00") == 0 ||
        replyOption.compare("01") == 0 ||
        replyOption.compare("02") == 0 ||
        replyOption.compare("03") == 0 ||
        replyOption.compare("04") == 0 ||
        replyOption.compare("05") == 0 )
    {
        sendCommand( command, true, "NDI::SFLIST" );
    }
    else
        throw std::runtime_error("NDIComm::SFLIST: unsupported reply option.");

    // parse the result
    if( replyOption.compare("00") == 0 && m_receiveBuffer.size() == 8 )
    {
        // 8 hex (32 bits)
        uint32_t result = 0;
        sscanf(&m_receiveBuffer[6],"%04x",&result);
        return result;
    }
    else if( replyOption.compare("01") == 0 ||
             replyOption.compare("02") == 0 ||
             replyOption.compare("04") == 0 ||
             replyOption.compare("05") == 0 )
    {
        // one hex (should be one byte)
        uint32_t result = 0;
        sscanf(m_receiveBuffer.data(),"%01x",&result);
        return result;
    }
    else
        throw std::runtime_error("NDIComm::SFLIST: unsupported reply option.");
}


void NDIComm::PHSR( const std::string &replyOption )
{
    std::string command("PHSR:");
    command.append(replyOption);

    // check if the reply option is supported
    if( replyOption.compare("00") == 0 ||
        replyOption.compare("01") == 0 ||
        replyOption.compare("02") == 0 ||
        replyOption.compare("03") == 0 ||
        replyOption.compare("04") == 0 )
    {
        sendCommand(command, true, "NDI:PHSR");
    }
    else
        throw std::runtime_error("NDIComm::PHSR: unsupported reply option.");
}


void NDIComm::PHF( const std::string &handle )
{
    std::string command("PHF:");
    command.append(handle);

    sendCommand(command, true, "NDIComm::PHF", "OKAY");
}


uint32_t NDIComm::PHRQ()
{
    std::string command("PHRQ:********01****");

    // send the command
    sendCommand(command, true, "NDIComm::PHRQ");

    // get the port handle
    uint32_t result = 0;
    sscanf(&m_receiveBuffer[0],"%02x",&result);

    return result;
}


void NDIComm::PVWR( uint32_t portHandle, uint32_t bytesCount, const char *buffer )
{
    for( uint32_t offset=0; offset<bytesCount*2; offset+=128 )
    {
        std::string command("PVWR:");

        // convert and append the port handle
        command.resize( command.size()+2 );
        sprintf(&command[command.size()-2],"%02X",portHandle);

        // convert and append the offset
        command.resize( command.size()+4 );
        sprintf(&command[command.size()-4],"%04X",static_cast<uint32_t>(offset/2));

        // assemble the command
        for( size_t i=0; i<128; i++ )
            command.push_back(buffer[offset+i]);

        // send the command
        sendCommand(command, true, "NDIComm::PVWR", "OKAY");
    }
}


void NDIComm::PINIT( uint32_t portHandle )
{
    std::string command("PINIT:");

    // convert and append the port handle
    command.resize( command.size()+2 );
    sprintf(&command[command.size()-2],"%02X",portHandle);

    // send the command
    sendCommand(command, true, "NDIComm::PINIT", "OKAY");
}


void NDIComm::PENA( uint32_t portHandle )
{
    std::string command("PENA:");

    // append the port handle
    command.resize( command.size()+2 );
    sprintf(&command[command.size()-2],"%02X",portHandle);

    // append the tool priority (set to dynamic as all are moving)
    command.append("D");

    // send the command
    sendCommand(command, true, "NDIComm::PENA", "OKAY");
}


void NDIComm::TSTART()
{
    std::string command("TSTART:80");

    sendCommand(command, true, "NDIComm::TSTART", "OKAY");
}


void NDIComm::TSTOP()
{
    std::string command("TSTOP:");

    sendCommand(command, true, "NDIComm::TSTOP", "OKAY");
}


void NDIComm::TX()
{
    std::string command("TX:0001");

    sendCommand(command, true, "NDIComm::TX");
}


bool NDIComm::sendCommand( const std::string &command, bool addCRC )
{
    // clear the send buffer
    m_sendBuffer.clear();

    // write the command to the send buffer
    m_sendBuffer.append( command );
    size_t length = m_sendBuffer.size();

    // optionally add CRC code
    if( addCRC )
    {
        // replace ' ' with ':'
        for( size_t i=0; i<length; i++ )
            if( ' ' == m_sendBuffer[i] )
                m_sendBuffer[i] = ':';

        // now compute the CRC
        uint32_t crc = getCRC( m_sendBuffer );

        // convert the crc to a string and append it to the send buffer
        m_sendBuffer.resize( length+4 );
        sprintf(&m_sendBuffer[length],"%04X",crc);
        length += 4;
    }

    // append the carriage return
    m_sendBuffer.resize( length + 2 );
    m_sendBuffer[ length ] = s_cr;
    m_sendBuffer[ length + 1 ] = 0;
#ifdef DEBUG
    std::cout << "out: " << m_sendBuffer << std::endl;
#endif
    // message complete, not send
    size_t out = transmit( m_sendBuffer.data(), m_sendBuffer.size()-1 );

    return out > 0;
}


bool NDIComm::sendCommand( const std::string &command, bool addCRC, const std::string &methode )
{
    if( sendCommand( command, addCRC ) )
    {
        // if sending was ok than receive the reply
        if( receiveCommand() )
            return true;
        else
        {
            std::string err(methode);
            err.append( ": did not receive any response." );
            throw std::runtime_error(err);
        }
    }
    else
    {
        std::string err(methode);
        err.append( ": nothing sent." );
        throw std::runtime_error(err);
    }
}


bool NDIComm::sendCommand( const std::string &command, bool addCRC, const std::string &methode, const std::string &expectedResponse )
{
    if( sendCommand( command, addCRC ) )
    {
        // if sending was ok than receive the reply
        if( receiveCommand() )
            return checkResponse(expectedResponse, methode);
        else
        {
            std::string err(methode);
            err.append( ": did not receive any response." );
            throw std::runtime_error(err);
        }
    }
    else
    {
        std::string err(methode);
        err.append( ": nothing sent." );
        throw std::runtime_error(err);
    }
}


bool NDIComm::receiveCommand( unsigned int timeout )
{
    // clear the receive buffer
    m_receiveBuffer.clear();

    // receive
    size_t in = 0;
    char buffer = 0;
    for( ; in<s_bufferSize; in++ )
    {
        // get char
        if( receive( &buffer, 1, timeout ) > 0 )
        {
            if( s_cr == buffer )
                break;
            else
            {
                m_receiveBuffer.resize( in + 1 );
                m_receiveBuffer[in] = buffer;
            }
        }
        else
            break;
    }

    // check if anything was received
    if( in > 0 )
    {
        // check the CRC of the command
        bool crcOK = checkCRC( m_receiveBuffer );

        // resize the string to hide the CRC code
        if( crcOK )
            m_receiveBuffer.resize( m_receiveBuffer.size() - 4 );

        // return true if something was received
        return crcOK;
    }
    else
        return false;
}


bool NDIComm::checkResponse()
{
    // init stuff
    std::string message;

    // check if it was an error
    if( checkError( message ) )
        throw std::runtime_error(message);

    // check if it was a warning
    if( checkWarning( message ) )
    {
#ifdef DEBUG
        std::cout << "NDIComm::checkResponse: WARNING: " << message << std::endl;
#endif
        return false;
    }

    // no error nor warning
    return true;
}


bool NDIComm::checkResponse( const std::string &expectedResponse )
{
    // try to breake appart the string in case there are multiple expected responces
    std::vector<std::string> expectedResponses;
    boost::split(expectedResponses, expectedResponse, boost::is_any_of(","));

    // remove empty strings
    expectedResponses.erase( std::remove_if( expectedResponses.begin(), expectedResponses.end(), boost::bind( &std::string::empty, _1 ) ), expectedResponses.end() );

    // compare the responses to the
    for( size_t i=0; i<expectedResponses.size(); i++ )
        if( m_receiveBuffer.compare( expectedResponses[i] ) == 0 )
            return true;

    // nothing found
    return false;
}


bool NDIComm::checkResponse( const std::string &expectedResponse, const std::string &methode )
{
    if( checkResponse(expectedResponse) )
        return true;
    else
    {
        // init stuff
        std::string message(methode);
        message.append(": ");

        // check if it was an error
        if( checkError( message ) )
            throw std::runtime_error(message);

        // check if it was a warning
        if( checkWarning( message ) )
        {
            std::cout << message << std::endl;
            return true;
        }

        // none of the previous
        message.append( "unexpected response: " );
        message.append( m_receiveBuffer );
        throw std::runtime_error(message);
    }
}

bool NDIComm::checkError( std::string &error )
{
    // check if this is an error
    std::string err("ERROR");
    if( m_receiveBuffer.size() >= 5 && err.compare( m_receiveBuffer.substr(0,err.size()) ) == 0 )
    {
        // check that the error also has an error code
        if( m_receiveBuffer.size() == 7 )
        {
            uint32_t errorCode = 0;
            sscanf(&(m_receiveBuffer[5]),"%02x",&errorCode);

            // check which code this is
            switch( errorCode )
            {
                case 0x01 : error.append( "Error: Invalid command." ); break;
                case 0x02 : error.append( "Error: Command too long." ); break;
                case 0x03 : error.append( "Error: Command too short." ); break;
                case 0x04 : error.append( "Error: Invalid CRC calculated for command; calculated CRC does not match the one sent." ); break;
                case 0x05 : error.append( "Error: Time-out on command execution." ); break;
                case 0x06 : error.append( "Error: Unable to set up new communication parameters. This occurs if one of the communication parameters is out of range." ); break;
                case 0x07 : error.append( "Error: Incorrect number of parameters." ); break;
                case 0x08 : error.append( "Error: Invalid port handle selected." ); break;
                case 0x09 : error.append( "Error: Invalid mode selected. Either the tracking priority is out of range, or an incorrect priority was selected (e.g. the tool has markers defined and \"button box\" was selected)." ); break;
                case 0x0A : error.append( "Error: Invalid LED selected. The LED selected is out of range." ); break;
                case 0x0B : error.append( "Error: Invalid LED state selected. The LED state selected is out of range." ); break;
                case 0x0C : error.append( "Error: Command is invalid while in the current mode." ); break;
                case 0x0D : error.append( "Error: No tool is assigned to the selected port handle." ); break;
                case 0x0E : error.append( "Error: Selected port handle not initialized. The port handle needs to be initialized before the command is sent." ); break;
                case 0x0F : error.append( "Error: Selected port handle not enabled. The port handle needs to be initialized before the command is sent." ); break;

                case 0x10 : error.append( "Error: System not initialized. The system must be initialized before the command is sent." ); break;
                case 0x11 : error.append( "Error: Unable to stop tracking. This occurs if there are hardware problems. Please contact NDI." ); break;
                case 0x12 : error.append( "Error: Unable to start tracking. This occurs if there are hardware problems. Please contact NDI." ); break;
                case 0x13 : error.append( "Error: Unable to initialize tool-in-port." ); break;
                case 0x14 : error.append( "Error: Invalid Position Sensor characterization parameters." ); break;
                case 0x15 : error.append( "Error: Unable to initialize the system." ); break;
                case 0x16 : error.append( "Error: Unable to start Diagnostic mode. This occurs if there are hardware problems. Please contact NDI." ); break;
                case 0x17 : error.append( "Error: Unable to stop Diagnostic mode. This occurs if there are hardware problems. Please contact NDI." ); break;
                case 0x18 : error.append( "Error: Unable to check for environmental infrared light." ); break;
                case 0x19 : error.append( "Error: Unable to read device's firmware version information." ); break;
                case 0x1A : error.append( "Error: Internal system error. This occurs when the system is unable to recover after too much IR or a system processing exception." ); break;
                case 0x1B : error.append( "Error: Unable to initialize for environmental infrared diagnostics." ); break;
                case 0x1C : error.append( "Error: Unable to set marker activation signature." ); break;
                case 0x1D : error.append( "Error: Unable to find SROM device IDs." ); break;
                case 0x1E : error.append( "Error: Unable to read SROM device data." ); break;
                case 0x1F : error.append( "Error: Unable to write SROM device data." ); break;

                // TODO: fill out the other error cases

                default :
                    error.append( "Error: unknown error code: " );
                    error.append( m_receiveBuffer.substr( 5, 2 ) );
                    error.append( "." );
                    break;
            }

            return true;
        }
        else
        {
            error.append( "Error." );
            return true;
        }
    }
    else
        return false;
}


bool NDIComm::checkWarning(std::string &warning)
{
    // find out which warning this is
    if( m_receiveBuffer.compare("WARNING") == 0 )
    {
        warning.append( "Warning: A non-fatal tool error has been encountered, e.g. a burnt out marker." );
        return true;
    }
    else if( m_receiveBuffer.compare("WARNING02") == 0 )
    {
        warning.append( "Warning: The tool you are trying to enable is a unique geometry tool that doesn’t meet the unique geometry requirements." );
        return true;
    }
    else if( m_receiveBuffer.compare("WARNING03") == 0 )
    {
        warning.append( "Warning: The tool you are trying to enable is a unique geometry tool that conflicts with another unique geometry tool already loaded and enabled." );
        return true;
    }
    else if( m_receiveBuffer.compare("WARNING04") == 0 )
    {
        warning.append( "Warning: The tool you are trying to enable is a unique geometry tool that doesn’t meet the unique geometry requirements, and conflicts with another unique geometry tool already loaded and enabled." );
        return true;
    }
    else
        return false;
}


uint32_t NDIComm::getCRC( const std::string &str)
{
    // init stuff
    uint32_t crc = 0;

    // compute the CRC
    for( size_t i=0; i<str.size(); i++ )
    {
        // update the CRC code
        crc = m_crcTable[ (crc ^ str[i]) & 0xFF] ^ (crc >> 8);
        crc = (crc & 0xFFFF);
    }

    return crc;
}


bool NDIComm::checkCRC( const std::string &str )
{
    // compute the CRC of the string without the appended CRC
    std::string cleanString = str.substr(0, str.size()-4);
    uint32_t crc = getCRC( cleanString );

    // get the CRC embedded in the string
    uint32_t stringCRC = 0;
    sscanf(&(str[str.size()-4]),"%04x",&stringCRC);

    // compare the two
    return stringCRC == crc;
}


//} // namespace eos
