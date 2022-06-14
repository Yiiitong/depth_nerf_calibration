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


#include <ndi_polaris/SerialComm.hpp>
#include <boost/optional.hpp>


boost::shared_ptr<boost::asio::io_service> SerialComm::m_io_service( new boost::asio::io_service() );


SerialComm::SerialComm() :
    m_serial_port(0)
{

}


SerialComm::~SerialComm()
{
    close();
}


void SerialComm::open(std::string port, unsigned int baudRate)
{
    if( !isOpen() )
    {
        try
        {
            m_serial_port = new boost::asio::serial_port(*(m_io_service).get(),port);
            m_serial_port->set_option( boost::asio::serial_port_base::baud_rate( baudRate ) );
        }
        catch( std::exception &e )
        {
            close();
            std::string err( "SerialComm::open: " );
            err.append(e.what());
            throw std::runtime_error(err);
        }
    }
    else
        throw std::runtime_error( "SerialComm::open: serial port already open" );
}


void SerialComm::close()
{
    if( isOpen() )
    {
        delete m_serial_port;
        m_serial_port = 0;
    }
}


size_t SerialComm::transmit(const void* tx, size_t len)
{
    // check if the port is open
    isOpen( true );

    size_t out;
    out = boost::asio::write(*m_serial_port,boost::asio::buffer(tx,len));
#ifdef DEBUG
    std::cout << "SerialComm::transmit(" << out << ")";
    if( out > 0 )
    {
        std::cout << ": ";
        for (unsigned int i = 0; i<out; i++)
            std::cout << ((char*)tx)[i];
    }
    std::cout << std::endl;
#endif
    return out;
}


void set_result(boost::optional<boost::system::error_code>* a, boost::system::error_code b)
{
    a->reset(b);
}


template <typename MutableBufferSequence> unsigned int read_with_timeout(boost::asio::serial_port &s, const MutableBufferSequence& buffers, unsigned int timeout)
{
    boost::optional<boost::system::error_code> timer_result;
    boost::asio::deadline_timer timer((boost::asio::io_context&)(s).get_executor().context());
    timer.expires_from_now(boost::posix_time::milliseconds(timeout));
    timer.async_wait(boost::bind(set_result, &timer_result, _1));

    boost::optional<boost::system::error_code> read_result;
    async_read(s, buffers,
                    boost::bind(set_result, &read_result, _1));

    if (read_result)
                {timer.cancel();}
    else if (timer_result)
                s.cancel();
    if (*read_result) return 0;
    else return boost::asio::buffer_size(buffers);
}


size_t SerialComm::receive(void* rx, size_t len,unsigned int timeout)
{
    if( timeout > 0 )
    {
        // check if the port is open
        isOpen( true );

        size_t received;
        //timeout in milliseconds
        received = read_with_timeout(*m_serial_port, boost::asio::buffer(rx,len),timeout);
#ifdef DEBUG
        std::cout << "SerialComm::receive(" << received << ")";
        if( received > 0 )
        {
            std::cout << ": ";
            for (unsigned int i = 0; i<received; i++)
                std::cout << ((char*)rx)[i];
        }
        std::cout << std::endl;
#endif
        return received;
    }
    else
        return receive( rx, len );
}


size_t SerialComm::receive( void* rx, size_t len )
{
    // check if the port is open
    isOpen( true );

    size_t received;
    received = boost::asio::read(*m_serial_port,boost::asio::buffer(rx,len));
#ifdef DEBUG
    std::cout << "SerialComm::receive(" << received << ")";
    if( received > 0 )
    {
        std::cout << ": ";
        for (unsigned int i = 0; i<received; i++)
            std::cout << ((char*)rx)[i];
    }
    std::cout << std::endl;
#endif
    return received;
}


unsigned int SerialComm::sendMsg(void *tx, unsigned int ltx, void *rx, unsigned int lrx, unsigned int timeout)
{
    unsigned int ret;
    boost::mutex::scoped_lock l(m_semaphore);

    transmit(tx,ltx);
    ret = receive(rx,lrx,timeout);

    l.unlock();

    return ret;
}


unsigned int SerialComm::MSBtoLSB(unsigned int i)
{
    return (i & 0xFF)*0x1000000 + (i & 0xFF00)*0x100 + (i & 0xFF0000)/0x100 + (i & 0xFF000000)/0x1000000;
}


unsigned int SerialComm::LSBtoMSB(unsigned int i)
{
    return (i & 0xFF)*0x1000000 + (i & 0xFF00)*0x100 + (i & 0xFF0000)/0x100 + (i & 0xFF000000)/0x1000000;
}


bool SerialComm::isOpen( bool complain )
{
    if( complain && (m_serial_port == 0) )
        throw std::runtime_error( "SerialComm::isOpen: connection not open" );
    else
        return m_serial_port != 0;
}


//} // namespace eos

