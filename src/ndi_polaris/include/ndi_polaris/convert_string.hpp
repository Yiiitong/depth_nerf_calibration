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

#include <iostream>
#include <string>
#include <vector>

#include <boost/algorithm/string.hpp>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>

#include "blas.hpp"


/*
 * convert_string.hpp
 *
 *  Created on: May 4, 2010
 *      Author: duliu
 */


/////
// matrix initialization from strings
///
namespace Eigen
{
    template<typename T, int Rows, int Cols>
    bool operator >> ( std::istream &s, Eigen::Matrix<T, Rows, Cols> &m )
    {
        // read the string from the istream
        std::string src;
        std::string line;
        while( !s.eof() )
        {
            getline( s, line );
            src.append( line );
        }

        // split the string
        std::vector<std::string > strings;
        boost::split(strings, src, boost::is_any_of("\n\t :;"), boost::algorithm::token_compress_on);

        // remove empty strings
        strings.erase( std::remove_if( strings.begin(), strings.end(), boost::bind( &std::string::empty, _1 ) ), strings.end() );

        // check if sizes match
        if( static_cast<size_t>(Rows*Cols) != strings.size() )
            return false;
        else
        {
            for( int row=0; row<Rows; row++ )
                for( int col=0; col<Cols; col++ )
                    m.data()[ col*Rows + row ] = boost::lexical_cast<T>( strings[ row*Cols + col ] );

            return true;
        }
    }


    template<typename T, int Dim, int Mode>
    bool operator >> ( std::istream &s, Eigen::Transform<T, Dim, Mode> &t )
    {
        return s >> t.matrix();
    }
}


/////
// the actual conversion functions
///

//namespace eos
//{

namespace util
{

/////
// ROOT TEMPLATE FUNCTION
///
template<typename T, typename R>
inline void convert( const T &src, R &dst );


/////
// STRING 2 STRING
///

template<typename T, typename R>
inline void convert( const std::basic_string<T> &src, std::basic_string<R> &dst )
{
    dst = src;
}



/////
// FROM STRING
///

template<typename T, typename R>
inline void convert( const std::basic_string<T> &src, R &dst )
{
    dst = boost::lexical_cast<R>( src );
}


// vector
template<typename T, typename R>
inline void convert( const std::basic_string<T> &src, std::vector<R> &dst )
{
    // split the string
    std::vector<std::basic_string<T> > strings;
    boost::split(strings, src, boost::is_any_of("\t :;"));

    // remove empty strings
    strings.erase( std::remove_if( strings.begin(), strings.end(), boost::bind( &std::string::empty, _1 ) ), strings.end() );

    // now convert the individual values
    if( strings.size() > 0 )
    {
        dst.resize( strings.size() );
        for( size_t i=0; i<strings.size(); i++ )
            convert( strings[i], dst[i] );
    }
}


//// vector of matrices
//template< typename T, typename R, int Rows, int Cols >
//inline void convert( const std::basic_string<T> &src, std::vector<Eigen::Matrix< R, Rows, Cols > > &dst )
//{
//    // get a vector of scalars
//    std::vector<R> values;
//    convert(src, values);

//    // determine the sizes
//    size_t mat_size = static_cast<size_t>(Rows*Cols);

//    // convert
//    dst.resize(values.size()/mat_size);
//    for( size_t i=0; i<values.size()/mat_size; i++ )
//        for( int row=0; row<Rows; row++ )
//            for( int col=0; col<Cols; col++ )
//                dst[i].data()[ col*Rows + row ] = static_cast<R>( values[ i*mat_size + row*Cols + col ] );
//}



/////
// TO STRING
///

template<typename T, typename R>
inline void convert( const T &src, std::basic_string<R> &dst )
{
    dst = boost::lexical_cast<std::basic_string<R> >( src );
}


// vector
template<typename T, typename R>
inline void convert( const std::vector<T> &src, std::basic_string<R> &dst )
{
    dst.clear();
    for( size_t idx=0; idx<src.size(); idx++ )
    {
        std::basic_string<R> tmp;
        convert( src[idx], tmp );
        dst.append( tmp );
        dst.append( " " );
    }
}


//// matrix
//template< typename T, typename R, int Rows, int Cols >
//inline void convert( const Eigen::Matrix< T, Rows, Cols > &src, std::basic_string<R> &dst )
//{
//    // init stuff
//    dst.clear();
//
//    // convert vector
//    for( int i=0; i<Rows*Cols; i++ )
//    {
//        std::basic_string<R> tmp;
//        convert( src.data()[i], tmp );
//        dst.append( tmp );
//        dst.append(" ");
//    }
//}
//
//
//// vector of matrices
//template< typename T, typename R, int Rows, int Cols >
//inline void convert( const std::vector< Eigen::Matrix< T, Rows, Cols > > &src, std::basic_string<R> &dst )
//{
//    dst.clear();
//    for( int idx=0; idx<src.size(); idx++ )
//    {
//        // convert the current matrix to string
//        std::basic_string<R> mat;
//        convert( src[idx], mat );
//
//        // push back
//        mat.append("; ");
//        dst.append( mat );
//    }
//}



///////
//// FROM STRING CONVENIENCE
/////
//template<typename T, typename R>
//inline R convert( const std::basic_string<T> &src)
//{
//    R result;
//    convert( src, result );
//    return result;
//}


//template<typename T, typename R>
//inline std::vector<R> convert( const std::basic_string<T> &src)
//{
//    return convert<std::vector<R> >(src);
//}


//template<class T>
//inline T convert( const std::string &src)
//{
//    T result;
//    convert( src, result );
//    return result;
//}


//template<typename T, typename R>
//inline std::basic_string<R> convert( const T &src)
//{
//    std::basic_string<R> result;
//    convert( src, result );
//    return result;
//}



} // end namespace util

//} // end namespace eos

