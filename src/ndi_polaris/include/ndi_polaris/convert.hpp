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
 * convert.hpp
 *
 *  Created on: May 4, 2010
 *      Author: duliu
 */

#include <complex>

#include <boost/property_tree/ptree.hpp>

#include "convert_string.hpp"


//namespace eos {
namespace util {

/////
// MISC CONVERSIONS
///

// convert between two vectors of different types
template<typename Tsrc, typename Tdst>
inline void convert( const std::vector<Tsrc> &src, std::vector<Tdst> &dst )
{
        dst.resize(src.size());

        for( size_t i=0; i<dst.size(); i++ )
                dst[i] = static_cast<Tdst>( src[i] );
}


/////
// BLAS
///

// Quaternion to Vector
template<typename T>
inline void convert( const Eigen::Quaternion<T> &src, Eigen::Matrix<T, 3, 1>  &dst )
{
    dst = Eigen::Matrix<T, 3, 1>( src.x(), src.y(), src.z() );
}

template<typename T>
inline void convert( const Eigen::Quaternion<T> &src, Eigen::Matrix<T, 4, 1>  &dst )
{
    dst = Eigen::Matrix<T, 4, 1>( src.x(), src.y(), src.z(), src.w() );
}

// Vector to Quaternion
template<typename T>
inline void convert( const Eigen::Matrix<T, 3, 1> &src, Eigen::Quaternion<T> &dst )
{
    dst = Eigen::Quaternion<T>( src(0), src(1), src(2), static_cast<T>(0) );
}

template<typename T>
inline void convert( const Eigen::Matrix<T, 4, 1> &src, Eigen::Quaternion<T> &dst )
{
    dst = Eigen::Quaternion<T>( src(0), src(1), src(2), src(4) );
}

// matrices with different sizes
template<typename Ti, int Ri, int Ci, typename To, int Ro, int Co>
inline void convert( const Eigen::Matrix<Ti, Ri, Ci> &src, Eigen::Matrix<To, Ro, Co> &dst )
{
    // determine which is the smallest common size
    int Row = (Ri < Ro) ? Ri : Ro;
    int Col = (Ci < Co) ? Ci : Co;

    // copy
    for( int c=0; c<Col; c++ )
        for( int r=0; r<Row; r++ )
            dst(r,c) = src(r,c);
}


/////
// Property Tree
///

template<typename T, typename R>
inline void convert( const boost::property_tree::basic_ptree<std::basic_string<T>, std::basic_string<T> > &pt, R &dst )
{
    util::convert( pt.template get_value< std::basic_string<T> >(), dst );
}


template<typename T, typename R>
inline void convert( const boost::property_tree::basic_ptree<std::basic_string<T>, std::basic_string<T> > &pt, std::basic_string<R> &dst )
{
    util::convert( pt.template get_value< std::basic_string<T> >(), dst );
}


template<typename T, typename R>
inline void convert( const boost::property_tree::basic_ptree<std::basic_string<T>, std::basic_string<T> > &pt, std::vector< std::vector<R> > &values )
{
    // init stuff
    values.clear();

    for( typename boost::property_tree::basic_ptree<std::basic_string<T>, std::basic_string<T> >::const_iterator it = pt.begin(); it != pt.end(); it++ )
    {
        std::vector<R> val;
        util::convert( it->second, val );

        values.push_back(val);
    }
}



} // end namespace util

//} // end namespace eos

