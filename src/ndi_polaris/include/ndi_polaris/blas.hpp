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
 * blas.hpp
 *
 *  Created on: Mar 29, 2011
 *      Author: duliu
 *
 *  Description: wrapper around Eigen's blas implementation
 */

/////
// define custom IOFormat
//
#define EIGEN_DEFAULT_IO_FORMAT Eigen::IOFormat(StreamPrecision, DontAlignCols, " ", " ", "", "", "", " ")

/////
// include Eigen
///
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>


namespace blas
{

/////
// Constants
///
static const double pi = 3.14159265358979323846;


} // end namespace blas
