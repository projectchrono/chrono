//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//
//
// File author: Alessandro Tasora


#ifndef CHMATRIXCOROTATION_H
#define CHMATRIXCOROTATION_H

#include "ChApiFEM.h"
#include "core/ChMatrix.h"


namespace chrono
{
namespace fem
{


/// Perform a corotation (warping) of a K matrix by pre- or post- multiplying
/// it with a C matrix that has 3x3 rotation matrices R as diagonal blocks,
/// so that C*K  means:
///            [R      ]  [       ]
///            [   R   ] *[   K   ]
///            [      R]  [       ]
///
/// This is often used in FEM codes to rotate a K local stiffness matrix 
/// and to obtain a global stiffness matrix. 
/// This class provides methods to do either C*K or also C*K*C' , without
/// explicitly building C, for improved performance.


class ChApiFem ChMatrixCorotation
{
public:

			/// Perform a corotation (warping) of a K matrix by pre-multiplying
			/// it with a C matrix; C has 3x3 rotation matrices R as diagonal blocks
	static void ComputeCK(const ChMatrix<>& K,    /// matrix to pre-corotate
						  const ChMatrix33<>& R,  /// 3x3 rotation matrix
						  const int	nblocks,	  /// number of rotation blocks
						  ChMatrix<>& CK);        /// result matrix: C*K

			/// Perform a corotation (warping) of a K matrix by post-multiplying
			/// it with a transposed C matrix; C has 3x3 rotation matrices R as diagonal blocks
	static void ComputeKCt(const ChMatrix<>& K,    /// matrix to post-corotate
						   const ChMatrix33<>& R,  /// 3x3 rotation matrix (will be used transposed)
						   const int	nblocks,   /// number of rotation blocks
						   ChMatrix<>& KC);        /// result matrix: C*K

};





} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif






