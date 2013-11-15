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
// File author: Alessandro Tasora


#include "ChMatrixCorotation.h"



namespace chrono
{
namespace fem
{

			/// Perform a corotation (warping) of a K matrix by pre-multiplying
			/// it with a C matrix; C has 3x3 rotation matrices R as diagonal blocks
void ChMatrixCorotation::ComputeCK(const ChMatrix<>& K,    /// matrix to corotate
						  const ChMatrix33<>& R,  /// 3x3 rotation matrix
						  const int	nblocks,		  /// number of rotation blocks
						  ChMatrix<>& CK)        /// result matrix: C*K
{
	for (int iblock=0; iblock < nblocks; iblock++)
	{
		double sum;
		for (int colres=0; colres < K.GetColumns(); ++colres)
			for (int row=0; row < 3; ++row)
			{
				sum = 0;
				for (int col=0; col < 3; ++col)
					sum+= R(row,col)* K((3*iblock)+col,colres);
				CK((3*iblock)+row, colres)= sum;
			}
	}
}

			/// Perform a corotation (warping) of a K matrix by post-multiplying
			/// it with a transposed C matrix; C has 3x3 rotation matrices R as diagonal blocks
void ChMatrixCorotation::ComputeKCt(const ChMatrix<>& K,    /// matrix to corotate
						   const ChMatrix33<>& R,				/// 3x3 rotation matrix (will be used transposed)
						   const int	nblocks,		  /// number of rotation blocks
						   ChMatrix<>& KC)        /// result matrix: C*K
{
	for (int iblock=0; iblock < nblocks; iblock++)
	{
		double sum;
		for (int rowres=0; rowres < K.GetRows(); ++rowres)
			for (int row=0; row < 3; ++row)
			{
				sum = 0;
				for (int col=0; col < 3; ++col)
					sum+= K(rowres,col+(3*iblock)) * R(row, col);
				KC(rowres, row+(3*iblock))= sum;
			}
	}
}

} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____








