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
//  This code uses the polar decomposition implementation provided as a companion to the book "Graphics Gems IV": 
//  Decompose.c 
//  Ken Shoemake, 1993 
//  Polar Decomposition of 3x3 matrix in 4x4, M = QS.  
//  The Graphics Gems IV implementation is available at: 
//  http://tog.acm.org/GraphicsGems/
//
// File author: Alessandro Tasora


#ifndef CHPOLARDECOMPOSITION_H
#define CHPOLARDECOMPOSITION_H

#include "ChApiFEM.h"
#include "core/ChMatrix.h"


namespace chrono
{
namespace fem
{



	/*
  Polar decomposition of a general 3x3 matrix

  This code uses the polar decomposition implementation provided as a companion to the book "Graphics Gems IV": 
  Decompose.c 
  Ken Shoemake, 1993 
  Polar Decomposition of 3x3 matrix in 4x4, M = QS.  
  The Graphics Gems IV implementation is available at: 
  http://tog.acm.org/GraphicsGems/

  The above website states that "All code here (on the GraphicsGems website) can be used without restrictions". It also lists the following EULA:
  "EULA: The Graphics Gems code is copyright-protected. In other words, you cannot claim the text of the code as your own and resell it. Using the code is permitted in any program, product, or library, non-commercial or commercial. Giving credit is not required, though is a nice gesture. The code comes as-is, and if there are any flaws or problems with any Gems code, nobody involved with Gems - authors, editors, publishers, or webmasters - are to be held responsible. Basically, don't be a jerk, and remember that anything free comes with no guarantee."

  Jernej Barbic made some adaptions to the polar decomposition code (wrap into a C++ class, some change in input/output format, etc.). 
  He releases his adaptions of the polar decomposition code into the public domain, free of charge. The above EULA still applies, of course.
*/
	// This class is wrapped by ChPolarDecomposition (see below)
	// It is based on VEGA (J. Barbic).
class PolarDecomposition
{
public:

  // Computes the Polar Decomposition of a general 3x3 matrix M.
  // M = Q * S
  // M is 3x3 input matrix
  // Q is 3x3 orthogonal output matrix, Q Q^T = Q^T Q = I 
  // S is 3x3 symmetric output matrix
  // Note: det(Q)=sgn(det(M)); this sign can be 1 or -1, depending on M
  // M is not modified
  // All matrices are row-major
  static double Compute(const double * M, double * Q, double * S, double tolerance = 1E-6);

protected:

  // one-norm of a 3 x 3 matrix
  static double oneNorm(const double * A);

  // infinity-norm of a 3 x 3 matrix
  static double infNorm(const double * A);

  // a, b, c are 3-vectors
  // compute cross product c = a x b
  inline static void crossProduct(const double * a, const double * b, double * c)
	{
		c[0] = a[1] * b[2] - a[2] * b[1];
		c[1] = a[2] * b[0] - a[0] * b[2];
		c[2] = a[0] * b[1] - a[1] * b[0];
	}

};

// compute the one-norm of a 3x3 matrix (row-major)
double PolarDecomposition::oneNorm(const double * A)
{
  double norm = 0.0;
  for (int i=0; i<3; i++) 
  {
    double columnAbsSum = fabs(A[i + 0]) + fabs(A[i + 3]) + fabs(A[i + 6]);
    if (columnAbsSum > norm) 
      norm = columnAbsSum;
  }
  return norm;
}

// compute the inf-norm of a 3x3 matrix (row-major)
double PolarDecomposition::infNorm(const double * A)
{
  double norm = 0.0;
  for (int i=0; i<3; i++) 
  {
    double rowSum = fabs(A[3 * i + 0]) + fabs(A[3 * i + 1]) + fabs(A[3 * i + 2]);
    if (rowSum > norm) 
      norm = rowSum;
  }
  return norm;
}

// Input: M (3x3 mtx)
// Output: Q (3x3 rotation mtx), S (3x3 symmetric mtx)
double PolarDecomposition::Compute(const double * M, double * Q, double * S, double tolerance)
{
  double Mk[9];
  double Ek[9];
  double det, M_oneNorm, M_infNorm, E_oneNorm;

  // Mk = M^T
  for(int i=0; i<3; i++)
    for(int j=0; j<3; j++)
      Mk[3 * i + j] = M[3 * j + i];

  M_oneNorm = oneNorm(Mk); 
  M_infNorm = infNorm(Mk);

  do 
  {
    double MadjTk[9];
 
    // row 2 x row 3
    crossProduct(&(Mk[3]), &(Mk[6]), &(MadjTk[0])); 
    // row 3 x row 1
    crossProduct(&(Mk[6]), &(Mk[0]), &(MadjTk[3]));
    // row 1 x row 2
    crossProduct(&(Mk[0]), &(Mk[3]), &(MadjTk[6]));

    det = Mk[0] * MadjTk[0] + Mk[1] * MadjTk[1] + Mk[2] * MadjTk[2];
    if (det == 0.0) 
    {
      printf("Warning (polarDecomposition) : zero determinant encountered.\n");
      break;
    }

    double MadjT_one = oneNorm(MadjTk); 
    double MadjT_inf = infNorm(MadjTk);

    double gamma = sqrt(sqrt((MadjT_one * MadjT_inf) / (M_oneNorm * M_infNorm)) / fabs(det));
    double g1 = gamma * 0.5;
    double g2 = 0.5 / (gamma * det);

    for(int i=0; i<9; i++)
    {
      Ek[i] = Mk[i];
      Mk[i] = g1 * Mk[i] + g2 * MadjTk[i];
      Ek[i] -= Mk[i];
    }

    E_oneNorm = oneNorm(Ek);
    M_oneNorm = oneNorm(Mk);  
    M_infNorm = infNorm(Mk);
  }
  while ( E_oneNorm > M_oneNorm * tolerance );

  // Q = Mk^T 
  for(int i=0; i<3; i++)
    for(int j=0; j<3; j++)
      Q[3*i+j] = Mk[3*j+i];

  for(int i=0; i<3; i++)
    for(int j=0; j<3; j++)
    {
      S[3*i+j] = 0;
      for(int k=0; k<3; k++)
        S[3*i+j] += Mk[3*i+k] * M[3*k+j];
    }
    
  // S must be symmetric; enforce the symmetry
  for (int i=0; i<3; i++) 
    for (int j=i; j<3; j++)
      S[3 * i + j] = S[3 * j + i] = 0.5 * (S[3 * i + j] + S[3 * j + i]);

  return (det);
}







// -----------------------------------------




/// Perform a polar decomposition of a 3x3 P matrix in order to retrieve
/// the orthogonal Q and the symmetric S form, as P=Q*S
/// 

template <class Real = double>
class ChApiFem ChPolarDecomposition 
{
public:

	// Computes the polar decomposition of a generic 3x3 matrix M, as M = Q * S
	// Input:
	//   M is a 3x3 input matrix
	// Output:
	//   Q is a 3x3 orthogonal output matrix
	//   S is a 3x3 symmetric output matrix
	//   return value: det(Q) that can be -1 or +1.


	static double Compute(const ChMatrix33<Real>& M,  /// a 3x3 input matrix to decompose
								ChMatrix33<Real>& Q,  /// resulting 3x3 orthogonal output matrix
								ChMatrix33<Real>& S,  /// resulting 3x3 symmetric output matrix
								double tolerance = 1E-6)
	{
		return PolarDecomposition::Compute(M.GetAddress(), Q.GetAddress(), S.GetAddress());
	}

};





} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif






