// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora
// =============================================================================
//
//  This code uses the polar decomposition implementation provided as a companion to the book "Graphics Gems IV":
//  Decompose.c
//  Ken Shoemake, 1993
//  Polar Decomposition of 3x3 matrix in 4x4, M = QS.
//  The Graphics Gems IV implementation is available at:
//  http://tog.acm.org/GraphicsGems/
//
// =============================================================================

#ifndef CHPOLARDECOMPOSITION_H
#define CHPOLARDECOMPOSITION_H

#include "chrono/core/ChMatrix33.h"
#include "chrono_fea/ChApiFEA.h"

namespace chrono {
namespace fea {

/*
Polar decomposition of a general 3x3 matrix

This code uses the polar decomposition implementation provided as a companion to the book "Graphics Gems IV":
Decompose.c
Ken Shoemake, 1993
Polar Decomposition of 3x3 matrix in 4x4, M = QS.
The Graphics Gems IV implementation is available at:
http://tog.acm.org/GraphicsGems/

The above website states that "All code here (on the GraphicsGems website) can be used without restrictions". It also
lists the following EULA:
"EULA: The Graphics Gems code is copyright-protected. In other words, you cannot claim the text of the code as your own
and resell it. Using the code is permitted in any program, product, or library, non-commercial or commercial. Giving
credit is not required, though is a nice gesture. The code comes as-is, and if there are any flaws or problems with any
Gems code, nobody involved with Gems - authors, editors, publishers, or webmasters - are to be held responsible.
Basically, don't be a jerk, and remember that anything free comes with no guarantee."

Jernej Barbic made some adaptations to the polar decomposition code (wrap into a C++ class, some change in input/output
format, etc.).
He releases his adaptations of the polar decomposition code into the public domain, free of charge. The above EULA still
applies, of course.
*/
// This class is wrapped by ChPolarDecomposition (see below)
// It is based on VEGA (J. Barbic).
class PolarDecomposition {
  public:
    // Computes the Polar Decomposition of a general 3x3 matrix M.
    // M = Q * S
    // M is 3x3 input matrix
    // Q is 3x3 orthogonal output matrix, Q Q^T = Q^T Q = I
    // S is 3x3 symmetric output matrix
    // Note: det(Q)=sgn(det(M)); this sign can be 1 or -1, depending on M
    // M is not modified
    // All matrices are row-major
    static double Compute(const double* M, double* Q, double* S, double tolerance = 1E-6);

  protected:
    // one-norm of a 3 x 3 matrix
    static double oneNorm(const double* A);

    // infinity-norm of a 3 x 3 matrix
    static double infNorm(const double* A);

    // a, b, c are 3-vectors
    // compute cross product c = a x b
    inline static void crossProduct(const double* a, const double* b, double* c) {
        c[0] = a[1] * b[2] - a[2] * b[1];
        c[1] = a[2] * b[0] - a[0] * b[2];
        c[2] = a[0] * b[1] - a[1] * b[0];
    }
};

// -----------------------------------------

/// Perform a polar decomposition of a 3x3 P matrix in order to retrieve
/// the orthogonal Q and the symmetric S form, as P=Q*S
///

template <class Real = double>
class ChApiFea ChPolarDecomposition {
  public:
    // Computes the polar decomposition of a generic 3x3 matrix M, as M = Q * S
    // Input:
    //   M is a 3x3 input matrix
    // Output:
    //   Q is a 3x3 orthogonal output matrix
    //   S is a 3x3 symmetric output matrix
    //   return value: det(Q) that can be -1 or +1.

    static double Compute(const ChMatrix33<Real>& M,  ///< a 3x3 input matrix to decompose
                          ChMatrix33<Real>& Q,        ///< resulting 3x3 orthogonal output matrix
                          ChMatrix33<Real>& S,        ///< resulting 3x3 symmetric output matrix
                          double tolerance = 1e-6     ///< tolerance of the computation
    ) {
        return PolarDecomposition::Compute(M.GetAddress(), Q.GetAddress(), S.GetAddress());
    }
};

}  // end namespace fea
}  // end namespace chrono

#endif
