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

#ifndef CHELEMENTSHELL_H
#define CHELEMENTSHELL_H

#include "ChApiFEA.h"
#include "ChElementGeneric.h"


namespace chrono {
namespace fea {


/// Base class for most structral elements of 'shell' type.

class ChApiFea ChElementShell : public ChElementGeneric {
  protected:
    double mass;

  public:
    ChElementShell() {}

    virtual ~ChElementShell() {}

    //
    // shell-specific functions
    //

    /// Gets the xyz displacement of a point on the shell,
    /// and the rotation RxRyRz of section reference, at parametric coordinates 'u' and 'v'.
    /// Note, u=-1..+1 , v= -1..+1.
    /// Note, 'displ' is the displ.state of nodes, ex. get it as GetStateBlock()
    /// Results are not corotated.
    virtual void EvaluateSectionDisplacement(const double u,
                                             const double v,
                                             const ChMatrix<>& displ,
                                             ChVector<>& u_displ,
                                             ChVector<>& u_rotaz) = 0;

    /// Gets the absolute xyz position of a point on the shell,
    /// and the absolute rotation of section reference,  at parametric coordinates 'u' and 'v'.
    /// Note, u=-1..+1 , v= -1..+1.
    /// Note, 'displ' is the displ.state of nodes, ex. get it as GetStateBlock()
    /// Results are corotated.
    virtual void EvaluateSectionFrame(const double u,
                                      const double v,
                                      const ChMatrix<>& displ,
                                      ChVector<>& point,
                                      ChQuaternion<>& rot) = 0;

    /// Gets the absolute xyz position of a point on the shell,
    /// at parametric coordinates 'u' and 'v'.
    /// Note, u=-1..+1 , v= -1..+1.
    /// Note, 'displ' is the displ.state of nodes, ex. get it as GetStateBlock()
    /// Results are corotated.
    virtual void EvaluateSectionPoint(const double u,
                                      const double v,
                                      const ChMatrix<>& displ,
                                      ChVector<>& point) = 0;

/*
    /// TODO?????
    /// Gets the tensional state at a point on the shell
    /// at parametric coordinates 'u' and 'v'.
    /// Note, u=-1..+1 , v= -1..+1.
    /// Note, 'displ' is the displ.state of  nodes, ex. get it as GetStateBlock().
    /// Results are not corotated, and are expressed in the reference system of beam.
    virtual void EvaluateSectionForceTorque(const double eta,
                                            const ChMatrix<>& displ,
                                            ChVector<>& Fforce,
                                            ChVector<>& Mtorque) = 0;
*/

};

}  // END_OF_NAMESPACE____
}  // END_OF_NAMESPACE____

#endif
