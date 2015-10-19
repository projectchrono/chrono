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

#ifndef CHELEMENTBEAM_H
#define CHELEMENTBEAM_H

#include "ChElementGeneric.h"

namespace chrono {
namespace fea {

/// Base class for most structral elements of 'beam' type.

class ChApiFea ChElementBeam : public ChElementGeneric {
  protected:
    double mass;
    double length;

  public:
    ChElementBeam() {
        length = 0;  // will be computed by Setup(), later
        mass = 0;    // will be computed by Setup(), later
    }

    virtual ~ChElementBeam() {}

    //
    // beam-specific functions
    //

    /// Gets the xyz displacement of a point on the beam line,
    /// and the rotation RxRyRz of section plane, at abscyssa 'eta'.
    /// Note, eta=-1 at node1, eta=+1 at node2.
    /// Note, 'displ' is the displ.state of 2 nodes, ex. get it as GetStateBlock()
    /// Results are not corotated.
    virtual void EvaluateSectionDisplacement(const double eta,
                                             const ChMatrix<>& displ,
                                             ChVector<>& u_displ,
                                             ChVector<>& u_rotaz) = 0;

    /// Gets the absolute xyz position of a point on the beam line,
    /// and the absolute rotation of section plane, at abscyssa 'eta'.
    /// Note, eta=-1 at node1, eta=+1 at node2.
    /// Note, 'displ' is the displ.state of 2 nodes, ex. get it as GetStateBlock()
    /// Results are corotated.
    virtual void EvaluateSectionFrame(const double eta,
                                      const ChMatrix<>& displ,
                                      ChVector<>& point,
                                      ChQuaternion<>& rot) = 0;

    /// Gets the force (traction x, shear y, shear z) and the
    /// torque (torsion on x, bending on y, on bending on z) at a section along
    /// the beam line, at abscyssa 'eta'.
    /// Note, eta=-1 at node1, eta=+1 at node2.
    /// Note, 'displ' is the displ.state of 2 nodes, ex. get it as GetStateBlock().
    /// Results are not corotated, and are expressed in the reference system of beam.
    virtual void EvaluateSectionForceTorque(const double eta,
                                            const ChMatrix<>& displ,
                                            ChVector<>& Fforce,
                                            ChVector<>& Mtorque) = 0;
    /// Gets the axial and bending strain of the ANCF "cable" element

    virtual void EvaluateSectionStrain(const double eta, const ChMatrix<>& displ, ChVector<>& StrainV) = 0;
    /// The full mass of the beam, (with const. section, density, etc.)
    double GetMass() { return this->mass; }

    /// The rest length of the bar
    double GetRestLength() { return this->length; }

    /// Set the rest length of the bar (ususlly this should be automatically done
    /// when SetupInitial is called on beams element, given the current state, but one
    /// might need to override this, ex for precompressed beams etc).
    void SetRestLength(double ml) { this->length = ml; }
};

}  // END_OF_NAMESPACE____
}  // END_OF_NAMESPACE____

#endif
