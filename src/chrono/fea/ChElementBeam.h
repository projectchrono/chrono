// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora
// =============================================================================

#ifndef CH_ELEMENT_BEAM_H
#define CH_ELEMENT_BEAM_H

#include "chrono/fea/ChElementGeneric.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_elements
/// @{

/// Base class for beam structural elements.
class ChApi ChElementBeam : public ChElementGeneric {
  public:
    ChElementBeam() : length(1), mass(1) {}

    virtual ~ChElementBeam() {}

    /// Evaluate the displacement of a point on the beam line and the rotation of section plane.
    /// Note, eta=-1 at node1, eta=+1 at node2.
    virtual void EvaluateSectionDisplacement(const double eta, ChVector3d& displ, ChVector3d& rot) = 0;

    /// Evaluate the linear and angular velocity of a point on the beam line.
    /// Note, eta=-1 at node1, eta=+1 at node2.
    virtual void EvaluateSectionVelocity(const double eta, ChVector3d& lin_vel, ChVector3d& ang_vel) {
        lin_vel = VNULL;
        ang_vel = VNULL;
    }

    /// Evaluate the absolute position of a point on the beam line and the absolute rotation of the section plane.
    /// Note, eta=-1 at node1, eta=+1 at node2.
    virtual void EvaluateSectionFrame(const double eta, ChVector3d& point, ChQuaternion<>& rot) = 0;

    /// Evaluate the force (traction x, shear y, shear z) and the torque (torsion on x, bending on y, on bending on z) at a section along the beam line.
    /// Note, eta=-1 at node1, eta=+1 at node2.
    /// Results are not corotated, and are expressed in the reference system of beam.
    virtual void EvaluateSectionForceTorque(const double eta, ChVector3d& force, ChVector3d& torque) = 0;

    /// Evaluate the axial and bending strain of the beam element.
    virtual void EvaluateSectionStrain(const double eta, ChVector3d& strain) = 0;

    /// Return the full mass of the beam element.
    double GetMass() const { return mass; }

    /// Return the rest length of the beam element.
    double GetRestLength() const { return length; }

    /// Set the rest length of the bar.
    /// This is automatically done when SetupInitial is called on beams element, given the current state.
    /// This function allows overriding the calculated value, e.g., for pre-compressed beams.
    void SetRestLength(double l) { length = l; }


    //
    // SERIALIZATION
    //

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override {
        // version number
        archive_out.VersionWrite<ChElementBeam>();
        // serialize parent class
        ChElementGeneric::ArchiveOut(archive_out);
        // serialize all member data:
        archive_out << CHNVP(this->length);
        archive_out << CHNVP(this->mass);
    }

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override {
        // version number
        /*int version =*/archive_in.VersionRead<ChElementBeam>();
        // deserialize parent class:
        ChElementGeneric::ArchiveIn(archive_in);
        // deserialize all member data:
        archive_in >> CHNVP(this->length);
        archive_in >> CHNVP(this->mass);
    }


  protected:
    double mass;
    double length;
};

/// @} fea_elements

}  // end namespace fea
}  // end namespace chrono

#endif
