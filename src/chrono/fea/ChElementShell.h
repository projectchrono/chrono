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

#ifndef CHELEMENTSHELL_H
#define CHELEMENTSHELL_H

#include "chrono/fea/ChElementGeneric.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_elements
/// @{

/// Base class for most structural elements of 'shell' type.
class ChApi ChElementShell : public ChElementGeneric {
  public:
    ChElementShell() : mass(0) {}

    virtual ~ChElementShell() {}

    //
    // shell-specific functions
    //

    /// Gets the xyz displacement of a point on the shell,
    /// and the rotation RxRyRz of section reference, at parametric coordinates 'u' and 'v'.
    /// Note, u=-1..+1 , v= -1..+1 parametric coordinates, except if triangular shell, where u=0..+1, v=0..+1, natural
    /// triangle coords. Results are not corotated.
    virtual void EvaluateSectionDisplacement(const double u,
                                             const double v,
                                             ChVector3d& u_displ,
                                             ChVector3d& u_rotaz) = 0;

    /// Gets the absolute xyz position of a point on the shell,
    /// and the absolute rotation of section reference,  at parametric coordinates 'u' and 'v'.
    /// Note, u=-1..+1 , v= -1..+1 parametric coordinates, except if triangular shell, where u=0..+1, v=0..+1, natural
    /// triangle coords. Results are corotated.
    virtual void EvaluateSectionFrame(const double u, const double v, ChVector3d& point, ChQuaternion<>& rot) = 0;

    /// Gets the absolute xyz position of a point on the shell,
    /// at parametric coordinates 'u' and 'v'.
    /// Note, u=-1..+1 , v= -1..+1 parametric coordinates, except if triangular shell, where u=0..+1, v=0..+1, natural
    /// triangle coords. Results are corotated.
    virtual void EvaluateSectionPoint(const double u, const double v, ChVector3d& point) = 0;

    /// Virtual method to plot velocity field distribution.
    /// Note, u=-1..+1 , v= -1..+1 parametric coordinates, except if triangular shell, where u=0..+1, v=0..+1, natural
    /// triangle coords.
    virtual void EvaluateSectionVelNorm(double U, double V, ChVector3d& Result) = 0;

    /*
        /// TODO?????
        /// Gets the tensional state at a point on the shell
        /// at parametric coordinates 'u' and 'v'.
        /// Note, u=-1..+1 , v= -1..+1 parametric coordinates, except if triangular shell,
        /// where u=0..+1, v=0..+1, natural triangle coords.
        /// Results are not corotated, and are expressed in the reference system of beam.
        virtual void EvaluateSectionForceTorque(const double eta,
                                                ChVector3d& Fforce,
                                                ChVector3d& Mtorque) = 0;
    */

    /// Return false if quadrilateral shell - hence u,v parametric coordinates assumed in -1..+1,
    /// return true if triangular shell - hence u,v are triangle natural coordinates assumed in 0..+1
    virtual bool IsTriangleShell() { return false; }

    //
    // SERIALIZATION
    //

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override { 
        // version number
        archive_out.VersionWrite<ChElementShell>();
        // serialize parent class
        ChElementGeneric::ArchiveOut(archive_out);
        // serialize all member data:
        archive_out << CHNVP(this->mass);
    };

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override { 
        // version number
        /*int version =*/archive_in.VersionRead<ChElementShell>();
        // deserialize parent class
        ChElementGeneric::ArchiveIn(archive_in);
        // deserialize all member data:
        archive_in >> CHNVP(this->mass);
    };

  protected:
    double mass;
};

/// @} fea_elements

}  // end namespace fea
}  // end namespace chrono

#endif
