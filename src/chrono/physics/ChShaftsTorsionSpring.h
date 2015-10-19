//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHSHAFTSTORSIONSPRING_H
#define CHSHAFTSTORSIONSPRING_H

//////////////////////////////////////////////////
//
//   ChShaftsTorsionSpring.h
//
//   Class for defining a torsional spring-damper between
//   two one-degree-of-freedom parts, that is,
//   shafts that can be used to build 1D models
//   of power trains. This is more efficient than
//   simulating power trains modeled full 3D ChBody
//   objects.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "physics/ChShaftsTorqueBase.h"

namespace chrono {

///  Class for defining a torsional spring-damper between
///  between two one-degree-of-freedom parts, that is,
///  shafts that can be used to build 1D models
///  of power trains. This is more efficient than
///  simulating power trains modeled with full 3D ChBody
///  objects.

class ChApi ChShaftsTorsionSpring : public ChShaftsTorqueBase {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChShaftsTorsionSpring, ChShaftsTorqueBase);

  private:
    //
    // DATA
    //

    double stiffness;
    double damping;

  public:
    //
    // CONSTRUCTORS
    //

    /// Constructor.
    ChShaftsTorsionSpring();
    /// Destructor
    ~ChShaftsTorsionSpring();

    /// Copy from another ChShaftsTorsionSpring.
    void Copy(ChShaftsTorsionSpring* source);

    //
    // FUNCTIONS
    //

    /// Set the torsional stiffness between the two shafts
    void SetTorsionalStiffness(double mt) { this->stiffness = mt; }
    /// Get the torsional stiffness between the two shafts
    double GetTorsionalStiffness() const { return this->stiffness; }

    /// Set the torsional damping between the two shafts
    void SetTorsionalDamping(double mt) { this->damping = mt; }
    /// Get the torsional damping between the two shafts
    double GetTorsionalDamping() const { return this->damping; }

    //
    // UPDATE FUNCTIONS
    //

    /// This is the function that actually contains the
    /// formula for computing T=T(rot,vel,time,etc)
    virtual double ComputeTorque();

    //
    // SERIALIZATION
    //

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive);

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive);
};

}  // END_OF_NAMESPACE____

#endif
