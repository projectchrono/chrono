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

#ifndef CHLOADSBEAM_H
#define CHLOADSBEAM_H

#include "chrono/physics/ChLoad.h"
#include "chrono/physics/ChLoaderU.h"
#include "chrono_fea/ChElementBeam.h"

namespace chrono {
namespace fea {

// This file contains a number of ready-to-use loads (ChLoad inherited classes and
// their embedded ChLoader classes) that can be applied to threedimensional beam
// elements in FEA.
// These are 'simplified' tools, that save you from inheriting your custom
// loads from ChLoaderUatomic ChLoaderUdistributed etc. Or just look at these
// classes and learn how to implement some special type of load.

/// Atomic wrench.
/// Loader for a wrench (force+torque) at a specific position of a beam.
/// An atomic load on the beam is a wrench, i.e. force+load applied at
/// a certain abscissa U, that is a six-dimensional load.
/// It is not a distributed load, so inherit it from ChLoaderUatomic.
class ChLoaderBeamWrench : public ChLoaderUatomic {
  private:
    ChVector<> torque;
    ChVector<> force;

  public:
    // Useful: a constructor that also sets ChLoadable
    ChLoaderBeamWrench(std::shared_ptr<ChLoadableU> mloadable) : ChLoaderUatomic(mloadable) {
        this->torque = VNULL;
        this->force = VNULL;
    }

    // Compute F=F(u)
    // This is the function that you have to implement. It should return the
    // load at U. For Eulero beams, loads are expected as 6-rows vectors, containing
    // a wrench: forceX, forceY, forceZ, torqueX, torqueY, torqueZ.
    virtual void ComputeF(const double U,        ///< parametric coordinate in line
                          ChVectorDynamic<>& F,  ///< Result F vector here, size must be = n.field coords.of loadable
                          ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate F
                          ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate F
                          ) {
        F.PasteVector(this->force, 0, 0);   // load, force part
        F.PasteVector(this->torque, 3, 0);  // load, torque part
    }

    /// Set force (ex. in [N] units)
    void SetForce(const ChVector<>& mf) { this->force = mf; }
    ChVector<> GetForce() const { return this->force; }

    /// Set torque (ex. in [Nm] units)
    void SetTorque(const ChVector<>& mt) { this->torque = mt; }
    ChVector<> GetTorque() const { return this->torque; }
};

/// Atomic wrench (ready to use load)
/// Load for a wrench (force+torque) at a specific position of a beam.
/// The ChLoaderBeamWrench is a 'container' for ChLoadBeamWrench.
/// An atomic load on the beam is a wrench, i.e. force+load applied at
/// a certain abscissa U, that is a six-dimensional load.
/// It is not a distributed load, so inherit it from ChLoaderUatomic:
class ChLoadBeamWrench : public ChLoad<ChLoaderBeamWrench> {
  public:
    ChLoadBeamWrench(std::shared_ptr<ChLoadableU> mloadable) : ChLoad<ChLoaderBeamWrench>(mloadable) {}
};

//-----------------------------------------------------------------------------------------

/// Distributed constant wrench.
/// Loader for a constant wrench (force+torque) distributed on a beam.
/// An distributed wrench on the beam contains a "force per unit length"
/// and a "torque per unit length"
class ChLoaderBeamWrenchDistributed : public ChLoaderUdistributed {
  private:
    ChVector<> torqueperunit;
    ChVector<> forceperunit;

  public:
    // Useful: a constructor that also sets ChLoadable
    ChLoaderBeamWrenchDistributed(std::shared_ptr<ChLoadableU> mloadable) : ChLoaderUdistributed(mloadable) {
        this->torqueperunit = VNULL;
        this->forceperunit = VNULL;
    };

    // Compute F=F(u)
    // This is the function that you have to implement. It should return the
    // load at U. For Eulero beams, loads are expected as 6-rows vectors, containing
    // a wrench: forceX, forceY, forceZ, torqueX, torqueY, torqueZ.
    virtual void ComputeF(const double U,        ///< parametric coordinate in line
                          ChVectorDynamic<>& F,  ///< Result F vector here, size must be = n.field coords.of loadable
                          ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate F
                          ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate F
                          ) {
        F.PasteVector(this->forceperunit, 0, 0);   // load, force part
        F.PasteVector(this->torqueperunit, 3, 0);  // load, torque part
    }

    virtual int GetIntegrationPointsU() { return 1; }

    /// Set force per unit length (ex. [N/m] )
    void SetForcePerUnit(const ChVector<>& mf) { this->forceperunit = mf; }
    ChVector<> GetForcePerUnit() const { return this->forceperunit; }

    /// Set torque per unit length (ex. [Nm/m] )
    void SetTorquePerUnit(const ChVector<>& mt) { this->torqueperunit = mt; }
    ChVector<> GetTorquePerUnit() const { return this->torqueperunit; }
};

/// Distributed constant wrench (ready to use load)
/// Load for a wrench (force+torque) at a specific position of a beam.
/// The ChLoaderBeamWrench is a 'container' for ChLoadBeamWrench.
/// An atomic load on the beam is a wrench, i.e. force+load applied at
/// a certain abscissa U, that is a six-dimensional load.
/// It is not a distributed load, so inherit it from ChLoaderUatomic:
class ChLoadBeamWrenchDistributed : public ChLoad<ChLoaderBeamWrenchDistributed> {
  public:
    ChLoadBeamWrenchDistributed(std::shared_ptr<ChLoadableU> mloadable)
        : ChLoad<ChLoaderBeamWrenchDistributed>(mloadable) {}
};

}  // end namespace fea
}  // end namespace chrono

#endif
