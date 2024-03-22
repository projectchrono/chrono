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
// This file contains a number of ready-to-use loads (ChLoad inherited classes and
// their embedded ChLoader classes) that can be applied to threedimensional beam
// elements in FEA.
// These are 'simplified' tools, that save you from inheriting your custom
// loads from ChLoaderUatomic ChLoaderUdistributed etc. Or just look at these
// classes and learn how to implement some special type of load.
// =============================================================================

#ifndef CHLOADSBEAM_H
#define CHLOADSBEAM_H

#include "chrono/physics/ChLoad.h"
#include "chrono/physics/ChLoaderU.h"
#include "chrono/fea/ChElementBeam.h"

namespace chrono {
namespace fea {

/// @addtogroup chrono_fea
/// @{

/// Atomic wrench.
/// Loader for a wrench (force+torque) at a specific position of a beam.
/// An atomic load on the beam is a wrench, i.e. force+load applied at
/// a certain abscissa U, that is a six-dimensional load.
/// It is not a distributed load, so inherit it from ChLoaderUatomic.
class ChLoaderBeamWrench : public ChLoaderUatomic {
  private:
    ChVector3d torque;
    ChVector3d force;

  public:
    // Useful: a constructor that also sets ChLoadable
    ChLoaderBeamWrench(std::shared_ptr<ChLoadableU> mloadable) : ChLoaderUatomic(mloadable) {
        this->torque = VNULL;
        this->force = VNULL;
    }

    // Compute F=F(u)
    // This is the function that you have to implement. It should return the
    // load at U. For Euler beams, loads are expected as 6-rows vectors, containing
    // a wrench: forceX, forceY, forceZ, torqueX, torqueY, torqueZ.
    virtual void ComputeF(double U,                    ///< parametric coordinate in line
                          ChVectorDynamic<>& F,        ///< result vector, size = field dim of loadable
                          ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate F
                          ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate F
    ) {
        F.segment(0, 3) = this->force.eigen();   // load, force part
        F.segment(3, 3) = this->torque.eigen();  // load, torque part
    }

    /// Set force (ex. in [N] units)
    void SetForce(const ChVector3d& mf) { this->force = mf; }
    ChVector3d GetForce() const { return this->force; }

    /// Set torque (ex. in [Nm] units)
    void SetTorque(const ChVector3d& mt) { this->torque = mt; }
    ChVector3d GetTorque() const { return this->torque; }
};

/// Atomic wrench (ready to use load)
/// Load for a wrench (force+torque) at a specific position of a beam.
/// The ChLoaderBeamWrench is a 'container' for ChLoadBeamWrench.
/// An atomic load on the beam is a wrench, i.e. force+load applied at
/// a certain abscissa U, that is a six-dimensional load.
/// It is not a distributed load, so inherit it from ChLoaderUatomic:
class ChLoadBeamWrench : public ChLoad {
  public:
    ChLoadBeamWrench(std::shared_ptr<ChLoadableU> loadable) {
        SetLoader(chrono_types::make_shared<ChLoaderBeamWrench>(loadable));
    }
    virtual ChLoadBeamWrench* Clone() const override { return new ChLoadBeamWrench(*this); }
    std::shared_ptr<ChLoaderBeamWrench> GetLoader() const {
        return std::static_pointer_cast<ChLoaderBeamWrench>(loader);
    }
};

//-----------------------------------------------------------------------------------------

/// Distributed constant wrench.
/// Loader for a constant wrench (force+torque) distributed on a beam.
/// An distributed wrench on the beam contains a "force per unit length"
/// and a "torque per unit length"
class ChLoaderBeamWrenchDistributed : public ChLoaderUdistributed {
  private:
    ChVector3d torqueperunit;
    ChVector3d forceperunit;

  public:
    // Useful: a constructor that also sets ChLoadable
    ChLoaderBeamWrenchDistributed(std::shared_ptr<ChLoadableU> mloadable) : ChLoaderUdistributed(mloadable) {
        this->torqueperunit = VNULL;
        this->forceperunit = VNULL;
    };

    // Compute F=F(u)
    // This is the function that you have to implement. It should return the
    // load at U. For Euler beams, loads are expected as 6-rows vectors, containing
    // a wrench: forceX, forceY, forceZ, torqueX, torqueY, torqueZ.
    virtual void ComputeF(double U,                    ///< parametric coordinate in line
                          ChVectorDynamic<>& F,        ///< result vector, size = field dim of loadable
                          ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate F
                          ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate F
    ) {
        F.segment(0, 3) = this->forceperunit.eigen();   // load, force part
        F.segment(3, 3) = this->torqueperunit.eigen();  // load, torque part
    }

    virtual int GetIntegrationPointsU() { return 1; }

    /// Set force per unit length (ex. [N/m] )
    void SetForcePerUnit(const ChVector3d& mf) { this->forceperunit = mf; }
    ChVector3d GetForcePerUnit() const { return this->forceperunit; }

    /// Set torque per unit length (ex. [Nm/m] )
    void SetTorquePerUnit(const ChVector3d& mt) { this->torqueperunit = mt; }
    ChVector3d GetTorquePerUnit() const { return this->torqueperunit; }
};

/// Distributed constant wrench (ready to use load)
/// Load for a wrench (force+torque) at a specific position of a beam.
/// The ChLoaderBeamWrench is a 'container' for ChLoadBeamWrench.
/// An atomic load on the beam is a wrench, i.e. force+load applied at
/// a certain abscissa U, that is a six-dimensional load.
/// It is not a distributed load, so inherit it from ChLoaderUatomic:
class ChLoadBeamWrenchDistributed : public ChLoad {
  public:
    ChLoadBeamWrenchDistributed(std::shared_ptr<ChLoadableU> loadable) {
        SetLoader(chrono_types::make_shared<ChLoaderBeamWrenchDistributed>(loadable));
    }
    virtual ChLoadBeamWrenchDistributed* Clone() const override { return new ChLoadBeamWrenchDistributed(*this); }
    std::shared_ptr<ChLoaderBeamWrenchDistributed> GetLoader() const {
        return std::static_pointer_cast<ChLoaderBeamWrenchDistributed>(loader);
    }
};

/// @} chrono_fea

}  // end namespace fea
}  // end namespace chrono

#endif
