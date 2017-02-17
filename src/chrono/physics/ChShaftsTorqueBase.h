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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHSHAFTSTORQUEBASE_H
#define CHSHAFTSTORQUEBASE_H

#include "chrono/physics/ChShaftsCouple.h"
#include "chrono/solver/ChConstraintTwoGeneric.h"

namespace chrono {

/// Base class for all stuff defining a torque between two one-degree-of-freedom
/// parts, for example torsional dampers, torsional springs, electric engines, etc.
/// This helps to keep things simple: derived classes just have to implement ComputeTorque().

class ChApi ChShaftsTorqueBase : public ChShaftsCouple {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChShaftsTorqueBase)

  protected:
    double torque;  ///< store actual value of torque

  public:
    ChShaftsTorqueBase();
    ChShaftsTorqueBase(const ChShaftsTorqueBase& other);
    ~ChShaftsTorqueBase() {}

    /// Number of scalar constraints
    virtual int GetDOC_c() override { return 0; }

    //
    // STATE FUNCTIONS
    //

    // (override/implement interfaces for global state vectors, see ChPhysicsItem for comments.)
    virtual void IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) override;

    // Override/implement system functions of ChShaftsCouple
    // (to assemble/manage data for system solver)

    // Adds the torsional torques in the 'fb' part: qf+=torques*factor
    // of both shafts
    void VariablesFbLoadForces(double factor = 1) override;

    /// Get the reaction torque exchanged between the two shafts,
    /// considered as applied to the 1st axis.
    virtual double GetTorqueReactionOn1() const override { return torque; }

    /// Get the reaction torque exchanged between the two shafts,
    /// considered as applied to the 2nd axis.
    virtual double GetTorqueReactionOn2() const override { return -torque; }

    //
    // UPDATE FUNCTIONS
    //

    /// NOTE: children classes MUST implement this.
    /// In most cases, this is the ONLY function you need to implement
    /// in children classes. It will be called at each Update().
    virtual double ComputeTorque() = 0;

    /// Update all auxiliary data of the gear transmission at given time
    virtual void Update(double mytime, bool update_assets = true) override;

    //
    // SERIALIZATION
    //

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};

CH_CLASS_VERSION(ChShaftsTorqueBase,0)


}  // end namespace chrono

#endif
