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

#ifndef CHSHAFTSBODY_H
#define CHSHAFTSBODY_H

#include "chrono/physics/ChBodyFrame.h"
#include "chrono/physics/ChShaft.h"
#include "chrono/solver/ChConstraintTwoGeneric.h"

namespace chrono {

// Forward references (for parent hierarchy pointer)
class ChShaft;
class ChBodyFrame;

/// Class for creating a constraint between a 3D ChBody object and a 1D ChShaft object.
/// A rotation axis must be specified (to tell along which direction the shaft inertia
/// and rotation affects the body).
/// This constraint is useful, for example, when you have modeled a 3D car using ChBody
/// items and a 1D powertrain (gears, differential, etc.) using ChShaft objects: you
/// can connect the former (at least, the wheels) to the latter using this constraint.

class ChApi ChShaftsBody : public ChPhysicsItem {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChShaftsBody)

  private:
    double torque_react;                ///< reaction torque
    ChConstraintTwoGeneric constraint;  ///< used as an interface to the solver
    ChShaft* shaft;                     ///< connected shaft
    ChBodyFrame* body;                  ///< connected body
    ChVector<> shaft_dir;               ///< shaft direction

  public:
    ChShaftsBody();
    ChShaftsBody(const ChShaftsBody& other);
    ~ChShaftsBody() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChShaftsBody* Clone() const override { return new ChShaftsBody(*this); }

    /// Get the number of scalar variables affected by constraints in this link
    virtual int GetNumCoords() const { return 6 + 1; }

    /// Number of scalar costraints
    virtual int GetDOC_c() override { return 1; }

    // Override/implement interfaces for global state vectors, see ChPhysicsItem for comments.

    virtual void IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) override;
    virtual void IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) override;
    virtual void IntLoadResidual_CqL(const unsigned int off_L,
                                     ChVectorDynamic<>& R,
                                     const ChVectorDynamic<>& L,
                                     const double c) override;
    virtual void IntLoadConstraint_C(const unsigned int off,
                                     ChVectorDynamic<>& Qc,
                                     const double c,
                                     bool do_clamp,
                                     double recovery_clamp) override;
    virtual void IntLoadConstraint_Ct(const unsigned int off, ChVectorDynamic<>& Qc, const double c) override {}
    virtual void IntToDescriptor(const unsigned int off_v,
                                 const ChStateDelta& v,
                                 const ChVectorDynamic<>& R,
                                 const unsigned int off_L,
                                 const ChVectorDynamic<>& L,
                                 const ChVectorDynamic<>& Qc) override;
    virtual void IntFromDescriptor(const unsigned int off_v,
                                   ChStateDelta& v,
                                   const unsigned int off_L,
                                   ChVectorDynamic<>& L) override;

    // Override/implement system functions of ChPhysicsItem
    // (to assemble/manage data for system solver)

    virtual void InjectConstraints(ChSystemDescriptor& mdescriptor) override;
    virtual void ConstraintsBiReset() override;
    virtual void ConstraintsBiLoad_C(double factor = 1, double recovery_clamp = 0.1, bool do_clamp = false) override;
    virtual void ConstraintsBiLoad_Ct(double factor = 1) override;
    virtual void ConstraintsLoadJacobians() override;
    virtual void ConstraintsFetch_react(double factor = 1) override;

    /// Use this function after object creation, to initialize it, given
    /// the 1D shaft and 3D body to join.
    /// Each item must belong to the same ChSystem.
    /// Direction is expressed in the local coordinates of the body.
    bool Initialize(std::shared_ptr<ChShaft> mshaft,  ///< shaft to join
                    std::shared_ptr<ChBodyFrame>
                        mbody,              ///< body to join
                    const ChVector<>& mdir  ///< the direction of the shaft on 3D body (applied on COG: pure torque)
                    );

    /// Get the shaft
    ChShaft* GetShaft() { return shaft; }
    /// Get the body
    ChBodyFrame* GetBody() { return body; }

    /// Set the direction of the shaft respect to 3D body, as a
    /// normalized vector expressed in the coordinates of the body.
    /// The shaft applies only torque, about this axis.
    void SetShaftDirection(ChVector<> md) { shaft_dir = Vnorm(md); }

    /// Get the direction of the shaft respect to 3D body, as a
    /// normalized vector expressed in the coordinates of the body.
    const ChVector<>& GetShaftDirection() const { return shaft_dir; }

    /// Get the reaction torque considered as applied to ChShaft.
    double GetTorqueReactionOnShaft() const { return -(torque_react); }

    /// Get the reaction torque considered as applied to ChBody,
    /// expressed in the coordinates of the body.
    ChVector<> GetTorqueReactionOnBody() const { return (shaft_dir * torque_react); }

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

CH_CLASS_VERSION(ChShaftsBody,0)


}  // end namespace chrono

#endif
