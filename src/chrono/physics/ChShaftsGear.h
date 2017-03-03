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

#ifndef CHSHAFTSGEAR_H
#define CHSHAFTSGEAR_H

#include "chrono/physics/ChShaftsCouple.h"
#include "chrono/solver/ChConstraintTwoGeneric.h"

namespace chrono {

/// Class for defining a 'transmission ratio' (a 1D gear) between two one-degree-of-freedom
/// parts; i.e., shafts that can be used to build 1D models of powertrains. This is more
/// efficient than simulating power trains modeled with full 3D ChBody objects.
/// Note that this really simple constraint does not provide a way to trasmit a reaction
/// force to the truss, if this is needed, just use the ChShaftsPlanetary with a fixed
/// carrier shaft, or the ChShaftGearbox.

class ChApi ChShaftsGear : public ChShaftsCouple {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChShaftsGear)

  private:
    double ratio;                       ///< transmission ratio t, as in w2=t*w1, or t=w2/w1
    double torque_react;                ///< reaction torque
    ChConstraintTwoGeneric constraint;  ///< used as an interface to the solver

  public:
    ChShaftsGear();
    ChShaftsGear(const ChShaftsGear& other);
    ~ChShaftsGear() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChShaftsGear* Clone() const override { return new ChShaftsGear(*this); }

    /// Number of scalar constraints
    virtual int GetDOC_c() override { return 1; }

    //
    // STATE FUNCTIONS
    //

    // (override/implement interfaces for global state vectors, see ChPhysicsItem for comments.)
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

    // Override/implement system functions of ChShaftsCouple
    // (to assemble/manage data for system solver)

    virtual void InjectConstraints(ChSystemDescriptor& mdescriptor) override;
    virtual void ConstraintsBiReset() override;
    virtual void ConstraintsBiLoad_C(double factor = 1, double recovery_clamp = 0.1, bool do_clamp = false) override;
    virtual void ConstraintsLoadJacobians() override;
    virtual void ConstraintsFetch_react(double factor = 1) override;

    // Other functions

    /// Use this function after gear creation, to initialize it, given
    /// two shafts to join.
    /// Each shaft must belong to the same ChSystem.
    bool Initialize(std::shared_ptr<ChShaft> mshaft1,  ///< first  shaft to join
                    std::shared_ptr<ChShaft> mshaft2   ///< second shaft to join
                    ) override;

    /// Set the transmission ratio t, as in w2=t*w1, or t=w2/w1 , or  t*w1 - w2 = 0.
    /// For example, t=1 for a rigid joint; t=-0.5 for representing
    /// a couple of spur gears with teeths z1=20 & z2=40; t=0.1 for
    /// a gear with inner teeths (or epicycloidal reducer), etc.
    void SetTransmissionRatio(double mt) { ratio = mt; }
    /// Get the transmission ratio t, as in w2=t*w1, or t=w2/w1
    double GetTransmissionRatio() const { return ratio; }

    /// Get the reaction torque exchanged between the two shafts,
    /// considered as applied to the 1st axis.
    double GetTorqueReactionOn1() const override { return ratio * torque_react; }

    /// Get the reaction torque exchanged between the two shafts,
    /// considered as applied to the 2nd axis.
    double GetTorqueReactionOn2() const override { return -torque_react; }

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

CH_CLASS_VERSION(ChShaftsGear,0)


}  // end namespace chrono

#endif
