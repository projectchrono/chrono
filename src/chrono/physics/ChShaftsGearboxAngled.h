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

#ifndef CHSHAFTSGEARBOXANGLED_H
#define CHSHAFTSGEARBOXANGLED_H

#include "chrono/physics/ChBodyFrame.h"
#include "chrono/physics/ChPhysicsItem.h"
#include "chrono/physics/ChShaft.h"
#include "chrono/solver/ChConstraintThreeGeneric.h"

namespace chrono {

// Forward references (for parent hierarchy pointer)

class ChShaft;
class ChBodyFrame;

///  Class for defining a gearbox with 1D input and 1D output, but with
///  different directions in 3D space. Basically it defines a
///  transmission ratio between two 1D entities of ChShaft type, and
///  transmits the reaction of the gearbox to a 3D body that acts as the
///  support truss.
///  A typical example is the case of a gearbox with bevel gears,
///  where input shaft and output shaft are at 90°.
///  Note that the more basic ChShaftsGear can do the same, except
///  that it does not provide a way to transmit reaction
///  to a truss body.

class ChApi ChShaftsGearboxAngled : public ChPhysicsItem {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChShaftsGearboxAngled)

  private:
    double t0;

    double torque_react;  ///< reaction torque

    ChConstraintThreeGeneric constraint;  ///< used as an interface to the solver

    ChShaft* shaft1;    ///< first connected shaft
    ChShaft* shaft2;    ///< second connected shaft
    ChBodyFrame* body;  ///< connected body

    ChVector<> shaft_dir1;  ///< direction of first shaft
    ChVector<> shaft_dir2;  ///< direction of second shaft

  public:
    ChShaftsGearboxAngled();
    ChShaftsGearboxAngled(const ChShaftsGearboxAngled& other);
    ~ChShaftsGearboxAngled() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChShaftsGearboxAngled* Clone() const override { return new ChShaftsGearboxAngled(*this); }

    /// Get the number of scalar variables affected by constraints in this link
    virtual int GetNumCoords() const { return 6 + 1 + 1; }

    /// Number of scalar costraints
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

    // Override/implement system functions of ChPhysicsItem
    // (to assemble/manage data for system solver)

    virtual void InjectConstraints(ChSystemDescriptor& mdescriptor) override;
    virtual void ConstraintsBiReset() override;
    virtual void ConstraintsBiLoad_C(double factor = 1, double recovery_clamp = 0.1, bool do_clamp = false) override;
    virtual void ConstraintsBiLoad_Ct(double factor = 1) override;
    virtual void ConstraintsLoadJacobians() override;
    virtual void ConstraintsFetch_react(double factor = 1) override;

    // Other functions

    /// Use this function after gear creation, to initialize it, given
    /// two shafts to join, and the 3D body that acts as a truss and
    /// receives the reaction torque of the gearbox.
    /// Each shaft and body must belong to the same ChSystem.
    /// Shafts directions are considered in local body coordinates.
    virtual bool
    Initialize(std::shared_ptr<ChShaft> mshaft1,    ///< first (input) shaft to join
               std::shared_ptr<ChShaft> mshaft2,    ///< second  (output) shaft to join
               std::shared_ptr<ChBodyFrame> mbody,  ///< 3D body to use as truss (also carrier, if rotates as in planetary gearboxes)
               ChVector<>& mdir1,  ///< the direction of the first shaft on 3D body defining the gearbox truss
               ChVector<>& mdir2   ///< the direction of the first shaft on 3D body defining the gearbox truss
               );

    /// Get the first shaft (carrier wheel)
    ChShaft* GetShaft1() { return shaft1; }
    /// Get the second shaft
    ChShaft* GetShaft2() { return shaft2; }
    /// Get the third shaft
    ChBodyFrame* GetBodyTruss() { return body; }

    /// Set the transmission ratio t, as in w2=t*w1, or t=w2/w1 , or  t*w1 - w2 = 0.
    /// For example, t=1 for equal bevel gears, etc.
    void SetTransmissionRatio(double mt0) { t0 = mt0; }

    /// Get the transmission ratio t, as in w2=t*w1, or t=w2/w1.
    double GetTransmissionRatio() const { return t0; }

    /// Set the direction of shaft 1 (input) respect to 3D body, as a
    /// normalized vector expressed in the coordinates of the body.
    /// The shaft applies only torque, about this axis.
    void SetShaftDirection1(ChVector<> md) { shaft_dir1 = Vnorm(md); }

    /// Set the direction of shaft 2 (output) respect to 3D body, as a
    /// normalized vector expressed in the coordinates of the body.
    /// The shaft applies only torque, about this axis.
    void SetShaftDirection2(ChVector<> md) { shaft_dir2 = Vnorm(md); }

    /// Get the direction of the shaft 1 (input) respect to 3D body, as a
    /// normalized vector expressed in the coordinates of the body.
    const ChVector<>& GetShaftDirection1() const { return shaft_dir1; }

    /// Get the direction of the shaft 2 (input) respect to 3D body, as a
    /// normalized vector expressed in the coordinates of the body.
    const ChVector<>& GetShaftDirection2() const { return shaft_dir2; }

    /// Get the reaction torque considered as applied to the 1st axis.
    double GetTorqueReactionOn1() const { return t0 * torque_react; }

    /// Get the reaction torque considered as applied to the 2nd axis.
    double GetTorqueReactionOn2() const { return (-1.0 * torque_react); }

    /// Get the reaction torque considered as applied to the body
    /// (the truss of the gearbox), expressed in local body coordinates.
    ChVector<> GetTorqueReactionOnBody() const {
        return (shaft_dir1 * t0 - shaft_dir2) * torque_react;
    }

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

CH_CLASS_VERSION(ChShaftsGearboxAngled,0)

}  // end namespace chrono

#endif
