//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHSHAFTSGEARBOXANGLED_H
#define CHSHAFTSGEARBOXANGLED_H

#include "physics/ChPhysicsItem.h"
#include "physics/ChBodyFrame.h"
#include "physics/ChShaft.h"
#include "lcp/ChLcpConstraintThreeGeneric.h"

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
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChShaftsGearboxAngled, ChPhysicsItem);

  private:
    //
    // DATA
    //

    double t0;

    double torque_react;

    // used as an interface to the LCP solver.
    ChLcpConstraintThreeGeneric constraint;

    float cache_li_speed;  // used to cache the last computed value of multiplier (solver warm starting)
    float cache_li_pos;    // used to cache the last computed value of multiplier (solver warm starting)

    ChShaft* shaft1;
    ChShaft* shaft2;
    ChBodyFrame* body;

    ChVector<> shaft_dir1;
    ChVector<> shaft_dir2;

  public:
    //
    // CONSTRUCTORS
    //

    /// Constructor.
    ChShaftsGearboxAngled();
    /// Destructor
    ~ChShaftsGearboxAngled();

    /// Copy from another ChShaftsGearboxAngled.
    void Copy(ChShaftsGearboxAngled* source);

    //
    // FLAGS
    //

    //
    // FUNCTIONS
    //
    /// Get the number of scalar variables affected by constraints in this link
    virtual int GetNumCoords() { return 6 + 1 + 1; }

    /// Number of scalar costraints
    virtual int GetDOC_c() { return 1; }

    //
    // STATE FUNCTIONS
    //

    // (override/implement interfaces for global state vectors, see ChPhysicsItem for comments.)
    virtual void IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L);
    virtual void IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L);
    virtual void IntLoadResidual_CqL(const unsigned int off_L,
                                     ChVectorDynamic<>& R,
                                     const ChVectorDynamic<>& L,
                                     const double c);
    virtual void IntLoadConstraint_C(const unsigned int off,
                                     ChVectorDynamic<>& Qc,
                                     const double c,
                                     bool do_clamp,
                                     double recovery_clamp);
    virtual void IntToLCP(const unsigned int off_v,
                          const ChStateDelta& v,
                          const ChVectorDynamic<>& R,
                          const unsigned int off_L,
                          const ChVectorDynamic<>& L,
                          const ChVectorDynamic<>& Qc);
    virtual void IntFromLCP(const unsigned int off_v, ChStateDelta& v, const unsigned int off_L, ChVectorDynamic<>& L);

    // Override/implement LCP system functions of ChPhysicsItem
    // (to assembly/manage data for LCP system solver

    virtual void InjectConstraints(ChLcpSystemDescriptor& mdescriptor);
    virtual void ConstraintsBiReset();
    virtual void ConstraintsBiLoad_C(double factor = 1., double recovery_clamp = 0.1, bool do_clamp = false);
    virtual void ConstraintsBiLoad_Ct(double factor = 1.);
    // virtual void ConstraintsFbLoadForces(double factor=1.);
    virtual void ConstraintsLoadJacobians();
    virtual void ConstraintsLiLoadSuggestedSpeedSolution();
    virtual void ConstraintsLiLoadSuggestedPositionSolution();
    virtual void ConstraintsLiFetchSuggestedSpeedSolution();
    virtual void ConstraintsLiFetchSuggestedPositionSolution();
    virtual void ConstraintsFetch_react(double factor = 1.);

    // Other functions

    /// Use this function after gear creation, to initialize it, given
    /// two shafts to join, and the 3D body that acts as a truss and
    /// receives the reaction torque of the gearbox.
    /// Each shaft and body must belong to the same ChSystem.
    /// Shafts directions are considered in local body coordinates.
    virtual int Initialize(
        std::shared_ptr<ChShaft> mshaft1,    ///< first (input) shaft to join
        std::shared_ptr<ChShaft> mshaft2,    ///< second  (output) shaft to join
        std::shared_ptr<ChBodyFrame> mbody,  ///< 3D body to use as truss (also carrier, if rotates as in planetary gearboxes)
        ChVector<>& mdir1,                   ///< the direction of the first shaft on 3D body defining the gearbox truss
        ChVector<>& mdir2                    ///< the direction of the first shaft on 3D body defining the gearbox truss
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
    double GetTorqueReactionOn1() const { return (this->t0 * torque_react); }

    /// Get the reaction torque considered as applied to the 2nd axis.
    double GetTorqueReactionOn2() const { return (-1.0 * torque_react); }

    /// Get the reaction torque considered as applied to the body
    /// (the truss of the gearbox), expressed in local body coordinates.
    ChVector<> GetTorqueReactionOnBody() const {
        return (this->shaft_dir1 * this->t0 - this->shaft_dir2) * torque_react;
    }

    //
    // UPDATE FUNCTIONS
    //

    /// Update all auxiliary data of the gear transmission at given time
    virtual void Update(double mytime, bool update_assets = true);

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
