//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010, 2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHSHAFTSBODY_H
#define CHSHAFTSBODY_H

//////////////////////////////////////////////////
//
//   ChShaftsBody.h
//
//   Class for creating a constraint between a 3D
//   ChBody object and a 1D ChShaft object. A rotation
//   axis must be specified (to tell along which direction
//   she shaft inertia and rotation affects the body).
//   This constraint is useful, for example, when you have modeled a
//   3D car using ChBody items and a 1D powertrain (gears,
//   differential, etc.) using ChShaft objects: you can connect
//   the former (at least, the wheels) to the latter using
//   this constraint.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "physics/ChBodyFrame.h"
#include "physics/ChShaft.h"
#include "lcp/ChLcpConstraintTwoGeneric.h"

namespace chrono {

// Forward references (for parent hierarchy pointer)

class ChShaft;
class ChBodyFrame;

/// Class for creating a constraint between a 3D
/// ChBody object and a 1D ChShaft object. A rotation
/// axis must be specified (to tell along which direction
/// she shaft inertia and rotation affects the body).
/// This constraint is useful, for example, when you have modeled a
/// 3D car using ChBody items and a 1D powertrain (gears,
/// differential, etc.) using ChShaft objects: you can connect
/// the former (at least, the wheels) to the latter using
/// this constraint.

class ChApi ChShaftsBody : public ChPhysicsItem {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChShaftsBody, ChPhysicsItem);

  private:
    //
    // DATA
    //

    double torque_react;

    // used as an interface to the LCP solver.
    ChLcpConstraintTwoGeneric constraint;

    ChShaft* shaft;
    ChBodyFrame* body;

    ChVector<> shaft_dir;

  public:
    //
    // CONSTRUCTORS
    //

    /// Build a shaft.
    ChShaftsBody();
    /// Destructor
    ~ChShaftsBody();

    /// Copy from another ChShaftsPlanetary.
    void Copy(ChShaftsBody* source);

    //
    // FLAGS
    //

    //
    // FUNCTIONS
    //
    /// Get the number of scalar variables affected by constraints in this link
    virtual int GetNumCoords() { return 6 + 1; }

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
    virtual void IntLoadConstraint_Ct(const unsigned int off, ChVectorDynamic<>& Qc, const double c){};
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
    virtual void ConstraintsLoadJacobians();
    virtual void ConstraintsFetch_react(double factor = 1.);

    // Other functions

    /// Use this function after object creation, to initialize it, given
    /// the 1D shaft and 3D body to join.
    /// Each item must belong to the same ChSystem.
    /// Direction is expressed in the local coordinates of the body.
    bool Initialize(std::shared_ptr<ChShaft> mshaft,     ///< shaft to join
                    std::shared_ptr<ChBodyFrame> mbody,  ///< body to join
                    const ChVector<>& mdir               ///< the direction of the shaft on 3D body (applied on COG: pure torque)
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
