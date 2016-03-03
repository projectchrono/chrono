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

#ifndef CHSHAFTSGEAR_H
#define CHSHAFTSGEAR_H

//////////////////////////////////////////////////
//
//   ChShaftsGear.h
//
//   Class for defining a transmission ratio between
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

#include "physics/ChShaftsCouple.h"
#include "lcp/ChLcpConstraintTwoGeneric.h"

namespace chrono {

///  Class for defining a 'transmission ratio' (a 1D gear)
///  between two one-degree-of-freedom parts, that is,
///  shafts that can be used to build 1D models
///  of power trains. This is more efficient than
///  simulating power trains modeled with full 3D ChBody
///  objects.
///  Note that this really simple constraint does not
///  provide a way to trasmit a reaction force to the truss,
///  if this is needed, just use the ChShaftsPlanetary with
///  a fixed carrier shaft, or the ChShaftGearbox.

class ChApi ChShaftsGear : public ChShaftsCouple {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChShaftsGear, ChShaftsCouple);

  private:
    //
    // DATA
    //

    double ratio;  // transmission ratio t, as in w2=t*w1, or t=w2/w1

    double torque_react;

    // used as an interface to the LCP solver.
    ChLcpConstraintTwoGeneric constraint;

    float cache_li_speed;  // used to cache the last computed value of multiplier (solver warm starting)
    float cache_li_pos;    // used to cache the last computed value of multiplier (solver warm starting)

  public:
    //
    // CONSTRUCTORS
    //

    /// Build a shaft.
    ChShaftsGear();
    /// Destructor
    ~ChShaftsGear();

    /// Copy from another ChShaftsGear.
    void Copy(ChShaftsGear* source);

    //
    // FLAGS
    //

    //
    // FUNCTIONS
    //

    /// Number of scalar constraints
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

    // Override/implement LCP system functions of ChShaftsCouple
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
    /// two shafts to join.
    /// Each shaft must belong to the same ChSystem.
    virtual bool Initialize(std::shared_ptr<ChShaft> mshaft1,  ///< first  shaft to join
                            std::shared_ptr<ChShaft> mshaft2   ///< second shaft to join
                            );

    /// Set the transmission ratio t, as in w2=t*w1, or t=w2/w1 , or  t*w1 - w2 = 0.
    /// For example, t=1 for a rigid joint; t=-0.5 for representing
    /// a couple of spur gears with teeths z1=20 & z2=40; t=0.1 for
    /// a gear with inner teeths (or epicycloidal reducer), etc.
    void SetTransmissionRatio(double mt) { this->ratio = mt; }
    /// Get the transmission ratio t, as in w2=t*w1, or t=w2/w1
    double GetTransmissionRatio() const { return this->ratio; }

    /// Get the reaction torque exchanged between the two shafts,
    /// considered as applied to the 1st axis.
    double GetTorqueReactionOn1() const { return (this->ratio * this->torque_react); }

    /// Get the reaction torque exchanged between the two shafts,
    /// considered as applied to the 2nd axis.
    double GetTorqueReactionOn2() const { return -(this->torque_react); }

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
