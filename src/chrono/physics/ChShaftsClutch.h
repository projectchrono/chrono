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

#ifndef CHSHAFTSCLUTCH_H
#define CHSHAFTSCLUTCH_H

//////////////////////////////////////////////////
//
//   ChShaftsClutch.h
//
//   Class for defining a clutch (or brake) between
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
#include "lcp/ChLcpConstraintTwoGenericBoxed.h"

namespace chrono {

///  Class for defining a clutch or a brake (1D model)
///  between two one-degree-of-freedom parts, that is,
///  shafts that can be used to build 1D models
///  of power trains. This is more efficient than
///  simulating power trains modeled with full 3D ChBody
///  objects.

class ChApi ChShaftsClutch : public ChShaftsCouple {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChShaftsClutch, ChShaftsCouple);

  private:
    //
    // DATA
    //

    double maxT;        // clutch max transmissible torque (for forward direction
    double minT;        // clutch min transmissible torque (for backward direction)
    double modulation;  // 0...1  (default 1).

    double torque_react;

    // used as an interface to the LCP solver.
    ChLcpConstraintTwoGenericBoxed constraint;

    float cache_li_speed;  // used to cache the last computed value of multiplier (solver warm starting)
    float cache_li_pos;    // used to cache the last computed value of multiplier (solver warm starting)

  public:
    //
    // CONSTRUCTORS
    //

    /// Constructor.
    ChShaftsClutch();
    /// Destructor
    ~ChShaftsClutch();

    /// Copy from another ChShaftsClutch.
    void Copy(ChShaftsClutch* source);

    //
    // FLAGS
    //

    //
    // FUNCTIONS
    //

    /// Number of scalar constraints, for statistical reasons
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
    virtual void IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c);
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
    virtual void ConstraintsFbLoadForces(double factor = 1.);
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

    /// Set the transmissible torque limit (the maximum torque that
    /// the clutch can transmit between the two shafts).
    /// You can specify two values for backward/forward directions: usually
    /// these are equal (ex. -100,100) in most commercial clutches, but
    /// if you define (0,100), for instance, you can create a so called
    /// freewheel or overrunning clutch that works only in one direction.
    void SetTorqueLimit(double ml, double mu);
    /// Set the transmissible torque limit (the maximum torque that
    /// the clutch can transmit between the two shafts), for both
    /// forward and backward direction.
    void SetTorqueLimit(double ml) { SetTorqueLimit(-fabs(ml), fabs(ml)); }

    /// Get the torque limit for forward rotation
    double GetTorqueLimitF() const { return this->maxT; }
    /// Get the torque limit for backward rotation
    double GetTorqueLimitB() const { return this->minT; }
    /// Get the torque limit (when this is a clutch with symmetric forw/backw limits)
    double GetTorqueLimit() const { return this->maxT; }

    /// Set the user modulation of the torque (or brake, if you use it between
    /// a fixed shaft and a free shaft). The modulation must range from
    /// 0 (switched off) to 1 (max torque). Default is 1, when clutch is created.
    /// You can update this during integration loop to simulate the pedal pushing by the driver.
    void SetModulation(double mm) { this->modulation = ChMax(ChMin(mm, 1.0), 0.0); }
    /// Get the the user modulation.
    double GetModulation() const { return this->modulation; }

    /// Get the actual angle slippage of the clutch, in terms of phase of shaft 1 respect to 2.
    double GetSlippage() const { return GetRelativeRotation(); }
    /// Get the actual slippage speed of the clutch, in terms of speed of shaft 1 respect to 2.
    double GetSlippage_dt() const { return GetRelativeRotation_dt(); }
    /// Get the actual slippage acceleration of the clutch, in terms of accel. of shaft 1 respect to 2.
    double GetSlippage_dtdt() const { return GetRelativeRotation_dtdt(); }

    /// Get the reaction torque exchanged between the two shafts,
    /// considered as applied to the 1st axis.
    virtual double GetTorqueReactionOn1() const { return (this->torque_react); }

    /// Get the reaction torque exchanged between the two shafts,
    /// considered as applied to the 2nd axis.
    virtual double GetTorqueReactionOn2() const { return -(this->torque_react); }

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
