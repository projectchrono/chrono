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

#ifndef CHSHAFTSPLANETARY_H
#define CHSHAFTSPLANETARY_H

//////////////////////////////////////////////////
//
//   ChShaftsPlanetary.h
//
//   Class for defining a transmission ratio between
//   three one-degree-of-freedom parts (that is,
//   shafts that can be used to build 1D models
//   of power trains), as in a planetary gear.
//   This is more efficient than
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

#include "physics/ChPhysicsItem.h"
#include "physics/ChShaft.h"
#include "lcp/ChLcpConstraintThreeGeneric.h"

namespace chrono {

// Forward references (for parent hierarchy pointer)

class ChShaft;

///  Class for defining a planetary gear
///  between three one-degree-of-freedom parts (that is,
///  shafts that can be used to build 1D models
///  of power trains - this is more efficient than
///  simulating power trains modeled full 3D ChBody
///  objects).
///  Planetary gears can be used to make, for instance,
///  the differentials of cars.
///  While traditional gear reducers have one input and one
///  output, the planetary gear have two inputs and one
///  output (or, if you prefer, one input and two outputs).
///  Note that you can use this class also to make a
///  gearbox if you are interested in knowing the reaction
///  torque transmitted to the truss (whereas the basic ChLinkGear
///  cannot do this because it has only in and out); in this case
///  you just use the shaft n.1 as truss and fix it.

class ChApi ChShaftsPlanetary : public ChPhysicsItem {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChShaftsPlanetary, ChPhysicsItem);

  private:
    //
    // DATA
    //

    double r1;  // transmission ratios  as in   r1*w1 + r2*w2 + r3*w3 = 0
    double r2;
    double r3;

    double torque_react;

    // used as an interface to the LCP solver.
    ChLcpConstraintThreeGeneric constraint;

    float cache_li_speed;  // used to cache the last computed value of multiplier (solver warm starting)
    float cache_li_pos;    // used to cache the last computed value of multiplier (solver warm starting)

    ChShaft* shaft1;
    ChShaft* shaft2;
    ChShaft* shaft3;

  public:
    //
    // CONSTRUCTORS
    //

    /// Constructor.
    ChShaftsPlanetary();
    /// Destructor
    ~ChShaftsPlanetary();

    /// Copy from another ChShaftsPlanetary.
    void Copy(ChShaftsPlanetary* source);

    //
    // FLAGS
    //

    //
    // FUNCTIONS
    //

    /// Get the number of scalar variables affected by constraints in this link
    virtual int GetNumCoords() { return 3; }

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

    /// Use this function after planetary gear creation, to initialize it, given
    /// three shafts to join.
    /// Although there's no special requirement, you may think of the three
    /// typical moving parts of an apycycloidal reducer: the carrier, the
    /// input gear, and the gear with inner teeth that usually is kept fixed (but the
    /// ChShaftsPlanetary does not require that one shaft is fixed - it's up to you)
    /// Each shaft must belong to the same ChSystem.
    virtual int Initialize(ChSharedPtr<ChShaft> mshaft1,  ///< first  shaft to join (carrier wheel)
                           ChSharedPtr<ChShaft> mshaft2,  ///< second shaft to join (wheel)
                           ChSharedPtr<ChShaft> mshaft3   ///< third  shaft to join (wheel)
                           );

    /// Get the first shaft (carrier wheel)
    ChShaft* GetShaft1() { return shaft1; }
    /// Get the second shaft
    ChShaft* GetShaft2() { return shaft2; }
    /// Get the third shaft
    ChShaft* GetShaft3() { return shaft3; }

    /// Return the speed of the first shaft (carrier wheel).
    double GetSpeedShaft1() const { return shaft1->GetPos_dt(); }
    /// Return the speed of the second shaft.
    double GetSpeedShaft2() const { return shaft2->GetPos_dt(); }
    /// Return the speed of the third shaft.
    double GetSpeedShaft3() const { return shaft3->GetPos_dt(); }

    /// Set the transmission ratios r1 r2 r3 as in
    ///     r1*w1 + r2*w2 + r3*w3 = 0
    /// For example, for the car differential, if you assume that shaft 1 is
    /// the carrier and shafts 2 and 3 go to the wheel hubs, you must use
    /// r1=-2, r2=1, r3=1 to satisfy the kinematics -2*w1+w2+w3=0 of the differential;
    /// equivalently, you may use r1=1, r2=-0.5, r3=-0.5 (the equation would hold the same).
    void SetTransmissionRatios(double mr1, double mr2, double mr3) {
        r1 = mr1;
        r2 = mr2;
        r3 = mr3;
    }

    /// Setting the transmission ratios r1 r2 r3 for  r1*w1 + r2*w2 + r3*w3 = 0
    /// may be cumbersome, but when you deal with typical planetary devices, this
    /// function provides a shortcut to setting them for you, given a single
    /// parameter t0, that is the speed ratio t'=w3'/w2' of the inverted planetary.
    /// That ratio is simple to get: to invert the planetary, imagine to hold fixed
    /// the carrier of shaft 1 (that is w1' =0), move the shaft 2 and see which is
    /// the speed of shaft 3, to get the ratio t0=w3'/w2'. Generally, shaft 1 is
    /// called the 'carrier'. For example, in normal operation of an epicycloidal
    /// reducer, the carrier (shaft 1) is used as output, shaft 2 is the input, and
    /// shaft 3 is hold fixed to get one degree of freedom only; but in 'inverted' operation
    /// imagine the carrier is fixed, so t0 can be easily got as t0=-z2/z3, with z=n.of teeth.
    /// In a car differential, again with shaft 1 as carrier, one can see that t0=w3'/w2'
    /// so t0=-1.     See the Willis theory for more details on these formulas.
    /// Note that t0 should be different from 1 (singularity).
    /// Once you get t0, simply use this function and it will set r1 r2 r3 automatically.
    void SetTransmissionRatioOrdinary(double t0) {
        r1 = (1. - t0);
        r2 = t0;
        r3 = -1.0;
    }

    /// Get the t0 transmission ratio of the equivalent ordinary
    /// gearbox, ie. the inverted planetary, that is the ratio  t0=w3'/w2' assuming
    /// that the carrier (shaft 1) is hold fixed.
    double GetTransmissionRatioOrdinary() const { return -r2 / r3; }

    /// Get the transmission ratio r1, as in  r1*w1+r2*w2+r3*w3 = 0
    double GetTransmissionR1() const { return this->r1; }
    /// Get the transmission ratio r1, as in  r1*w1+r2*w2+r3*w3 = 0
    double GetTransmissionR2() const { return this->r2; }
    /// Get the transmission ratio r1, as in  r1*w1+r2*w2+r3*w3 = 0
    double GetTransmissionR3() const { return this->r3; }

    /// Get the reaction torque considered as applied to the 1st axis.
    double GetTorqueReactionOn1() const { return (this->r1 * torque_react); }

    /// Get the reaction torque considered as applied to the 2nd axis.
    double GetTorqueReactionOn2() const { return (this->r2 * torque_react); }

    /// Get the reaction torque considered as applied to the 3rd axis.
    double GetTorqueReactionOn3() const { return (this->r3 * torque_react); }

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

typedef ChSharedPtr<ChShaftsPlanetary> ChSharedPlanetaryPtr;

}  // END_OF_NAMESPACE____

#endif
