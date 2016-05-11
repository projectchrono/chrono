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

#ifndef CHLINKDISTANCE_H
#define CHLINKDISTANCE_H

///////////////////////////////////////////////////
//
//   ChLinkDistance.h
//
//   Class for enforcing a fixed polar distance
//   between two points on two ChBody objects.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "physics/ChLink.h"
#include "lcp/ChLcpConstraintTwoBodies.h"

namespace chrono {
/// Class for enforcing a fixed polar distance
/// between two points on two ChBodyFrame objects.
/// The two points which are used to define the end points
/// of the distance are assumed not to move respect to the
/// two owner ChBody, as well as the amount of the distance
/// is assumed not to change during the simulation. If you
/// need to have a time-varying distance, or distance between
/// two points which move respect to the bodies, please use
/// the more advanced ChLinkLinActuator.

class ChApi ChLinkDistance : public ChLink {
    CH_RTTI(ChLinkDistance, ChLink);

  protected:
    //
    // DATA
    //
    // the imposed distance
    double distance;
    // the distance endpoints, in body rel.coords
    ChVector<> pos1;
    ChVector<> pos2;
    // the constraint object
    ChLcpConstraintTwoBodies Cx;

    double curr_dist;  // used for internal optimizations

  public:
    //
    // CONSTRUCTORS
    //

    ChLinkDistance();

    virtual ~ChLinkDistance();
    virtual void Copy(ChLinkDistance* source);
    virtual ChLink* new_Duplicate();  // always return base link class pointer

    //
    // FUNCTIONS
    //

    virtual int GetType() { return LNK_GEOMETRICDISTANCE; }

    /// Initialize this constraint, given the two bodies to be connected, the
    /// positions of the two anchor endpoints of the distance (each expressed
    /// in body or abs. coordinates) and the imposed distance.
    virtual int Initialize(
        std::shared_ptr<ChBodyFrame> mbody1,  ///< first frame to link
        std::shared_ptr<ChBodyFrame> mbody2,  ///< second frame to link
        bool pos_are_relative,                ///< true: following posit. are considered relative to bodies. false: pos.are absolute
        ChVector<> mpos1,                     ///< position of distance endpoint, for 1st body (rel. or abs., see flag above)
        ChVector<> mpos2,                     ///< position of distance endpoint, for 2nd body (rel. or abs., see flag above)
        bool auto_distance = true,            ///< if true, initializes the imposed distance as the distance between mpos1 and mpos2
        double mdistance = 0                  ///< imposed distance (no need to define, if auto_distance=true.)
        );

    /// Get the number of (bilateral) constraints introduced by this link.
    virtual int GetDOC_c() { return 1; }

    /// Get the link coordinate system, expressed relative to Body2 (the 'master'
    /// body). This represents the 'main' reference of the link: reaction forces
    /// are expressed in this coordinate system.
    /// (It is the coordinate system of the contact plane relative to Body2)
    virtual ChCoordsys<> GetLinkRelativeCoords();

    /// Get the 1st anchor endpoint for the distance (expressed in Body1 coordinate system)
    ChVector<> GetEndPoint1Rel() const { return pos1; }
    /// Set the 1st anchor endpoint for the distance (expressed in Body1 coordinate system)
    void SetEndPoint1Rel(const ChVector<>& mset) { pos1 = mset; }
    /// Get the 1st anchor endpoint for the distance (expressed in absolute coordinate system)
    ChVector<> GetEndPoint1Abs() const { return ((ChFrame<double>*)Body1)->TransformLocalToParent(pos1); }
    /// Set the 1st anchor endpoint for the distance (expressed in absolute coordinate system)
    void SetEndPoint1Abs(ChVector<>& mset) { pos1 = ((ChFrame<double>*)Body1)->TransformParentToLocal(mset); }

    /// Get the 2nd anchor endpoint for the distance (expressed in Body2 coordinate system)
    ChVector<> GetEndPoint2Rel() const { return pos2; };
    /// Set the 2nd anchor endpoint for the distance (expressed in Body2 coordinate system)
    void SetEndPoint2Rel(const ChVector<>& mset) { pos2 = mset; }
    /// Get the 1st anchor endpoint for the distance (expressed in absolute coordinate system)
    ChVector<> GetEndPoint2Abs() const { return ((ChFrame<double>*)Body2)->TransformLocalToParent(pos2); }
    /// Set the 1st anchor endpoint for the distance (expressed in absolute coordinate system)
    void SetEndPoint2Abs(ChVector<>& mset) { pos2 = ((ChFrame<double>*)Body2)->TransformParentToLocal(mset); }

    /// Get the imposed distance
    double GetImposedDistance() const { return distance; };
    /// Set the imposed distance
    void SetImposedDistance(const double mset) { distance = mset; }
    /// Get the distance currently existing between the two endpoints
    double GetCurrentDistance() const {
        return (((ChFrame<double>*)Body1)->TransformLocalToParent(pos1) -
                ((ChFrame<double>*)Body2)->TransformLocalToParent(pos2)).Length();
    };
    /// Get the constraint violation
    double GetC() const { return GetCurrentDistance() - distance; }

    //
    // UPDATING FUNCTIONS
    //

    /// Override _all_ time, jacobian etc. updating.
    /// In detail, it computes jacobians, violations, etc. and stores
    /// results in inner structures.
    virtual void Update(double mtime, bool update_assets = true);

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

    //
    // LCP INTERFACE
    //

    virtual void InjectConstraints(ChLcpSystemDescriptor& mdescriptor);
    virtual void ConstraintsBiReset();
    virtual void ConstraintsBiLoad_C(double factor = 1., double recovery_clamp = 0.1, bool do_clamp = false);
    virtual void ConstraintsLoadJacobians();
    virtual void ConstraintsFetch_react(double factor = 1.);

    //
    // SERIALIZATION
    //

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive);

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive);
};

//////////////////////////////////////////////////////
//////////////////////////////////////////////////////

}  // END_OF_NAMESPACE____

#endif
