// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHLINKDISTANCE_H
#define CHLINKDISTANCE_H

#include "chrono/physics/ChLink.h"
#include "chrono/solver/ChConstraintTwoBodies.h"

namespace chrono {

/// Fixed distance constraint between two points on two ChBodyFrame objects.
/// The two points which are used to define the end points of the distance are assumed not to move respect to the two
/// owner ChBody, and the amount of the distance is assumed not to change during the simulation.
class ChApi ChLinkDistance : public ChLink {
  public:
    enum class Mode {
        BILATERAL,               ///< bilateral joint
        UNILATERAL_MAXDISTANCE,  ///< unilateral, imposed max distance, current <= imposed
        UNILATERAL_MINDISTANCE   ///< unilateral, imposed min distance, current >= imposed
    };

    ChLinkDistance();
    ChLinkDistance(const ChLinkDistance& other);
    ~ChLinkDistance() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkDistance* Clone() const override { return new ChLinkDistance(*this); }

    /// Initialize this constraint, given the two bodies to be connected, the positions of the two anchor endpoints of
    /// the distance (each expressed in body or abs. coordinates) and the imposed distance.
    int Initialize(std::shared_ptr<ChBodyFrame> mbody1,  ///< first frame to link
                   std::shared_ptr<ChBodyFrame> mbody2,  ///< second frame to link
                   bool pos_are_relative,                ///< true: following pos. are relative to bodies
                   ChVector<> mpos1,  ///< pos. of distance endpoint, for 1st body (rel. or abs., see flag above)
                   ChVector<> mpos2,  ///< pos. of distance endpoint, for 2nd body (rel. or abs., see flag above)
                   bool auto_distance =
                       true,  ///< if true, initializes the imposed distance as the distance between mpos1 and mpos2
                   double mdistance = 0,        ///< imposed distance (no need to define, if auto_distance=true.)
                   Mode mode = Mode::BILATERAL  ///< set distance constraining mode
    );

    /// Get the number of (bilateral) constraints introduced by this link.
    virtual int GetDOC_c() override { return 1; }

    /// Get the link coordinate system, expressed relative to Body2 (the 'master' body). This represents the 'main'
    /// reference of the link: reaction forces are expressed in this coordinate system. (It is the coordinate system of
    /// the contact plane relative to Body2)
    virtual ChCoordsys<> GetLinkRelativeCoords() override;

    /// Get the 1st anchor endpoint for the distance (expressed in Body1 coordinate system)
    ChVector<> GetEndPoint1Rel() const { return pos1; }
    /// Set the 1st anchor endpoint for the distance (expressed in Body1 coordinate system)
    void SetEndPoint1Rel(const ChVector<>& mset) { pos1 = mset; }
    /// Get the 1st anchor endpoint for the distance (expressed in absolute coordinate system)
    ChVector<> GetEndPoint1Abs() const { return ((ChFrame<double>*)Body1)->TransformLocalToParent(pos1); }
    /// Set the 1st anchor endpoint for the distance (expressed in absolute coordinate system)
    void SetEndPoint1Abs(ChVector<>& mset) { pos1 = ((ChFrame<double>*)Body1)->TransformParentToLocal(mset); }

    /// Get the 2nd anchor endpoint for the distance (expressed in Body2 coordinate system)
    ChVector<> GetEndPoint2Rel() const { return pos2; }
    /// Set the 2nd anchor endpoint for the distance (expressed in Body2 coordinate system)
    void SetEndPoint2Rel(const ChVector<>& mset) { pos2 = mset; }
    /// Get the 1st anchor endpoint for the distance (expressed in absolute coordinate system)
    ChVector<> GetEndPoint2Abs() const { return ((ChFrame<double>*)Body2)->TransformLocalToParent(pos2); }
    /// Set the 1st anchor endpoint for the distance (expressed in absolute coordinate system)
    void SetEndPoint2Abs(ChVector<>& mset) { pos2 = ((ChFrame<double>*)Body2)->TransformParentToLocal(mset); }

    /// Set the imposed distance
    void SetImposedDistance(const double mset) { distance = mset; }

    /// Get the imposed distance
    double GetImposedDistance() const { return distance; };

    /// Get the distance currently existing between the two endpoints
    double GetCurrentDistance() const { return curr_dist; }

    /// Set link mode
    /// The joint can act as bilateral or unilateral; in this latter case, it can either limit the maximum or minimum
    /// distance; if in unilateral mode, a VI solver is required!
    void SetMode(Mode mode);

    /// Get link mode
    Mode GetMode() const { return this->mode; }

    /// Get the constraint violation
    virtual ChVectorDynamic<> GetConstraintViolation() const override { return C; }

    /// Override _all_ time, jacobian etc. updating.
    /// In detail, it computes jacobians, violations, etc. and stores
    /// results in inner structures.
    virtual void Update(double mtime, bool update_assets = true) override;

    // STATE FUNCTIONS

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

    // SOLVER INTERFACE

    virtual void InjectConstraints(ChSystemDescriptor& mdescriptor) override;
    virtual void ConstraintsBiReset() override;
    virtual void ConstraintsBiLoad_C(double factor = 1, double recovery_clamp = 0.1, bool do_clamp = false) override;
    virtual void ConstraintsLoadJacobians() override;
    virtual void ConstraintsFetch_react(double factor = 1) override;

    // SERIALIZATION

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

  private:
    Mode mode;                 ///< current mode
    double mode_sign;          ///< current mode
    double distance;           ///< imposed distance
    double curr_dist;          ///< current distance
    ChVector<> pos1;           ///< first endpoint, in body rel. coords
    ChVector<> pos2;           ///< second endpoint, in body rel. coords
    ChConstraintTwoBodies Cx;  ///< the constraint object
    ChVectorN<double, 1> C;    ///< constraint violation
};

CH_CLASS_VERSION(ChLinkDistance, 0)

}  // end namespace chrono

#endif
