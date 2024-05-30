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
    int Initialize(
        std::shared_ptr<ChBodyFrame> body1,  ///< first frame to link
        std::shared_ptr<ChBodyFrame> body2,  ///< second frame to link
        bool pos_are_relative,               ///< true: following pos. are relative to bodies
        ChVector3d pos1,                     ///< pos. of distance endpoint, for 1st body (rel. or abs., see flag above)
        ChVector3d pos2,                     ///< pos. of distance endpoint, for 2nd body (rel. or abs., see flag above)
        bool auto_distance = true,  ///< if true, initializes the imposed distance as the distance between pos1 and pos2
        double mdistance = 0,       ///< imposed distance (no need to define, if auto_distance=true.)
        Mode mode = Mode::BILATERAL  ///< set distance constraining mode
    );

    /// Get the number of (bilateral) constraints introduced by this link.
    virtual unsigned int GetNumConstraintsBilateral() override { return 1; }

    /// Get the link frame1, relative to body1.
    /// Link frame1 is located on body1 at pos1, with the X axis pointing towards pos2.
    virtual ChFramed GetFrame1Rel() const override;

    /// Get the link frame2, relative to body2.
    /// Link frame2 is located on body2 at pos2, with the X axis pointing towards pos1.
    virtual ChFramed GetFrame2Rel() const override;

    /// Get the 1st anchor endpoint for the distance (expressed in body 1 coordinate system)
    ChVector3d GetEndPoint1Rel() const { return m_pos1; }
    /// Set the 1st anchor endpoint for the distance (expressed in body 1 coordinate system)
    void SetEndPoint1Rel(const ChVector3d& mset) { m_pos1 = mset; }
    /// Get the 1st anchor endpoint for the distance (expressed in absolute coordinate system)
    ChVector3d GetEndPoint1Abs() const { return m_body1->TransformPointLocalToParent(m_pos1); }
    /// Set the 1st anchor endpoint for the distance (expressed in absolute coordinate system)
    void SetEndPoint1Abs(const ChVector3d& mset) { m_pos1 = m_body1->TransformPointParentToLocal(mset); }

    /// Get the 2nd anchor endpoint for the distance (expressed in body 2 coordinate system)
    ChVector3d GetEndPoint2Rel() const { return m_pos2; }
    /// Set the 2nd anchor endpoint for the distance (expressed in body 2 coordinate system)
    void SetEndPoint2Rel(const ChVector3d& mset) { m_pos2 = mset; }
    /// Get the 1st anchor endpoint for the distance (expressed in absolute coordinate system)
    ChVector3d GetEndPoint2Abs() const { return m_body2->TransformPointLocalToParent(m_pos2); }
    /// Set the 1st anchor endpoint for the distance (expressed in absolute coordinate system)
    void SetEndPoint2Abs(const ChVector3d& mset) { m_pos2 = m_body2->TransformPointParentToLocal(mset); }

    /// Set the imposed distance.
    void SetImposedDistance(const double mset) { distance = mset; }

    /// Get the imposed distance.
    double GetImposedDistance() const { return distance; };

    /// Get the distance currently existing between the two endpoints.
    double GetCurrentDistance() const { return curr_dist; }

    /// Set link mode.
    /// The joint can act as bilateral or unilateral; in this latter case, it can either limit the maximum or minimum
    /// distance; if in unilateral mode, a VI solver is required!
    void SetMode(Mode mode);

    /// Get link mode.
    Mode GetMode() const { return this->mode; }

    /// Get the constraint violation.
    virtual ChVectorDynamic<> GetConstraintViolation() const override { return C; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    Mode mode;                 ///< current mode
    double mode_sign;          ///< current mode
    double distance;           ///< imposed distance
    double curr_dist;          ///< current distance
    ChVector3d m_pos1;         ///< first endpoint, in body rel. coords
    ChVector3d m_pos2;         ///< second endpoint, in body rel. coords
    ChConstraintTwoBodies Cx;  ///< the constraint object
    ChVectorN<double, 1> C;    ///< constraint violation

    // Solver and integrator interface functions

    virtual void Update(double mtime, bool update_assets = true) override;

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

    virtual void InjectConstraints(ChSystemDescriptor& descriptor) override;
    virtual void ConstraintsBiReset() override;
    virtual void ConstraintsBiLoad_C(double factor = 1, double recovery_clamp = 0.1, bool do_clamp = false) override;
    virtual void LoadConstraintJacobians() override;
    virtual void ConstraintsFetch_react(double factor = 1) override;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

CH_CLASS_VERSION(ChLinkDistance, 0)

}  // end namespace chrono

#endif
