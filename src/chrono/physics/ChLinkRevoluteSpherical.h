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
// Authors: Radu Serban
// =============================================================================

#ifndef CHLINKREVOLUTESPHERICAL_H
#define CHLINKREVOLUTESPHERICAL_H

#include "chrono/physics/ChLink.h"
#include "chrono/physics/ChBody.h"
#include "chrono/solver/ChConstraintTwoBodies.h"

namespace chrono {

/// Composite revolute-spherical joint between two two bodies.
/// This joint is defined through a point and direction on the first body (the revolute side), a point on the second
/// body (the spherical side), and a distance.  Kinematically, the two points are maintained at the prescribed distance
/// while the vector between the two points is always perpendicular to the provided direction of the revolute joint.
class ChApi ChLinkRevoluteSpherical : public ChLink {
  public:
    ChLinkRevoluteSpherical();
    ChLinkRevoluteSpherical(const ChLinkRevoluteSpherical& other);
    ~ChLinkRevoluteSpherical() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkRevoluteSpherical* Clone() const override { return new ChLinkRevoluteSpherical(*this); }

    /// Get the number of (bilateral) constraints introduced by this joint.
    virtual unsigned int GetNumConstraintsBilateral() override { return 2; }

    /// Get the point on m_body1 (revolute side), expressed in body 1 coordinate system.
    const ChVector3d& GetPoint1Rel() const { return m_pos1; }

    /// Get the direction of the revolute joint, expressed in body 1 coordinate system.
    const ChVector3d& GetDir1Rel() const { return m_dir1; }

    /// Get the point on m_body2 (spherical side), expressed in body 2 coordinate system.
    const ChVector3d& GetPoint2Rel() const { return m_pos2; }

    /// Get the imposed distance (length of massless connector).
    double GetImposedDistance() const { return m_dist; }

    /// Get the current distance between the two points.
    double GetCurrentDistance() const { return m_cur_dist; }

    /// Get the point on m_body1 (revolute side), expressed in absolute coordinate system.
    ChVector3d GetPoint1Abs() const { return m_body1->TransformPointLocalToParent(m_pos1); }

    /// Get the direction of the revolute joint, expressed in absolute coordinate system.
    ChVector3d GetDir1Abs() const { return m_body1->TransformDirectionLocalToParent(m_dir1); }

    /// Get the point on m_body2 (spherical side), expressed in absolute coordinate system.
    ChVector3d GetPoint2Abs() const { return m_body2->TransformPointLocalToParent(m_pos2); }

    /// Get the link frame 1, relative to body 1.
    /// This frame, defined on body 1 (the revolute side), is centered at the revolute joint location, has its X axis
    /// along the joint connector, and its Z axis aligned with the revolute axis.
    virtual ChFrame<> GetFrame1Rel() const override;

    /// Get the link frame 2, relative to body 2.
    /// This frame, defined on body 2 (the spherical side), is centered at the spherical joint location, has its X axis
    /// along the joint connector, and its Z axis aligned with the revolute axis.
    virtual ChFrame<> GetFrame2Rel() const override;

    /// Get the reaction force and torque on the 1st body, expressed in the link frame 1.
    virtual ChWrenchd GetReaction1() const override;

    /// Get the reaction force and torque on the 2nd body, expressed in the link frame 2.
    virtual ChWrenchd GetReaction2() const override;

    /// Get the joint violation (residuals of the constraint equations)
    virtual ChVectorDynamic<> GetConstraintViolation() const override { return m_C; }

    /// Initialize this joint by specifying the two bodies to be connected, a
    /// coordinate system specified in the absolute frame, and the distance of
    /// the massless connector. The composite joint is constructed such that the
    /// direction of the revolute joint is aligned with the Z axis of the specified
    /// coordinate system and the spherical joint is at the specified distance
    /// along the X axis.
    void Initialize(std::shared_ptr<ChBody> body1,  ///< first frame (revolute side)
                    std::shared_ptr<ChBody> body2,  ///< second frame (spherical side)
                    const ChCoordsys<>& csys,       ///< joint coordinate system (in absolute frame)
                    double distance                 ///< imposed distance
    );

    /// Initialize this joint by specifying the two bodies to be connected, a point
    /// and a direction on body1 defining the revolute joint, and a point on the
    /// second body defining the spherical joint. If local = true, it is assumed
    /// that these quantities are specified in the local body frames. Otherwise,
    /// it is assumed that they are specified in the absolute frame. The imposed
    /// distance between the two points can be either inferred from the provided
    /// configuration (auto_distance = true) or specified explicitly.
    void Initialize(std::shared_ptr<ChBody> body1,  ///< first frame (revolute side)
                    std::shared_ptr<ChBody> body2,  ///< second frame (spherical side)
                    bool local,                     ///< true if data given in body local frames
                    const ChVector3d& pos1,         ///< point on first frame (center of revolute)
                    const ChVector3d& dir1,         ///< direction of revolute on first frame
                    const ChVector3d& pos2,         ///< point on second frame (center of spherical)
                    bool auto_distance = true,      ///< true if imposed distance equal to |pos1 - po2|
                    double distance = 0             ///< imposed distance (used only if auto_distance = false)
    );

    /// Perform the update of this joint at the specified time: compute jacobians,
    /// constraint violations, etc. and cache in internal structures
    virtual void Update(double time, bool update_assets) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    ChVector3d m_pos1;  ///< point on first frame (in local frame)
    ChVector3d m_pos2;  ///< point on second frame (in local frame)
    ChVector3d m_dir1;  ///< direction of revolute on first frame (in local frame)
    double m_dist;      ///< imposed distance between pos1 and pos2

    double m_cur_dist;  ///< actual distance between pos1 and pos2
    double m_cur_dot;   ///< actual value of dot constraint

    ChConstraintTwoBodies m_cnstr_dist;  ///< constraint: ||pos2_abs - pos1_abs|| - dist = 0
    ChConstraintTwoBodies m_cnstr_dot;   ///< constraint: dot(dir1_abs, pos2_abs - pos1_abs) = 0

    ChVectorN<double, 2> m_C;  ////< current constraint violations

    double m_multipliers[2];  ///< Lagrange multipliers

    // Solver and integrator interface functions

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

CH_CLASS_VERSION(ChLinkRevoluteSpherical, 0)

}  // end namespace chrono

#endif
