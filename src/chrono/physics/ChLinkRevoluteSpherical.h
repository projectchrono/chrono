//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHLINKREVOLUTESPHERICAL_H
#define CHLINKREVOLUTESPHERICAL_H

#include "physics/ChLink.h"
#include "lcp/ChLcpConstraintTwoBodies.h"

namespace chrono {

/// Class for modeling a composite revolute-spherical joint between two
/// two ChBodyFrame objects.  This joint is defined through a point and
/// direction on the first body (the revolute side), a point on the second
/// body (the spherical side), and a distance.  Kinematically, the two points
/// are maintained at the prescribed distance while the vector between the
/// two points is always perpendicular to the provided direction of the
/// revolute joint.

class ChApi ChLinkRevoluteSpherical : public ChLink {
    CH_RTTI(ChLinkRevoluteSpherical, ChLink);

  public:
    //
    // CONSTRUCTORS
    //

    ChLinkRevoluteSpherical();
    ~ChLinkRevoluteSpherical();

    virtual void Copy(ChLinkRevoluteSpherical* source);
    virtual ChLink* new_Duplicate();

    //
    // FUNCTIONS
    //

    /// Get the type of this joint.
    virtual int GetType() { return LNK_REVOLUTESPHERICAL; }

    /// Get the number of (bilateral) constraints introduced by this joint.
    virtual int GetDOC_c() { return 2; }

    /// Get the point on Body1 (revolute side), expressed in Body1 coordinate system.
    const ChVector<>& GetPoint1Rel() const { return m_pos1; }
    /// Get the direction of the revolute joint, expressed in Body1 coordinate system.
    const ChVector<>& GetDir1Rel() const { return m_dir1; }
    /// Get the point on Body2 (spherical side), expressed in Body2 coordinate system.
    const ChVector<>& GetPoint2Rel() const { return m_pos2; }

    /// Get the imposed distance (length of massless connector).
    double GetImposedDistance() const { return m_dist; }
    /// Get the current distance between the two points.
    double GetCurrentDistance() const { return m_cur_dist; }

    /// Get the point on Body1 (revolute side), expressed in absolute coordinate system.
    ChVector<> GetPoint1Abs() const { return Body1->TransformPointLocalToParent(m_pos1); }
    /// Get the direction of the revolute joint, expressed in absolute coordinate system.
    ChVector<> GetDir1Abs() const { return Body1->TransformDirectionLocalToParent(m_dir1); }
    /// Get the point on Body2 (spherical side), expressed in absolute coordinate system.
    ChVector<> GetPoint2Abs() const { return Body2->TransformPointLocalToParent(m_pos2); }

    /// Get the link coordinate system, expressed relative to Body2 (spherical side).
    /// This represents the 'main' reference of the link: reaction forces
    /// and reaction torques are reported in this coordinate system.
    virtual ChCoordsys<> GetLinkRelativeCoords();

    /// Get the joint violation (residuals of the constraint equations)
    ChMatrix<>* GetC() { return m_C; }

    /// Initialize this joint by specifying the two bodies to be connected, a
    /// coordinate system specified in the absolute frame, and the distance of
    /// the massless connector.  The composite joint is constructed such that the
    /// direction of the revolute joint is aligned with the z axis of the specified
    /// coordinate system and the spherical joint is at the specified distance
    /// along the x axis.
    void Initialize(ChSharedPtr<ChBodyFrame> body1,  ///< first frame (revolute side)
                    ChSharedPtr<ChBodyFrame> body2,  ///< second frame (spherical side)
                    const ChCoordsys<>& csys,        ///< joint coordinate system (in absolute frame)
                    double distance                  ///< imposed distance
                    );

    /// Initialize this joint by specifying the two bodies to be connected, a point
    /// and a direction on body1 defining the revolute joint, and a point on the
    /// second body defining the spherical joint. If local = true, it is assumed
    /// that these quantities are specified in the local body frames. Otherwise,
    /// it is assumed that they are specified in the absolute frame. The imposed
    /// distance between the two points can be either inferred from the provided
    /// configuration (auto_distance = true) or specified explicitly.
    void Initialize(ChSharedPtr<ChBodyFrame> body1,  ///< first frame (revolute side)
                    ChSharedPtr<ChBodyFrame> body2,  ///< second frame (spherical side)
                    bool local,                      ///< true if data given in body local frames
                    const ChVector<>& pos1,          ///< point on first frame (center of revolute)
                    const ChVector<>& dir1,          ///< direction of revolute on first frame
                    const ChVector<>& pos2,          ///< point on second frame (center of spherical)
                    bool auto_distance = true,       ///< true if imposed distance equal to |pos1 - po2|
                    double distance = 0              ///< imposed distance (used only if auto_distance = false)
                    );

    //
    // UPDATING FUNCTIONS
    //

    /// Perform the update of this joint at the specified time: compute jacobians,
    /// constraint violations, etc. and cache in internal structures
    virtual void Update(double time, bool update_assets = true);

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
    // SOLVER INTERFACE
    //

    virtual void InjectConstraints(ChLcpSystemDescriptor& descriptor);
    virtual void ConstraintsBiReset();
    virtual void ConstraintsBiLoad_C(double factor = 1., double recovery_clamp = 0.1, bool do_clamp = false);
    virtual void ConstraintsLoadJacobians();
    virtual void ConstraintsFetch_react(double factor = 1.);
    virtual void ConstraintsLiLoadSuggestedSpeedSolution();
    virtual void ConstraintsLiLoadSuggestedPositionSolution();
    virtual void ConstraintsLiFetchSuggestedSpeedSolution();
    virtual void ConstraintsLiFetchSuggestedPositionSolution();

    //
    // EXTRA REACTION FORCE & TORQUE FUNCTIONS
    //

    ChVector<> Get_react_force_body1();
    ChVector<> Get_react_torque_body1();
    ChVector<> Get_react_force_body2();
    ChVector<> Get_react_torque_body2();

    //
    // SERIALIZATION
    //

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive);

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive);

  private:
    ChVector<> m_pos1;  // point on first frame (in local frame)
    ChVector<> m_pos2;  // point on second frame (in local frame)
    ChVector<> m_dir1;  // direction of revolute on first frame (in local frame)
    double m_dist;      // imposed distance between pos1 and pos2

    double m_cur_dist;  // actual distance between pos1 and pos2
    double m_cur_dot;   // actual value of dot constraint

    // The constraint objects
    ChLcpConstraintTwoBodies m_cnstr_dist;  // ||pos2_abs - pos1_abs|| - dist = 0
    ChLcpConstraintTwoBodies m_cnstr_dot;   // dot(dir1_abs, pos2_abs - pos1_abs) = 0

    // Current constraint violations
    ChMatrix<>* m_C;

    // Caching of multipliers to allow warm starting
    double m_cache_speed[2];
    double m_cache_pos[2];
};

}  // END_OF_NAMESPACE____

#endif
