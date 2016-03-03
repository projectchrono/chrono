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

#ifndef CH_LINK_REVOLUTE_TRANSLATIONAL_H
#define CH_LINK_REVOLUTE_TRANSLATIONAL_H

#include "physics/ChLink.h"
#include "lcp/ChLcpConstraintTwoBodies.h"

namespace chrono {

/// Class for modeling a composite revolute-translational joint between two
/// ChBodyFrame objects.  This joint is defined through a point and
/// direction on the first body (the revolute side), a point and two mutually
/// orthogonal directions on the second body (the translational side),
/// and a distance.
class ChApi ChLinkRevoluteTranslational : public ChLink {
    CH_RTTI(ChLinkRevoluteTranslational, ChLink);

  public:
    //
    // CONSTRUCTORS
    //

    ChLinkRevoluteTranslational();
    ~ChLinkRevoluteTranslational();

    virtual void Copy(ChLinkRevoluteTranslational* source);
    virtual ChLink* new_Duplicate();

    //
    // FUNCTIONS
    //

    /// Get the type of this joint.
    virtual int GetType() { return LNK_REVOLUTETRANSLATIONAL; }

    /// Get the number of (bilateral) constraints introduced by this joint.
    virtual int GetDOC_c() { return 4; }

    /// Get the point on Body1 (revolute side), expressed in Body1 coordinate system.
    const ChVector<>& GetPoint1Rel() const { return m_p1; }
    /// Get the direction of the revolute joint, expressed in Body1 coordinate system.
    const ChVector<>& GetDirZ1Rel() const { return m_z1; }
    /// Get the point on Body2 (spherical side), expressed in Body2 coordinate system.
    const ChVector<>& GetPoint2Rel() const { return m_p2; }
    /// Get the first direction of the translational joint, expressed in Body2 coordinate system.
    /// The translational axis is orthogonal to the direction.
    const ChVector<>& GetDirX2Rel() const { return m_x2; }
    /// Get the second direction of the translational joint, expressed in Body2 coordinate system.
    /// The translational axis is orthogonal to the direction.
    const ChVector<>& GetDirY2Rel() const { return m_y2; }

    /// Get the imposed distance (length of massless connector).
    double GetImposedDistance() const { return m_dist; }
    /// Get the current distance between the two points.
    double GetCurrentDistance() const { return m_cur_dist; }

    /// Get the point on Body1 (revolute side), expressed in absolute coordinate system.
    ChVector<> GetPoint1Abs() const { return Body1->TransformPointLocalToParent(m_p1); }
    /// Get the direction of the revolute joint, expressed in absolute coordinate system.
    ChVector<> GetDirZ1Abs() const { return Body1->TransformDirectionLocalToParent(m_z1); }
    /// Get the point on Body2 (translational side), expressed in absolute coordinate system.
    ChVector<> GetPoint2Abs() const { return Body2->TransformPointLocalToParent(m_p2); }
    /// Get the first direction of the translational joint, expressed in absolute coordinate system.
    /// The translational axis is orthogonal to the direction.
    ChVector<> GetDirX2Abs() const { return Body2->TransformDirectionLocalToParent(m_x2); }
    /// Get the second direction of the translational joint, expressed in absolute coordinate system.
    /// The translational axis is orthogonal to the direction.
    ChVector<> GetDirY2Abs() const { return Body2->TransformDirectionLocalToParent(m_y2); }

    /// Get the link coordinate system, expressed relative to Body2 (translational side).
    /// This represents the 'main' reference of the link: reaction forces
    /// and reaction torques are reported in this coordinate system.
    virtual ChCoordsys<> GetLinkRelativeCoords();

    /// Get the joint violation (residuals of the constraint equations)
    ChMatrix<>* GetC() { return m_C; }

    /// Initialize this joint by specifying the two bodies to be connected, a
    /// coordinate system specified in the absolute frame, and the distance of
    /// the massless connector.  The composite joint is constructed such that the
    /// revolute joint is centered at the origin of the specified coordinate system.
    /// The revolute joint rotates about the z axis, the translational joint moves
    /// along the y axis, and the translation axis is at the specified distance
    /// along the x axis.
    void Initialize(std::shared_ptr<ChBodyFrame> body1,  ///< first frame (revolute side)
                    std::shared_ptr<ChBodyFrame> body2,  ///< second frame (translational side)
                    const ChCoordsys<>& csys,        ///< joint coordinate system (in absolute frame)
                    double distance                  ///< imposed distance
                    );

    /// Initialize this joint by specifying the two bodies to be connected, a point
    /// and a direction on body1 defining the revolute joint, and two directions and
    /// a point on the second body defining the translational joint. If local = true,
    /// it is assumed that these quantities are specified in the local body frames.
    /// Otherwise, it is assumed that they are specified in the absolute frame. The
    /// imposed distance between the two points can be either inferred from the provided
    /// configuration (auto_distance = true) or specified explicitly.
    void Initialize(std::shared_ptr<ChBodyFrame> body1,  ///< first frame (revolute side)
                    std::shared_ptr<ChBodyFrame> body2,  ///< second frame (spherical side)
                    bool local,                      ///< true if data given in body local frames
                    const ChVector<>& p1,            ///< point on first frame (revolute side)
                    const ChVector<>& dirZ1,         ///< direction of revolute on first frame
                    const ChVector<>& p2,            ///< point on second frame (translational side)
                    const ChVector<>& dirX2,         ///< first direction of translational joint
                    const ChVector<>& dirY2,         ///< second direction of translational joint
                    bool auto_distance = true,       ///< true if imposed distance equal to distance between axes
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
    ChVector<> m_p1;  // point on first frame (in local frame)
    ChVector<> m_p2;  // point on second frame (in local frame)
    ChVector<> m_z1;  // direction of revolute on first frame (in local frame)
    ChVector<> m_x2;  // first direction of translational on second frame (in local frame)
    ChVector<> m_y2;  // first direction of translational on second frame (in local frame)
    double m_dist;    // imposed distance between rotational and translation axes

    double m_cur_par1;  // actual value of par1 constraint
    double m_cur_par2;  // actual value of par2 constraint
    double m_cur_dot;   // actual value of dot constraint
    double m_cur_dist;  // actual distance between pos1 and pos2

    // The constraint objects
    ChLcpConstraintTwoBodies m_cnstr_par1;  // z1 perpendicualr to x2
    ChLcpConstraintTwoBodies m_cnstr_par2;  // z1 perpendicular to y2
    ChLcpConstraintTwoBodies m_cnstr_dot;   // d12 perpendicular to z1
    ChLcpConstraintTwoBodies m_cnstr_dist;  // distance between axes

    // Current constraint violations
    ChMatrix<>* m_C;

    // Lagrange multipliers
    // Note that their order corresponds to the following order of the constraints:
    // par1, par2, dot, dist.
    double m_multipliers[4];
};

}  // end namespace chrono

#endif
