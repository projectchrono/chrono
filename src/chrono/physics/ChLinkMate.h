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

#ifndef CHLINKMATE_H
#define CHLINKMATE_H

#include "chrono/physics/ChLink.h"
#include "chrono/physics/ChLinkMask.h"
#include "chrono/solver/ChKRMBlock.h"

namespace chrono {

/// Base class for constraints between two frames attached to two bodies.
/// Contrary to links of the ChLinkLock type, these links do not allow the constrained frame to move with respect to the
/// constrained body. Since this assumpion is often verified, these constraints might be a better choice for the most
/// common cases.
class ChApi ChLinkMate : public ChLink {
  public:
    ChLinkMate() {}
    ChLinkMate(const ChLinkMate& other) : ChLink(other) {}
    virtual ~ChLinkMate() {}

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;
};

CH_CLASS_VERSION(ChLinkMate, 0)

// -----------------------------------------------------------------------------

/// Generic mate constraint.
/// This class allows selecting the DOFs to be constrained.
class ChApi ChLinkMateGeneric : public ChLinkMate {
  public:
    using ChConstraintVectorX = Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::ColMajor, 6, 1>;

    ChLinkMateGeneric(bool mc_x = true,
                      bool mc_y = true,
                      bool mc_z = true,
                      bool mc_rx = true,
                      bool mc_ry = true,
                      bool mc_rz = true);

    ChLinkMateGeneric(const ChLinkMateGeneric& other);

    virtual ~ChLinkMateGeneric() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkMateGeneric* Clone() const override { return new ChLinkMateGeneric(*this); }

    /// Get the reference frame (expressed in and relative to the absolute frame) of the visual model.
    /// For a ChLinkMate, this returns the absolute coordinate system of the second body.
    virtual ChFrame<> GetVisualModelFrame(unsigned int nclone = 0) const override { return frame2 >> *GetBody2(); }

    /// Get the link frame 1, relative to body 1.
    virtual ChFramed GetFrame1Rel() const override { return frame1; }

    /// Get the link frame 2, relative to body 2.
    virtual ChFramed GetFrame2Rel() const override { return frame2; }

    bool IsConstrainedX() const { return c_x; }
    bool IsConstrainedY() const { return c_y; }
    bool IsConstrainedZ() const { return c_z; }
    bool IsConstrainedRx() const { return c_rx; }
    bool IsConstrainedRy() const { return c_ry; }
    bool IsConstrainedRz() const { return c_rz; }

    /// Sets which movements (of frame 1 respect to frame 2) are constrained.
    void SetConstrainedCoords(bool mc_x, bool mc_y, bool mc_z, bool mc_rx, bool mc_ry, bool mc_rz);

    /// Initialize the link given the two bodies to be connected and the absolute position of the link.
    /// Two frames, moving together with each of the two bodies, will be automatically created.
    /// This method guarantees that the constraint is satisfied, given that the bodies will not be moved before the
    /// simulation starts.
    virtual void Initialize(std::shared_ptr<ChBodyFrame> body1,  ///< first body to link
                            std::shared_ptr<ChBodyFrame> body2,  ///< second body to link
                            ChFrame<> absframe                   ///< link frame, in abs. coordinate
    );

    /// Initialize the link given the two bodies to be connected and two frames (either referring to absolute or body
    /// coordinates) in which the link must be placed. It is recommended to place the bodies so that the constraints are
    /// satisfied. Starting the simulation with constraints violation might lead to unstable results.
    virtual void Initialize(std::shared_ptr<ChBodyFrame> body1,  ///< first body to link
                            std::shared_ptr<ChBodyFrame> body2,  ///< second body to link
                            bool pos_are_relative,               ///< true: following pos. are relative to bodies
                            ChFrame<> frame1,                    ///< slave frame 1 (rel. or abs.)
                            ChFrame<> frame2                     ///< master frame 2 (rel. or abs.)
    );

    /// Initialization based on passing two vectors (point + dir) on the two bodies, which will represent the Z axes of
    /// the two frames (X and Y will be built from the Z vector via Gram-Schmidt orthonormalization).
    /// It is recommended to place the bodies so that the constraints are satisfied.
    /// Starting the simulation with constraints violation might lead to unstable results.
    virtual void Initialize(std::shared_ptr<ChBodyFrame> body1,  ///< first body to link
                            std::shared_ptr<ChBodyFrame> body2,  ///< second body to link
                            bool pos_are_relative,               ///< true: following pos. are relative to bodies
                            const ChVector3d& point1,            ///< origin of slave frame 1 (rel. or abs.)
                            const ChVector3d& point2,            ///< origin of master frame 2 (rel. or abs.)
                            const ChVector3d& dir1,              ///< X axis of slave plane 1 (rel. or abs.)
                            const ChVector3d& dir2               ///< X axis of master plane 2 (rel. or abs.)
    );

    /// Enable/disable all the constraint of the link as desired.
    virtual void SetDisabled(bool mdis) override;

    /// Set this link as 'broken'.
    virtual void SetBroken(bool mon) override;

    /// Enable/disable calculation of the tangent stiffness matrix (Kc) of this constraint (default: false).
    void SetUseTangentStiffness(bool useKc);

    virtual unsigned int GetNumConstraints() override { return m_num_constr; }
    virtual unsigned int GetNumConstraintsBilateral() override { return m_num_constr_bil; }
    virtual unsigned int GetNumConstraintsUnilateral() override { return m_num_constr_uni; }

    /// Return link mask.
    ChLinkMask& GetLinkMask() { return mask; }

    // LINK VIOLATIONS
    // Get the constraint violations, i.e. the residual of the constraint equations and their time derivatives (TODO)

    /// Link violation (residuals of the link constraint equations).
    virtual ChVectorDynamic<> GetConstraintViolation() const override { return C; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  protected:
    void SetupLinkMask();
    void ChangedLinkMask();

    ChFrame<> frame1;
    ChFrame<> frame2;

    bool c_x;
    bool c_y;
    bool c_z;
    bool c_rx;
    bool c_ry;
    bool c_rz;

    int m_num_constr;      ///< number of constraints
    int m_num_constr_bil;  ///< number of bilateral constraints
    int m_num_constr_uni;  ///< number of unilateral constraints

    ChLinkMask mask;

    ChConstraintVectorX C;  ///< residuals

    ChMatrix33<> P;  ///< projection matrix from Lagrange multiplier to reaction torque

    ChVector3d gamma_f;  ///< translational Lagrange multipliers
    ChVector3d gamma_m;  ///< rotational Lagrange multipliers

    std::unique_ptr<ChKRMBlock> Kmatr = nullptr;  ///< the tangent stiffness matrix of constraint

    /// Update link state, constraint Jacobian, and frames.
    /// This is called automatically by the solver at each time step.
    /// Derived classes must call this parent method and then take care of updating their own assets.
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
    virtual void IntLoadConstraint_Ct(const unsigned int off, ChVectorDynamic<>& Qc, const double c) override;
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
    virtual void ConstraintsBiLoad_Ct(double factor = 1) override;
    virtual void LoadConstraintJacobians() override;
    virtual void ConstraintsFetch_react(double factor = 1) override;

    /// Register with the given system descriptor any ChKRMBlock objects associated with this item.
    virtual void InjectKRMMatrices(ChSystemDescriptor& descriptor) override;

    /// Add the current stiffness K matrix in encapsulated ChKRMBlock item(s), if any.
    /// The K matrix is loaded with scaling value Kfactor.
    virtual void LoadKRMMatrices(double Kfactor, double Rfactor, double Mfactor) override;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

CH_CLASS_VERSION(ChLinkMateGeneric, 0)

// -----------------------------------------------------------------------------

/// Mate constraint of plane-to-plane type.
/// The planes are defined by the X and Y axes of the two frames i.e. the two Z axes are parallel.
/// An offset distance can be provided.
class ChApi ChLinkMatePlanar : public ChLinkMateGeneric {
  public:
    ChLinkMatePlanar() : ChLinkMateGeneric(false, false, true, true, true, false), m_flipped(false), m_distance(0) {}
    ChLinkMatePlanar(const ChLinkMatePlanar& other);
    ~ChLinkMatePlanar() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkMatePlanar* Clone() const override { return new ChLinkMatePlanar(*this); }

    /// Tell if the two normals must be opposed (flipped=true) or must have the same direction (flipped=false).
    void SetFlipped(bool doflip);

    /// Tell if the two normals are opposed (flipped=true) or have the same direction (flipped=false).
    bool IsFlipped() const { return m_flipped; }

    /// Set the distance between the two planes, in normal direction.
    void SetDistance(double distance) { m_distance = distance; }

    /// Get the requested distance between the two planes, in normal direction.
    double GetDistance() const { return m_distance; }

    /// Initialize the link by providing a point and a normal direction on each plane, each expressed in body or abs
    /// reference. Normals can be either aligned or opposed depending on the SetFlipped() method.
    virtual void Initialize(std::shared_ptr<ChBodyFrame> body1,  ///< first body to link
                            std::shared_ptr<ChBodyFrame> body2,  ///< second body to link
                            bool pos_are_relative,               ///< true: following pos. are relative to bodies
                            const ChVector3d& point1,            ///< point on slave plane 1 (rel. or abs.)
                            const ChVector3d& point2,            ///< point on master plane 2 (rel. or abs.)
                            const ChVector3d& norm1,             ///< normal of slave plane 1 (rel. or abs.)
                            const ChVector3d& norm2              ///< normal of master plane 2 (rel. or abs.)
                            ) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  protected:
    bool m_flipped;
    double m_distance;

    /// Update link state. This is called automatically by the solver at each time step.
    /// Update constraint jacobian and frames.
    virtual void Update(double time, bool update_assets = true) override;
};

CH_CLASS_VERSION(ChLinkMatePlanar, 0)

// -----------------------------------------------------------------------------

/// Mate constraint of coaxial type.
/// This link corresponds to the typical cylinder-vs-cylinder mating used in 3D CAD assemblies.
/// The two coaxial axes are the Z axes of the two frames.
class ChApi ChLinkMateCylindrical : public ChLinkMateGeneric {
  public:
    ChLinkMateCylindrical() : ChLinkMateGeneric(true, true, false, true, true, false), m_flipped(false) {}
    ChLinkMateCylindrical(const ChLinkMateCylindrical& other);
    virtual ~ChLinkMateCylindrical() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkMateCylindrical* Clone() const override { return new ChLinkMateCylindrical(*this); }

    /// Tell if the two axes must be opposed (flipped=true) or must have the same verse (flipped=false)
    void SetFlipped(bool doflip);
    bool IsFlipped() const { return m_flipped; }

    using ChLinkMateGeneric::Initialize;

    /// Specialized initialization for coaxial mate, given the two bodies to be connected, two points, two directions
    /// (each expressed in body or abs. coordinates).
    virtual void Initialize(std::shared_ptr<ChBodyFrame> body1,  ///< first body to link
                            std::shared_ptr<ChBodyFrame> body2,  ///< second body to link
                            bool pos_are_relative,               ///< true: following pos. are relative to bodies
                            const ChVector3d& point1,            ///< point on slave axis 1 (rel. or abs.)
                            const ChVector3d& point2,            ///< point on master axis 2 (rel. or abs.)
                            const ChVector3d& dir1,              ///< direction of slave axis 1 (rel. or abs.)
                            const ChVector3d& dir2               ///< direction of master axis 2 (rel. or abs.)
                            ) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  protected:
    bool m_flipped;
};

CH_CLASS_VERSION(ChLinkMateCylindrical, 0)

// -----------------------------------------------------------------------------

/// Mate constraint of revolute type.
/// The two revolute axes are the Z axes of the two frames.
class ChApi ChLinkMateRevolute : public ChLinkMateGeneric {
  public:
    ChLinkMateRevolute() : ChLinkMateGeneric(true, true, true, true, true, false), m_flipped(false) {}
    ChLinkMateRevolute(const ChLinkMateRevolute& other);
    virtual ~ChLinkMateRevolute() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkMateRevolute* Clone() const override { return new ChLinkMateRevolute(*this); }

    using ChLinkMateGeneric::Initialize;

    /// Tell if the two axes must be opposed (flipped=true) or must have the same verse (flipped=false)
    void SetFlipped(bool doflip);
    bool IsFlipped() const { return m_flipped; }

    /// Specialized initialization for revolute mate, given the two bodies to be connected, two points, two directions
    /// (each expressed in body or abs. coordinates). These two directions are the Z axes of slave frame F1 and master
    /// frame F2
    virtual void Initialize(std::shared_ptr<ChBodyFrame> body1,  ///< first body to link
                            std::shared_ptr<ChBodyFrame> body2,  ///< second body to link
                            bool pos_are_relative,               ///< true: following pos. are relative to bodies
                            const ChVector3d& point1,            ///< point on slave axis 1 (rel. or abs.)
                            const ChVector3d& point2,            ///< point on master axis 2 (rel. or abs.)
                            const ChVector3d& dir1,              ///< direction of slave axis 1 (rel. or abs.)
                            const ChVector3d& dir2               ///< direction of master axis 2 (rel. or abs.)
                            ) override;

    /// Get relative angle of slave frame with respect to master frame.
    double GetRelativeAngle();

    /// Get relative angular velocity of slave frame with respect to master frame.
    double GetRelativeAngleDt();

    /// Get relative angular acceleration of slave frame with respect to master frame.
    double GetRelativeAngleDt2();

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  protected:
    bool m_flipped;
};

CH_CLASS_VERSION(ChLinkMateRevolute, 0)

// -----------------------------------------------------------------------------

/// Mate constraint of prismatic type.
/// Allowed relative movements are along the Z axes of the two frames.
class ChApi ChLinkMatePrismatic : public ChLinkMateGeneric {
  public:
    ChLinkMatePrismatic() : ChLinkMateGeneric(true, true, false, true, true, true), m_flipped(false) {}
    ChLinkMatePrismatic(const ChLinkMatePrismatic& other);
    virtual ~ChLinkMatePrismatic() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkMatePrismatic* Clone() const override { return new ChLinkMatePrismatic(*this); }

    using ChLinkMateGeneric::Initialize;

    /// Tell if the two axes must be opposed (flipped=true) or must have the same verse (flipped=false)
    void SetFlipped(bool doflip);
    bool IsFlipped() const { return m_flipped; }

    /// Specialized initialization for prismatic mate, given the two bodies to be connected, two points, two directions.
    /// These two directions are the X axes of secondary frame F1 and principal frame F2.
    /// All quantities can be expressed in body or in absolute coordinates.
    virtual void Initialize(std::shared_ptr<ChBodyFrame> body1,  ///< first body to link
                            std::shared_ptr<ChBodyFrame> body2,  ///< second body to link
                            bool pos_are_relative,               ///< true: following pos. are relative to bodies
                            const ChVector3d& point1,            ///< point on slave axis 1 (rel. or abs.)
                            const ChVector3d& point2,            ///< point on master axis 2 (rel. or abs.)
                            const ChVector3d& dir1,              ///< direction of slave axis 1 (rel. or abs.)
                            const ChVector3d& dir2               ///< direction of master axis 2 (rel. or abs.)
                            ) override;

    /// Get relative position of slave frame with respect to master frame.
    double GetRelativePos();

    /// Get relative velocity of slave frame with respect to master frame.
    double GetRelativePosDt();

    /// Get relative acceleration of slave frame with respect to master frame.
    double GetRelativePosDt2();

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  protected:
    bool m_flipped;
};

CH_CLASS_VERSION(ChLinkMatePrismatic, 0)

// -----------------------------------------------------------------------------

/// Mate constraint of spherical type.
/// This link corresponds to the typical point-on-point or spherical joint mating used in 3D CAD assemblies.
class ChApi ChLinkMateSpherical : public ChLinkMateGeneric {
  public:
    ChLinkMateSpherical() : ChLinkMateGeneric(true, true, true, false, false, false) {}
    ChLinkMateSpherical(const ChLinkMateSpherical& other);
    virtual ~ChLinkMateSpherical() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkMateSpherical* Clone() const override { return new ChLinkMateSpherical(*this); }

    using ChLinkMateGeneric::Initialize;

    /// Specialized initialization for coincident mate, given the two bodies to be connected, and two points
    /// (each expressed in body or abs. coordinates).
    void Initialize(std::shared_ptr<ChBodyFrame> body1,  ///< first body to link
                    std::shared_ptr<ChBodyFrame> body2,  ///< second body to link
                    bool pos_are_relative,               ///< true: following pos. are relative to bodies.
                    ChVector3d point1,                   ///< point slave 1 (rel. or abs.)
                    ChVector3d point2                    ///< point master 2 (rel. or abs.)
    );
};

CH_CLASS_VERSION(ChLinkMateSpherical, 0)

// -----------------------------------------------------------------------------

/// Mate constraining distance of origin of frame 2 respect to Z axis of frame 1.
class ChApi ChLinkMateDistanceZ : public ChLinkMateGeneric {
  public:
    ChLinkMateDistanceZ() : ChLinkMateGeneric(false, false, true, false, false, false), m_distance(0) {}
    ChLinkMateDistanceZ(const ChLinkMateDistanceZ& other);
    virtual ~ChLinkMateDistanceZ() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkMateDistanceZ* Clone() const override { return new ChLinkMateDistanceZ(*this); }

    /// Set the distance of the two constrainted frames along the Z axis of frame 2.
    void SetDistance(double distance) { m_distance = distance; }

    /// Get the imposed distance on Z of frame 2.
    double GetDistance() const { return m_distance; }

    using ChLinkMateGeneric::Initialize;

    /// Initialize the link by providing two points and a direction along which the distance must be considered.
    /// \a dir2 will be the Z axis of both frame 1 and 2.
    void Initialize(std::shared_ptr<ChBodyFrame> body1,  ///< first body to link
                    std::shared_ptr<ChBodyFrame> body2,  ///< second body to link
                    bool pos_are_relative,               ///< true: following pos. are relative to bodies
                    ChVector3d point1,                   ///< point slave 1 (rel. or abs.)
                    ChVector3d point2,                   ///< point master 2 (rel. or abs.)
                    ChVector3d dir2                      ///< direction of master axis 2 (rel. or abs.)
    );

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  protected:
    double m_distance;

    /// Update link state, constraint Jacobian, and frames.
    /// Called automatically by the solver at each time step.
    virtual void Update(double mtime, bool update_assets = true) override;
};

CH_CLASS_VERSION(ChLinkMateDistanceZ, 0)

// -----------------------------------------------------------------------------

/// Mate constraint of parallel type.
/// This link corresponds to the typical axis-is-parallel-to-axis (or edge to edge, etc.) mating used in 3D CAD
/// assemblies. The axes to be kept parallel are the two Z axes of the two frames.
class ChApi ChLinkMateParallel : public ChLinkMateGeneric {
  public:
    ChLinkMateParallel() : ChLinkMateGeneric(false, false, false, true, true, false), m_flipped(false) {}
    ChLinkMateParallel(const ChLinkMateParallel& other);
    virtual ~ChLinkMateParallel() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkMateParallel* Clone() const override { return new ChLinkMateParallel(*this); }

    /// Tell if the two axes must be opposed (flipped=true) or must have the same verse (flipped=false)
    void SetFlipped(bool doflip);
    bool IsFlipped() const { return m_flipped; }

    /// Specialized initialization for parallel mate, given the two bodies to be connected, two points and two
    /// directions (each expressed in body or abs. coordinates).
    virtual void Initialize(std::shared_ptr<ChBodyFrame> body1,  ///< first body to link
                            std::shared_ptr<ChBodyFrame> body2,  ///< second body to link
                            bool pos_are_relative,               ///< true: following pos. are relative to bodies
                            const ChVector3d& point1,            ///< point on slave axis 1 (rel. or abs.)
                            const ChVector3d& point2,            ///< point on master axis 2 (rel. or abs.)
                            const ChVector3d& dir1,              ///< direction of slave axis 1 (rel. or abs.)
                            const ChVector3d& dir2               ///< direction of master axis 2 (rel. or abs.)
                            ) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  protected:
    bool m_flipped;
};

CH_CLASS_VERSION(ChLinkMateParallel, 0)

// -----------------------------------------------------------------------------

/// Mate constraint of orthogonal type.
/// This link corresponds to the typical axis-is-orthogonal-to-axis (or edge to edge, etc.) mating used in 3D CAD
/// assemblies. Then the two Z axes of the two frames are aligned to the cross product of the two directions.
class ChApi ChLinkMateOrthogonal : public ChLinkMateGeneric {
  public:
    ChLinkMateOrthogonal()
        : ChLinkMateGeneric(false, false, false, false, false, true), m_reldir1(VNULL), m_reldir2(VNULL) {}
    ChLinkMateOrthogonal(const ChLinkMateOrthogonal& other);
    virtual ~ChLinkMateOrthogonal() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkMateOrthogonal* Clone() const override { return new ChLinkMateOrthogonal(*this); }

    /// Specialized initialization for orthogonal mate, given the two bodies to be connected, two points and two
    /// directions (each expressed in body or abs. coordinates).
    virtual void Initialize(std::shared_ptr<ChBodyFrame> body1,  ///< first body to link
                            std::shared_ptr<ChBodyFrame> body2,  ///< second body to link
                            bool pos_are_relative,               ///< true: following pos. are relative to bodies
                            const ChVector3d& point1,            ///< point on slave axis 1 (rel. or abs.)
                            const ChVector3d& point2,            ///< point on master axis 2 (rel. or abs.)
                            const ChVector3d& dir1,              ///< direction of slave axis 1 (rel. or abs.)
                            const ChVector3d& dir2               ///< direction of master axis 2 (rel. or abs.)
                            ) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  protected:
    ChVector3d m_reldir1;
    ChVector3d m_reldir2;

    /// Update link state. This is called automatically by the solver at each time step.
    /// Update constraint jacobian and frames.
    virtual void Update(double mtime, bool update_assets = true) override;
};

CH_CLASS_VERSION(ChLinkMateOrthogonal, 0)

// -----------------------------------------------------------------------------

/// Mate constraint to completely fix relative motion of two frames.
class ChApi ChLinkMateFix : public ChLinkMateGeneric {
  public:
    ChLinkMateFix() : ChLinkMateGeneric(true, true, true, true, true, true) {}
    ChLinkMateFix(const ChLinkMateFix& other);
    virtual ~ChLinkMateFix() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkMateFix* Clone() const override { return new ChLinkMateFix(*this); }

    using ChLinkMateGeneric::Initialize;

    /// Specialized initialization for "fix" mate, given the two bodies to be connected, the positions of the two
    /// auxiliary frames where the two bodies are connected are both automatically initialized as the current absolute
    /// position of body1.
    void Initialize(std::shared_ptr<ChBodyFrame> body1,  ///< first body to link
                    std::shared_ptr<ChBodyFrame> body2   ///< second body to link
    );
};

CH_CLASS_VERSION(ChLinkMateFix, 0)

// -----------------------------------------------------------------------------

/// Rack-pinion link between two body frames.
/// It correctly handles the direction of transmitted force given the teeth pressure angle.
class ChApi ChLinkMateRackPinion : public ChLinkMateGeneric {
  public:
    ChLinkMateRackPinion();
    ChLinkMateRackPinion(const ChLinkMateRackPinion& other);
    virtual ~ChLinkMateRackPinion() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkMateRackPinion* Clone() const override { return new ChLinkMateRackPinion(*this); }

    // Updates aux frames positions
    virtual void UpdateTime(double mytime) override;

    // data get/set

    /// Get the primitive radius of the pinion.
    double GetPinionRadius() const { return R; }

    /// Set the primitive radius of the pinion.
    void SetPinionRadius(double mR) { R = mR; }

    /// Get the pressure angle (usually 20 deg for typical gears).
    double GetPressureAngle() const { return alpha; }

    /// Set the pressure angle (usually 20 deg for typical gears).
    void SetPressureAngle(double mset) { alpha = mset; }

    /// Get the angle of teeth in bevel gears (0 deg for spur gears).
    double GetPitchAngle() const { return beta; }

    /// Set the angle of teeth in bevel gears (0 deg for spur gears).
    void SetPitchAngle(double mset) { beta = mset; }

    /// Get the initial phase of rotation of pinion respect to rack.
    double GetPhase() const { return phase; }

    /// Set the initial phase of rotation of pinion respect to rack.
    void SetPhase(double mset) { phase = mset; }

    /// Enable/disable enforcement check on exact phase between gears (default: false).
    /// If false, after many simulation steps the phasing may be affected by numerical error accumulation.
    /// Note that, to ensure the correct phasing during the many rotations, an algorithm will update an accumulator with
    /// total rotation values, which might be affected by loss of numerical precision after many revolutions.
    void SetEnforcePhase(bool mset) { checkphase = mset; }

    bool GetEnforcePhase() const { return checkphase; }

    /// Get total rotation of 1st gear, respect to interaxis, in radians.
    double GetRotation1() const { return a1; }

    /// Reset the total rotations of a1 and a2.
    void ResetRotation1() { a1 = 0; }

    /// Set pinion shaft position and direction, in body1-relative reference.
    /// The shaft direction is the Z axis of that frame.
    void SetPinionFrame(ChFrame<double> mf) { local_pinion = mf; }

    /// Get pinion shaft position and direction, in body1-relative reference.
    /// The shaft direction is the Z axis of that frame.
    ChFrame<double> GetPinionFrame() const { return local_pinion; }

    /// Set rack position and direction, in body2-relative reference.
    /// The rack direction is the X axis of that frame.
    void SetRackFrame(ChFrame<double> mf) { local_rack = mf; }

    /// Get rack position and direction, in body2-relative reference.
    /// The rack direction is the X axis of that frame.
    ChFrame<double> GetRackFrame() const { return local_rack; }

    /// Get pinion shaft direction in absolute reference.
    ChVector3d GetAbsPinionDir();

    /// Get pinion position in absolute reference.
    ChVector3d GetAbsPinionPos();

    /// Get rack direction in absolute reference.
    ChVector3d GetAbsRackDir();

    /// Get rack position in absolute reference.
    ChVector3d GetAbsRackPos();

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  protected:
    double R;         ///< primitive radius of the pinion
    double alpha;     ///< inclination of action line
    double beta;      ///< helix angle
    double phase;     ///< mounting phase angle
    bool checkphase;  ///< keep gear always on phase

    double a1;  ///< auxiliary

    ChVector3d contact_pt;

    ChFrame<double> local_pinion;  ///< pinion shaft pos & dir (frame Z axis), relative to body1
    ChFrame<double> local_rack;    ///< rack direction (frame X axis), relative to body2
};

CH_CLASS_VERSION(ChLinkMateRackPinion, 0)

}  // end namespace chrono

#endif
