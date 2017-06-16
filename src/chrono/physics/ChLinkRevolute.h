// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

#ifndef CHLINKREVOLUTE_H
#define CHLINKREVOLUTE_H

#include "chrono/physics/ChLink.h"
#include "chrono/solver/ChConstraintTwoBodies.h"

namespace chrono {

/// Class for modeling a revolute joint between two two ChBodyFrame objects.
/// This joint is defined through 5 constraint equations between two marker
/// frames, one on each body.  Kinematically, these constraints impose the
/// condition that the two marker origins coincide (3 constraints) and that
/// two directions (one on each body, namely the Z axes of the marker frames)
/// are always parallel.

class ChApi ChLinkRevolute : public ChLink {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChLinkRevolute)

  public:
    ChLinkRevolute();
    ChLinkRevolute(const ChLinkRevolute& other);
    ~ChLinkRevolute();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkRevolute* Clone() const override { return new ChLinkRevolute(*this); }

    /// Get the number of (bilateral) constraints introduced by this joint.
    virtual int GetDOC_c() override { return 5; }

    /// Get the link coordinate system, expressed relative to Body2.
    virtual ChCoordsys<> GetLinkRelativeCoords() override { return m_frame2.GetCoord(); }

    /// Get the joint frame on Body1, expressed in Body1 coordinate system.
    const ChFrame<>& GetFrame1Rel() const { return m_frame1; }
    /// Get the joint frame on Body2, expressed in Body2 coordinate system.
    const ChFrame<>& GetFrame2Rel() const { return m_frame2; }

    /// Get the joint frame on Body1, expressed in absolute coordinate system.
    ChFrame<> GetFrame1Abs() const { return m_frame1 >> *Body1; }
    /// Get the joint frame on Body2, expressed in absolute coordinate system.
    ChFrame<> GetFrame2Abs() const { return m_frame2 >> *Body2; }

    /// Get the joint violation (residuals of the constraint equations)
    ChMatrix<>* GetC() { return m_C; }

    /// Initialize this joint by specifying the two bodies to be connected and a
    /// joint frame specified in the absolute frame. The revolute joint is
    /// constructed such that ...
    void Initialize(std::shared_ptr<ChBodyFrame> body1,  ///< first body frame
                    std::shared_ptr<ChBodyFrame> body2,  ///< second body frame
                    const ChFrame<>& frame               ///< joint frame (in absolute frame)
                    );

    /// Initialize this joint by specifying the two bodies to be connected and the
    /// joint frames on each body. If local = true, it is assumed that these quantities
    /// are specified in the local body frames. Otherwise, it is assumed that they are
    /// specified in the absolute frame.
    void Initialize(std::shared_ptr<ChBodyFrame> body1,  ///< first body frame
                    std::shared_ptr<ChBodyFrame> body2,  ///< second body frame
                    bool local,                          ///< true if data given in body local frames
                    const ChFrame<>& frame1,             ///< joint frame on body 1
                    const ChFrame<>& frame2              ///< joint frame on body 2
                    );

    //
    // UPDATING FUNCTIONS
    //

    /// Perform the update of this joint at the specified time: compute jacobians
    /// and constraint violations, cache in internal structures
    virtual void Update(double time, bool update_assets = true) override;

    //
    // STATE FUNCTIONS
    //

    // (override/implement interfaces for global state vectors, see ChPhysicsItem for comments.)
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

    //
    // SOLVER INTERFACE
    //

    virtual void InjectConstraints(ChSystemDescriptor& descriptor) override;
    virtual void ConstraintsBiReset() override;
    virtual void ConstraintsBiLoad_C(double factor = 1, double recovery_clamp = 0.1, bool do_clamp = false) override;
    virtual void ConstraintsLoadJacobians() override;
    virtual void ConstraintsFetch_react(double factor = 1) override;

    //
    // SERIALIZATION
    //

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

  private:
    // Joint frames (in body local frames)
    ChFrame<> m_frame1;  ///< joint frame on body 1
    ChFrame<> m_frame2;  ///< joint frame on body 2

    // Cached matrices
    ChMatrix33<> m_u1_tilde;
    ChMatrix33<> m_v1_tilde;
    ChMatrix33<> m_w2_tilde;

    // The constraint objects
    ChConstraintTwoBodies m_cnstr_x;   ///< x1_abs - x2_abs = 0
    ChConstraintTwoBodies m_cnstr_y;   ///< y1_abs - y2_abs = 0
    ChConstraintTwoBodies m_cnstr_z;   ///< z1_abs - z2_abs = 0
    ChConstraintTwoBodies m_cnstr_uw;  ///< dot(u1_abs, w2_abs) = 0
    ChConstraintTwoBodies m_cnstr_vw;  ///< dot(u1_abs, w2_abs) = 0

    // Current constraint violations
    ChMatrix<>* m_C;

    // Lagrange multipliers
    double m_multipliers[5];
};

CH_CLASS_VERSION(ChLinkRevolute,0)

}  // end namespace chrono

#endif
