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

#ifndef CHLINKUNIVERSAL_H
#define CHLINKUNIVERSAL_H


#include "physics/ChLink.h"
#include "lcp/ChLcpConstraintTwoBodies.h"


namespace chrono {

/// Class for modeling a universal joint between two two ChBodyFrame objects.
/// This joint is defined through 4 constraint equations between two marker
/// frames, one on each body.  Kinematically, these constraints impose the
/// condition that the two marker origins coincide (3 constraints) and that
/// two directions (one on each body) are always perpendicular. Together,
/// these constraints model the cross in a physical universal joint.

class ChApi ChLinkUniversal : public ChLink {

  CH_RTTI(ChLinkUniversal, ChLink);

public:
    //
    // CONSTRUCTORS
    //

  ChLinkUniversal();
  ~ChLinkUniversal();

  virtual void Copy(ChLinkUniversal* source);
  virtual ChLink* new_Duplicate();

    //
    // FUNCTIONS
    //

  /// Get the type of this joint.
  virtual int GetType()  { return LNK_UNIVERSAL; }

  /// Get the number of constraints imposed by this joint.
  virtual int GetDOC_d() { return 4; }

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
  /// joint frame specified in the absolute frame. The universal joint is
  /// constructed such that ... 

  void Initialize(
    ChSharedPtr<ChBodyFrame> body1,   ///< first body frame
    ChSharedPtr<ChBodyFrame> body2,   ///< second body frame
    const ChFrame<>&         frame    ///< joint frame (in absolute frame)
    );

  /// Initialize this joint by specifying the two bodies to be connected and the
  /// joint frames on each body. If local = true, it is assumed that these quantities
  /// are specified in the local body frames. Otherwise, it is assumed that they are
  /// specified in the absolute frame.
  void Initialize(
    ChSharedPtr<ChBodyFrame> body1,   ///< first body frame
    ChSharedPtr<ChBodyFrame> body2,   ///< second body frame
    bool                     local,   ///< true if data given in body local frames
    const ChFrame<>&         frame1,  ///< joint frame on body 1
    const ChFrame<>&         frame2   ///< joint frame on body 2
    );

    //
    // UPDATING FUNCTIONS
    //

  /// Perform the update of this joint at the specified time: compute jacobians
  /// and constraint violations, cache in internal structures
  virtual void Update(double time);

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

private:

  // Joint frames (in body local frames)
  ChFrame<> m_frame1;                     // joint frame on body 1
  ChFrame<> m_frame2;                     // joint frame on body 2

  // The constraint objects
  ChLcpConstraintTwoBodies m_cnstr_x;     // x1_abs - x2_abs = 0
  ChLcpConstraintTwoBodies m_cnstr_y;     // y1_abs - y2_abs = 0
  ChLcpConstraintTwoBodies m_cnstr_z;     // z1_abs - z2_abs = 0
  ChLcpConstraintTwoBodies m_cnstr_dot;   // dot(u1_abs, v2_abs) = 0

  // Current constraint violations
  ChMatrix<>* m_C;

  // Caching of multipliers to allow warm starting
  double m_cache_speed[4];
  double m_cache_pos[4];
};


} // END_OF_NAMESPACE____


#endif
