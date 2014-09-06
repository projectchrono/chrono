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


namespace chrono
{
///
/// Class for modeling a composite revolute-spherical joint between
/// two ChBodyFrame objects. 
///

class ChApi ChLinkRevoluteSpherical : public ChLink {

  CH_RTTI(ChLinkRevoluteSpherical, ChLink);

public:
    //
    // CONSTRUCTORS
    //

  ChLinkRevoluteSpherical();
  ~ChLinkRevoluteSpherical();

  virtual void Copy(ChLinkRevoluteSpherical* source);
  virtual ChLink* new_Duplicate();  // always return base link class pointer

    //
    // FUNCTIONS
    //

  virtual int GetType() { return LNK_REVOLUTESPHERICAL; }

    /// Initialize this joint by specifying the two bodies to be connected, the
    /// ...
  void Initialize(
    ChSharedPtr<ChBodyFrame> body1,                  ///< first frame (revolute side)
    ChSharedPtr<ChBodyFrame> body2,                  ///< second frame (spherical side)
    bool                     local,                  ///< true if data given in body local frames
    const ChVector<>&        pos1,                   ///< point on first frame
    const ChVector<>&        pos2,                   ///< point on second frame
    const ChVector<>&        dir1,                   ///< direction of revolute on first frame
    bool                     auto_distance = true,   ///< true if imposed distance equal to |pos1 - po2|
    double                   distance = 0            ///< imposed distance (used only if auto_distance = false)
    );

  virtual int GetDOC_d() { return 2; }

    //
    // UPDATING FUNCTIONS
    //

    /// Override _all_ time, jacobian etc. updating.
    /// In detail, it computes jacobians, violations, etc. and stores 
    /// results in inner structures.
  virtual void Update(double mtime);

    //
    // LCP INTERFACE
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
  ChVector<> m_pos1;     // point on first frame (in local frame)
  ChVector<> m_pos2;     // point on second frame (in local frame)
  ChVector<> m_dir1;     // direction of revolute on first frame (in local frame)
  double     m_dist;     // imposed distance between pos1 and pos2

  double     m_cur_dist; // actual distance between pos1 and pos2
  double     m_cur_dot;  // actual value of dot constraint

  // The constraint objects
  ChLcpConstraintTwoBodies m_cnstr_dist;  // ||pos2_abs - pos1_abs|| = dist
  ChLcpConstraintTwoBodies m_cnstr_dot;   // dot(dir1_abs, pos2_abs - pos1_abs) = 0
};


} // END_OF_NAMESPACE____


#endif
