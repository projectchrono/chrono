//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHLINKTRAJECTORY_H
#define CHLINKTRAJECTORY_H

///////////////////////////////////////////////////
//
//   ChLinkTrajectory.h
//
//
//   Class for point-on-imposed-trajectory constraint
//
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "physics/ChLinkLock.h"
#include "geometry/ChCLine.h"

namespace chrono {
///
/// ChLinkTrajectory class.
/// This class implements the 'point on an imposed
/// trajectory' constraint.
/// It can be used also to simulate the imposed motion
/// of objects in space (for motion capture, for example).
///

class ChApi ChLinkTrajectory : public ChLinkLock {
    CH_RTTI(ChLinkTrajectory, ChLinkLock);

  protected:
    /// Function s=s(t) telling how the curvilinear
    /// parameter of the trajectory is visited in time.
    ChSharedPtr<ChFunction> space_fx;

    /// The line for the trajectory.
    ChSharedPtr<geometry::ChLine> trajectory_line;

    bool modulo_s;

  public:
    // builders and destroyers
    ChLinkTrajectory();
    virtual ~ChLinkTrajectory();
    virtual void Copy(ChLinkTrajectory* source);
    virtual ChLink* new_Duplicate();  // always return base link class pointer

    /// Gets the address of the function s=s(t) telling
    /// how the curvilinear parameter of the trajectory changes in time.
    ChSharedPtr<ChFunction> Get_space_fx() { return space_fx; };

    /// Sets the function s=s(t) telling how the curvilinear parameter
    /// of the trajectory changes in time.
    void Set_space_fx(ChSharedPtr<ChFunction> m_funct);

    /// Tells that the s in  s=s(t)  function will be wrapped in 0..1 if it is outside 0..1
    void Set_modulo_one_fx(bool mmod) { modulo_s = mmod; }

    /// Get the address of the trajectory line
    ChSharedPtr<geometry::ChLine> Get_trajectory_line() { return trajectory_line; }

    /// Sets the trajectory line (take ownership - does not copy line)
    void Set_trajectory_line(ChSharedPtr<geometry::ChLine> mline);

    /// Use this function after link creation, to initialize the link from
    /// two joined rigid bodies.
    /// Both rigid bodies must belong to the same ChSystem.
    /// Two markers will be created and added to the rigid bodies (later,
    /// you can use GetMarker1() and GetMarker2() to access them.
    /// Marker2 will stay in origin of body2. Trajectory is considered relative to body2.
    void Initialize(
        ChSharedPtr<ChBody> mbody1,  ///< first  body to join (the one that follows the trajectory)
        ChSharedPtr<ChBody> mbody2,  ///< second body to join (the one that contains the trajectory)
        const ChVector<>& mpos1,     ///< position of the 'following point' on body1, relative to coordinate of body1.
        ChSharedPtr<geometry::ChLine> mline  ///< the line on mbody2 to be followed by point mpos1 of mbody1
        );

    // UPDATING FUNCTIONS - "lock formulation" custom implementations

    // Overrides the parent class function. Here it moves the
    // constraint mmain marker tangent to the line.
    virtual void UpdateTime(double mytime);

    // STREAMING

    virtual void StreamIN(ChStreamInBinary& mstream);
    virtual void StreamOUT(ChStreamOutBinary& mstream);
};

}  // END_OF_NAMESPACE____

#endif
