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

#ifndef CHLINKPOINTSPLINE_H
#define CHLINKPOINTSPLINE_H

///////////////////////////////////////////////////
//
//   ChLinkPointSpline.h
//
//
//   Class for point-on-spline constraint
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
/// ChLinkPointSpline class.
/// This class implements the 'point on a spline curve'
/// constraint. It can be used also to simulate
/// curvilinear glyphs, etc.
///

class ChApi ChLinkPointSpline : public ChLinkLock {
    CH_RTTI(ChLinkPointSpline, ChLinkLock);

  protected:
    /// The line for the trajectory.
    std::shared_ptr<geometry::ChLine> trajectory_line;

  public:
    // builders and destroyers
    ChLinkPointSpline();
    virtual ~ChLinkPointSpline();
    virtual void Copy(ChLinkPointSpline* source);
    virtual ChLink* new_Duplicate();  // always return base link class pointer

    /// Get the address of the trajectory line
    std::shared_ptr<geometry::ChLine> Get_trajectory_line() { return trajectory_line; }

    /// Sets the trajectory line (take ownership - does not copy line)
    void Set_trajectory_line(std::shared_ptr<geometry::ChLine> mline);

    // UPDATING FUNCTIONS - "lock formulation" custom implementations

    // Overrides the parent class function. Here it moves the
    // constraint mmain marker tangent to the line.
    virtual void UpdateTime(double mytime);

    //
    // SERIALIZATION
    //

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive);

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive);
};

}  // END_OF_NAMESPACE____

#endif
