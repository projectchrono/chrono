//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2012 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHBODYFRAME_H
#define CHBODYFRAME_H

#include "core/ChFrameMoving.h"
#include "lcp/ChLcpVariablesBodyOwnMass.h"

namespace chrono {

/// Class for objects that represent moving frames in space and
/// that contain  ChLcpVariables proxies to the solver.
/// i.e. items with translational and rotational degrees of freedom
/// This class is used as a base for the very used ChBody class

class ChApi ChBodyFrame : public ChFrameMoving<double> {
  public:
    /// Returns reference to the encapsulated ChLcpVariablesBody,
    /// representing body variables (pos, speed or accel.- see VariablesLoad...() )
    /// and forces.
    /// The ChLcpVariablesBodyOwnMass is the interface to the LCP system solver.
    virtual ChLcpVariablesBodyOwnMass& VariablesBody() = 0;
	virtual ChLcpVariables& Variables() = 0;

    /// Transform generic cartesian force into absolute force+torque applied to body COG.
    /// If local=1, force & application point are intended as expressed in local
    /// coordinates, if =0, in absolute.
    void To_abs_forcetorque(const ChVector<>& force,
                            const ChVector<>& appl_point,
                            int local,
                            ChVector<>& resultforce,
                            ChVector<>& resulttorque);

    /// Transform generic cartesian torque into absolute torque applied to body COG.
    /// If local=1, torque is intended as expressed in local coordinates, if =0, in absolute.
    void To_abs_torque(const ChVector<>& torque, int local, ChVector<>& resulttorque);


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
