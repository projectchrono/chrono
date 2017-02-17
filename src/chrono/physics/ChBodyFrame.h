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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHBODYFRAME_H
#define CHBODYFRAME_H

#include "chrono/core/ChFrameMoving.h"
#include "chrono/solver/ChVariablesBodyOwnMass.h"

namespace chrono {

/// Class for objects that represent moving frames in space and
/// that contain  ChVariables proxies to the solver.
/// i.e. items with translational and rotational degrees of freedom
/// This class is used as a base for the very used ChBody class

class ChApi ChBodyFrame : public ChFrameMoving<double> {
  public:
    ChBodyFrame() {}
    ChBodyFrame(const ChBodyFrame& other) : ChFrameMoving<double>(other) {}

    virtual ~ChBodyFrame() {}

    /// Returns reference to the encapsulated ChVariablesBody,
    /// representing body variables (pos, speed or accel.- see VariablesLoad...() )
    /// and forces.
    /// The ChVariablesBodyOwnMass is the interface to the system solver.
    virtual ChVariablesBodyOwnMass& VariablesBody() = 0;
    virtual ChVariables& Variables() = 0;

    /// Transform generic cartesian force into absolute force+torque applied to body COG.
    /// If local=1, force & application point are intended as expressed in local
    /// coordinates, if =0, in absolute.
    void To_abs_forcetorque(const ChVector<>& force,
                            const ChVector<>& appl_point,
                            bool local,
                            ChVector<>& resultforce,
                            ChVector<>& resulttorque);

    /// Transform generic cartesian torque into absolute torque applied to body COG.
    /// If local=1, torque is intended as expressed in local coordinates, if =0, in absolute.
    void To_abs_torque(const ChVector<>& torque, bool local, ChVector<>& resulttorque);

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};

CH_CLASS_VERSION(ChBodyFrame,0)


}  // end namespace chrono

#endif
