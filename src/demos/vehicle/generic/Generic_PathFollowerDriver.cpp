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
//
//
// =============================================================================

#include "generic/Generic_PathFollowerDriver.h"

using namespace chrono;

Generic_PathFollowerDriver::Generic_PathFollowerDriver(ChVehicle& vehicle, ChBezierCurve* path)
    : m_vehicle(vehicle), m_PID(path) {
    m_PID.Reset(vehicle);
}

Generic_PathFollowerDriver::Generic_PathFollowerDriver(ChVehicle& vehicle,
                                                       const std::string& filename,
                                                       ChBezierCurve* path)
    : m_vehicle(vehicle), m_PID(filename, path) {
    m_PID.Reset(vehicle);
}

void Generic_PathFollowerDriver::Reset() {
    m_PID.Reset(m_vehicle);
}

void Generic_PathFollowerDriver::Advance(double step) {
    // Set trottle and braking driver values.
    m_throttle = 0.12;
    m_braking = 0;

    // Let the underlying steering PID controller calculate the driver steering
    // value (make sure the value is clamped between -1 and +1).
    SetSteering(m_PID.Advance(m_vehicle, step), -1, 1);
}
