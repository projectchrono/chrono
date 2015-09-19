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
// Utility classes implementing PID steering controllers. The base class
// implements the basic functionality to control the error between the location
// of a sentinel point (a point at a look-ahead distance in front of the vehicle)
// and the current target point.
// Derived classes differ in how they specify the target point.  This can be the
// closest point to the sentinel point on a pre-defined curve path (currently
// using a ChBezierCurve) or from some other external sources (e.g. interfacing
// with a camera sensor).
//
// An object of this type can be used within a Chrono::Vehicle driver model to
// provide the steering output.
//
// =============================================================================

#include "chrono/core/ChMathematics.h"

#include "chrono_vehicle/utils/ChSteeringController.h"

namespace chrono {

// -----------------------------------------------------------------------------
// Implementation of the base class ChSteeringController
// -----------------------------------------------------------------------------
ChSteeringController::ChSteeringController() : m_dist(0), m_sentinel(0, 0, 0), m_target(0, 0, 0) {
    // Default PID controller gains all zero (no control).
    SetGains(0, 0, 0);
}

void ChSteeringController::Reset(const ChVehicle& vehicle) {
    // Base class only calculates an updated sentinel location.
    m_sentinel = vehicle.GetChassis()->GetFrame_REF_to_abs().TransformPointLocalToParent(ChVector<>(m_dist, 0, 0));
}

double ChSteeringController::Advance(const ChVehicle& vehicle, double step) {
    // Calculate current "sentinel" location.  This is a point at the look-ahead
    // distance in front of the vehicle.
    m_sentinel = vehicle.GetChassis()->GetFrame_REF_to_abs().TransformPointLocalToParent(ChVector<>(m_dist, 0, 0));

    // Calculate current "target" location.
    CalcTargetLocation();

    // The "error" vector is the projection onto the horizontal plane (z=0) of
    // the vector between sentinel and target.
    ChVector<> err_vec = m_target - m_sentinel;
    err_vec.z = 0;

    // Calculate the sign of the angle between the projections of the sentinel
    // vector and the target vector (with origin at vehicle location).
    ChVector<> sentinel_vec = m_sentinel - vehicle.GetChassisPos();
    sentinel_vec.z = 0;
    ChVector<> target_vec = m_target - vehicle.GetChassisPos();
    target_vec.z = 0;

    double temp = Vdot(Vcross(sentinel_vec, target_vec), ChVector<>(0, 0, 1));

    // Calculate current error (magnitude).
    double err = ChSignum(temp) * err_vec.Length();

    // Estimate error derivative (backward FD approximation).
    m_errd = (err - m_err) / step;

    // Calculate current error integral (trapezoidal rule).
    m_erri += (err + m_err) * step / 2;

    // Cache new error
    m_err = err;

    // Return PID output (steering value)
    return m_Kp * m_err + m_Ki * m_erri + m_Kd * m_errd;
}

// -----------------------------------------------------------------------------
// Implementation of the derived class ChPathSteeringController.
// -----------------------------------------------------------------------------
ChPathSteeringController::ChPathSteeringController(ChBezierCurve* path) : m_path(path) {
    // Create a tracker object associated with the given path.
    m_tracker = new ChBezierCurveTracker(path);
}

ChPathSteeringController::~ChPathSteeringController() {
    delete m_tracker;
}

void ChPathSteeringController::CalcTargetLocation() {
    // Let the underlying tracker do its magic.
    m_tracker->calcClosestPoint(m_sentinel, m_target);
}

void ChPathSteeringController::Reset(const ChVehicle& vehicle) {
    // Let the base class calculate the current location of the sentinel point.
    ChSteeringController::Reset(vehicle);

    // Reset the path tracker with the new sentinel location.
    m_tracker->reset(m_sentinel);
}

}  // end namespace chrono
