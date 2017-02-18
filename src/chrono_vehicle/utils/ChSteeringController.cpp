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

#include <cstdio>

#include "chrono/core/ChMathematics.h"

#include "chrono_vehicle/utils/ChSteeringController.h"

#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_thirdparty/rapidjson/filereadstream.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Implementation of the base class ChSteeringController
// -----------------------------------------------------------------------------
ChSteeringController::ChSteeringController()
    : m_dist(0), m_sentinel(0, 0, 0), m_target(0, 0, 0), m_err(0), m_errd(0), m_erri(0), m_collect(false), m_csv(NULL) {
    // Default PID controller gains all zero (no control).
    SetGains(0, 0, 0);
}

ChSteeringController::ChSteeringController(const std::string& filename)
    : m_sentinel(0, 0, 0), m_target(0, 0, 0), m_collect(false), m_csv(NULL) {
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

    m_Kp = d["Gains"]["Kp"].GetDouble();
    m_Ki = d["Gains"]["Ki"].GetDouble();
    m_Kd = d["Gains"]["Kd"].GetDouble();

    m_dist = d["Lookahead Distance"].GetDouble();

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

ChSteeringController::~ChSteeringController() {
    delete m_csv;
}

void ChSteeringController::Reset(const ChVehicle& vehicle) {
    // Base class only calculates an updated sentinel location.
    m_sentinel = vehicle.GetChassisBody()->GetFrame_REF_to_abs().TransformPointLocalToParent(ChVector<>(m_dist, 0, 0));
    m_err = 0;
    m_erri = 0;
    m_errd = 0;
}

double ChSteeringController::Advance(const ChVehicle& vehicle, double step) {
    // Calculate current "sentinel" location.  This is a point at the look-ahead
    // distance in front of the vehicle.
    m_sentinel = vehicle.GetChassisBody()->GetFrame_REF_to_abs().TransformPointLocalToParent(ChVector<>(m_dist, 0, 0));

    // Calculate current "target" location.
    CalcTargetLocation();

    // If data collection is enabled, append current target and sentinel locations.
    if (m_collect) {
        *m_csv << vehicle.GetChTime() << m_target << m_sentinel << std::endl;
    }

    // The "error" vector is the projection onto the horizontal plane (z=0) of
    // the vector between sentinel and target.
    ChVector<> err_vec = m_target - m_sentinel;
    err_vec.z() = 0;

    // Calculate the sign of the angle between the projections of the sentinel
    // vector and the target vector (with origin at vehicle location).
    ChVector<> sentinel_vec = m_sentinel - vehicle.GetVehiclePos();
    sentinel_vec.z() = 0;
    ChVector<> target_vec = m_target - vehicle.GetVehiclePos();
    target_vec.z() = 0;

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

void ChSteeringController::StartDataCollection() {
    // Return now if currently collecting data.
    if (m_collect)
        return;
    // Create the CSV_writer object if needed (first call to this function).
    if (!m_csv) {
        m_csv = new utils::CSV_writer("\t");
        m_csv->stream().setf(std::ios::scientific | std::ios::showpos);
        m_csv->stream().precision(6);
    }
    // Enable data collection.
    m_collect = true;
}

void ChSteeringController::StopDataCollection() {
    // Suspend data collection.
    m_collect = false;
}

void ChSteeringController::WriteOutputFile(const std::string& filename) {
    // Do nothing if data collection was never enabled.
    if (m_csv)
        m_csv->write_to_file(filename);
}

// -----------------------------------------------------------------------------
// Implementation of the derived class ChPathSteeringController.
// -----------------------------------------------------------------------------
ChPathSteeringController::ChPathSteeringController(ChBezierCurve* path, bool isClosedPath) 
    : m_path(path) {
    // Create a tracker object associated with the given path.
    m_tracker = new ChBezierCurveTracker(path, isClosedPath);
}

ChPathSteeringController::ChPathSteeringController(const std::string& filename,
                                                   ChBezierCurve* path, 
                                                   bool isClosedPath)
    : ChSteeringController(filename), m_path(path) {
    // Create a tracker object associated with the given path.
    m_tracker = new ChBezierCurveTracker(path, isClosedPath);
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

}  // end namespace vehicle
}  // end namespace chrono
