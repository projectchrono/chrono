// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Various utility classes for vehicle subsystems.
//
// =============================================================================

#include <limits>

#include "chrono_vehicle/ChSubsysDefs.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------

SpringForce::SpringForce(double preload) : m_f(preload), m_stops(false), m_min_length(0), m_max_length(0) {}

void SpringForce::enable_stops(double min_length, double max_length) {
    m_stops = true;
    m_min_length = min_length;
    m_max_length = max_length;

    // From ADAMS/Car example
    m_bump.AddPoint(0.0, 0.0);
    m_bump.AddPoint(2.0e-3, 200.0);
    m_bump.AddPoint(4.0e-3, 400.0);
    m_bump.AddPoint(6.0e-3, 600.0);
    m_bump.AddPoint(8.0e-3, 800.0);
    m_bump.AddPoint(10.0e-3, 1000.0);
    m_bump.AddPoint(20.0e-3, 2500.0);
    m_bump.AddPoint(30.0e-3, 4500.0);
    m_bump.AddPoint(40.0e-3, 7500.0);
    m_bump.AddPoint(50.0e-3, 12500.0);
    m_bump.AddPoint(60.0e-3, 125000.0);
    //
    m_rebound.AddPoint(0.0, 0.0);
    m_rebound.AddPoint(2.0e-3, 200.0);
    m_rebound.AddPoint(4.0e-3, 400.0);
    m_rebound.AddPoint(6.0e-3, 600.0);
    m_rebound.AddPoint(8.0e-3, 800.0);
    m_rebound.AddPoint(10.0e-3, 1000.0);
    m_rebound.AddPoint(20.0e-3, 2500.0);
    m_rebound.AddPoint(30.0e-3, 4500.0);
    m_rebound.AddPoint(40.0e-3, 7500.0);
    m_rebound.AddPoint(50.0e-3, 12500.0);
    m_rebound.AddPoint(60.0e-3, 125000.0);
}

double SpringForce::evaluate_stops(double length) {
    double f = 0;
    if (m_stops) {
        if (length < m_min_length)
            f += m_bump.Get_y(m_min_length - length);
        if (length > m_max_length)
            f -= m_rebound.Get_y(length - m_max_length);
    }
    return f;
}

LinearSpringForce::LinearSpringForce(double k, double preload) : SpringForce(preload), m_k(k) {}

double LinearSpringForce::evaluate(double time, double rest_length, double length, double vel, const ChLinkTSDA& link) {
    return m_f - m_k * (length - rest_length) + evaluate_stops(length);
}

NonlinearSpringForce::NonlinearSpringForce(double preload) : SpringForce(preload) {}

NonlinearSpringForce::NonlinearSpringForce(const std::vector<std::pair<double, double>>& data, double preload)
    : SpringForce(preload) {
    for (unsigned int i = 0; i < data.size(); ++i)
        m_mapK.AddPoint(data[i].first, data[i].second);
}

void NonlinearSpringForce::add_pointK(double x, double y) {
    m_mapK.AddPoint(x, y);
}

double NonlinearSpringForce::evaluate(double time,
                                      double rest_length,
                                      double length,
                                      double vel,
                                      const ChLinkTSDA& link) {
    return m_f - m_mapK.Get_y(length - rest_length) + evaluate_stops(length);
}

// -----------------------------------------------------------------------------

LinearDamperForce::LinearDamperForce(double c, double preload) : m_c(c) {}

double LinearDamperForce::evaluate(double time, double rest_length, double length, double vel, const ChLinkTSDA& link) {
    return -m_c * vel;
}

NonlinearDamperForce::NonlinearDamperForce() {}

NonlinearDamperForce::NonlinearDamperForce(const std::vector<std::pair<double, double>>& data) {
    for (unsigned int i = 0; i < data.size(); ++i) {
        m_mapC.AddPoint(data[i].first, data[i].second);
    }
}

void NonlinearDamperForce::add_pointC(double x, double y) {
    m_mapC.AddPoint(x, y);
}

double NonlinearDamperForce::evaluate(double time,
                                      double rest_length,
                                      double length,
                                      double vel,
                                      const ChLinkTSDA& link) {
    return -m_mapC.Get_y(vel);
}

// -----------------------------------------------------------------------------

LinearSpringDamperForce::LinearSpringDamperForce(double k, double c, double preload)
    : SpringForce(preload), m_k(k), m_c(c) {}

double LinearSpringDamperForce::evaluate(double time,
                                         double rest_length,
                                         double length,
                                         double vel,
                                         const ChLinkTSDA& link) {
    return m_f - m_k * (length - rest_length) - m_c * vel + evaluate_stops(length);
}

NonlinearSpringDamperForce::NonlinearSpringDamperForce(double preload) : SpringForce(preload) {}

NonlinearSpringDamperForce::NonlinearSpringDamperForce(const std::vector<std::pair<double, double>>& dataK,
                                                       const std::vector<std::pair<double, double>>& dataC,
                                                       double preload)
    : SpringForce(preload) {
    for (unsigned int i = 0; i < dataK.size(); ++i)
        m_mapK.AddPoint(dataK[i].first, dataK[i].second);
    for (unsigned int i = 0; i < dataC.size(); ++i)
        m_mapC.AddPoint(dataC[i].first, dataC[i].second);
}

void NonlinearSpringDamperForce::add_pointK(double x, double y) {
    m_mapK.AddPoint(x, y);
}

void NonlinearSpringDamperForce::add_pointC(double x, double y) {
    m_mapC.AddPoint(x, y);
}

double NonlinearSpringDamperForce::evaluate(double time,
                                            double rest_length,
                                            double length,
                                            double vel,
                                            const ChLinkTSDA& link) {
    return m_f - m_mapK.Get_y(length - rest_length) - m_mapC.Get_y(vel) + evaluate_stops(length);
}

// -----------------------------------------------------------------------------

DegressiveDamperForce::DegressiveDamperForce(double c_compression)
    : m_c_compression(c_compression), m_c_expansion(c_compression), m_degr_compression(0), m_degr_expansion(0) {}

DegressiveDamperForce::DegressiveDamperForce(double c_compression, double c_expansion)
    : m_c_compression(c_compression), m_c_expansion(c_expansion), m_degr_compression(0), m_degr_expansion(0) {}

DegressiveDamperForce::DegressiveDamperForce(double c_compression, double degr_compression, double degr_expansion)
    : m_c_compression(c_compression),
      m_c_expansion(c_compression),
      m_degr_compression(degr_compression),
      m_degr_expansion(degr_expansion) {}

DegressiveDamperForce::DegressiveDamperForce(double c_compression,
                                             double degr_compression,
                                             double c_expansion,
                                             double degr_expansion)
    : m_c_compression(c_compression),
      m_c_expansion(c_expansion),
      m_degr_compression(degr_compression),
      m_degr_expansion(degr_expansion) {}

double DegressiveDamperForce::evaluate(double time,
                                       double rest_length,
                                       double length,
                                       double vel,
                                       const ChLinkTSDA& link) {
    if (vel >= 0) {
        return -m_c_expansion * vel / (1.0 + m_degr_expansion * vel);
    } else {
        return -m_c_compression * vel / (1.0 - m_degr_compression * vel);
    }
}

// -----------------------------------------------------------------------------

LinearSpringTorque::LinearSpringTorque(double k, double rest_angle, double preload)
    : m_k(k), m_rest_angle(rest_angle), m_t(preload) {}

double LinearSpringTorque::evaluate(double time, double angle, double vel, const ChLinkRSDA& link) {
    return m_t - m_k * (angle - m_rest_angle);
}

// -----------------------------------------------------------------------------

LinearDamperTorque::LinearDamperTorque(double c) : m_c(c) {}

double LinearDamperTorque::evaluate(double time, double angle, double vel, const ChLinkRSDA& link) {
    return -m_c * vel;
}

// -----------------------------------------------------------------------------

LinearSpringDamperTorque::LinearSpringDamperTorque(double k, double c, double rest_angle, double preload)
    : m_k(k), m_c(c), m_rest_angle(rest_angle), m_t(preload) {}

double LinearSpringDamperTorque::evaluate(double time, double angle, double vel, const ChLinkRSDA& link) {
    return m_t - m_k * (angle - m_rest_angle) - m_c * vel;
}

// -----------------------------------------------------------------------------

MapSpringTorque::MapSpringTorque(double rest_angle, double preload) : m_rest_angle(rest_angle), m_t(preload) {}

MapSpringTorque::MapSpringTorque(const std::vector<std::pair<double, double>>& data, double rest_angle, double preload)
    : m_rest_angle(rest_angle), m_t(preload) {
    for (unsigned int i = 0; i < data.size(); ++i) {
        m_map.AddPoint(data[i].first, data[i].second);
    }
}

void MapSpringTorque::add_point(double x, double y) {
    m_map.AddPoint(x, y);
}

double MapSpringTorque::evaluate(double time, double angle, double vel, const ChLinkRSDA& link) {
    return m_t - m_map.Get_y(angle - m_rest_angle);
}

// -----------------------------------------------------------------------------

MapDamperTorque::MapDamperTorque() {}
MapDamperTorque::MapDamperTorque(const std::vector<std::pair<double, double>>& data) {
    for (unsigned int i = 0; i < data.size(); ++i) {
        m_map.AddPoint(data[i].first, data[i].second);
    }
}
void MapDamperTorque::add_point(double x, double y) {
    m_map.AddPoint(x, y);
}
double MapDamperTorque::evaluate(double time, double angle, double vel, const ChLinkRSDA& link) {
    return -m_map.Get_y(vel);
}

}  // end namespace vehicle
}  // end namespace chrono
