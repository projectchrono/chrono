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
#include <iterator>

#include "chrono_vehicle/ChSubsysDefs.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------

SpringForce::SpringForce(double preload) : m_P(preload), m_stops(false), m_min_length(0), m_max_length(0) {}

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

void SpringForce::set_stops(const std::vector<std::pair<double, double>>& data_bump,
                            const std::vector<std::pair<double, double>>& data_rebound) {
    m_bump.Reset();
    m_rebound.Reset();
    for (unsigned int i = 0; i < data_bump.size(); ++i)
        m_bump.AddPoint(data_bump[i].first, data_bump[i].second);
    for (unsigned int i = 0; i < data_rebound.size(); ++i)
        m_rebound.AddPoint(data_rebound[i].first, data_rebound[i].second);
}

void SpringForce::set_stops(double bump_coefficient, double rebound_coefficient) {
    for (auto& p : m_bump.GetPoints())
        p.y = bump_coefficient * p.x;
    for (auto& p : m_rebound.GetPoints())
        p.y = rebound_coefficient * p.x;
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

rapidjson::Value SpringForce::exportJSON(rapidjson::Document::AllocatorType& allocator) {
    rapidjson::Value obj(rapidjson::kObjectType);

    obj.AddMember("min length", m_min_length, allocator);
    obj.AddMember("max length", m_max_length, allocator);

    rapidjson::Value dataB(rapidjson::kArrayType);
    for (const auto& p : m_bump.GetPoints()) {
        rapidjson::Value xy(rapidjson::kArrayType);
        xy.PushBack(p.x, allocator);
        xy.PushBack(p.y, allocator);
        dataB.PushBack(xy, allocator);
    }

    rapidjson::Value dataR(rapidjson::kArrayType);
    for (const auto& p : m_rebound.GetPoints()) {
        rapidjson::Value xy(rapidjson::kArrayType);
        xy.PushBack(p.x, allocator);
        xy.PushBack(p.y, allocator);
        dataR.PushBack(xy, allocator);
    }

    obj.AddMember("bump curve data", dataB, allocator);
    obj.AddMember("rebound curve data", dataR, allocator);

    return obj;
}

// -----------------------------------------------------------------------------

LinearSpringForce::LinearSpringForce(double k, double preload) : SpringForce(preload), m_k(k) {}

double LinearSpringForce::evaluate(double time, double rest_length, double length, double vel, const ChLinkTSDA& link) {
    return m_P - m_k * (length - rest_length) + evaluate_stops(length);
}

rapidjson::Value LinearSpringForce::exportJSON(rapidjson::Document::AllocatorType& allocator) {
    rapidjson::Value obj(rapidjson::kObjectType);

    obj.AddMember("type", "LinearSpringForce", allocator);
    obj.AddMember("spring coefficient", m_k, allocator);
    obj.AddMember("preload", m_P, allocator);
    if (m_stops)
        obj.AddMember("Stops", SpringForce::exportJSON(allocator), allocator);

    return obj;
}

// -----------------------------------------------------------------------------

NonlinearSpringForce::NonlinearSpringForce(double preload) : SpringForce(preload) {}

NonlinearSpringForce::NonlinearSpringForce(const std::vector<std::pair<double, double>>& dataK, double preload)
    : SpringForce(preload) {
    for (unsigned int i = 0; i < dataK.size(); ++i)
        m_mapK.AddPoint(dataK[i].first, dataK[i].second);
}

void NonlinearSpringForce::add_pointK(double x, double y) {
    m_mapK.AddPoint(x, y);
}

double NonlinearSpringForce::evaluate(double time,
                                      double rest_length,
                                      double length,
                                      double vel,
                                      const ChLinkTSDA& link) {
    return m_P - m_mapK.Get_y(length - rest_length) + evaluate_stops(length);
}

rapidjson::Value NonlinearSpringForce::exportJSON(rapidjson::Document::AllocatorType& allocator) {
    rapidjson::Value obj(rapidjson::kObjectType);

    obj.AddMember("type", "NonlinearSpringForce", allocator);

    rapidjson::Value dataK(rapidjson::kArrayType);
    for (const auto& p : m_mapK.GetPoints()) {
        rapidjson::Value xy(rapidjson::kArrayType);
        xy.PushBack(p.x, allocator);
        xy.PushBack(p.y, allocator);
        dataK.PushBack(xy, allocator);
    }

    obj.AddMember("spring curve data", dataK, allocator);
    obj.AddMember("preload", m_P, allocator);
    if (m_stops)
        obj.AddMember("Stops", SpringForce::exportJSON(allocator), allocator);

    return obj;
}

// -----------------------------------------------------------------------------

LinearDamperForce::LinearDamperForce(double c, double preload) : m_c(c) {}

double LinearDamperForce::evaluate(double time, double rest_length, double length, double vel, const ChLinkTSDA& link) {
    return -m_c * vel;
}

rapidjson::Value LinearDamperForce::exportJSON(rapidjson::Document::AllocatorType& allocator) {
    rapidjson::Value obj(rapidjson::kObjectType);

    obj.AddMember("type", "LinearDamperForce", allocator);
    obj.AddMember("damping coefficient", m_c, allocator);

    return obj;
}

// -----------------------------------------------------------------------------

NonlinearDamperForce::NonlinearDamperForce() {}

NonlinearDamperForce::NonlinearDamperForce(const std::vector<std::pair<double, double>>& dataC) {
    for (unsigned int i = 0; i < dataC.size(); ++i) {
        m_mapC.AddPoint(dataC[i].first, dataC[i].second);
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

rapidjson::Value NonlinearDamperForce::exportJSON(rapidjson::Document::AllocatorType& allocator) {
    rapidjson::Value obj(rapidjson::kObjectType);

    obj.AddMember("type", "NonlinearDamperForce", allocator);

    rapidjson::Value dataC(rapidjson::kArrayType);
    for (const auto& p : m_mapC.GetPoints()) {
        rapidjson::Value xy(rapidjson::kArrayType);
        xy.PushBack(p.x, allocator);
        xy.PushBack(p.y, allocator);
        dataC.PushBack(xy, allocator);
    }

    obj.AddMember("damping curve data", dataC, allocator);

    return obj;
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

rapidjson::Value DegressiveDamperForce::exportJSON(rapidjson::Document::AllocatorType& allocator) {
    rapidjson::Value obj(rapidjson::kObjectType);

    obj.AddMember("type", "DegressiveDamperForce", allocator);
    obj.AddMember("damping coefficient compression", m_c_compression, allocator);
    obj.AddMember("damping coefficient expansion", m_c_expansion, allocator);
    obj.AddMember("degression coefficient compression", m_degr_compression, allocator);
    obj.AddMember("degression coefficient expansion", m_degr_expansion, allocator);

    return obj;
}

// -----------------------------------------------------------------------------

LinearSpringDamperForce::LinearSpringDamperForce(double k, double c, double preload)
    : SpringForce(preload), m_k(k), m_c(c) {}

double LinearSpringDamperForce::evaluate(double time,
                                         double rest_length,
                                         double length,
                                         double vel,
                                         const ChLinkTSDA& link) {
    return m_P - m_k * (length - rest_length) - m_c * vel + evaluate_stops(length);
}

rapidjson::Value LinearSpringDamperForce::exportJSON(rapidjson::Document::AllocatorType& allocator) {
    rapidjson::Value obj(rapidjson::kObjectType);

    obj.AddMember("type", "LinearSpringDamperForce", allocator);
    obj.AddMember("spring coefficient", m_k, allocator);
    obj.AddMember("damping coefficient", m_c, allocator);
    obj.AddMember("preload", m_P, allocator);
    if (m_stops)
        obj.AddMember("Stops", SpringForce::exportJSON(allocator), allocator);

    return obj;
}

// -----------------------------------------------------------------------------

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
    return m_P - m_mapK.Get_y(length - rest_length) - m_mapC.Get_y(vel) + evaluate_stops(length);
}

rapidjson::Value NonlinearSpringDamperForce::exportJSON(rapidjson::Document::AllocatorType& allocator) {
    rapidjson::Value obj(rapidjson::kObjectType);

    obj.AddMember("type", "NonlinearSpringDamperForce", allocator);

    rapidjson::Value dataK(rapidjson::kArrayType);
    for (const auto& p : m_mapK.GetPoints()) {
        rapidjson::Value xy(rapidjson::kArrayType);
        xy.PushBack(p.x, allocator);
        xy.PushBack(p.y, allocator);
        dataK.PushBack(xy, allocator);
    }

    rapidjson::Value dataC(rapidjson::kArrayType);
    for (const auto& p : m_mapC.GetPoints()) {
        rapidjson::Value xy(rapidjson::kArrayType);
        xy.PushBack(p.x, allocator);
        xy.PushBack(p.y, allocator);
        dataC.PushBack(xy, allocator);
    }

    obj.AddMember("spring curve data", dataK, allocator);
    obj.AddMember("damping curve data", dataC, allocator);
    obj.AddMember("preload", m_P, allocator);
    if (m_stops)
        obj.AddMember("Stops", SpringForce::exportJSON(allocator), allocator);

    return obj;
}

// -----------------------------------------------------------------------------

MapSpringDamperForce::MapSpringDamperForce(double preload) : SpringForce(preload), m_last({0, 0}) {}

MapSpringDamperForce::MapSpringDamperForce(const std::vector<double>& defs,
                                           const std::vector<double>& vels,
                                           ChMatrixConstRef data,
                                           double preload)
    : SpringForce(preload), m_defs(defs), m_vels(vels), m_data(data), m_last({0, 0}) {}

void MapSpringDamperForce::set_deformations(const std::vector<double> defs) {
    assert(m_data.rows() == 0 || (size_t)m_data.cols() == defs.size());
    m_defs = defs;
}

void MapSpringDamperForce::add_pointC(double x, const std::vector<double>& y) {
    assert(m_defs.size() == 0 || m_defs.size() == y.size());
    auto nrow = m_data.rows();

    if (nrow == 0) {
        // First insertion
        assert(m_data.cols() == 0);
        assert(m_vels.size() == 0);
        m_data.resize(1, y.size());
    } else {
        // Subsequent insertions
        assert((size_t)m_data.cols() == y.size());
        m_data.conservativeResize(nrow + 1, Eigen::NoChange);
    }

    m_vels.push_back(x);
    for (auto i = 0; i < y.size(); i++)
        m_data(nrow, i) = y[i];
}

void MapSpringDamperForce::print_data() {
    std::copy(m_defs.begin(), m_defs.end(), std::ostream_iterator<double>(std::cout, " "));
    std::cout << "\n------" << std::endl;
    std::copy(m_vels.begin(), m_vels.end(), std::ostream_iterator<double>(std::cout, " "));
    std::cout << "\n------" << std::endl;
    for (int iv = 0; iv < m_vels.size(); iv++)
        std::cout << m_data.row(iv) << std::endl;
    std::cout << "------" << std::endl;
}

double MapSpringDamperForce::evaluate(double time,
                                      double rest_length,
                                      double length,
                                      double vel,
                                      const ChLinkTSDA& link) {
    assert(m_data.rows() > 0 && m_data.cols() > 0);

    double def = length - rest_length;

    // Find lower bounds in defs and vels vectors
    auto lb_def = std::lower_bound(m_defs.begin(), m_defs.end(), def);
    auto lb_vel = std::lower_bound(m_vels.begin(), m_vels.end(), vel);

    // Extrapolate outside ranges
    if (lb_def == m_defs.end())
        lb_def = std::prev(m_defs.end());
    int d2 = (int)std::distance(m_defs.begin(), lb_def);
    if (d2 == 0)
        d2 = 1;
    int d1 = d2 - 1;

    if (lb_vel == m_vels.end())
        lb_vel = std::prev(m_vels.end());
    int v2 = (int)std::distance(m_vels.begin(), lb_vel);
    if (v2 == 0)
        v2 = 1;
    int v1 = v2 - 1;

    // Calculate weights
    double dd = m_defs[d2] - m_defs[d1];
    double dv = m_vels[v2] - m_vels[v1];

    double wd1 = (m_defs[d2] - def) / dd;
    double wd2 = (def - m_defs[d1]) / dd;

    double wv1 = (m_vels[v2] - vel) / dv;
    double wv2 = (vel - m_vels[v1]) / dv;

    // Calculate force (bi-linear interpolation)
    auto F1 = wv1 * m_data(v1, d1) + wv2 * m_data(v2, d1);
    auto F2 = wv1 * m_data(v1, d2) + wv2 * m_data(v2, d2);

    auto F = wd1 * F1 + wd2 * F2;

    return m_P - F;
}

rapidjson::Value MapSpringDamperForce::exportJSON(rapidjson::Document::AllocatorType& allocator) {
    rapidjson::Value obj(rapidjson::kObjectType);

    obj.AddMember("type", "MapSpringDamperForce", allocator);

    //// TODO: m_defs, m_vels, m_data

    obj.AddMember("preload", m_P, allocator);
    if (m_stops)
        obj.AddMember("Stops", SpringForce::exportJSON(allocator), allocator);

    return obj;
}

// -----------------------------------------------------------------------------

LinearSpringTorque::LinearSpringTorque(double k, double preload) : m_k(k), m_P(preload) {}

double LinearSpringTorque::evaluate(double time, double rest_angle, double angle, double vel, const ChLinkRSDA& link) {
    return m_P - m_k * (angle - rest_angle);
}

rapidjson::Value LinearSpringTorque::exportJSON(rapidjson::Document::AllocatorType& allocator) {
    rapidjson::Value obj(rapidjson::kObjectType);

    obj.AddMember("type", "LinearSpringTorque", allocator);
    obj.AddMember("spring coefficient", m_k, allocator);
    obj.AddMember("preload", m_P, allocator);

    return obj;
}

// -----------------------------------------------------------------------------

NonlinearSpringTorque::NonlinearSpringTorque(double preload) : m_P(preload) {}

NonlinearSpringTorque::NonlinearSpringTorque(const std::vector<std::pair<double, double>>& dataK, double preload)
    : m_P(preload) {
    for (unsigned int i = 0; i < dataK.size(); ++i) {
        m_mapK.AddPoint(dataK[i].first, dataK[i].second);
    }
}

void NonlinearSpringTorque::add_pointK(double x, double y) {
    m_mapK.AddPoint(x, y);
}

double NonlinearSpringTorque::evaluate(double time,
                                       double rest_angle,
                                       double angle,
                                       double vel,
                                       const ChLinkRSDA& link) {
    return m_P - m_mapK.Get_y(angle - rest_angle);
}

rapidjson::Value NonlinearSpringTorque::exportJSON(rapidjson::Document::AllocatorType& allocator) {
    rapidjson::Value obj(rapidjson::kObjectType);

    obj.AddMember("type", "NonlinearSpringTorque", allocator);

    rapidjson::Value dataK(rapidjson::kArrayType);
    for (const auto& p : m_mapK.GetPoints()) {
        rapidjson::Value xy(rapidjson::kArrayType);
        xy.PushBack(p.x, allocator);
        xy.PushBack(p.y, allocator);
        dataK.PushBack(xy, allocator);
    }

    obj.AddMember("spring curve data", dataK, allocator);
    obj.AddMember("preload", m_P, allocator);

    return obj;
}

// -----------------------------------------------------------------------------

LinearDamperTorque::LinearDamperTorque(double c) : m_c(c) {}

double LinearDamperTorque::evaluate(double time, double rest_angle, double angle, double vel, const ChLinkRSDA& link) {
    return -m_c * vel;
}

rapidjson::Value LinearDamperTorque::exportJSON(rapidjson::Document::AllocatorType& allocator) {
    rapidjson::Value obj(rapidjson::kObjectType);

    obj.AddMember("type", "LinearDamperTorque", allocator);
    obj.AddMember("damping coefficient", m_c, allocator);

    return obj;
}

// -----------------------------------------------------------------------------

NonlinearDamperTorque::NonlinearDamperTorque() {}

NonlinearDamperTorque::NonlinearDamperTorque(const std::vector<std::pair<double, double>>& dataC) {
    for (unsigned int i = 0; i < dataC.size(); ++i) {
        m_mapC.AddPoint(dataC[i].first, dataC[i].second);
    }
}
void NonlinearDamperTorque::add_pointC(double x, double y) {
    m_mapC.AddPoint(x, y);
}
double NonlinearDamperTorque::evaluate(double time,
                                       double rest_angle,
                                       double angle,
                                       double vel,
                                       const ChLinkRSDA& link) {
    return -m_mapC.Get_y(vel);
}

rapidjson::Value NonlinearDamperTorque::exportJSON(rapidjson::Document::AllocatorType& allocator) {
    rapidjson::Value obj(rapidjson::kObjectType);

    obj.AddMember("type", "NonlinearDamperTorque", allocator);

    rapidjson::Value dataC(rapidjson::kArrayType);
    for (const auto& p : m_mapC.GetPoints()) {
        rapidjson::Value xy(rapidjson::kArrayType);
        xy.PushBack(p.x, allocator);
        xy.PushBack(p.y, allocator);
        dataC.PushBack(xy, allocator);
    }

    obj.AddMember("damping curve data", dataC, allocator);

    return obj;
}

// -----------------------------------------------------------------------------

LinearSpringDamperTorque::LinearSpringDamperTorque(double k, double c, double preload) : m_k(k), m_c(c), m_P(preload) {}

double LinearSpringDamperTorque::evaluate(double time,
                                          double rest_angle,
                                          double angle,
                                          double vel,
                                          const ChLinkRSDA& link) {
    return m_P - m_k * (angle - rest_angle) - m_c * vel;
}

rapidjson::Value LinearSpringDamperTorque::exportJSON(rapidjson::Document::AllocatorType& allocator) {
    rapidjson::Value obj(rapidjson::kObjectType);

    obj.AddMember("type", "LinearSpringDamperTorque", allocator);
    obj.AddMember("spring coefficient", m_k, allocator);
    obj.AddMember("damping coefficient", m_c, allocator);
    obj.AddMember("preload", m_P, allocator);

    return obj;
}

// -----------------------------------------------------------------------------

NonlinearSpringDamperTorque::NonlinearSpringDamperTorque(double preload) : m_P(preload) {}

NonlinearSpringDamperTorque::NonlinearSpringDamperTorque(const std::vector<std::pair<double, double>>& dataK,
                                                         const std::vector<std::pair<double, double>>& dataC,
                                                         double preload)
    : m_P(preload) {
    for (unsigned int i = 0; i < dataK.size(); ++i) {
        m_mapK.AddPoint(dataK[i].first, dataK[i].second);
    }
    for (unsigned int i = 0; i < dataC.size(); ++i) {
        m_mapC.AddPoint(dataC[i].first, dataC[i].second);
    }
}

void NonlinearSpringDamperTorque::add_pointK(double x, double y) {
    m_mapK.AddPoint(x, y);
}

void NonlinearSpringDamperTorque::add_pointC(double x, double y) {
    m_mapC.AddPoint(x, y);
}

double NonlinearSpringDamperTorque::evaluate(double time,
                                             double rest_angle,
                                             double angle,
                                             double vel,
                                             const ChLinkRSDA& link) {
    return m_P - m_mapK.Get_y(angle - rest_angle) - m_mapC.Get_y(vel);
}

rapidjson::Value NonlinearSpringDamperTorque::exportJSON(rapidjson::Document::AllocatorType& allocator) {
    rapidjson::Value obj(rapidjson::kObjectType);

    obj.AddMember("type", "NonlinearSpringDamperTorque", allocator);

    rapidjson::Value dataK(rapidjson::kArrayType);
    for (const auto& p : m_mapK.GetPoints()) {
        rapidjson::Value xy(rapidjson::kArrayType);
        xy.PushBack(p.x, allocator);
        xy.PushBack(p.y, allocator);
        dataK.PushBack(xy, allocator);
    }

    rapidjson::Value dataC(rapidjson::kArrayType);
    for (const auto& p : m_mapC.GetPoints()) {
        rapidjson::Value xy(rapidjson::kArrayType);
        xy.PushBack(p.x, allocator);
        xy.PushBack(p.y, allocator);
        dataC.PushBack(xy, allocator);
    }

    obj.AddMember("spring curve data", dataK, allocator);
    obj.AddMember("damping curve data", dataC, allocator);
    obj.AddMember("preload", m_P, allocator);

    return obj;
}

}  // end namespace vehicle
}  // end namespace chrono
