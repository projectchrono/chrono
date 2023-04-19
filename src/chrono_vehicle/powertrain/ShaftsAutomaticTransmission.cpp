// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
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
// ChShaft-based automatic transmission model constructed with data from file
// (JSON format).
//
// =============================================================================

#include "chrono/core/ChGlobal.h"

#include "chrono_vehicle/powertrain/ShaftsAutomaticTransmission.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

const double rpm2rads = CH_C_PI / 30;

ShaftsAutomaticTransmission::ShaftsAutomaticTransmission(const std::string& filename)
    : ChShaftsAutomaticTransmission("") {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

ShaftsAutomaticTransmission::ShaftsAutomaticTransmission(const rapidjson::Document& d)
    : ChShaftsAutomaticTransmission("") {
    Create(d);
}

void ShaftsAutomaticTransmission::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    m_transmissionblock_inertia = d["Transmission Block Inertia"].GetDouble();
    m_ingear_shaft_inertia = d["Input Shaft Inertia"].GetDouble();

    // Read torque converter data
    assert(d.HasMember("Torque Converter"));

    ReadMapData(d["Torque Converter"]["Capacity Factor Map"], m_tc_capacity_factor);
    ReadMapData(d["Torque Converter"]["Torque Ratio Map"], m_tc_torque_ratio);

    // Read transmission data
    assert(d.HasMember("Gear Box"));

    m_rev_gear = d["Gear Box"]["Reverse Gear Ratio"].GetDouble();
    unsigned int np = d["Gear Box"]["Forward Gear Ratios"].Size();
    m_fwd_gear.resize(np);
    for (unsigned int i = 0; i < np; i++)
        m_fwd_gear[i] = d["Gear Box"]["Forward Gear Ratios"][i].GetDouble();

    m_upshift_RPM = d["Gear Box"]["Upshift RPM"].GetDouble();
    m_downshift_RPM = d["Gear Box"]["Downshift RPM"].GetDouble();

    if (d["Gear Box"].HasMember("Shift Latency")) {
        SetGearShiftLatency(d["Gear Box"]["Shift Latency"].GetDouble());
    }
}

// -----------------------------------------------------------------------------

void ShaftsAutomaticTransmission::SetGearRatios(std::vector<double>& fwd, double& rev) {
    rev = m_rev_gear;
    fwd = m_fwd_gear;
}

// -----------------------------------------------------------------------------

void ShaftsAutomaticTransmission::ReadMapData(const rapidjson::Value& a, MapData& map_data) {
    assert(a.IsArray());
    map_data.m_n = a.Size();
    for (unsigned int i = 0; i < map_data.m_n; i++) {
        map_data.m_x.push_back(a[i][0u].GetDouble());
        map_data.m_y.push_back(a[i][1u].GetDouble());
    }
}

void ShaftsAutomaticTransmission::SetMapData(const MapData& map_data,
                                             double x_factor,
                                             double y_factor,
                                             std::shared_ptr<ChFunction_Recorder>& map) {
    for (unsigned int i = 0; i < map_data.m_n; i++) {
        map->AddPoint(x_factor * map_data.m_x[i], y_factor * map_data.m_y[i]);
    }
}

void ShaftsAutomaticTransmission::SetTorqueConverterCapacityFactorMap(std::shared_ptr<ChFunction_Recorder>& map) {
    SetMapData(m_tc_capacity_factor, 1.0, 1.0, map);
}

void ShaftsAutomaticTransmission::SetTorqeConverterTorqueRatioMap(std::shared_ptr<ChFunction_Recorder>& map) {
    SetMapData(m_tc_torque_ratio, 1.0, 1.0, map);
}

}  // end namespace vehicle
}  // end namespace chrono
