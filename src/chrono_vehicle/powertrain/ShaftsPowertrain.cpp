// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
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
// ChShaft-based powertrain model constructed with data from file (JSON format).
//
// =============================================================================

#include "chrono/core/ChGlobal.h"

#include "chrono_vehicle/powertrain/ShaftsPowertrain.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

const double rpm2rads = CH_C_PI / 30;

// -----------------------------------------------------------------------------
// Constructor a shafts powertrain using data from the specified JSON file.
// -----------------------------------------------------------------------------
ShaftsPowertrain::ShaftsPowertrain(const std::string& filename) : ChShaftsPowertrain("") {
    Document d; ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

ShaftsPowertrain::ShaftsPowertrain(const rapidjson::Document& d) : ChShaftsPowertrain("") {
    Create(d);
}

// -----------------------------------------------------------------------------
// Worker function for creating a ShaftsPowertrain powertrain using data in the
// specified RapidJSON document.
// -----------------------------------------------------------------------------
void ShaftsPowertrain::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    // Read engine data
    assert(d.HasMember("Engine"));

    m_motorblock_inertia = d["Engine"]["Motor Block Inertia"].GetDouble();
    m_crankshaft_inertia = d["Engine"]["Crankshaft Inertia"].GetDouble();
    ReadMapData(d["Engine"]["Torque Map"], m_engine_torque);
    ReadMapData(d["Engine"]["Losses Map"], m_engine_losses);

    // Read transmission data
    assert(d.HasMember("Transmission"));

    m_ingear_shaft_inertia = d["Transmission"]["Input Shaft Inertia"].GetDouble();
    m_rev_gear = d["Transmission"]["Reverse Gear Ratio"].GetDouble();
    unsigned int np = d["Transmission"]["Forward Gear Ratios"].Size();
    m_fwd_gear.resize(np);
    for (unsigned int i = 0; i < np; i++)
        m_fwd_gear[i] = d["Transmission"]["Forward Gear Ratios"][i].GetDouble();

    m_upshift_RPM = d["Transmission"]["Upshift RPM"].GetDouble();
    m_downshift_RPM = d["Transmission"]["Downshift RPM"].GetDouble();

    if (d["Transmission"].HasMember("Shift Latency")) {
        SetGearShiftLatency(d["Transmission"]["Shift Latency"].GetDouble());
    }

    // Read torque converter data
    assert(d.HasMember("Torque Converter"));
    ReadMapData(d["Torque Converter"]["Capacity Factor Map"], m_tc_capacity_factor);
    ReadMapData(d["Torque Converter"]["Torque Ratio Map"], m_tc_torque_ratio);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ShaftsPowertrain::SetGearRatios(std::vector<double>& fwd, double& rev) {
    rev = m_rev_gear;
    fwd = m_fwd_gear;
}

// -----------------------------------------------------------------------------
// Utility functions for reading (from a JSON object) and assigning (to a
// recorder function) map data specified as (x,y) pairs.
// -----------------------------------------------------------------------------
void ShaftsPowertrain::ReadMapData(const rapidjson::Value& a, MapData& map_data) {
    assert(a.IsArray());
    map_data.m_n = a.Size();
    for (unsigned int i = 0; i < map_data.m_n; i++) {
        map_data.m_x.push_back(a[i][0u].GetDouble());
        map_data.m_y.push_back(a[i][1u].GetDouble());
    }
}

void ShaftsPowertrain::SetMapData(const MapData& map_data,
                                  double x_factor,
                                  double y_factor,
                                  std::shared_ptr<ChFunction_Recorder>& map) {
    for (unsigned int i = 0; i < map_data.m_n; i++) {
        map->AddPoint(x_factor * map_data.m_x[i], y_factor * map_data.m_y[i]);
    }
}

void ShaftsPowertrain::SetEngineTorqueMap(std::shared_ptr<ChFunction_Recorder>& map) {
    SetMapData(m_engine_torque, rpm2rads, 1.0, map);
}
void ShaftsPowertrain::SetEngineLossesMap(std::shared_ptr<ChFunction_Recorder>& map) {
    SetMapData(m_engine_losses, rpm2rads, 1.0, map);
}
void ShaftsPowertrain::SetTorqueConverterCapacityFactorMap(std::shared_ptr<ChFunction_Recorder>& map) {
    SetMapData(m_tc_capacity_factor, 1.0, 1.0, map);
}
void ShaftsPowertrain::SetTorqeConverterTorqueRatioMap(std::shared_ptr<ChFunction_Recorder>& map) {
    SetMapData(m_tc_torque_ratio, 1.0, 1.0, map);
}

}  // end namespace vehicle
}  // end namespace chrono
