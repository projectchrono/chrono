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

#include "chrono_vehicle/powertrain/AutomaticTransmissionShafts.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

AutomaticTransmissionShafts::AutomaticTransmissionShafts(const std::string& filename)
    : ChAutomaticTransmissionShafts("") {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

AutomaticTransmissionShafts::AutomaticTransmissionShafts(const rapidjson::Document& d)
    : ChAutomaticTransmissionShafts("") {
    Create(d);
}

void AutomaticTransmissionShafts::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    m_transmissionblock_inertia = d["Transmission Block Inertia"].GetDouble();
    m_ingear_shaft_inertia = d["Input Shaft Inertia"].GetDouble();
    m_motorshaft_inertia = d["Motorshaft Inertia"].GetDouble();
    m_driveshaft_inertia = d["Driveshaft Inertia"].GetDouble();

    // Read torque converter data
    assert(d.HasMember("Torque Converter"));

    m_tc_capacity_factor.Read(d["Torque Converter"]["Capacity Factor Map"]);
    m_tc_torque_ratio.Read(d["Torque Converter"]["Torque Ratio Map"]);

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

void AutomaticTransmissionShafts::SetGearRatios(std::vector<double>& fwd, double& rev) {
    rev = m_rev_gear;
    fwd = m_fwd_gear;
}

void AutomaticTransmissionShafts::SetTorqueConverterCapacityFactorMap(std::shared_ptr<ChFunction_Recorder>& map) {
    m_tc_capacity_factor.Set(*map);
}

void AutomaticTransmissionShafts::SetTorqeConverterTorqueRatioMap(std::shared_ptr<ChFunction_Recorder>& map) {
    m_tc_torque_ratio.Set(*map);
}

}  // end namespace vehicle
}  // end namespace chrono
