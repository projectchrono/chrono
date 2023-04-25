
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
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// Simple engine model for the MAN_5t vehicle.
// - based on torque-speed engine maps
// - both power and torque limited
// - no torque converter
//
// =============================================================================

#include "chrono_models/vehicle/man/powertrain/MAN_5t_EngineSimpleMap.h"

namespace chrono {
namespace vehicle {
namespace man {

const double rpm2rads = CH_C_PI / 30;
const double lbft2nm = 1.3558;

MAN_5t_EngineSimpleMap::MAN_5t_EngineSimpleMap(const std::string& name) : ChEngineSimpleMap(name) {}

double MAN_5t_EngineSimpleMap::GetMaxEngineSpeed() {
    return 2300 * rpm2rads;
}

void MAN_5t_EngineSimpleMap::SetEngineTorqueMaps(ChFunction_Recorder& map0, ChFunction_Recorder& mapF) {
    map0.AddPoint(-100 * rpm2rads, 0.000 * lbft2nm);
    map0.AddPoint(0 * rpm2rads, 0.0 * lbft2nm);
    map0.AddPoint(100 * rpm2rads, 0.0 * lbft2nm);
    map0.AddPoint(400 * rpm2rads, -20.0 * lbft2nm);
    map0.AddPoint(600 * rpm2rads, -20.0 * lbft2nm);
    map0.AddPoint(800 * rpm2rads, -20.0 * lbft2nm);
    map0.AddPoint(1000 * rpm2rads, -20.0 * lbft2nm);
    map0.AddPoint(1200 * rpm2rads, -20.0 * lbft2nm);
    map0.AddPoint(1400 * rpm2rads, -20.0 * lbft2nm);
    map0.AddPoint(1600 * rpm2rads, -20.0 * lbft2nm);
    map0.AddPoint(1800 * rpm2rads, -30.0 * lbft2nm);
    map0.AddPoint(2000 * rpm2rads, -30.0 * lbft2nm);
    map0.AddPoint(2100 * rpm2rads, -40.0 * lbft2nm);
    map0.AddPoint(2300 * rpm2rads, -100.0 * lbft2nm);
    map0.AddPoint(2500 * rpm2rads, -150.0 * lbft2nm);

    mapF.AddPoint(-100.0 * rpm2rads, 0.00);
    mapF.AddPoint(0.0 * rpm2rads, 200.00);
    mapF.AddPoint(400.0 * rpm2rads, 300.00);
    mapF.AddPoint(800.0 * rpm2rads, 600.00);
    mapF.AddPoint(985.22 * rpm2rads, 759.93);
    mapF.AddPoint(1047.46 * rpm2rads, 828.31);
    mapF.AddPoint(1115.06 * rpm2rads, 889.74);
    mapF.AddPoint(1164.93 * rpm2rads, 914.07);
    mapF.AddPoint(1211.27 * rpm2rads, 925.66);
    mapF.AddPoint(1255.84 * rpm2rads, 930.3);
    mapF.AddPoint(1320.03 * rpm2rads, 933.77);
    mapF.AddPoint(1389.57 * rpm2rads, 936.09);
    mapF.AddPoint(1450.2 * rpm2rads, 936.09);
    mapF.AddPoint(1539.36 * rpm2rads, 936.09);
    mapF.AddPoint(1589.3 * rpm2rads, 936.09);
    mapF.AddPoint(1673.11 * rpm2rads, 933.77);
    mapF.AddPoint(1739.1 * rpm2rads, 930.3);
    mapF.AddPoint(1797.97 * rpm2rads, 922.19);
    mapF.AddPoint(1849.71 * rpm2rads, 912.91);
    mapF.AddPoint(1896.1 * rpm2rads, 902.48);
    mapF.AddPoint(1937.14 * rpm2rads, 892.05);
    mapF.AddPoint(1976.41 * rpm2rads, 879.3);
    mapF.AddPoint(2021.03 * rpm2rads, 863.08);
    mapF.AddPoint(2054.95 * rpm2rads, 846.85);
    mapF.AddPoint(2094.22 * rpm2rads, 831.79);
    mapF.AddPoint(2129.93 * rpm2rads, 815.56);
    mapF.AddPoint(2156.71 * rpm2rads, 802.81);
    mapF.AddPoint(2195.99 * rpm2rads, 786.59);
    mapF.AddPoint(2226.33 * rpm2rads, 773.84);
    mapF.AddPoint(2254.89 * rpm2rads, 763.41);
    mapF.AddPoint(2297.73 * rpm2rads, 747.19);
    mapF.AddPoint(2300.0 * rpm2rads, -100.0);
}

}  // namespace man
}  // end namespace vehicle
}  // end namespace chrono
