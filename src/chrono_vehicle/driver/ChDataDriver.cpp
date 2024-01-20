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
// A driver model based on user inputs provided as time series. If provided as a
// text file, each line in the file must contain 4 values:
//   time steering throttle braking
// It is assumed that the time values are unique.
// If the time values are not sorted, this must be specified at construction.
// Driver inputs at intermediate times are obtained through linear interpolation.
//
// =============================================================================

#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <algorithm>

#include "chrono_vehicle/driver/ChDataDriver.h"

namespace chrono {
namespace vehicle {

ChDataDriver::ChDataDriver(ChVehicle& vehicle, const std::string& filename, bool sorted) : ChDriver(vehicle) {
    bool auto_transmission = vehicle.GetPowertrainAssembly() && vehicle.GetPowertrainAssembly()->GetTransmission() &&
                             vehicle.GetPowertrainAssembly()->GetTransmission()->IsAutomatic();

    std::ifstream ifile(filename.c_str());
    std::string line;

    while (std::getline(ifile, line)) {
        // skip empty line, if present
        if (line.empty())
            continue;

        std::istringstream iss(line);

        double time, steering, throttle, braking;
        double clutch = 0;

        if (auto_transmission)
            iss >> time >> steering >> throttle >> braking;
        else
            iss >> time >> steering >> throttle >> braking >> clutch;

        if (iss.fail())
            break;

        m_data.push_back(Entry(time, steering, throttle, braking, clutch));
    }

    ifile.close();

    if (!sorted)
        std::sort(m_data.begin(), m_data.end(), ChDataDriver::compare);

    GetLog() << "Loaded driver file: " << filename.c_str() << "\n";
}

ChDataDriver::ChDataDriver(ChVehicle& vehicle, const std::vector<Entry>& data, bool sorted)
    : ChDriver(vehicle), m_data(data) {
    if (!sorted)
        std::sort(m_data.begin(), m_data.end(), ChDataDriver::compare);
}

// -----------------------------------------------------------------------------

void ChDataDriver::Synchronize(double time) {
    if (time <= m_data[0].m_time) {
        m_steering = m_data[0].m_steering;
        m_throttle = m_data[0].m_throttle;
        m_braking = m_data[0].m_braking;
        m_clutch = m_data[0].m_clutch;
        return;
    } else if (time >= m_data.back().m_time) {
        m_steering = m_data.back().m_steering;
        m_throttle = m_data.back().m_throttle;
        m_braking = m_data.back().m_braking;
        m_clutch = m_data.back().m_clutch;
        return;
    }

    std::vector<Entry>::iterator right =
        std::lower_bound(m_data.begin(), m_data.end(), Entry(time, 0, 0, 0, 0), ChDataDriver::compare);

    std::vector<Entry>::iterator left = right - 1;

    double tbar = (time - left->m_time) / (right->m_time - left->m_time);

    m_steering = left->m_steering + tbar * (right->m_steering - left->m_steering);
    m_throttle = left->m_throttle + tbar * (right->m_throttle - left->m_throttle);
    m_braking = left->m_braking + tbar * (right->m_braking - left->m_braking);
    m_clutch = left->m_clutch + tbar * (right->m_clutch - left->m_clutch);
}

}  // end namespace vehicle
}  // end namespace chrono
