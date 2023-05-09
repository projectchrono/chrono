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
// Authors: Radu Serban, Mike Taylor, Marcel Offermans
// =============================================================================

#include "chrono_models/vehicle/generic/powertrain/Generic_EngineSimpleMap.h"

namespace chrono {
namespace vehicle {
namespace generic {

Generic_EngineSimpleMap::Generic_EngineSimpleMap(const std::string& name)
    : ChEngineSimpleMap(name) {}

double Generic_EngineSimpleMap::GetMaxEngineSpeed() {
    return (8000.0 * CH_C_PI / 30.0);
}

void Generic_EngineSimpleMap::SetEngineTorqueMaps(chrono::ChFunction_Recorder& map0, chrono::ChFunction_Recorder& mapF) {
    map0.AddPoint(0,0.000);
    map0.AddPoint(52.360,-39.262);
    map0.AddPoint(104.720,-39.263);
    map0.AddPoint(157.080,-39.266);
    map0.AddPoint(209.440,-39.272);
    map0.AddPoint(261.799,-39.284);
    map0.AddPoint(314.159,-39.307);
    map0.AddPoint(366.519,-39.353);
    map0.AddPoint(418.879,-39.447);
    map0.AddPoint(471.239,-39.625);
    map0.AddPoint(523.599,-39.750);
    map0.AddPoint(575.959,-39.875);
    map0.AddPoint(628.319,-40.250);
    map0.AddPoint(654.498,-41.500);
    map0.AddPoint(680.678,-43.000);
    map0.AddPoint(706.858,-46.000);
    map0.AddPoint(733.038,-52.000);
    map0.AddPoint(785.398,-64.000);
    map0.AddPoint(837.758,-800.000);

    mapF.AddPoint(0, 80.000);
    mapF.AddPoint(52.360, 80.000);
    mapF.AddPoint(104.720, 135.000);
    mapF.AddPoint(157.080, 200.000);
    mapF.AddPoint(209.440, 245.000);
    mapF.AddPoint(261.799, 263.000);
    mapF.AddPoint(314.159, 310.000);
    mapF.AddPoint(366.519, 358.000);
    mapF.AddPoint(418.879, 404.000);
    mapF.AddPoint(471.239, 455.000);
    mapF.AddPoint(523.599, 475.000);
    mapF.AddPoint(575.959, 485.000);
    mapF.AddPoint(628.319, 468.000);
    mapF.AddPoint(654.498, 462.000);
    mapF.AddPoint(680.678, 455.000);
    mapF.AddPoint(706.858, 427.000);
    mapF.AddPoint(733.038, 370.000);
    mapF.AddPoint(785.398, 259.000);
    mapF.AddPoint(837.758, -700.000);
}

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono
