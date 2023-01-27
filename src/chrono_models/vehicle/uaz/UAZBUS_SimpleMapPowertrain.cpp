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
// Simple powertrain model for the UAZBUS vehicle.
// - based on torque-speed engine maps
// - both power and torque limited
// - no torque converter
// - simple gear-shifting model (in automatic mode)
//
// =============================================================================

#include "chrono_models/vehicle/uaz/UAZBUS_SimpleMapPowertrain.h"

namespace chrono {
namespace vehicle {
namespace uaz {

const double rpm2rads = CH_C_PI / 30;

UAZBUS_SimpleMapPowertrain::UAZBUS_SimpleMapPowertrain(const std::string& name) : ChSimpleMapPowertrain(name) {}

double UAZBUS_SimpleMapPowertrain::GetMaxEngineSpeed() {
    return 4000 * rpm2rads;
}

void UAZBUS_SimpleMapPowertrain::SetEngineTorqueMaps(ChFunction_Recorder& map0, ChFunction_Recorder& mapF) {
   map0.AddPoint(-10.0,	                          0.0);
   map0.AddPoint(10.0,	                          0.0);
   map0.AddPoint(rpm2rads*7.3654652540894313e+02,	   -10.0);
   map0.AddPoint(rpm2rads*9.8766401357229211e+02,	   -10.0);
   map0.AddPoint(rpm2rads*1.5002376354371584e+03,	   -15.0);
   map0.AddPoint(rpm2rads*1.9805319424140143e+03,	   -15.0);
   map0.AddPoint(rpm2rads*2.3483645267445136e+03,	   -15.0);
   map0.AddPoint(rpm2rads*2.7370849651969356e+03,	   -20.0);
   map0.AddPoint(rpm2rads*3.1899218510296519e+03,	   -20.0);
   map0.AddPoint(rpm2rads*3.6847450081517964e+03,	   -30.0);
   map0.AddPoint(rpm2rads*3.9080471179935125e+03,	   -100.0);

   mapF.AddPoint(-10.0,	                          0.6*1.7442687747035581e+02);
   mapF.AddPoint(rpm2rads*7.3654652540894313e+02,	   1.7442687747035581e+02);
   mapF.AddPoint(rpm2rads*9.8766401357229211e+02,	   1.8272727272727280e+02);
   mapF.AddPoint(rpm2rads*1.2386762202128491e+03,	   1.8984189723320162e+02);
   mapF.AddPoint(rpm2rads*1.5002376354371584e+03,	   1.9577075098814234e+02);
   mapF.AddPoint(rpm2rads*1.7244030537657691e+03,	   2.0051383399209493e+02);
   mapF.AddPoint(rpm2rads*1.9805319424140143e+03,	   2.0525691699604749e+02);
   mapF.AddPoint(rpm2rads*2.1885050625372241e+03,	   2.0762845849802375e+02);
   mapF.AddPoint(rpm2rads*2.3483645267445136e+03,	   2.0810276679841903e+02);
   mapF.AddPoint(rpm2rads*2.4920316927464028e+03,	   2.0620553359683799e+02);
   mapF.AddPoint(rpm2rads*2.7370849651969356e+03,	   2.0620553359683799e+02);
   mapF.AddPoint(rpm2rads*2.9927927277540134e+03,	   2.0620553359683799e+02);
   mapF.AddPoint(rpm2rads*3.1899218510296519e+03,	   2.0644268774703562e+02);
   mapF.AddPoint(rpm2rads*3.4879527857490921e+03,	   2.0312252964426881e+02);
   mapF.AddPoint(rpm2rads*3.6847450081517964e+03,	   1.9956521739130440e+02);
   mapF.AddPoint(rpm2rads*3.9080471179935125e+03,	   1.9458498023715421e+02);
   mapF.AddPoint(rpm2rads*4100.0,	                  -100.0);
   mapF.AddPoint(rpm2rads*4200.0,	                  -200.0);
}

void UAZBUS_SimpleMapPowertrain::SetGearRatios(std::vector<double>& fwd, double& rev) {
    rev = -1.0/5.224;

    fwd.push_back(1.0/4.124/1.94);
    fwd.push_back(1.0/4.124);
    fwd.push_back(1.0/2.641);
    fwd.push_back(1.0/1.58);
    fwd.push_back(1.0);
}

void UAZBUS_SimpleMapPowertrain::SetShiftPoints(std::vector<std::pair<double, double>>& shift_bands) {
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 3000 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 3700 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 3700 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 3700 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 3700 * rpm2rads));
}

}  // end namespace uaz
}  // end namespace vehicle
}  // end namespace chrono
