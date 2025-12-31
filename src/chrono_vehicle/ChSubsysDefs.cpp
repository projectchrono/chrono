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

int VehicleObjTag::Generate(uint16_t vehicle_tag, uint16_t part_tag) {
    return (int32_t)(vehicle_tag << 16 | part_tag);
}

uint16_t VehicleObjTag::ExtractVehicleTag(int tag) {
  return (uint16_t)(tag >> 16);
}

uint16_t VehicleObjTag::ExtractPartTag(int tag) {
    return (uint16_t)tag;
}

}  // end namespace vehicle
}  // end namespace chrono
