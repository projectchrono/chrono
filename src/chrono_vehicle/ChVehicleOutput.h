// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
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
// Definition of vehicle output data structures.
//
// =============================================================================

#ifndef CH_VEHICLE_OUTPUT_H
#define CH_VEHICLE_OUTPUT_H

#include "chrono_vehicle/ChConfigVehicle.h"

#include "chrono/input_output/ChOutputASCII.h"
#ifdef CHRONO_HAS_HDF5
    #include "chrono/input_output/ChOutputHDF5.h"
#endif

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle
/// @{

/// Output data for a vehicle subsystem.
struct OutputData {
    std::vector<const ChAssembly::Components*> comp;  ///< subsystem part components
    std::unique_ptr<ChOutput> db;                     ///< output database

    /// Write the components to the output DB.
    void Write(int frame, double time) const { db->Write(frame, time, comp); }
};

/// @} vehicle

}  // end namespace vehicle
}  // end namespace chrono

#endif
