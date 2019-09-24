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
// Irrlicht-based visualization wrapper for wheeled vehicles.
// This class extends ChVehicleIrrApp.
//
// =============================================================================

#ifndef CH_WHEELED_VEHICLE_IRRAPP_H
#define CH_WHEELED_VEHICLE_IRRAPP_H

#include "chrono_vehicle/utils/ChVehicleIrrApp.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_utils
/// @{

/// Customized Chrono Irrlicht application for wheeled vehicle visualization.
class CH_VEHICLE_API ChWheeledVehicleIrrApp : public ChVehicleIrrApp {
  public:
    /// Construct a wheeled vehicle Irrlicht application.
    ChWheeledVehicleIrrApp(
        ChVehicle* vehicle,        ///< pointer to the associated vehicle system
        const wchar_t* title = 0,  ///< window title
        irr::core::dimension2d<irr::u32> dims = irr::core::dimension2d<irr::u32>(1000, 800),  ///< window dimensions
        irr::ELOG_LEVEL log_level = irr::ELL_INFORMATION  ///< Irrlicht logging level
    );

    ~ChWheeledVehicleIrrApp() {}

  private:
    virtual void renderOtherStats(int left, int top) override;

    ChWheeledVehicle* m_wvehicle;
};

/// @} vehicle_wheeled_utils

}  // end namespace vehicle
}  // end namespace chrono

#endif
