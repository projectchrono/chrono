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
// Irrlicht-based GUI driver for the a track test rig. This class extends
// the ChIrrGuiDriver for a vehicle with controls for the shaker post.
//
// =============================================================================

#ifndef CH_IRRGUIDRIVER_TTR_H
#define CH_IRRGUIDRIVER_TTR_H

#include <string>

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/driver/ChIrrGuiDriver.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked_utils
/// @{

/// Irrlicht-based GUI driver for the a track test rig.
/// This class extends the ChIrrGuiDriver for a vehicle with controls for the shaker posts.
class CH_VEHICLE_API ChIrrGuiDriverTTR : public ChIrrGuiDriver {
  public:
    ChIrrGuiDriverTTR(ChVehicleIrrApp& app,            ///< handle to the vehicle Irrlicht application
                      double displacement_limit = 0.1  ///< limits for post displacement
                      );

    ~ChIrrGuiDriverTTR() {}

    /// Get the post vertical displacement
    double GetDisplacement() const { return m_displacement; }

    /// Override for OnEvent.
    virtual bool OnEvent(const irr::SEvent& event) override;

    void SetDisplacementDelta(double delta) { m_displacementDelta = delta; }

  private:
    void SetDisplacement(double vertical_disp);

    double m_displacementDelta;
    double m_displacement;
    double m_minDisplacement;
    double m_maxDisplacement;
};

/// @} vehicle_tracked_utils

}  // end namespace vehicle
}  // end namespace chrono

#endif
