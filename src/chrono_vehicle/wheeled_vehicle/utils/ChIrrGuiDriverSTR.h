// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Irrlicht-based GUI driver for the a suspension test rig. This class extends
// the ChIrrGuiDriver for a vehicle with controls for the shaker posts.
//
// =============================================================================

#ifndef CH_IRRGUIDRIVER_STR_H
#define CH_IRRGUIDRIVER_STR_H

#include <string>

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/driver/ChIrrGuiDriver.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_utils
/// @{

/// Irrlicht-based GUI driver for the a suspension test rig. This class extends
/// the ChIrrGuiDriver for a vehicle with controls for the shaker posts.
class CH_VEHICLE_API ChIrrGuiDriverSTR : public ChIrrGuiDriver {
  public:
    ChIrrGuiDriverSTR(ChVehicleIrrApp& app,            ///< handle to the vehicle Irrlicht application
                      double displacement_limit = 0.1  ///< limits for post displacement
                      );

    ~ChIrrGuiDriverSTR() {}

    /// Get the left post vertical displacement
    double GetDisplacementLeft() const { return m_displacementLeft; }

    /// Get the right post vertical displacement
    double GetDisplacementRight() const { return m_displacementRight; }

    /// Override for OnEvent.
    virtual bool OnEvent(const irr::SEvent& event) override;

    void SetDisplacementDelta(double delta) { m_displacementDelta = delta; }

  private:
    void SetDisplacementLeft(double vertical_disp);
    void SetDisplacementRight(double vertical_disp);

    double m_displacementDelta;
    double m_displacementLeft;
    double m_displacementRight;
    double m_minDisplacement;
    double m_maxDisplacement;
};

/// @} vehicle_wheeled_utils

}  // end namespace vehicle
}  // end namespace chrono

#endif
