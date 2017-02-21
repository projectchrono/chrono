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
// Irrlicht-based GUI driver for the a suspension test rig.
// This class implements the functionality required by its base ChDriverSTR
// class using keyboard inputs.
// As an Irrlicht event receiver, its OnEvent() callback is used to keep track
// and update the current driver inputs. As such it does not need to override
// the default no-op Advance() virtual method.
//
// =============================================================================

#ifndef CH_IRRGUIDRIVER_STR_H
#define CH_IRRGUIDRIVER_STR_H

#include <string>

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/utils/ChVehicleIrrApp.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChDriverSTR.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_test_rig
/// @{

/// Irrlicht-based GUI driver for the a suspension test rig.  This class implements
/// the functionality required by its base ChDriverSTR class using keyboard inputs.
/// As an Irrlicht event receiver, its OnEvent() callback is used to keep track
/// and update the current driver inputs.
class CH_VEHICLE_API ChIrrGuiDriverSTR : public ChDriverSTR, public irr::IEventReceiver {
  public:
    ChIrrGuiDriverSTR(ChVehicleIrrApp& app  ///< handle to the vehicle Irrlicht application
                      );

    ~ChIrrGuiDriverSTR() {}

    /// Override for OnEvent.
    virtual bool OnEvent(const irr::SEvent& event) override;

    /// Set the time response for steering control.
    /// The provided value represents the time (in seconds) for increasing the
    /// steering input from 0 to 1 (or decreasing it from 0 to -1).
    void SetSteeringDelta(double delta) { m_steeringDelta = delta; }

    /// Set the time response for post displacement control.
    /// The provided value represents the time (in seconds) for increasing the
    /// displacement input from 0 to 1 (or decreasing it from 0 to -1).
    void SetDisplacementDelta(double delta) { m_displDelta = delta; }

  private:
    ChVehicleIrrApp& m_app;

    double m_steeringDelta;
    double m_displDelta;
};

/// @} vehicle_wheeled_test_rig

}  // end namespace vehicle
}  // end namespace chrono

#endif
