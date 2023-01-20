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
// Irrlicht-based GUI driver for the a suspension test rig.
// This class implements the functionality required by its base class using
// keyboard inputs.
// As an Irrlicht event receiver, its OnEvent() callback is used to keep track
// and update the current inputs.
//
// =============================================================================

#ifndef CH_STR_INTERACTIVE_DRIVER_IRR_H
#define CH_STR_INTERACTIVE_DRIVER_IRR_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChVehicleVisualSystemIrrlicht.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRigDriver.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_test_rig
/// @{

/// Irrlicht-based GUI driver for the a suspension test rig.  This class implements
/// the functionality required by its base ChSuspensionTestRigDriver class using keyboard inputs.
/// As an Irrlicht event receiver, its OnEvent() callback is used to keep track
/// and update the current inputs.
class CH_VEHICLE_API ChSuspensionTestRigInteractiveDriverIRR : public ChSuspensionTestRigDriver,
                                                               public irr::IEventReceiver {
  public:
    ChSuspensionTestRigInteractiveDriverIRR(irrlicht::ChVisualSystemIrrlicht& vsys);

    ~ChSuspensionTestRigInteractiveDriverIRR() {}

    /// Set the time response for steering control.
    /// The provided value represents the time (in seconds) for increasing the
    /// steering input from 0 to 1 (or decreasing it from 0 to -1).
    void SetSteeringDelta(double delta) { m_steeringDelta = delta; }

    /// Set the time response for post displacement control.
    /// The provided value represents the time (in seconds) for increasing the
    /// displacement input from 0 to 1 (or decreasing it from 0 to -1).
    void SetDisplacementDelta(double delta) { m_displDelta = delta; }

  private:
    /// Override for OnEvent.
    virtual bool OnEvent(const irr::SEvent& event) override;

    /// Get string message.
    virtual std::string GetInfoMessage() const override { return m_msg; }

    irrlicht::ChVisualSystemIrrlicht& m_vsys;

    double m_steeringDelta;
    double m_displDelta;
    int m_current_post;
    std::string m_msg;
};

/// @} vehicle_wheeled_test_rig

}  // end namespace vehicle
}  // end namespace chrono

#endif
