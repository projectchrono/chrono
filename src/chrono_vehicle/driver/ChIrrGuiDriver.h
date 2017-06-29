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
// Authors: Radu Serban, Justin Madsen, Conlain Kelly
// =============================================================================
//
// Irrlicht-based GUI driver for the a vehicle. This class implements the
// functionality required by its base ChDriver class using keyboard or joystick
// inputs. If a joystick is present it will use that as an input; it will
// otherwise default to a keyboard input.
// As an Irrlicht event receiver, its OnEvent() callback is used to keep track
// and update the current driver inputs. As such it does not need to override
// the default no-op Advance() virtual method.
//
// =============================================================================

#ifndef CH_IRRGUIDRIVER_H
#define CH_IRRGUIDRIVER_H

#include <string>

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChDriver.h"

#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/utils/ChVehicleIrrApp.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_driver
/// @{

/// Interactive driver model using keyboard inputs.
/// Irrlicht-based GUI driver for the a vehicle. This class implements the
/// functionality required by its base ChDriver class using keyboard inputs.
/// As an Irrlicht event receiver, its OnEvent() callback is used to keep track
/// and update the current driver inputs. As such it does not need to override
/// the default no-op Advance() virtual method.
///
/// @sa ChDataDriver
class CH_VEHICLE_API ChIrrGuiDriver : public ChDriver, public irr::IEventReceiver {
  public:
    /// Functioning modes for a ChIrrGuiDriver
    enum InputMode {
        LOCK,      ///< driver inputs locked at current values
        KEYBOARD,  ///< driver inputs from keyboard
        DATAFILE,  ///< driver inputs from data file
        JOYSTICK   ///< driver inputs from joystick
    };

    /// Construct an Irrlicht GUI driver.
    ChIrrGuiDriver(ChVehicleIrrApp& app);

    virtual ~ChIrrGuiDriver() {}

    /// Intercept and process keyboard inputs.
    virtual bool OnEvent(const irr::SEvent& event) override;

    /// Update the state of this driver system at the specified time.
    virtual void Synchronize(double time) override;

    /// Set the time response for throttle control.
    void SetThrottleDelta(double delta  ///< time (in seconds) to go from 0 to 1
                          ) {
        m_throttleDelta = delta;
    }

    /// Set the time response for steering control.
    void SetSteeringDelta(double delta  ///< time (in seconds) to go from 0 to 1 (or to -1)
                          ) {
        m_steeringDelta = delta;
    }

    /// Set the time response for braking control.
    void SetBrakingDelta(double delta  ///< time (in seconds) to go from 0 to 1
                         ) {
        m_brakingDelta = delta;
    }

    /// Set the input file for the underlying data driver.
    void SetInputDataFile(const std::string& filename);

    /// Set the current functioning mode.
    void SetInputMode(InputMode mode);

    /// Return the current functioning mode as a string.
    std::string GetInputModeAsString() const;

  protected:
    ChVehicleIrrApp& m_app;

    double m_throttleDelta;
    double m_steeringDelta;
    double m_brakingDelta;
    int m_dT;

    InputMode m_mode;
    double m_time_shift;
    std::shared_ptr<ChDataDriver> m_data_driver;
};

/// @} vehicle_driver

}  // end namespace vehicle
}  // end namespace chrono

#endif
