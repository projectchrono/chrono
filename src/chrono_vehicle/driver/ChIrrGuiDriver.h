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
// Authors: Radu Serban, Justin Madsen
// =============================================================================
//
// Irrlicht-based GUI driver for the a vehicle. This class implements the
// functionality required by its base ChDriver class using keyboard inputs.
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

#include "chrono_vehicle/utils/ChVehicleIrrApp.h"
#include "chrono_vehicle/driver/ChDataDriver.h"

namespace chrono {

class CH_VEHICLE_API ChIrrGuiDriver : public ChDriver, public irr::IEventReceiver {
  public:
    enum InputMode { LOCK, KEYBOARD, DATAFILE };

    ChIrrGuiDriver(ChVehicleIrrApp& app);

    ~ChIrrGuiDriver() {}

    virtual bool OnEvent(const irr::SEvent& event);
    virtual void Update(double time);

    void SetThrottleDelta(double delta) { m_throttleDelta = delta; }
    void SetSteeringDelta(double delta) { m_steeringDelta = delta; }
    void SetBrakingDelta(double delta) { m_brakingDelta = delta; }

    void SetInputDataFile(const std::string& filename);
    void SetInputMode(InputMode mode);
    std::string GetInputModeAsString() const;

  private:
    ChVehicleIrrApp& m_app;

    double m_throttleDelta;
    double m_steeringDelta;
    double m_brakingDelta;

    InputMode m_mode;
    double m_time_shift;
    ChSharedPtr<ChDataDriver> m_data_driver;
};

}  // end namespace chrono

#endif
