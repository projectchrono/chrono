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
// Authors: Rainer Gericke
// =============================================================================
//
// Marder support roller subsystem.
//
// =============================================================================

#ifndef MARDER_SUPPORT_ROLLER_H
#define MARDER_SUPPORT_ROLLER_H

#include <string>

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/tracked_vehicle/track_wheel/ChDoubleTrackWheel.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace marder {

/// @addtogroup vehicle_models_marder
/// @{

/// Support roller model for the Marder vehicle (base class).
class CH_MODELS_API Marder_SupportRoller : public ChDoubleTrackWheel {
  public:
    virtual ~Marder_SupportRoller() {}

    /// Return the mass of the idler wheel body.
    virtual double GetMass() const override { return m_wheel_mass; }
    /// Return the moments of inertia of the idler wheel body.
    virtual const ChVector<>& GetInertia() override { return m_wheel_inertia; }
    /// Return the radius of the idler wheel.
    virtual double GetRadius() const override { return m_wheel_radius; }
    /// Return the total width of the idler wheel.
    virtual double GetWidth() const override { return m_wheel_width; }
    /// Return the gap width.
    virtual double GetGap() const override { return m_wheel_gap; }

  protected:
    Marder_SupportRoller(const std::string& name);

    virtual VehicleSide GetVehicleSide() const = 0;

    virtual std::string GetMeshFile() const = 0;

    /// Create the contact material consistent with the specified contact method.
    virtual void CreateContactMaterial(ChContactMethod contact_method) override;

    /// Add visualization of the road wheel.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    static const double m_wheel_mass;
    static const ChVector<> m_wheel_inertia;
    static const double m_wheel_radius;
    static const double m_wheel_width;
    static const double m_wheel_gap;
};

/// Road-wheel model for the M113 vehicle (left side).
class CH_MODELS_API Marder_SupportRollerLeft : public Marder_SupportRoller {
  public:
    Marder_SupportRollerLeft(int index) : Marder_SupportRoller("Marder_SupportRollerLeft_" + std::to_string(index)) {}
    ~Marder_SupportRollerLeft() {}

    virtual VehicleSide GetVehicleSide() const override { return LEFT; }

    virtual std::string GetMeshFile() const override { return GetDataFile(m_meshFile); }

  private:
    static const std::string m_meshFile;
};

/// Road-wheel model for the M113 vehicle (right side).
class CH_MODELS_API Marder_SupportRollerRight : public Marder_SupportRoller {
  public:
    Marder_SupportRollerRight(int index) : Marder_SupportRoller("Marder_SupportRollerRight_" + std::to_string(index)) {}
    ~Marder_SupportRollerRight() {}

    virtual VehicleSide GetVehicleSide() const override { return RIGHT; }

    virtual std::string GetMeshFile() const override { return GetDataFile(m_meshFile); }

  private:
    static const std::string m_meshFile;
};

/// @} vehicle_models_marder

}  // namespace marder
}  // end namespace vehicle
}  // end namespace chrono

#endif
