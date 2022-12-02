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
// M113 road wheel subsystem.
//
// =============================================================================

#ifndef M113_ROAD_WHEEL_H
#define M113_ROAD_WHEEL_H

#include <string>

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/tracked_vehicle/track_wheel/ChDoubleTrackWheel.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace m113 {

/// @addtogroup vehicle_models_m113
/// @{

/// Road-wheel model for the M113 vehicle (base class).
class CH_MODELS_API M113_RoadWheel : public ChDoubleTrackWheel {
  public:
    virtual ~M113_RoadWheel() {}

    /// Return the mass of the road wheel body.
    virtual double GetMass() const override { return m_wheel_mass; }
    /// Return the moments of inertia of the road wheel body.
    virtual const ChVector<>& GetInertia() override { return m_wheel_inertia; }
    /// Return the radius of the road wheel.
    virtual double GetRadius() const override { return m_wheel_radius; }
    /// Return the total width of the road wheel.
    virtual double GetWidth() const override { return m_wheel_width; }
    /// Return the gap width.
    virtual double GetGap() const override { return m_wheel_gap; }

  protected:
    M113_RoadWheel(const std::string& name);

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
class CH_MODELS_API M113_RoadWheelLeft : public M113_RoadWheel {
  public:
    M113_RoadWheelLeft(int index) : M113_RoadWheel("M113_RoadWheelLeft_" + std::to_string(index)) {}
    ~M113_RoadWheelLeft() {}

    virtual VehicleSide GetVehicleSide() const override { return LEFT; }

    virtual std::string GetMeshFile() const override { return GetDataFile(m_meshFile); }

  private:
    static const std::string m_meshFile;
};

/// Road-wheel model for the M113 vehicle (right side).
class CH_MODELS_API M113_RoadWheelRight : public M113_RoadWheel {
  public:
    M113_RoadWheelRight(int index) : M113_RoadWheel("M113_RoadWheelRight_" + std::to_string(index)) {}
    ~M113_RoadWheelRight() {}

    virtual VehicleSide GetVehicleSide() const override { return RIGHT; }

    virtual std::string GetMeshFile() const override { return GetDataFile(m_meshFile); }

  private:
    static const std::string m_meshFile;
};

/// @} vehicle_models_m113

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono

#endif
