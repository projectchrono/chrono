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
// Authors: Radu Serban, Michael Taylor
// =============================================================================
//
// M113 sprocket subsystem (continuous band track).
//
// =============================================================================

#ifndef M113_SPROCKET_CB_H
#define M113_SPROCKET_CB_H

#include <string>

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/tracked_vehicle/sprocket/ChSprocketBandBushing.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace m113 {

/// @addtogroup vehicle_models_m113
/// @{

/// M113 sprocket subsystem for continuous band track (base class).
class CH_MODELS_API M113_SprocketBandBushing : public ChSprocketBandBushing {
  public:
    virtual ~M113_SprocketBandBushing() {}

    /// Get the number of teeth of the gear.
    virtual int GetNumTeeth() const override { return m_num_teeth; }

    /// Get the radius of the gear.
    /// This quantity is used during the automatic track assembly.
    virtual double GetAssemblyRadius() const override { return m_gear_RA; }

    /// Return the mass of the gear body.
    virtual double GetGearMass() const override { return m_gear_mass; }
    /// Return the moments of inertia of the gear body.
    virtual const ChVector<>& GetGearInertia() override { return m_gear_inertia; }
    /// Return the inertia of the axle shaft.
    virtual double GetAxleInertia() const override { return m_axle_inertia; }
    /// Return the distance between the two gear profiles.
    virtual double GetSeparation() const override { return m_separation; }

    /// Return the radius of the addendum circle.
    virtual double GetOuterRadius() const override { return m_gear_outer_radius; }
    /// Return the base width of the sprocket profile
    /// length of the chord where the tooth profile meets the sprocket's outer radius
    virtual double GetBaseWidth() const override { return m_gear_base_width; }
    /// Return the width of the inner tip of the sprocket profile
    virtual double GetTipWidth() const override { return m_gear_tip_width; }
    /// Return the depth of the sprocket profile
    /// measured as the distance from the center of the profile tip line to the
    /// center of the base width line
    virtual double GetToothDepth() const override { return m_gear_tooth_depth; }
    /// Return the radius of the (concave) tooth circular arc.
    virtual double GetArcRadius() const override { return m_gear_arc_radius; }

    /// Return the total width of the sprocket guiding wheel that acts similar to another road wheel
    virtual double GetGuideWheelWidth() const override { return m_gear_guide_wheel_width; }

    /// Return the gap width of the sprocket guiding wheel that acts similar to another road wheel
    virtual double GetGuideWheelGap() const override { return m_gear_guide_wheel_gap; }

    /// Add visualization of the sprocket.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

  protected:
    M113_SprocketBandBushing(const std::string& name);

    virtual std::string GetMeshName() const = 0;
    virtual std::string GetMeshFile() const = 0;

    static const int m_num_teeth;

    static const double m_gear_mass;
    static const ChVector<> m_gear_inertia;
    static const double m_axle_inertia;
    static const double m_separation;

    // Gear profile data
    static const double m_gear_outer_radius;
    static const double m_gear_base_width;
    static const double m_gear_tip_width;
    static const double m_gear_tooth_depth;
    static const double m_gear_arc_radius;
    static const double m_gear_RA;

    static const double m_gear_guide_wheel_width;
    static const double m_gear_guide_wheel_gap;
};

/// M113 sprocket subsystem for continuous band track (left side).
class CH_MODELS_API M113_SprocketBandBushingLeft : public M113_SprocketBandBushing {
  public:
    M113_SprocketBandBushingLeft() : M113_SprocketBandBushing("M113_SprocketLeft") {}
    ~M113_SprocketBandBushingLeft() {}

    virtual std::string GetMeshName() const override { return m_meshName; }
    virtual std::string GetMeshFile() const override { return GetDataFile(m_meshFile); }

  private:
    static const std::string m_meshName;
    static const std::string m_meshFile;
};

/// M113 sprocket subsystem for continuous band track (right side).
class CH_MODELS_API M113_SprocketBandBushingRight : public M113_SprocketBandBushing {
  public:
    M113_SprocketBandBushingRight() : M113_SprocketBandBushing("M113_SprocketRight") {}
    ~M113_SprocketBandBushingRight() {}

    virtual std::string GetMeshName() const override { return m_meshName; }
    virtual std::string GetMeshFile() const override { return GetDataFile(m_meshFile); }

  private:
    static const std::string m_meshName;
    static const std::string m_meshFile;
};

/// @} vehicle_models_m113

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono

#endif
