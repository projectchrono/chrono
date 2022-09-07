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
// M113 sprocket subsystem (double pin).
//
// =============================================================================

#ifndef M113_SPROCKET_DOUBLE_PIN_H
#define M113_SPROCKET_DOUBLE_PIN_H

#include <string>

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/tracked_vehicle/sprocket/ChSprocketDoublePin.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace m113 {

/// @addtogroup vehicle_models_m113
/// @{

/// M113 sprocket subsystem, suitable for interaction with double-pin track shoes (base class).
class CH_MODELS_API M113_SprocketDoublePin : public ChSprocketDoublePin {
  public:
    virtual ~M113_SprocketDoublePin() {}

    /// Get the number of teeth of the gear.
    virtual int GetNumTeeth() const override { return m_num_teeth; }

    /// Get the radius of the gear.
    /// This quantity is used during the automatic track assembly.
    virtual double GetAssemblyRadius() const override { return m_gear_RA; }

    /// Get the addendum radius.
    /// This quantity is an average radius for sprocket-track engagement used to estimate longitudinal slip.
    virtual double GetAddendumRadius() const override { return m_gear_RT; }

    /// Return the mass of the gear body.
    virtual double GetGearMass() const override { return m_gear_mass; }
    /// Return the moments of inertia of the gear body.
    virtual const ChVector<>& GetGearInertia() override { return m_gear_inertia; }
    /// Return the inertia of the axle shaft.
    virtual double GetAxleInertia() const override { return m_axle_inertia; }
    /// Return the distance between the two gear profiles.
    virtual double GetSeparation() const override { return m_separation; }

    /// Return the radius of the addendum circle.
    virtual double GetOuterRadius() const override { return m_gear_RT; }
    /// Return the radius of the (concave) tooth circular arc.
    virtual double GetArcRadius() const override { return m_gear_R; }
    /// Return height of arc center.
    virtual double GetArcCenterHeight() const override { return m_gear_C; }
    /// Return offset of arc center.
    virtual double GetArcCenterOffset() const override { return m_gear_W; }

    /// Return the allowed backlash (play) before lateral contact with track shoes is enabled (to prevent detracking).
    virtual double GetLateralBacklash() const override { return m_lateral_backlash; }

  protected:
    M113_SprocketDoublePin(const std::string& name);

    /// Create the contact material consistent with the specified contact method.
    virtual void CreateContactMaterial(ChContactMethod contact_method) override;

    /// Add visualization of the sprocket.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    virtual std::string GetMeshFile() const = 0;

    static const int m_num_teeth;

    static const double m_gear_mass;
    static const ChVector<> m_gear_inertia;
    static const double m_axle_inertia;
    static const double m_separation;

    // Gear profile data
    static const double m_gear_RT;
    static const double m_gear_R;
    static const double m_gear_C;
    static const double m_gear_W;
    static const double m_gear_RA;

    static const double m_lateral_backlash;
};

/// M113 sprocket subsystem, suitable for interaction with double-pin track shoes (left side).
class CH_MODELS_API M113_SprocketDoublePinLeft : public M113_SprocketDoublePin {
  public:
    M113_SprocketDoublePinLeft() : M113_SprocketDoublePin("M113_SprocketLeft") {}
    ~M113_SprocketDoublePinLeft() {}

    virtual std::string GetMeshFile() const override { return GetDataFile(m_meshFile); }

  private:
    static const std::string m_meshFile;
};

/// M113 sprocket subsystem, suitable for interaction with double-pin track shoes (right side).
class CH_MODELS_API M113_SprocketDoublePinRight : public M113_SprocketDoublePin {
  public:
    M113_SprocketDoublePinRight() : M113_SprocketDoublePin("M113_SprocketRight") {}
    ~M113_SprocketDoublePinRight() {}

    virtual std::string GetMeshFile() const override { return GetDataFile(m_meshFile); }

  private:
    static const std::string m_meshFile;
};

/// @} vehicle_models_m113

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono

#endif
