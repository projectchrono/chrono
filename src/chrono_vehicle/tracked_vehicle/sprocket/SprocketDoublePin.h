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
// Tracked vehicle double-pin sprocket model constructed with data from file
// (JSON format).
//
// =============================================================================

#ifndef SPROCKET_DOUBLE_PIN_H
#define SPROCKET_DOUBLE_PIN_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/tracked_vehicle/sprocket/ChSprocketDoublePin.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked_sprocket
/// @{

/// Tracked vehicle double-pin sprocket model constructed with data from file (JSON format).
class CH_VEHICLE_API SprocketDoublePin : public ChSprocketDoublePin {
  public:
    SprocketDoublePin(const std::string& filename);
    SprocketDoublePin(const rapidjson::Document& d);
    ~SprocketDoublePin() {}

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

  private:
    virtual void Create(const rapidjson::Document& d) override;

    /// Create the contact material consistent with the specified contact method.
    virtual void CreateContactMaterial(ChContactMethod contact_method) override;

    /// Add visualization of the sprocket.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    int m_num_teeth;

    double m_gear_mass;
    ChVector<> m_gear_inertia;
    double m_axle_inertia;
    double m_separation;

    double m_gear_RT;
    double m_gear_R;
    double m_gear_RA;
    double m_gear_C;
    double m_gear_W;

    double m_lateral_backlash;

    bool m_has_mesh;
    std::string m_meshFile;

    ChContactMaterialData m_mat_info;
};

/// @} vehicle_tracked_sprocket

}  // end namespace vehicle
}  // end namespace chrono

#endif
