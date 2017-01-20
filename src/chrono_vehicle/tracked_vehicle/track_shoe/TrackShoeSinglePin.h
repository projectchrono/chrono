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
// Single-pin track shoe with central guide constructed with data from file
// (JSON format).
//
// =============================================================================

#ifndef TRACK_SHOE_SINGLE_PIN_H
#define TRACK_SHOE_SINGLE_PIN_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeSinglePin.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked_shoe
/// @{

/// Single-pin track shoe with central guide constructed with data from file (JSON format).
class CH_VEHICLE_API TrackShoeSinglePin : public ChTrackShoeSinglePin {
  public:
    TrackShoeSinglePin(const std::string& filename);
    TrackShoeSinglePin(const rapidjson::Document& d);
    ~TrackShoeSinglePin() {}

    virtual GuidePinType GetType() const final override { return GuidePinType::CENTRAL_PIN; }

    /// Return the height of the track shoe.
    virtual double GetHeight() const override { return m_shoe_height; }
    /// Return the pitch length of the track shoe.
    /// This quantity must agree with the pitch of the sprocket gear.
    virtual double GetPitch() const override { return m_shoe_pitch; }

    /// Return the mass of the shoe body.
    virtual double GetShoeMass() const override { return m_shoe_mass; }
    /// Return the moments of inertia of the shoe body.
    virtual const ChVector<>& GetShoeInertia() const override { return m_shoe_inertia; }

    /// Return the location of the front contact cylinder.
    virtual double GetFrontCylinderLoc() const override { return m_front_cyl_loc; }
    /// Return the location of the rear contact cylinder.
    virtual double GetRearCylinderLoc() const override { return m_rear_cyl_loc; }
    /// Return the radius of the contact cylinders.
    virtual double GetCylinderRadius() const override { return m_cyl_radius; }

    /// Return dimensions and locations of the contact boxes for the shoe and guiding pin.
    /// Note that this is for contact with wheels, idler, and ground only.
    /// This contact geometry does not affect contact with the sprocket.
    virtual const ChVector<>& GetPadBoxDimensions() const override { return m_pad_box_dims; }
    virtual const ChVector<>& GetPadBoxLocation() const override { return m_pad_box_loc; }
    virtual const ChVector<>& GetGuideBoxDimensions() const override { return m_guide_box_dims; }
    virtual const ChVector<>& GetGuideBoxLocation() const override { return m_guide_box_loc; }

    /// Add visualization assets for the idler subsystem.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

  private:
    void Create(const rapidjson::Document& d);

    double m_shoe_height;
    double m_shoe_pitch;
    double m_shoe_mass;
    ChVector<> m_shoe_inertia;

    double m_cyl_radius;
    double m_front_cyl_loc;
    double m_rear_cyl_loc;

    ChVector<> m_pad_box_dims;
    ChVector<> m_pad_box_loc;
    ChVector<> m_guide_box_dims;
    ChVector<> m_guide_box_loc;

    bool m_has_mesh;
    std::string m_meshName;
    std::string m_meshFile;
};

/// @} vehicle_tracked_shoe

}  // end namespace vehicle
}  // end namespace chrono

#endif
