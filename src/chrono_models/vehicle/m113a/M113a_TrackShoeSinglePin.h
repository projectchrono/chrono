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
// M113 track shoe subsystem (single pin).
//
// =============================================================================

#ifndef M113a_TRACK_SHOE_SINGLE_PIN_H
#define M113a_TRACK_SHOE_SINGLE_PIN_H

#include <string>

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeSinglePin.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace m113 {

class CH_MODELS_API M113a_TrackShoeSinglePin : public ChTrackShoeSinglePin {
  public:
    M113a_TrackShoeSinglePin();
    ~M113a_TrackShoeSinglePin() {}

    /// Return the type of track shoe (guiding pin).
    /// A derived class must specify the type of track shoe (which must be
    /// consistent with the idler and road wheels in the containing track assembly).
    virtual GuidePinType GetType() const override { return GuidePinType::CENTRAL_PIN; }
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

    /// Add visualization of the track shoe.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

  private:
    static const double m_shoe_height;
    static const double m_shoe_pitch;
    static const double m_shoe_mass;
    static const ChVector<> m_shoe_inertia;

    static const double m_cyl_radius;
    static const double m_front_cyl_loc;
    static const double m_rear_cyl_loc;

    static const ChVector<> m_pad_box_dims;
    static const ChVector<> m_pad_box_loc;
    static const ChVector<> m_guide_box_dims;
    static const ChVector<> m_guide_box_loc;

    static const std::string m_meshName;
    static const std::string m_meshFile;
};

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono

#endif
