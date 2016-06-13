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

#ifndef M113_TRACK_SHOE_SINGLE_PIN_H
#define M113_TRACK_SHOE_SINGLE_PIN_H

#include <string>

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeSinglePin.h"

#include "models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace m113 {

///
///
///
class CH_MODELS_API M113_TrackShoeSinglePin : public ChTrackShoeSinglePin {
  public:
    M113_TrackShoeSinglePin();
    ~M113_TrackShoeSinglePin() {}

    /// Return the type of track shoe (guiding pin).
    /// A derived class must specify the type of track shoe (which must be
    /// consistent with the idler and road wheels in the containing track assembly).
    virtual GuidePinType GetType() const override { return CENTRAL_PIN; }
    /// Return the height of the track shoe.
    virtual double GetHeight() const override { return m_shoe_height; }
    /// Return the pitch length of the track shoe.
    /// This quantity must agree with the pitch of the sprocket gear.
    virtual double GetPitch() const override { return m_shoe_pitch; }

    /// Return the mass of the shoe body.
    virtual double GetShoeMass() const { return m_shoe_mass; }
    /// Return the moments of inertia of the shoe body.
    virtual const ChVector<>& GetShoeInertia() const override { return m_shoe_inertia; }

    /// Return the location of the front contact cylinder.
    virtual double GetFrontCylinderLoc() const { return m_front_cyl_loc; }
    /// Return the location of the rear contact cylinder.
    virtual double GetRearCylinderLoc() const { return m_rear_cyl_loc; }
    /// Return the radius of the contact cylinders.
    virtual double GetCylinderRadius() const { return m_cyl_radius; }

    /// Add visualization of the track shoe.
    virtual void AddShoeVisualization() override;

    /// Add contact geometry for the track shoe.
    virtual void AddShoeContact() override;

    /// Set the track shoe visualization type.
    void SetVisType(VisualizationType vis) { m_vis_type = vis; }

  private:
    static const double m_shoe_height;
    static const double m_shoe_pitch;
    static const double m_shoe_mass;
    static const ChVector<> m_shoe_inertia;

    static const double m_cyl_radius;
    static const double m_front_cyl_loc;
    static const double m_rear_cyl_loc;

    static const std::string m_meshName;
    static const std::string m_meshFile;

    VisualizationType m_vis_type;
};

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono

#endif
