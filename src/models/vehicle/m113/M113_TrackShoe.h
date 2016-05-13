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

#ifndef M113_TRACK_SHOE_H
#define M113_TRACK_SHOE_H

#include <string>

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/ChSinglePinShoe.h"

#include "models/ChApiModels.h"

namespace m113 {

///
///
///
class CH_MODELS_API M113_TrackShoe : public chrono::vehicle::ChSinglePinShoe {
  public:
    M113_TrackShoe();
    ~M113_TrackShoe() {}

    /// Return the type of track shoe (guiding pin).
    /// A derived class must specify the type of track shoe (which must be
    /// consistent with the idler and road wheels in the containing track assembly).
    virtual chrono::vehicle::TrackShoeType GetType() const override { return chrono::vehicle::CENTRAL_PIN; }
    /// Return the height of the track shoe.
    virtual double GetHeight() const override { return m_shoe_height; }
    /// Return the pitch length of the track shoe.
    /// This quantity must agree with the pitch of the sprocket gear.
    virtual double GetPitch() const override { return m_shoe_pitch; }

    /// Return the mass of the shoe body.
    virtual double GetShoeMass() const { return m_shoe_mass; }
    /// Return the moments of inertia of the shoe body.
    virtual const chrono::ChVector<>& GetShoeInertia() const override { return m_shoe_inertia; }

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
    void SetVisType(chrono::vehicle::VisualizationType vis) { m_vis_type = vis; }

  private:
    static const double m_shoe_height;
    static const double m_shoe_pitch;
    static const double m_shoe_mass;
    static const chrono::ChVector<> m_shoe_inertia;

    static const double m_cyl_radius;
    static const double m_front_cyl_loc;
    static const double m_rear_cyl_loc;

    static const std::string m_meshName;
    static const std::string m_meshFile;

    chrono::vehicle::VisualizationType m_vis_type;
};

}  // end namespace m113

#endif
