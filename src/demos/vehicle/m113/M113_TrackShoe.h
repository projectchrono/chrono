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

namespace m113 {

///
///
///
class M113_TrackShoe : public chrono::vehicle::ChSinglePinShoe {
  public:
    M113_TrackShoe(chrono::vehicle::VisualizationType vis_type);
    ~M113_TrackShoe() {}

    /// Return the type of track shoe (guiding pin).
    /// A derived class must specify the type of track shoe (which must be
    /// consistent with the idler and road wheels in the containing track assembly.
    virtual chrono::vehicle::TrackShoeType GetType() const override { return chrono::vehicle::CENTRAL_PIN; }
    /// Return the height of the track shoe.
    virtual double GetHeight() const override { return m_shoe_height; }
    /// Return the pitch length of the track shoe.
    /// This quantity must agree with the pitch of the sprocket gear.
    virtual double GetPitch() const override { return m_shoe_pitch; }

    /// Return the mass of the shoe body.
    virtual double GetShoeMass() const { return m_shoe_mass; }
    /// Return the moments of inertia of the shoe body.
    virtual chrono::ChVector<> GetShoeInertia() const override { return m_shoe_inertia; }
    /// Return the radius of the track shoe pin.
    virtual double GetPinRadius() const override { return m_pin_radius; }
    /// Return the length of the track shoe pin.
    /// This is the total pin length.
    virtual double GetPinLength() const override { return m_pin_length; }

    /// Initialize this track shoe subsystem.
    /// The track shoe is created within the specified system and initialized
    /// at the specified location and orientation (expressed in the global frame).
    /// A derived class must extend this default implementation and specify the contact
    /// geometry for the track shoe body.
    virtual void Initialize(chrono::ChSharedPtr<chrono::ChBodyAuxRef> chassis,  ///< [in] handle to the chassis body
                            const chrono::ChVector<>& location,      ///< [in] location relative to the chassis frame
                            const chrono::ChQuaternion<>& rotation,  ///< [in] orientation relative to the chassis frame
                            size_t index = 0                         ///< [in] index of this track shoe
                            ) override;

  private:
    static const double m_shoe_height;
    static const double m_shoe_pitch;
    static const double m_shoe_mass;
    static const chrono::ChVector<> m_shoe_inertia;
    static const double m_pin_radius;
    static const double m_pin_length;

    static const std::string m_meshName;
    static const std::string m_meshFile;

    chrono::vehicle::VisualizationType m_vis_type;
};

}  // end namespace m113

#endif
