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
// M113 track shoe subsystem (double pin).
//
// =============================================================================

#ifndef M113_TRACK_SHOE_DOUBLE_PIN_H
#define M113_TRACK_SHOE_DOUBLE_PIN_H

#include <string>

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeDoublePin.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace m113 {

/// @addtogroup vehicle_models_m113
/// @{

/// Double-pin track shoe subsystem for the M113 vehicle.
class CH_MODELS_API M113_TrackShoeDoublePin : public ChTrackShoeDoublePin {
  public:
    M113_TrackShoeDoublePin(const std::string& name, DoublePinTrackShoeType topology);
    ~M113_TrackShoeDoublePin() {}

    /// Return the type of track shoe (guiding pin).
    /// A derived class must specify the type of track shoe (which must be
    /// consistent with the idler and road wheels in the containing track assembly).
    virtual GuidePinType GetType() const override { return GuidePinType::CENTRAL_PIN; }

    /// Return the height of the track shoe.
    virtual double GetHeight() const override { return m_shoe_height; }

    /// Return the mass of the shoe body.
    virtual double GetShoeMass() const override { return m_shoe_mass; }
    /// Return the moments of inertia of the shoe body.
    virtual const ChVector<>& GetShoeInertia() const override { return m_shoe_inertia; }
    /// Return shoe length (distance between pins).
    virtual double GetShoeLength() const override { return m_shoe_length; }
    /// Return shoe width (separation between connectors).
    virtual double GetShoeWidth() const override { return m_shoe_width; }

    /// Return the location of the guiding pin center, expressed in the shoe reference frame.
    virtual ChVector<> GetLateralContactPoint() const override { return m_pin_center; }

    /// Return the mass of a connector body.
    virtual double GetConnectorMass() const override { return m_connector_mass; }
    /// Return the moments of inertia of a connector body.
    virtual const ChVector<>& GetConnectorInertia() const override { return m_connector_inertia; }
    /// Return the length of a connector body.
    virtual double GetConnectorLength() const override { return m_connector_length; }
    /// Return the radius of a connector body.
    virtual double GetConnectorRadius() const override { return m_connector_radius; }
    /// Return the width of a connector body (visualization only).
    virtual double GetConnectorWidth() const override { return m_connector_width; }

    /// Return contact geometry and material for interaction with terrain.
    virtual ChVehicleGeometry GetGroundContactGeometry() const override { return m_ground_geometry; }

  private:
    static const double m_shoe_mass;
    static const ChVector<> m_shoe_inertia;
    static const double m_shoe_length;
    static const double m_shoe_width;
    static const double m_shoe_height;

    static const ChVector<> m_pin_center;

    static const double m_connector_mass;
    static const ChVector<> m_connector_inertia;
    static const double m_connector_radius;
    static const double m_connector_length;
    static const double m_connector_width;

    ChVehicleGeometry m_ground_geometry;
};

/// @} vehicle_models_m113

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono

#endif
