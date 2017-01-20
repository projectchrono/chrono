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
// Double-pin track shoe with central guide constructed with data from file
// (JSON format).
//
// =============================================================================

#ifndef TRACK_SHOE_DOUBLE_PIN_H
#define TRACK_SHOE_DOUBLE_PIN_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeDoublePin.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked_shoe
/// @{

/// Double-pin track shoe with central guide constructed with data from file (JSON format).
class CH_VEHICLE_API TrackShoeDoublePin : public ChTrackShoeDoublePin {
  public:
    TrackShoeDoublePin(const std::string& filename);
    TrackShoeDoublePin(const rapidjson::Document& d);
    ~TrackShoeDoublePin() {}

    virtual GuidePinType GetType() const final override { return GuidePinType::CENTRAL_PIN; }

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

    double m_shoe_mass;
    ChVector<> m_shoe_inertia;
    double m_shoe_length;
    double m_shoe_width;
    double m_shoe_height;

    double m_connector_mass;
    ChVector<> m_connector_inertia;
    double m_connector_radius;
    double m_connector_length;
    double m_connector_width;

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
