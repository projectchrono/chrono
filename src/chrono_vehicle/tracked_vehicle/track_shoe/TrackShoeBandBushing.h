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
// Band-bushing track shoe with central guide constructed with data from file
// (JSON format).
//
// =============================================================================

#ifndef TRACK_SHOE_BAND_BUSHING_H
#define TRACK_SHOE_BAND_BUSHING_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeBandBushing.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked_shoe
/// @{

/// Band-bushing track shoe with central guide constructed with data from file (JSON format).
class CH_VEHICLE_API TrackShoeBandBushing : public ChTrackShoeBandBushing {
  public:
    TrackShoeBandBushing(const std::string& filename);
    TrackShoeBandBushing(const rapidjson::Document& d);
    ~TrackShoeBandBushing() {}

    virtual GuidePinType GetType() const final override { return GuidePinType::CENTRAL_PIN; }

    /// Return the height of the track shoe.
    virtual double GetHeight() const override { return m_shoe_height; }

    /// Return the mass of the tread body.
    virtual double GetTreadMass() const override { return m_tread_mass; }

    /// Return the mass of the web.
    /// This will be equally distributed over the specified number of web segments.
    virtual double GetWebMass() const override { return m_web_mass; }

    /// Return the moments of inertia of the tread body.
    virtual const ChVector<>& GetTreadInertia() const override { return m_tread_inertias; }

    /// Return the moments of inertia of the web.
    /// These will be distributed over the specified number of web segments.
    virtual const ChVector<>& GetWebInertia() const override { return m_web_inertias; }

    /// Return the dimensions of the contact box for the guiding pin.
    /// Note that this is for contact with wheels, idler, and ground only.
    /// This contact geometry does not affect contact with the sprocket.
    virtual const ChVector<>& GetGuideBoxDimensions() const override { return m_guide_box_dims; }

    /// Return the offset (in X direction) of the guiding pin.
    virtual double GetGuideBoxOffsetX() const override { return m_guide_box_offset_x; }

    /// Return belt geometry parameters
    virtual double GetBeltWidth() const override { return m_belt_width; }

    /// Return the length of the flat tip of the tread tooth tip (in the X direction)
    virtual double GetToothTipLength() const override { return m_tooth_tip_length; }
    /// Return the length of the base of the tread tooth (in the X direction) where the tooth circular profile ends
    virtual double GetToothBaseLength() const override { return m_tooth_base_length; }
    /// Return the width of the one of the tooth profile sections of the tread tooth (in the Y direction)
    virtual double GetToothWidth() const override { return m_tooth_width; }
    /// Return the height from the base to the tip of the tread tooth profile (in the Z direction)
    virtual double GetToothHeight() const override { return m_tooth_height; }
    /// Return the radius of the tooth profile arc that connects the tooth tip and base lines
    virtual double GetToothArcRadius() const override { return m_tooth_arc_radius; }

    /// Return the number of segments that the web section is broken up into.
    virtual int GetNumWebSegments() const override { return m_num_web_segments; }
    /// Return the combined length of all of the web sections (in the X direction)
    virtual double GetWebLength() const override { return m_web_length; }
    /// Return the thickness of the web section (in the Z direction)
    virtual double GetWebThickness() const override { return m_web_thickness; }

    /// Return the length of the tread below the web area (in the X direction, tread pad for ground contact)
    virtual double GetTreadLength() const override { return m_tread_length; }
    /// Return the thickness of the tread below the web area (tread pad for ground contact)
    virtual double GetTreadThickness() const override { return m_tread_thickness; }

    /// Return bushing stiffness and damping data.
    virtual std::shared_ptr<ChVehicleBushingData> GetBushingData() const override { return m_bushingData; }

    /// Specify the name assigned to the procedurally-generated tread body visualization mesh.
    virtual const std::string& GetTreadVisualizationMeshName() const override { return m_tread_meshName; }

  private:
    virtual void Create(const rapidjson::Document& d) override;

    /// Create the 4 contact materials, consistent with the specified contact method, for interactionss with the
    /// sprocket, wheels, and ground.
    virtual void CreateContactMaterials(ChContactMethod contact_method) override;

    /// Add visualization assets for the idler subsystem.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    double m_tread_mass;
    ChVector<> m_tread_inertias;

    double m_web_mass;
    ChVector<> m_web_inertias;

    double m_shoe_height;

    double m_belt_width;
    double m_tooth_width;
    double m_tooth_tip_length;
    double m_tooth_base_length;
    double m_tooth_height;
    double m_tooth_arc_radius;
    double m_web_length;
    double m_web_thickness;
    int m_num_web_segments;
    double m_tread_length;
    double m_tread_thickness;

    ChVector<> m_guide_box_dims;
    double m_guide_box_offset_x;

    bool m_has_mesh;               ///< OBJ file provided
    std::string m_meshFile;        ///< name of OBJ file with tread visualization mesh
    std::string m_tread_meshName;  ///< name for procedurally-generated tread visualization mesh

    std::shared_ptr<ChVehicleBushingData> m_bushingData;  ///< bushing parameters

    MaterialInfo m_pad_mat_info;
    MaterialInfo m_body_mat_info;
    MaterialInfo m_guide_mat_info;
    MaterialInfo m_tooth_mat_info;
};

/// @} vehicle_tracked_shoe

}  // end namespace vehicle
}  // end namespace chrono

#endif
