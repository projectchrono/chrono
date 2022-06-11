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
// Authors: Radu Serban, Michael Taylor
// =============================================================================
//
// M113 track shoe subsystem (continuous band with rigid links).
//
// =============================================================================

#ifndef M113_TRACK_SHOE_BAND_BUSHING_H
#define M113_TRACK_SHOE_BAND_BUSHING_H

#include <string>

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeBandBushing.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace m113 {

/// @addtogroup vehicle_models_m113
/// @{

/// Continuous band rigid-link track shoe subsystem for the M113 vehicle.
class CH_MODELS_API M113_TrackShoeBandBushing : public ChTrackShoeBandBushing {
  public:
    M113_TrackShoeBandBushing(const std::string& name);
    ~M113_TrackShoeBandBushing() {}

    /// Return the type of track shoe (guiding pin).
    /// A derived class must specify the type of track shoe (which must be
    /// consistent with the idler and road wheels in the containing track assembly).
    virtual GuidePinType GetType() const override { return GuidePinType::CENTRAL_PIN; }

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
    /// Create the 4 contact materials, consistent with the specified contact method, for interactionss with the
    /// sprocket, wheels, and ground.
    virtual void CreateContactMaterials(ChContactMethod contact_method) override;

    /// Add visualization assets for the track shoe subsystem.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    std::shared_ptr<ChVehicleBushingData> m_bushingData;

    static const double m_tread_mass;
    static const ChVector<> m_tread_inertias;

    static const double m_web_mass;
    static const ChVector<> m_web_inertias;

    static const double m_shoe_height;

    static const double m_belt_width;
    static const double m_tooth_width;
    static const double m_tooth_tip_length;
    static const double m_tooth_base_length;
    static const double m_tooth_height;
    static const double m_tooth_arc_radius;
    static const double m_web_length;
    static const double m_web_thickness;
    static const int m_num_web_segments;
    static const double m_tread_length;
    static const double m_tread_thickness;

    static const ChVector<> m_guide_box_dims;
    static const double m_guide_box_offset_x;

    static const std::string m_meshFile;        // name of OBJ file with tread visualization mesh
    static const std::string m_tread_meshName;  // name for procedurally-generated tread visualization mesh
};

/// @} vehicle_models_m113

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono

#endif
