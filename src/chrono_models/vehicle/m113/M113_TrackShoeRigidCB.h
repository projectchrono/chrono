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
// M113 track shoe subsystem (rigid-web continuous band track).
//
// =============================================================================

#ifndef M113_TRACK_SHOE_DOUBLE_PIN_H
#define M113_TRACK_SHOE_DOUBLE_PIN_H

#include <string>

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeRigidCB.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace m113 {

/// @addtogroup vehicle_models_m113
/// @{

/// Double-pin track shoe subsystem for the M113 vehicle.
class CH_MODELS_API M113_TrackShoeRigidCB : public ChTrackShoeRigidCB {
  public:
    M113_TrackShoeRigidCB();
    ~M113_TrackShoeRigidCB() {}

    /// Return the type of track shoe (guiding pin).
    /// A derived class must specify the type of track shoe (which must be
    /// consistent with the idler and road wheels in the containing track assembly).
    virtual GuidePinType GetType() const override { return GuidePinType::CENTRAL_PIN; }

    /// Return the height of the track shoe.
    virtual double GetHeight() const override { return m_shoe_height; }


    /// Return the mass of the shoe body.
    virtual std::vector<double> GetShoeMasses() const override { return m_shoe_masses; }
    /// Return the moments of inertia of the shoe body.
    virtual std::vector<ChVector<>> GetShoeInertias() const override { return m_shoe_inertias; }

    /// Add visualization assets for the track shoe subsystem.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    /// Return dimensions and locations of the contact boxes for the shoe and guiding pin.
    /// Note that this is for contact with wheels, idler, and ground only.
    /// This contact geometry does not affect contact with the sprocket.
    virtual const ChVector<>& GetGuideBoxDimensions() const override { return m_guide_box_dims; }
    virtual const double GetGuideBoxOffsetX() const override{ return m_guide_box_offset_x; }


    //// TODO - Add comments here
    /// Return belt geometry parameters
    virtual const double GetBeltWidth() const override{ return m_belt_width; }
    virtual const double GetToothWidth() const override{ return m_tooth_width; }
    virtual const double GetToothTipLength() const override{ return m_tooth_tip_length; }
    virtual const double GetToothBaseLength() const override{ return m_tooth_base_length; }
    virtual const double GetToothHeight() const override{ return m_tooth_height; }
    virtual const double GetToothArcRadius() const override{ return m_tooth_arc_radius; }
    virtual ChVector2<> GetToothArcCenter() const override{ return m_tooth_arc_center; }
    virtual const double GetBushingDepth() const override{ return m_web_thickness * m_bushing_fractional_depth;}
    virtual const double GetWebThickness() const override{ return m_web_thickness; }
    virtual const int GetNumWebSegments() const override{ return m_num_web_segments; }
    virtual std::vector<double> GetWebLengths() const override{ return m_web_lengths; }
    virtual const double GetTreadThickness() const override{ return m_tread_thickness; }
    virtual const double GetTreadLength() const override{ return m_tread_length; }


  private:
    std::vector<double> m_shoe_masses;
    std::vector<ChVector<>> m_shoe_inertias;

    static const double m_shoe_height;

    static const double m_belt_width;
    static const double m_tooth_width;
    static const double m_tooth_tip_length;
    static const double m_tooth_base_length;
    static const double m_tooth_height;
    static const double m_tooth_arc_radius;
    ChVector2<> m_tooth_arc_center;
    static const double m_web_thickness;
    static const double m_bushing_fractional_depth;
    static const double m_web_length;
    static const int m_num_web_segments;
    std::vector<double> m_web_lengths;
    static const double m_tread_thickness;
    static const double m_tread_length;

    static const ChVector<> m_guide_box_dims;
    static const double m_guide_box_offset_x;

    static const std::string m_meshName;
    static const std::string m_meshFile;
};

/// @} vehicle_models_m113

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono

#endif
