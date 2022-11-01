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
// Authors: Radu Serban, Michael Taylor
// =============================================================================
//
// Base class for continuous band track shoes using rigid treads.
// Derived classes specify actual template definitions, using different models
// for the track web.
//
// =============================================================================

#ifndef CH_TRACK_SHOE_BAND_H
#define CH_TRACK_SHOE_BAND_H

#include "chrono/assets/ChTriangleMeshShape.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChSubsysDefs.h"

#include "chrono_vehicle/tracked_vehicle/ChTrackShoe.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked_shoe
/// @{

/// Base class for continuous band track shoes using rigid treads.
/// Derived classes specify actual template definitions, using different models for the track web.
class CH_VEHICLE_API ChTrackShoeBand : public ChTrackShoe {
  public:
    ChTrackShoeBand(const std::string& name  ///< [in] name of the subsystem
    );

    virtual ~ChTrackShoeBand() {}

    /// Return the pitch length of the track shoe.
    /// This quantity must agree with the pitch of the sprocket gear.
    virtual double GetPitch() const override;

    /// Initialize this track shoe subsystem.
    /// The track shoe is created within the specified system and initialized
    /// at the specified location and orientation (expressed in the global frame).
    /// This version initializes the bodies of a CB rigid-link track shoe such that
    /// the center of the track shoe subsystem is at the specified location and all
    /// bodies have the specified orientation.
    virtual void Initialize(std::shared_ptr<ChBodyAuxRef> chassis,  ///< [in] handle to the chassis body
                            const ChVector<>& location,             ///< [in] location relative to the chassis frame
                            const ChQuaternion<>& rotation          ///< [in] orientation relative to the chassis frame
                            ) override;

    /// Write the procedurally-generated tread body visualization mesh to a Wavefront OBJ file.
    void WriteTreadVisualizationMesh(const std::string& out_dir);

    /// Export the procedurally-generated tread body visualization mesh as a macro in a PovRay include file.
    void ExportTreadVisualizationMeshPovray(const std::string& out_dir);

  protected:
    /// Return the mass of the tread body.
    virtual double GetTreadMass() const = 0;

    /// Return the mass of the web.
    /// This will be equally distributed over the specified number of web segments.
    virtual double GetWebMass() const = 0;

    /// Return the moments of inertia of the tread body.
    virtual const ChVector<>& GetTreadInertia() const = 0;

    /// Return the moments of inertia of the web.
    /// These will be distributed over the specified number of web segments.
    virtual const ChVector<>& GetWebInertia() const = 0;

    /// Return the dimensions of the contact box for the guiding pin.
    /// Note that this is for contact with wheels, idler, and ground only.
    /// This contact geometry does not affect contact with the sprocket.
    virtual const ChVector<>& GetGuideBoxDimensions() const = 0;

    /// Return the offset (in X direction) of the guiding pin.
    virtual double GetGuideBoxOffsetX() const = 0;

    /// Return the location of the guiding pin center, expressed in the shoe reference frame.
    virtual ChVector<> GetLateralContactPoint() const override;

    /// Return the width of the CB track belt (in the Y direction)
    virtual double GetBeltWidth() const = 0;

    /// Return the length of the flat tip of the tread tooth tip (in the X direction)
    virtual double GetToothTipLength() const = 0;
    /// Return the length of the base of the tread tooth (in the X direction) where the tooth circular profile ends
    virtual double GetToothBaseLength() const = 0;
    /// Return the width of the one of the tooth profile sections of the tread tooth (in the Y direction)
    virtual double GetToothWidth() const = 0;
    /// Return the height from the base to the tip of the tread tooth profile (in the Z direction)
    virtual double GetToothHeight() const = 0;
    /// Return the radius of the tooth profile arc that connects the tooth tip and base lines
    virtual double GetToothArcRadius() const = 0;

    /// Return the combined length of all of the web sections (in the X direction)
    virtual double GetWebLength() const = 0;
    /// Return the thickness of the web section (in the Z direction)
    virtual double GetWebThickness() const = 0;

    /// Return the length of the tread below the web area (in the X direction, tread pad for ground contact)
    virtual double GetTreadLength() const = 0;
    /// Return the thickness of the tread below the web area (tread pad for ground contact)
    virtual double GetTreadThickness() const = 0;

    /// Specify the name assigned to the procedurally-generated tread body visualization mesh.
    virtual const std::string& GetTreadVisualizationMeshName() const = 0;

    /// Add contact geometry for the tread body.
    /// Note that this is for contact with wheels, idler, and ground only.
    /// This contact geometry does not affect contact with the sprocket.
    void AddShoeContact(ChContactMethod contact_method);

    /// Add visualization of the tread body, based on primitives corresponding to the contact shapes.
    /// Note that the "primitive" shape for the tread body is a procedurally-generated mesh.
    void AddShoeVisualization();

    /// Get index-specific color (for visualization)
    static ChColor GetColor(size_t index);

    // Contact materials
    ChContactMaterialData m_tooth_matinfo;  ///< data for contact material for teeth (sprocket interaction)
    ChContactMaterialData m_body_matinfo;   ///< date for contact material for main body (wheel interaction)
    ChContactMaterialData m_pad_matinfo;    ///< data for contact material for pad (ground interaction)
    ChContactMaterialData m_guide_matinfo;  ///< date for contact material for guide pin (wheel interaction)

  private:
    /// Utilities for creating the tooth visualization mesh.
    int ProfilePoints(std::vector<ChVector2<>>& points, std::vector<ChVector2<>>& normals);
    std::shared_ptr<ChTriangleMeshShape> ToothMesh(double y);

    ChVector2<> m_center_p;       ///< center of (+x) arc, in tread body x-z plane
    ChVector2<> m_center_m;       ///< center of (-x) arc, in tread body x-z plane
    double m_center_p_arc_start;  ///< starting angle of the (+x) arc, in tread body x-z plane
    double m_center_p_arc_end;    ///< ending angle of the (+x) arc, in tread body x-z plane
    double m_center_m_arc_start;  ///< starting angle of the (-x) arc, in tread body x-z plane
    double m_center_m_arc_end;    ///< ending angle of the (-x) arc, in tread body x-z plane

    std::shared_ptr<ChMaterialSurface> m_tooth_material;  ///< contact material for teeth (sprocket interaction)

    friend class ChSprocketBand;
    friend class SprocketBandContactCB;
};

/// Vector of handles to continuous band track shoe subsystems.
typedef std::vector<std::shared_ptr<ChTrackShoeBand>> ChTrackShoeBandList;

/// @} vehicle_tracked_shoe

}  // end namespace vehicle
}  // end namespace chrono

#endif
