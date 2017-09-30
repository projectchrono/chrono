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
// Base class for a continuous band rigid-link track shoe (template definition).
//
// =============================================================================

#ifndef CH_TRACK_SHOE_RIGID_CB_H
#define CH_TRACK_SHOE_RIGID_CB_H

#include "chrono/assets/ChTriangleMeshShape.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChSubsysDefs.h"

#include "chrono_vehicle/tracked_vehicle/ChTrackShoe.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked_shoe
/// @{

/// Base class for a continuous band rigid-link track shoe (template definition).
class CH_VEHICLE_API ChTrackShoeRigidCB : public ChTrackShoe {
  public:
    ChTrackShoeRigidCB(const std::string& name  ///< [in] name of the subsystem
                       );

    virtual ~ChTrackShoeRigidCB() {}

    /// Get the mass of the track shoe.
    virtual double GetMass() const override;

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

    /// Initialize this track shoe system.
    /// This version specifies the locations and orientations of the tread body and of
    /// the web link bodies (relative to the chassis frame).
    void Initialize(std::shared_ptr<ChBodyAuxRef> chassis,          ///< [in] handle to chassis body
                    const std::vector<ChCoordsys<>>& component_pos  ///< [in] location & orientation of the shoe bodies
                    );

    /// Connect this track shoe to the specified neighbor.
    /// This function must be called only after both track shoes have been initialized.
    virtual void Connect(std::shared_ptr<ChTrackShoe> next  ///< [in] handle to the neighbor track shoe
                         ) override;

    /// Add visualization assets for the track shoe subsystem.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    /// Remove visualization assets for the track shoe subsystem.
    virtual void RemoveVisualizationAssets() override final;

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

    /// Return the number of segments that the web section is broken up into
    virtual int GetNumWebSegments() const = 0;
    /// Return the combined length of all of the web sections (in the X direction)
    virtual double GetWebLength() const = 0;
    /// Return the thickness of the web section (in the Z direction)
    virtual double GetWebThickness() const = 0;
    /// Return the length of just one of the web sections (in the X direction)
    double GetGetWebSegmentLength() { return m_seg_length; };
    /// Return a pointer to the web segment body with the provided index
    std::shared_ptr<ChBody> GetGetWebSegment(size_t index) { return m_web_segments[index]; };

    /// Return the length of the tread below the web area (in the X direction, tread pad that would contact with the
    /// ground)
    virtual double GetTreadLength() const = 0;
    /// Return the thickness of the tread below the web area (tread pad that would contact with the ground)
    virtual double GetTreadThickness() const = 0;

    /// Return the center of the tread circle for the arc on the positive x side of the shoe
    ChVector2<> GetTreadCircleCenterP() { return m_center_p; };
    /// Return the center of the tread circle for the arc on the negative x side of the shoe
    ChVector2<> GetTreadCircleCenterM() { return m_center_m; };
    /// Return the starting (smallest) angle for the starting point for the postive tooth arc
    double GetTreadCircleCenterPArcStart() { return m_center_p_arc_start; };
    /// Return the ending (largest) angle for the starting point for the postive tooth arc
    double GetTreadCircleCenterPArcEnd() { return m_center_p_arc_end; };
    /// Return the starting (smallest) angle for the starting point for the negative tooth arc
    double GetTreadCircleCenterMArcStart() { return m_center_m_arc_start; };
    /// Return the ending (largest) angle for the starting point for the negative tooth arc
    double GetTreadCircleCenterMArcEnd() { return m_center_m_arc_end; };

    /// Add contact geometry for the tread body.
    /// Note that this is for contact with wheels, idler, and ground only.
    /// This contact geometry does not affect contact with the sprocket.
    virtual void AddShoeContact();

    /// Add contact geometry for a web segment body.
    /// Note that this is for contact with wheels, idler, and ground only.
    /// This contact geometry does not affect contact with the sprocket.
    virtual void AddWebContact(std::shared_ptr<ChBody> segment);

    friend class ChSprocketCB;
    friend class SprocketCBContactCB;
    friend class ChTrackAssemblyRigidCB;

  private:
    /// Add visualization of the tread body, based on primitives corresponding to the contact shapes.
    void AddShoeVisualization();

    /// Add visualization of a web segment, body based on primitives corresponding to the contact shapes.
    void AddWebVisualization(std::shared_ptr<ChBody> segment);

    /// Utilities for creating the tooth mesh.
    size_t ProfilePoints(std::vector<ChVector2<>>& points, std::vector<ChVector2<>>& normals);
    std::shared_ptr<ChTriangleMeshShape> ToothMesh(double y);

    std::vector<std::shared_ptr<ChBody>> m_web_segments;  ///< handles to track shoe's web segment bodies
    double m_seg_length;                                  ///< length of a web segment
    double m_seg_mass;                                    ///< mass of a web segment
    ChVector<> m_seg_inertia;                             ///< moments of inertia of a web segment
    ChVector2<> m_center_p;                               ///< center of (+x) arc, in tread body x-z plane
    ChVector2<> m_center_m;                               ///< center of (-x) arc, in tread body x-z plane
    double m_center_p_arc_start;                          ///< starting angle of the (+x) arc, in tread body x-z plane
    double m_center_p_arc_end;                            ///< ending angle of the (+x) arc, in tread body x-z plane
    double m_center_m_arc_start;                          ///< starting angle of the (-x) arc, in tread body x-z plane
    double m_center_m_arc_end;                            ///< ending angle of the (-x) arc, in tread body x-z plane
};

/// Vector of handles to continuous band rigid-link track shoe subsystems.
typedef std::vector<std::shared_ptr<ChTrackShoeRigidCB>> ChTrackShoeRigidCBList;

/// @} vehicle_tracked_shoe

}  // end namespace vehicle
}  // end namespace chrono

#endif
