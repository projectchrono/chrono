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
    void Initialize(std::shared_ptr<ChBodyAuxRef> chassis,                    ///< [in] handle to chassis body
                    const std::vector<ChCoordsys<>> shoe_components_coordsys  ///< [in] location & orientation of the
                                                                              /// tread body followed by each web
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
    /// Return the mass of the shoe body.
    virtual std::vector<double> GetShoeMasses() const = 0;
    /// Return the moments of inertia of the shoe body.
    virtual std::vector<ChVector<>> GetShoeInertias() const = 0;

    /// Return the dimensions of the contact box for the guiding pin.
    /// Note that this is for contact with wheels, idler, and ground only.
    /// This contact geometry does not affect contact with the sprocket.
    virtual const ChVector<>& GetGuideBoxDimensions() const = 0;

    /// Return the offset (in X direction) of the guiding pin.
    virtual const double GetGuideBoxOffsetX() const = 0;

    //// TODO - Add comments here
    /// Return belt geometry parameters
    virtual const double GetBeltWidth() const = 0;
    virtual const double GetToothWidth() const = 0;
    virtual const double GetToothTipLength() const = 0;
    virtual const double GetToothBaseLength() const = 0;
    virtual const double GetToothHeight() const = 0;
    virtual const double GetToothArcRadius() const = 0;
    virtual ChVector2<> GetToothArcCenter() const = 0;
    virtual const double GetBushingDepth() const = 0;
    virtual const double GetWebThickness() const = 0;
    virtual const int GetNumWebSegments() const = 0;
    virtual std::vector<double> GetWebLengths() const = 0;
    virtual const double GetTreadThickness() const = 0;
    virtual const double GetTreadLength() const = 0;

    std::vector<std::shared_ptr<ChBody>> m_web_segments;  ///< handles to track shoe's web segment bodies

    /// Add contact geometry for the tread body.
    /// Note that this is for contact with wheels, idler, and ground only.
    /// This contact geometry does not affect contact with the sprocket.
    virtual void AddShoeContact();

    /// Add contact geometry for a web segment body.
    /// Note that this is for contact with wheels, idler, and ground only.
    /// This contact geometry does not affect contact with the sprocket.
    virtual void AddWebContact();

    friend class ChSprocketCB;
    friend class SprocketCBContactCB;
    friend class ChTrackAssemblyRigidCB;

  private:
    /// Add visualization of the tread body, based on primitives corresponding to the contact shapes.
    void AddShoeVisualization();

    /// Add visualization of a web segment, body based on primitives corresponding to the contact shapes.
    void AddWebVisualization();
};

/// Vector of handles to continuous band rigid-link track shoe subsystems.
typedef std::vector<std::shared_ptr<ChTrackShoeRigidCB>> ChTrackShoeRigidCBList;

/// @} vehicle_tracked_shoe

}  // end namespace vehicle
}  // end namespace chrono

#endif
