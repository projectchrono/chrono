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
// Base class for a rigid-link track shoe in a continuous band track.
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

/// Base class for a rigid-link track shoe in a continuous band track (template definition).
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
    /// This version initializes the bodies of a rigid-link track shoe such that
    /// the center of the track shoe subsystem is at the specified location and all
    /// bodies have the specified orientation.
    virtual void Initialize(std::shared_ptr<ChBodyAuxRef> chassis,  ///< [in] handle to the chassis body
                            const ChVector<>& location,             ///< [in] location relative to the chassis frame
                            const ChQuaternion<>& rotation          ///< [in] orientation relative to the chassis frame
                            ) override;

    /// Initialize this track shoe system.
    /// This version specifies the locations and orientations of the shoe body and of
    /// the connector bodies (relative to the chassis frame).
    void Initialize(std::shared_ptr<ChBodyAuxRef> chassis,        ///< [in] handle to chassis body
                    const ChVector<>& loc_shoe,                   ///< [in] location of shoe body
                    const ChQuaternion<>& rot_shoe,               ///< [in] orientation of shoe body
                    const ChVector<>& loc_tooth,                  ///< [in] location of tooth body
                    const ChQuaternion<>& rot_tooth,              ///< [in] orientation of tooth body
                    const std::vector<ChVector<>>& loc_links,     ///< [in] locations of link bodies
                    const std::vector<ChQuaternion<>>& rot_links  ///< [in] orientations of link bodies
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
    /// Return the number of rigid links
    virtual int GetNumLinks() const = 0;

    /// Return the mass of the shoe body.
    virtual double GetShoeMass() const = 0;
    /// Return the moments of inertia of the shoe body.
    virtual const ChVector<>& GetShoeInertia() const = 0;
    /// Return shoe length (distance between pins).
    virtual double GetShoeLength() const = 0;
    /// Return shoe width.
    virtual double GetShoeWidth() const = 0;

    /// Return the mass of the tooth body.
    virtual double GetToothMass() const = 0;
    /// Return the moments of inertia of the tooth body.
    virtual const ChVector<>& GetToothInertia() const = 0;

    //// TODO:  specification of tooth geometry (for contact with sprocket)

    /// Return the mass of a link body.
    virtual double GetLinkMass() const = 0;
    /// Return the moments of inertia of a connector body.
    virtual const ChVector<>& GetLinkInertia() const = 0;
    /// Return link length (distance between pins).
    virtual double GetLinkLength() const = 0;
    /// Return link height.
    virtual double GetLinkHeight() const = 0;

    //// TODO: specification of bushing elements (link-link, link-tooth)

    /// Add contact geometry for the track shoe.
    /// Note that this is for contact with wheels, idler, and ground only.
    /// This contact geometry does not affect contact with the sprocket.
    virtual void AddShoeContact();

    /// Add contact geometry for the link bodies.
    /// Note that this is for contact with wheels, idler, and ground only.
    /// This contact geometry does not affect contact with the sprocket.
    virtual void AddLinkContact(int link_index);

    std::shared_ptr<ChBody> m_tooth;               ///< tooth body
    std::vector<std::shared_ptr<ChBody>> m_links;  ///< connecting rigid links

    friend class ChSprocketCB;
    friend class SprocketCBContactCB;
    friend class ChTrackAssemblyRigidCB;

  private:
      /// Add visualization of the tooth body.
      void AddToothVisualization();

    /// Add visualization of the shoe body.
    void AddShoeVisualization();

    /// Add visualization of a connector body.
    void AddLinkVisualization(int link_index);
};

/// Vector of handles to CB rigid-link track shoe subsystems.
typedef std::vector<std::shared_ptr<ChTrackShoeRigidCB> > ChTrackShoeRigidCBList;

/// @} vehicle_tracked_shoe

}  // end namespace vehicle
}  // end namespace chrono

#endif
