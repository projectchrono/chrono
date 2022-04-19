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
// Base class for a double-pin track shoe (template definition).
//
// =============================================================================

#ifndef CH_TRACK_SHOE_DOUBLE_PIN_H
#define CH_TRACK_SHOE_DOUBLE_PIN_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChSubsysDefs.h"

#include "chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeSegmented.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked_shoe
/// @{

/// Base class for a double-pin track shoe (template definition).
class CH_VEHICLE_API ChTrackShoeDoublePin : public ChTrackShoeSegmented {
  public:
    ChTrackShoeDoublePin(const std::string& name  ///< [in] name of the subsystem
                         );

    virtual ~ChTrackShoeDoublePin();

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "TrackShoeDoublePin"; }

    /// Return the pitch length of the track shoe.
    /// This quantity must agree with the pitch of the sprocket gear.
    virtual double GetPitch() const override;

    /// Initialize this track shoe subsystem.
    /// The track shoe is created within the specified system and initialized
    /// at the specified location and orientation (expressed in the global frame).
    /// This version initializes the bodies of a double-pin track shoe such that
    /// the center of the track shoe subsystem is at the specified location and all
    /// bodies have the specified orientation.
    virtual void Initialize(std::shared_ptr<ChBodyAuxRef> chassis,  ///< [in] chassis body
                            const ChVector<>& location,             ///< [in] location relative to the chassis frame
                            const ChQuaternion<>& rotation          ///< [in] orientation relative to the chassis frame
                            ) override;

    /// Initialize this track shoe system.
    /// This version specifies the locations and orientations of the shoe body and of
    /// the connector bodies (relative to the chassis frame).
    void Initialize(std::shared_ptr<ChBodyAuxRef> chassis,  ///< [in] chassis body
                    const ChVector<>& loc_shoe,             ///< [in] location of shoe body
                    const ChQuaternion<>& rot_shoe,         ///< [in] orientation of shoe body
                    const ChVector<>& loc_connector_L,      ///< [in] location of left connector body
                    const ChVector<>& loc_connector_R,      ///< [in] location of right connector body
                    const ChQuaternion<>& rot_connector     ///< [in] orientation of connector bodies
                    );

    /// Add visualization assets for the track shoe subsystem.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    /// Remove visualization assets for the track shoe subsystem.
    virtual void RemoveVisualizationAssets() override final;

  protected:
    virtual void InitializeInertiaProperties() override;
    virtual void UpdateInertiaProperties() override;

    /// Return the mass of the shoe body.
    virtual double GetShoeMass() const = 0;
    /// Return the moments of inertia of the shoe body.
    virtual const ChVector<>& GetShoeInertia() const = 0;
    /// Return shoe length (distance between pins).
    virtual double GetShoeLength() const = 0;
    /// Return shoe width (separation between connectors).
    virtual double GetShoeWidth() const = 0;

    /// Return the mass of a connector body.
    virtual double GetConnectorMass() const = 0;
    /// Return the moments of inertia of a connector body.
    virtual const ChVector<>& GetConnectorInertia() const = 0;
    /// Return the length of a connector body (distance between pins).
    virtual double GetConnectorLength() const = 0;
    /// Return the width of a connector body (for visualization only).
    virtual double GetConnectorWidth() const = 0;
    /// Return the radius of a connector body.
    virtual double GetConnectorRadius() const = 0;

    virtual void ExportComponentList(rapidjson::Document& jsonDocument) const override;

    virtual void Output(ChVehicleOutput& database) const override;

    std::shared_ptr<ChBody> m_connector_L;  ///< left connector body
    std::shared_ptr<ChBody> m_connector_R;  ///< right connector body

    std::shared_ptr<ChVehicleJoint> m_revolute_L;      ///< shoe - left connector joint
    std::shared_ptr<ChVehicleJoint> m_revolute_R;      ///< shoe - right connector joint
    std::shared_ptr<ChLinkRSDA> m_rsda_L;              ///< optional RSDA on left revolute
    std::shared_ptr<ChLinkRSDA> m_rsda_R;              ///< optional RSDA on right revolute

    std::shared_ptr<ChVehicleJoint> m_connection_joint_L;    ///< connection to neighboring track shoe
    std::shared_ptr<ChVehicleJoint> m_connection_joint_R;    ///< connection to neighboring track shoe
    std::shared_ptr<ChLinkRSDA> m_connection_rsda_L;         ///< optional RSDA on connection
    std::shared_ptr<ChLinkRSDA> m_connection_rsda_R;         ///< optional RSDA on connection

  private:
    /// Connect this track shoe to the specified neighbor.
    /// This function must be called only after both track shoes have been initialized.
    virtual void Connect(std::shared_ptr<ChTrackShoe> next,  ///< [in] neighbor track shoe
                         ChTrackAssembly* assembly,          ///< [in] containing track assembly
                         ChChassis* chassis,                 ///< [in] associated chassis
                         bool ccw                            ///< [in] track assembled in counter clockwise direction
                         ) override final;

    /// Add visualization of a connector body based on primitives corresponding to the contact shapes.
    void AddConnectorVisualization(std::shared_ptr<ChBody> connector, VisualizationType vis);

    virtual void EnableTrackBendingStiffness(bool val) override final;

    friend class ChSprocketDoublePin;
    friend class SprocketDoublePinContactCB;
    friend class ChTrackAssemblyDoublePin;
};

/// Vector of double-pin track shoe subsystems.
typedef std::vector<std::shared_ptr<ChTrackShoeDoublePin> > ChTrackShoeDoublePinList;

/// @} vehicle_tracked_shoe

}  // end namespace vehicle
}  // end namespace chrono

#endif
