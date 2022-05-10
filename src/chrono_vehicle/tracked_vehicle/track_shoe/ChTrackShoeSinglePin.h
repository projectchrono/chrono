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
// Base class for a single-pin track shoe (template definition).
// A single-pin track shoe can be either of CENTRAL_PIN or LATERAL_PIN type.
//
// =============================================================================

#ifndef CH_TRACK_SHOE_SINGLE_PIN_H
#define CH_TRACK_SHOE_SINGLE_PIN_H

#include "chrono/physics/ChLinkRSDA.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChSubsysDefs.h"

#include "chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeSegmented.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked_shoe
/// @{

/// Base class for a single-pin track shoe (template definition).
/// A single-pin track shoe can be either of CENTRAL_PIN or LATERAL_PIN type.
class CH_VEHICLE_API ChTrackShoeSinglePin : public ChTrackShoeSegmented {
  public:
    ChTrackShoeSinglePin(const std::string& name  ///< [in] name of the subsystem
                         );

    virtual ~ChTrackShoeSinglePin();

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "TrackShoeSinglePin"; }

    /// Get track tension at this track shoe.
    /// Return is the force due to the connections of this track shoe, expressed in the track shoe reference frame.
    virtual ChVector<> GetTension() const override;

    /// Initialize this track shoe subsystem.
    /// The track shoe is created within the specified system and initialized
    /// at the specified location and orientation (expressed in the global frame).
    /// A derived class must extend this default implementation and specify the contact
    /// geometry for the track shoe body.
    virtual void Initialize(std::shared_ptr<ChBodyAuxRef> chassis,  ///< [in] chassis body
                            const ChVector<>& location,             ///< [in] location relative to the chassis frame
                            const ChQuaternion<>& rotation          ///< [in] orientation relative to the chassis frame
                            ) override;

  protected:
    virtual void InitializeInertiaProperties() override;
    virtual void UpdateInertiaProperties() override;

    /// Return the mass of the shoe body.
    virtual double GetShoeMass() const = 0;

    /// Return the moments of inertia of the shoe body.
    virtual const ChVector<>& GetShoeInertia() const = 0;

    /// Return the location of the front contact cylinder.
    /// This location is relative to the shoe reference frame (in the positive x direction)
    virtual double GetFrontCylinderLoc() const = 0;

    /// Return the location of the rear contact cylinder.
    /// This location is relative to the shoe reference frame (in the negative x direction)
    virtual double GetRearCylinderLoc() const = 0;

    /// Return the radius of the contact cylinders.
    virtual double GetCylinderRadius() const = 0;

    virtual void ExportComponentList(rapidjson::Document& jsonDocument) const override;

    virtual void Output(ChVehicleOutput& database) const override;

  private:
    /// Connect this track shoe to the specified neighbor.
    /// This function must be called only after both track shoes have been initialized.
    virtual void Connect(std::shared_ptr<ChTrackShoe> next,  ///< [in] neighbor track shoe
                         ChTrackAssembly* assembly,          ///< [in] containing track assembly
                         ChChassis* chassis,                 ///< [in] associated chassis
                         bool ccw                            ///< [in] track assembled in counter clockwise direction
                         ) override final;

    virtual void EnableTrackBendingStiffness(bool val) override;

    std::shared_ptr<ChVehicleJoint> m_joint;  ///< connection to neighboring track shoe
    std::shared_ptr<ChLinkRSDA> m_rsda;       ///< optional RSDA on connection

    friend class ChSprocketSinglePin;
    friend class SprocketSinglePinContactCB;
    friend class ChTrackAssemblySinglePin;
};

/// Vector of single-pin track shoe subsystems.
typedef std::vector<std::shared_ptr<ChTrackShoeSinglePin> > ChTrackShoeSinglePinList;

/// @} vehicle_tracked_shoe

}  // end namespace vehicle
}  // end namespace chrono

#endif
