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
// Base class for a track wheel (road wheel, idler wheel, or roller wheel).
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#ifndef CH_TRACK_WHEEL_H
#define CH_TRACK_WHEEL_H

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChLinkLock.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChPart.h"

namespace chrono {
namespace vehicle {

class ChTrackAssembly;

/// @addtogroup vehicle_tracked_suspension
/// @{

/// Base class for a track wheel subsystem.
class CH_VEHICLE_API ChTrackWheel : public ChPart {
  public:
    virtual ~ChTrackWheel();

    /// Return the type of track shoe consistent with this track wheel.
    virtual GuidePinType GetType() const = 0;

    /// Get a handle to the wheel body.
    std::shared_ptr<ChBody> GetWheelBody() const { return m_wheel; }

    /// Get a handle to the revolute joint.
    std::shared_ptr<ChLinkLockRevolute> GetRevolute() const { return m_revolute; }

    /// Return the mass of the track wheel body.
    virtual double GetWheelMass() const = 0;
  
    /// Return the moments of inertia of the track wheel body.
    virtual const ChVector<>& GetWheelInertia() = 0;

    /// Get the radius of the track wheel.
    virtual double GetWheelRadius() const = 0;

    /// Turn on/off collision flag for the track wheel.
    void SetCollide(bool val) { m_wheel->SetCollide(val); }

    /// Initialize this track wheel subsystem.
    /// The track wheel subsystem is initialized by attaching it to the specified
    /// carrier body at the specified location (with respect to and expressed in the
    /// reference frame of the chassis). It is assumed that the track wheel subsystem
    /// reference frame is always aligned with the chassis reference frame.
    /// A derived track wheel subsystem template class must extend this default
    /// implementation and specify contact geometry for the track wheel.
    virtual void Initialize(std::shared_ptr<ChBodyAuxRef> chassis,  ///< [in] handle to the chassis body
                            std::shared_ptr<ChBody> carrier,        ///< [in] handle to the carrier body
                            const ChVector<>& location,             ///< [in] location relative to the chassis frame
                            ChTrackAssembly* track                  ///< [in] containing track assembly
    );

    /// Log current constraint violations.
    void LogConstraintViolations();

  protected:
    /// Construct a road-wheel subsystem with given name.
    ChTrackWheel(const std::string& name);

    virtual void InitializeInertiaProperties() override;
    virtual void UpdateInertiaProperties() override;

    /// Create the contact material consistent with the specified contact method.
    virtual void CreateContactMaterial(ChContactMethod contact_method) = 0;

    virtual void ExportComponentList(rapidjson::Document& jsonDocument) const override;

    virtual void Output(ChVehicleOutput& database) const override;

    std::shared_ptr<ChBody> m_wheel;                 ///< handle to the track wheel body
    std::shared_ptr<ChLinkLockRevolute> m_revolute;  ///< handle to wheel revolute joint
    std::shared_ptr<ChMaterialSurface> m_material;   ///< contact material;
    ChTrackAssembly* m_track;                        ///< containing track assembly

    friend class ChTrackAssembly;
    friend class ChTrackSuspension;
};

/// @} vehicle_tracked_suspension

}  // end namespace vehicle
}  // end namespace chrono

#endif
