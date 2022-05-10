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
// Base class for a tracked vehicle roller.
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#ifndef CH_ROLLER_H
#define CH_ROLLER_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChChassis.h"

namespace chrono {
namespace vehicle {

class ChTrackAssembly;

/// @addtogroup vehicle_tracked_roller
/// @{

/// Base class for a roller wheel subsystem.
class CH_VEHICLE_API ChRoller : public ChPart {
  public:
    virtual ~ChRoller();

    /// Return the type of track shoe consistent with this roller wheel.
    virtual GuidePinType GetType() const = 0;

    /// Get a handle to the roller body.
    std::shared_ptr<ChBody> GetBody() const { return m_wheel; }

    /// Get a handle to the revolute joint.
    std::shared_ptr<ChLinkLockRevolute> GetRevolute() const { return m_revolute; }

    /// Get the radius of the road wheel.
    virtual double GetRadius() const = 0;

    virtual double GetRollerMass() const = 0;

    virtual const ChVector<>& GetRollerInertia() const = 0;

    /// Turn on/off collision flag for the road wheel.
    void SetCollide(bool val) { m_wheel->SetCollide(val); }

    /// Initialize this roller subsystem.
    /// The roller subsystem is initialized by attaching it to the chassis body at the specified location (with respect
    /// to and expressed in the reference frame of the chassis). It is assumed that the roller subsystem reference frame
    /// is always aligned with the chassis reference frame. A derived roller subsystem template class must extend this
    /// default implementation and specify contact geometry for the roller wheel.
    virtual void Initialize(std::shared_ptr<ChChassis> chassis,  ///< [in]associated chassis
                            const ChVector<>& location,          ///< [in] location relative to the chassis frame
                            ChTrackAssembly* track               ///< [in] containing track assembly
    );

    /// Log current constraint violations.
    void LogConstraintViolations();

  protected:
    /// Construct a roller subsystem with given name.
    ChRoller(const std::string& name);

    virtual void InitializeInertiaProperties() override;
    virtual void UpdateInertiaProperties() override;

    /// Create the contact material consistent with the specified contact method.
    virtual void CreateContactMaterial(ChContactMethod contact_method) = 0;

    virtual void ExportComponentList(rapidjson::Document& jsonDocument) const override;

    virtual void Output(ChVehicleOutput& database) const override;

    ChVector<> m_rel_loc;                            ///< idler subsystem location relative to chassis
    std::shared_ptr<ChBody> m_wheel;                 ///< roller body
    std::shared_ptr<ChLinkLockRevolute> m_revolute;  ///< roller revolute joint
    std::shared_ptr<ChMaterialSurface> m_material;   ///< contact material;
    ChTrackAssembly* m_track;                        ///< containing track assembly

    friend class ChTrackAssembly;
};

/// Vector of handles to roller subsystems.
typedef std::vector<std::shared_ptr<ChRoller> > ChRollerList;

/// @} vehicle_tracked_roller

}  // end namespace vehicle
}  // end namespace chrono

#endif
