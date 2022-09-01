// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2022 projectchrono.org
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
// Base class for an idler subsystem.  An idler consists of an idler wheel and
// a tensioner mechanism with different topologies.
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#ifndef CH_IDLER_H
#define CH_IDLER_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChChassis.h"

#include "chrono_vehicle/tracked_vehicle/ChTrackWheel.h"

namespace chrono {
namespace vehicle {

class ChTrackAssembly;

/// @addtogroup vehicle_tracked_idler
/// @{

/// Base class for an idler subsystem.
class CH_VEHICLE_API ChIdler : public ChPart {
  public:
    virtual ~ChIdler() {}

    /// Return the type of track shoe consistent with this road wheel.
    GuidePinType GetType() const { return m_type; }

    /// Return the idler wheel subsystem.
    std::shared_ptr<ChTrackWheel> GetIdlerWheel() const { return m_idler_wheel; }

    /// Return the body of the idler wheel.
    std::shared_ptr<ChBody> GetWheelBody() const { return m_idler_wheel->GetBody(); }

    /// Return a handle to the carrier body to which the idler wheel is connected.
    virtual std::shared_ptr<ChBody> GetCarrierBody() const = 0;

    /// Get the radius of the idler wheel.
    double GetWheelRadius() const { return m_idler_wheel->GetRadius(); }

    /// Initialize this idler subsystem.
    /// The idler subsystem is initialized by attaching it to the specified chassis at the specified location (with
    /// respect to and expressed in the reference frame of the chassis). It is assumed that the idler subsystem
    /// reference frame is always aligned with the chassis reference frame. A derived idler subsystem template class
    /// must extend this default implementation and specify contact geometry for the idler wheel.
    virtual void Initialize(std::shared_ptr<ChChassis> chassis,  ///< [in] associated chassis
                            const ChVector<>& location,          ///< [in] location relative to the chassis frame
                            ChTrackAssembly* track               ///< [in] containing track assembly
    );

    /// Enable/disable output for this subsystem.
    /// This function overrides the output setting for all components of this suspension assembly.
    virtual void SetOutput(bool state) override;

    /// Log current constraint violations.
    virtual void LogConstraintViolations() = 0;

  protected:
    /// Construct an idler subsystem with given name.
    ChIdler(const std::string& name);

    virtual void ExportComponentList(rapidjson::Document& jsonDocument) const override;
    
    GuidePinType m_type;                          ///< type of the track shoe matching this road wheel
    ChVector<> m_rel_loc;                         ///< idler subsystem location relative to chassis
    std::shared_ptr<ChTrackWheel> m_idler_wheel;  ///< idler-wheel subsystem
    ChTrackAssembly* m_track;                    ///< containing track assembly

    friend class ChTrackAssembly;
};

/// @} vehicle_tracked_idler

}  // end namespace vehicle
}  // end namespace chrono

#endif
