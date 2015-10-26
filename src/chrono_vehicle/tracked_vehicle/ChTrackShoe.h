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
// Base class for a track shoe.
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#ifndef CH_TRACK_SHOE_H
#define CH_TRACK_SHOE_H

#include "chrono/core/ChShared.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyAuxRef.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChSubsysDefs.h"

namespace chrono {
namespace vehicle {

///
///
///
class CH_VEHICLE_API ChTrackShoe : public ChShared {
  public:
    ChTrackShoe(const std::string& name  ///< [in] name of the subsystem
                );

    virtual ~ChTrackShoe() {}

    /// Return the type of track shoe (guiding pin).
    /// A derived class must specify the type of track shoe (which must be
    /// consistent with the idler and road wheels in the containing track assembly.
    virtual TrackShoeType GetType() const = 0;

    /// Get the name identifier for this track shoe subsystem.
    const std::string& GetName() const { return m_name; }

    /// Set the name identifier for this track shoe subsystem.
    void SetName(const std::string& name) { m_name = name; }

    /// Get a handle to the shoe body.
    ChSharedPtr<ChBody> GetShoeBody() const { return m_shoe; }

    /// Return the height of the track shoe.
    virtual double GetHeight() const = 0;

    /// Return the length of the track shoe.
    /// This quantity relates to the pitch of the sprocket gear.
    virtual double GetLength() const = 0;

    /// Set contact material properties.
    /// This function must be called before Initialize().
    void SetContactMaterial(float friction_coefficient = 0.6f,    ///< [in] coefficient of friction
                            float restitution_coefficient = 0.1,  ///< [in] coefficient of restitution
                            float young_modulus = 2e5f,           ///< [in] Young's modulus of elasticity
                            float poisson_ratio = 0.3f            ///< [in] Poisson ratio
                            );

    /// Initialize this track shoe subsystem.
    /// The track shoe is created within the specified system and initialized
    /// at the specified location and orientation (expressed in the global frame).
    virtual void Initialize(ChSystem* system,               ///< [in] containing system
                            const ChVector<>& location,     ///< [in] location relative to the chassis frame
                            const ChQuaternion<>& rotation  ///< [in] orientation relative to the chassis frame
                            ) = 0;

    /// Connect this track shoe to the specified neighbor.
    /// This function must be called only after both track shoes have been initialized.
    virtual void Connect(ChSharedPtr<ChTrackShoe> next  ///< [in] handle to the neighbor track shoe
                         ) = 0;

  protected:
    std::string m_name;          ///< name of the subsystem
    ChSharedPtr<ChBody> m_shoe;  ///< handle to the shoe body

    float m_friction;       ///< coefficient of friction
    float m_restitution;    ///< coefficient of restitution
    float m_young_modulus;  ///< Young's modulus
    float m_poisson_ratio;  ///< Poisson ratio

    friend class ChTrackAssembly;
};

/// Vector of handles to track shoe subsystems.
typedef std::vector<ChSharedPtr<ChTrackShoe> > ChTrackShoeList;

}  // end namespace vehicle
}  // end namespace chrono

#endif
