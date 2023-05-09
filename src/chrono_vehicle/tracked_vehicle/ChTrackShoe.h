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
// Base class for a track shoe.
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#ifndef CH_TRACK_SHOE_H
#define CH_TRACK_SHOE_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChChassis.h"
#include "chrono/core/ChException.h"

namespace chrono {
namespace vehicle {

class ChTrackAssembly;

/// @addtogroup vehicle_tracked_shoe
/// @{

/// Base class for a track shoe.
class CH_VEHICLE_API ChTrackShoe : public ChPart {
  public:
    virtual ~ChTrackShoe();

    /// Return the type of track shoe (guiding pin).
    /// A derived class must specify the type of track shoe (which must be
    /// consistent with the idler and road wheels in the containing track assembly.
    virtual GuidePinType GetType() const = 0;

    /// Get the index of this track shoe within its containing track assembly.
    size_t GetIndex() const { return m_index; }

    /// Get the shoe body.
    std::shared_ptr<ChBody> GetShoeBody() const { return m_shoe; }

    /// Get track tension at this track shoe.
    /// Return is the force due to the connections of this track shoe, expressed in the track shoe reference frame.
    virtual ChVector<> GetTension() const = 0;

    /// Return the height of the track shoe.
    virtual double GetHeight() const = 0;

    /// Return the pitch length of the track shoe.
    /// This quantity must agree with the pitch of the sprocket gear.
    virtual double GetPitch() const = 0;

    /// Return the location for lateral contact with the sprocket, expressed in the shoe reference frame.
    /// This point, which must be in the median plane of the track shoe, is used to enforce lateral contact with the
    /// sprocket as a detracking prevention mechanism. For track shoes with a central guiding pin, this can be the
    /// center of the guiding pin collision shape.
    virtual ChVector<> GetLateralContactPoint() const = 0;

    /// Return contact geometry and material for interaction with terrain.
    virtual ChVehicleGeometry GetGroundContactGeometry() const { return ChVehicleGeometry(); }

    /// Turn on/off collision flag for the shoe body.
    void SetCollide(bool val) { m_shoe->SetCollide(val); }

    /// Initialize this track shoe subsystem.
    /// The track shoe is created within the specified system and initialized
    /// at the specified location and orientation (expressed in the global frame).
    /// All actual work is deferred to derived classes (subsystem templates) which
    /// must create the bodies, joints, etc.  In addition, a derived class must set
    /// the track shoe body's identifier to BodyID::SHOES.
    virtual void Initialize(std::shared_ptr<ChBodyAuxRef> chassis,  ///< [in] handle to the chassis body
                            const ChVector<>& location,             ///< [in] location relative to the chassis frame
                            const ChQuaternion<>& rotation          ///< [in] orientation relative to the chassis frame
    );

    /// Set the index of this track shoe within its containing track assembly.
    void SetIndex(size_t index) { m_index = index; }

    /// Connect this track shoe to the specified neighbor.
    /// This function must be called only after all track shoes have been initialized.
    virtual void Connect(std::shared_ptr<ChTrackShoe> next,  ///< [in] handle to the neighbor track shoe
                         ChTrackAssembly* assembly,          ///< [in] containing track assembly
                         ChChassis* chassis,                 ///< [in] associated chassis
                         bool ccw                            ///< [in] track assembled in counter clockwise direction
                         ) = 0;

  protected:
    /// Construct a track shoe subsystem with given name.
    ChTrackShoe(const std::string& name);

    size_t m_index;                  ///< index of this track shoe within its containing track assembly
    std::shared_ptr<ChBody> m_shoe;  ///< handle to the shoe body

    friend class ChTrackAssembly;
};

/// Vector of handles to track shoe subsystems.
typedef std::vector<std::shared_ptr<ChTrackShoe> > ChTrackShoeList;

/// @} vehicle_tracked_shoe

}  // end namespace vehicle
}  // end namespace chrono

#endif
