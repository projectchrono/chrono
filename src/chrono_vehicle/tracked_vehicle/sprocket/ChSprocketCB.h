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
// Base class for a sprocket template with gear profile composed of circular arcs
// and line segments, suitable for interaction with a continuous band track.
//
// =============================================================================

#ifndef CH_SPROCKET_CB_H
#define CH_SPROCKET_CB_H

#include "chrono_vehicle/ChApiVehicle.h"

#include "chrono_vehicle/tracked_vehicle/ChSprocket.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked_sprocket
/// @{

/// Base class for a sprocket template with gear profile composed of circular arcs
/// and line segments, suitable for interaction with a continuous band track.
class CH_VEHICLE_API ChSprocketCB : public ChSprocket {
  public:
    ChSprocketCB(const std::string& name  ///< [in] name of the subsystem
                 );

    virtual ~ChSprocketCB() {}

    /// Return the 2D gear profile.
    /// The gear profile, a ChLinePath geometric object, is made up of an arbitrary number
    /// of sub-paths of type ChLineArc or ChLineSegment sub-lines. These must be added in
    /// clockwise order, and the end of sub-path i must be coincident with beginning of
    /// sub-path i+1.
    virtual std::shared_ptr<geometry::ChLinePath> GetProfile() override;

    /// Return the custom collision callback object.
    virtual ChSystem::CustomCollisionCallback* GetCollisionCallback(
        ChTrackAssembly* track  ///< [in] pointer to containing track assembly
        ) override;

  protected:
    //// TODO: specification of sprocket profile
};

/// @} vehicle_tracked_sprocket

}  // end namespace vehicle
}  // end namespace chrono

#endif
