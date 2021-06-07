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
// Base class for a sprocket template with gear profile composed of circular arcs
// and line segments, suitable for interaction with a continuous band track.
//
// =============================================================================

#ifndef CH_SPROCKET_BAND_H
#define CH_SPROCKET_BAND_H

#include "chrono_vehicle/ChApiVehicle.h"

#include "chrono_vehicle/tracked_vehicle/ChSprocket.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked_sprocket
/// @{

/// Base class for a sprocket template with gear profile composed of circular arcs
/// and a flat seat. This sprocket type is suitable for interaction with a continuous
/// band track.
class CH_VEHICLE_API ChSprocketBand : public ChSprocket {
  public:
    ChSprocketBand(const std::string& name  ///< [in] name of the subsystem
    );

    virtual ~ChSprocketBand() {}

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "SprocketBand"; }

    /// Return the 2D gear profile.
    /// The gear profile, a ChLinePath geometric object, is made up of an arbitrary number
    /// of sub-paths of type ChLineArc or ChLineSegment sub-lines. These must be added in
    /// clockwise order, and the end of sub-path i must be coincident with beginning of
    /// sub-path i+1.
    virtual std::shared_ptr<geometry::ChLinePath> GetProfile() const override;

    /// Return the custom collision callback object.
    virtual std::shared_ptr<ChSystem::CustomCollisionCallback> GetCollisionCallback(
        ChTrackAssembly* track  ///< [in] pointer to containing track assembly
        ) override;

  protected:
    /// Return the radius of the outer sprocket circle.
    virtual double GetOuterRadius() const = 0;

    /// Return the base width of the sprocket profile
    /// length of the chord where the tooth profile meets the sprocket's outer radius
    virtual double GetBaseWidth() const = 0;

    /// Return the width of the inner tip of the sprocket profile
    virtual double GetTipWidth() const = 0;

    /// Return the depth of the sprocket profile
    /// measured as the distance from the center of the profile tip line to the
    /// center of the base width line
    virtual double GetToothDepth() const = 0;

    /// Return the radius of the (concave) tooth circular arcs.
    virtual double GetArcRadius() const = 0;

    friend class SprocketBandContactCB;
};

/// @} vehicle_tracked_sprocket

}  // end namespace vehicle
}  // end namespace chrono

#endif
