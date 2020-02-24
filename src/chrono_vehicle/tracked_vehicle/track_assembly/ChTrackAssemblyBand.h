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
// Base class for continuous band track assembly using rigid tread.
// Derived classes specify the actual template defintions, using different track
// shoes.
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#ifndef CH_TRACK_ASSEMBLY_BAND_H
#define CH_TRACK_ASSEMBLY_BAND_H

#include <vector>

#include "chrono/core/ChVector2.h"
#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackAssembly.h"
#include "chrono_vehicle/tracked_vehicle/sprocket/ChSprocketBand.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked
/// @{

/// Definition of a continuous band track assembly.
/// A track assembly consists of a sprocket, an idler (with tensioner mechanism),
/// a set of suspensions (road-wheel assemblies), and a collection of track shoes.
/// This is the base class for continuous band track assembly using rigid tread.
/// Derived classes specify the actual template defintions, using different track shoes.
class CH_VEHICLE_API ChTrackAssemblyBand : public ChTrackAssembly {
  public:
    ChTrackAssemblyBand(const std::string& name,  ///< [in] name of the subsystem
                        VehicleSide side          ///< [in] assembly on left/right vehicle side
                        )
        : ChTrackAssembly(name, side) {}

    virtual ~ChTrackAssemblyBand() {}

    /// Get a handle to the sprocket.
    virtual std::shared_ptr<ChSprocket> GetSprocket() const override { return m_sprocket; }

  protected:
    std::shared_ptr<ChSprocketBand> m_sprocket;  ///< sprocket subsystem

    /// Assembly Algorithm Utility Functions
    bool FindAssemblyPoints(std::shared_ptr<ChBodyAuxRef> chassis,
                            int num_shoes,
                            const std::vector<double>& connection_lengths,
                            std::vector<ChVector2<>>& shoe_points);

    void FindCircleTangentPoints(
        ChVector2<> Circle1Pos,  ///< Center Position of Circle 1
        double Circle1Rad,       ///< Radius of Circle 1
        ChVector2<> Circle2Pos,  ///< Center Position of Circle 2
        double Circle2Rad,       ///< Radius of Circle 2
        ChVector2<>& Tan1Pnt1,   ///< Point on Circle 1 for the first calculated outside tangent
        ChVector2<>& Tan1Pnt2,   ///< Point on Circle 2 for the first calculated outside tangent
        ChVector2<>& Tan2Pnt1,   ///< Point on Circle 1 for the second calculated outside tangent
        ChVector2<>& Tan2Pnt2    ///< Point on Circle 2 for the second calculated outside tangent
    );
    void CheckCircleCircle(
        bool& found,  ///< Does an intersection point exist between the circle formed by StartingPoint and Radius with
                      ///< the current circle belt feature
        ChVector2<>& Point,  ///< Intersection Point, if it exists between the circle formed by StartingPoint and Radius
                             ///< with the current circle belt feature
        ChMatrixDynamic<>& Features,  ///< Table with the tagent or arc information for the entire belt wrap
        int FeatureIdx,               ///< Current belt feature to check the intersection of
        ChVector2<>& StartingPoint,   ///< Current Point on the belt wrap
        double Radius                 ///< Length of the current belt segment that needs to be placed on the belt wrap
    );
    void CheckCircleLine(
        bool& found,  ///< Does an intersection point exist between the circle formed by StartingPoint and Radius with
                      ///< the current circle belt feature
        ChVector2<>& Point,  ///< Intersection Point, if it exists between the circle formed by StartingPoint and Radius
                             ///< with the current circle belt feature
        ChMatrixDynamic<>& Features,  ///< Table with the tagent or arc information for the entire belt wrap
        int FeatureIdx,               ///< Current belt feature to check the intersection of
        ChVector2<>& StartingPoint,   ///< Current Point on the belt wrap
        double Radius                 ///< Length of the current belt segment that needs to be placed on the belt wrap
    );

    double m_sprocket_offset;
};

/// @} vehicle_tracked

}  // end namespace vehicle
}  // end namespace chrono

#endif
