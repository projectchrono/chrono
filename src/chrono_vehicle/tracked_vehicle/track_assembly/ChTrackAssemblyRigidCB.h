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
// Base class for a continuous band track assembly using rigid-link track shoes
// (template definition).
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#ifndef CH_TRACK_ASSEMBLY_RIGID_CB_H
#define CH_TRACK_ASSEMBLY_RIGID_CB_H

#include <vector>

#include "chrono/core/ChVector2.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackAssembly.h"
#include "chrono_vehicle/tracked_vehicle/sprocket/ChSprocketCB.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeRigidCB.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked
/// @{

/// Definition of a continuous band rigid-link track assembly.
/// A track assembly consists of a sprocket, an idler (with tensioner mechanism),
/// a set of suspensions (road-wheel assemblies), and a collection of track shoes.
/// This class defines the template for a track assembly using continuous band rigid-body
/// track shoes.
class CH_VEHICLE_API ChTrackAssemblyRigidCB : public ChTrackAssembly {
  public:
    ChTrackAssemblyRigidCB(const std::string& name,  ///< [in] name of the subsystem
                           VehicleSide side          ///< [in] assembly on left/right vehicle side
                           )
        : ChTrackAssembly(name, side) {}

    virtual ~ChTrackAssemblyRigidCB() {}

    /// Get the number of track shoes.
    virtual size_t GetNumTrackShoes() const override { return m_shoes.size(); }

    /// Get a handle to the sprocket.
    virtual std::shared_ptr<ChSprocket> GetSprocket() const override { return m_sprocket; }

    /// Get a handle to the specified track shoe subsystem.
    virtual std::shared_ptr<ChTrackShoe> GetTrackShoe(size_t id) const override { return m_shoes[id]; }

  protected:
    std::shared_ptr<ChSprocketCB> m_sprocket;  ///< sprocket subsystem
    ChTrackShoeRigidCBList m_shoes;            ///< track shoes

  private:
    /// Assemble track shoes over wheels.
    /// Return true if the track shoes were initialized in a counter clockwise
    /// direction and false otherwise.
    virtual bool Assemble(std::shared_ptr<ChBodyAuxRef> chassis) override;

    /// Assembly Algorithm Utility Functions
    void FindCircleTangentPoints(
        ChVector2<> Circle1Pos,  ///< Center Position of Circle 1
        double Circle1Rad,       ///< Radius of Circle 1
        ChVector2<> Circle2Pos,  ///< Center Position of Circle 2
        double Circle2Rad,       ///< Radius of Circle 2
        ChVector2<>& Tan1Pnt1,   ///> Point on Circle 1 for the first calculated outside tangent
        ChVector2<>& Tan1Pnt2,   ///> Point on Circle 2 for the first calculated outside tangent
        ChVector2<>& Tan2Pnt1,   ///> Point on Circle 1 for the second calculated outside tangent
        ChVector2<>& Tan2Pnt2    ///> Point on Circle 2 for the second calculated outside tangent
        );
    void CheckCircleCircle(
        bool& found,  ///> Does an intersection point exist between the circle formed by StartingPoint and Radius with
                      /// the current circle belt feature
        ChVector2<>& Point,  ///> Intersection Point, if it exists between the circle formed by StartingPoint and Radius
                             /// with the current circle belt feature
        ChMatrixDynamic<>& Features,  ///< Table with the tagent or arc information for the entire belt wrap
        int FeatureIdx,               ///< Current belt feature to check the intersection of
        ChVector2<>& StartingPoint,   ///< Current Point on the belt wrap
        double Radius                 ///< Length of the current belt segment that needs to be placed on the belt wrap
        );
    void CheckCircleLine(
        bool& found,  ///> Does an intersection point exist between the circle formed by StartingPoint and Radius with
                      /// the current circle belt feature
        ChVector2<>& Point,  ///> Intersection Point, if it exists between the circle formed by StartingPoint and Radius
                             /// with the current circle belt feature
        ChMatrixDynamic<>& Features,  ///< Table with the tagent or arc information for the entire belt wrap
        int FeatureIdx,               ///< Current belt feature to check the intersection of
        ChVector2<>& StartingPoint,   ///< Current Point on the belt wrap
        double Radius                 ///< Length of the current belt segment that needs to be placed on the belt wrap
        );

    std::shared_ptr<ChBodyAuxRef> m_chassis;
    double m_sprocket_offset;
};

/// @} vehicle_tracked

}  // end namespace vehicle
}  // end namespace chrono

#endif
