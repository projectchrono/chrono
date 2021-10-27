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
// Base class for segmented track assemblies.
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#ifndef CH_TRACK_ASSEMBLY_SEGMENTED_H
#define CH_TRACK_ASSEMBLY_SEGMENTED_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackAssembly.h"

#include "chrono/physics/ChLinkRotSpringCB.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked
/// @{

/// Base class for segmented track assemblies.
/// Track shoes in such an assembly are modeled with one or more rigid bodies connected through joints or bushings.
class CH_VEHICLE_API ChTrackAssemblySegmented : public ChTrackAssembly {
  public:
    virtual ~ChTrackAssemblySegmented() {}

    std::shared_ptr<ChLinkRotSpringCB::TorqueFunctor> GetTorqueFunctor() const { return m_torque_funct; }
    std::shared_ptr<ChVehicleBushingData> GetBushingData() const { return m_bushing_data; }

  protected:
    /// Default torque functor for implementing track bending stiffness.
    class CH_VEHICLE_API TrackBendingFunctor : public ChLinkRotSpringCB::TorqueFunctor {
      public:
        TrackBendingFunctor(double k, double c, double t = 0) : m_k(k), m_c(c), m_t(t) {}
        virtual double operator()(double time, double angle, double vel, ChLinkRotSpringCB* link) override;
      private:
        double m_k;
        double m_c;
        double m_t;
    };

    ChTrackAssemblySegmented(const std::string& name,  ///< [in] name of the subsystem
                             VehicleSide side          ///< [in] assembly on left/right vehicle side
    );

    std::shared_ptr<ChLinkRotSpringCB::TorqueFunctor> m_torque_funct;  ///< torque for track bending stiffness
    std::shared_ptr<ChVehicleBushingData> m_bushing_data;              ///< track pin bushings
};

/// @} vehicle_tracked

}  // end namespace vehicle
}  // end namespace chrono

#endif
