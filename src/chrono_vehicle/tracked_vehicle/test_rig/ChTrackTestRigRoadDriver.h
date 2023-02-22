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
// A driver model based on a user-specified road profile.  Post displacements
// are computed based on current road height at each post location and a given
// translation speed of the road profile. The road profile input data is assumed
// to contain (x,z) pairs, with x locations in increasing order.
//
// =============================================================================

#ifndef CH_TTR_ROAD_DRIVER_H
#define CH_TTR_ROAD_DRIVER_H

#include <string>
#include <vector>
#include <utility>

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/tracked_vehicle/test_rig/ChTrackTestRigDriver.h"

#include "chrono/core/ChCubicSpline.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked_test_rig
/// @{

/// Driver inputs for a track test rig based on road profile.
/// A driver model based on a user-specified road profile.  Post displacements are computed based on current road height
/// at each post location and a given translation speed of the road profile. The road profile input data is assumed to
/// contain (x,z) pairs, with x locations in increasing order.
class CH_VEHICLE_API ChTrackTestRigRoadDriver : public ChTrackTestRigDriver {
  public:
    /// Construct using data from the specified file.
    ChTrackTestRigRoadDriver(const std::string& filename,  ///< name of data file
                    double speed                  ///< translation speed
    );

    ~ChTrackTestRigRoadDriver();
    
    /// Return true when driver stopped producing inputs (end of data).
    virtual bool Ended() const override;

  private:
    /// Initialize the driver system.
    virtual void Initialize(size_t num_posts, const std::vector<double>& locations) override;

    /// Update the driver system at the specified time.
    virtual void Synchronize(double time) override;

    /// Get string message.
    virtual std::string GetInfoMessage() const override { return "Road driver inputs"; }

    std::string m_filename;       ///< input file name
    double m_speed;               ///< vehicle speed over road profile
    ChCubicSpline* m_curve_road;  ///< spline for road profile
    double m_min;                 ///< first value of road x
    double m_max;                 ///< last value fo road x
    bool m_ended;                 ///< flag indicating end of input data
};

/// @} vehicle_tracked_test_rig

}  // end namespace vehicle
}  // end namespace chrono

#endif
