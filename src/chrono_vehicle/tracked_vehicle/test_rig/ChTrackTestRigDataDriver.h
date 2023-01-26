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
// A driver model based on user inputs provided as time series. If provided as a
// text file, each line in the file must contain the following values:
//   time displ_post1 displ_post_2 ... throttle
// It is assumed that the time values are unique.
// If the time values are not sorted, this must be specified at construction.
// Inputs for post displacements and throttle are assumed to be in [-1,1].
// Driver inputs at intermediate times are obtained through cubic spline
// interpolation.
//
// =============================================================================

#ifndef CH_TTR_DATA_DRIVER_H
#define CH_TTR_DATA_DRIVER_H

#include <string>
#include <vector>

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/tracked_vehicle/test_rig/ChTrackTestRigDriver.h"

#include "chrono/core/ChCubicSpline.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked_test_rig
/// @{

/// Driver inputs for a track test rig from data file.
/// A driver model based on user inputs provided as time series. If provided as a
/// text file, each line in the file must contain the following values:
/// <pre>
///   time  displ_post1 displ_post_2 ... throttle
/// </pre>
/// It is assumed that the time values are unique.
/// If the time values are not sorted, this must be specified at construction.
/// Inputs for post displacements and throttle are assumed to be in [-1, 1].
/// Driver inputs at intermediate times are obtained through linear interpolation.
class CH_VEHICLE_API ChTrackTestRigDataDriver : public ChTrackTestRigDriver {
  public:
    /// Construct using data from the specified file.
    ChTrackTestRigDataDriver(const std::string& filename);

    ~ChTrackTestRigDataDriver();

    /// Return true when driver stopped producing inputs (end of data).
    virtual bool Ended() const override;

  private:
    /// Initialize the driver system.
    virtual void Initialize(size_t num_posts, const std::vector<double>& locations) override;

    /// Update the driver system at the specified time.
    /// The driver inputs are obtained through linear interpolation between the provided data points.
    virtual void Synchronize(double time) override;

    std::string m_filename;                     ///< input file name
    std::vector<ChCubicSpline*> m_curve_displ;  ///< splines for post displacements
    ChCubicSpline* m_curve_throttle;            ///< spline for throttle
    bool m_ended;                               ///< flag indicating end of input data
    double m_last_time;                         ///< last time entry in input file
    std::vector<double> m_last_displ;           ///< last displacements in input file
    double m_last_throttle;                     ///< last throttle value in input file
};

/// @} vehicle_tracked_test_rig

}  // end namespace vehicle
}  // end namespace chrono

#endif
