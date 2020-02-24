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
// text file, each line in the file must contain 4 values:
//   time left_post right_post steering
// It is assumed that the time values are unique.
// If the time values are not sorted, this must be specified at construction.
// Inputs for post_left, post_right, and steering are assumed to be in [-1,1].
// Driver inputs at intermediate times are obtained through cubic spline
// interpolation.
//
// =============================================================================

#ifndef CH_DATADRIVER_STR_H
#define CH_DATADRIVER_STR_H

#include <string>
#include <vector>

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChDriverSTR.h"

#include "chrono/core/ChCubicSpline.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_test_rig
/// @{

/// Driver inputs for a suspension test rig from data file.
/// A driver model based on user inputs provided as time series. If provided as a
/// text file, each line in the file must contain 4 values:
/// <pre>
///   time  left_post  right_post  steering
/// </pre>
/// It is assumed that the time values are unique.
/// If the time values are not sorted, this must be specified at construction.
/// Inputs for post_left, post_right, and steering are assumed to be normalized in the range [-1,1].
/// Driver inputs at intermediate times are obtained through cubic spline interpolation.
class CH_VEHICLE_API ChDataDriverSTR : public ChDriverSTR {
  public:
    /// Construct using data from the specified file.
    ChDataDriverSTR(const std::string& filename);

    ~ChDataDriverSTR();

    /// Return true when driver stopped producing inputs (end of data).
    virtual bool Ended() const override;

  private:
    /// Update the driver system at the specified time.
    /// The driver inputs are obtained through cubic spline interpolation using the provided data points.
    virtual void Synchronize(double time) override;

    ChCubicSpline* m_curve_left;      ///< spline for left post displacement
    ChCubicSpline* m_curve_right;     ///< spline for right post displacement
    ChCubicSpline* m_curve_steering;  ///< spline for steering
    bool m_ended;                     ///< flag indicating end of input data
    double m_last_time;               ///< last time entry in input file
    double m_last_displLeft;          ///< last left displacement in input file
    double m_last_displRight;         ///< last right displacement in input file
    double m_last_steering;           ///< last steering value in input file
};

/// @} vehicle_wheeled_test_rig

}  // end namespace vehicle
}  // end namespace chrono

#endif
