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
// Driver inputs at intermediate times are obtained through linear interpolation.
//
// =============================================================================

#ifndef CH_DATADRIVER_TTR_H
#define CH_DATADRIVER_TTR_H

#include <string>
#include <vector>

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/tracked_vehicle/test_rig/ChDriverTTR.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked_test_rig
/// @{

/// Driver inputs for a track test rig from data file.
/// A driver model based on user inputs provided as time series. If provided as a
/// text file, each line in the file must contain 4 values:
/// <pre>
///   time  displ_post1 displ_post_2 ... throttle
/// </pre>
/// It is assumed that the time values are unique.
/// If the time values are not sorted, this must be specified at construction.
/// Inputs for post displacements and throttle are assumed to be in[-1, 1].
/// Driver inputs at intermediate times are obtained through linear interpolation.
class CH_VEHICLE_API ChDataDriverTTR : public ChDriverTTR {
  public:
    /// Definition of driver inputs at a given time.
    struct Entry {
        Entry() {}
        Entry(double time) : m_time(time) {}
        Entry(double time, std::vector<double> displ, double throttle)
            : m_time(time), m_displ(displ), m_throttle(throttle) {}
        double m_time;
        std::vector<double> m_displ;
        double m_throttle;
    };

    /// Construct using data from the specified file.
    ChDataDriverTTR(const std::string& filename,  ///< name of data file
                    bool sorted = true            ///< indicate whether entries are sorted by time stamps
                    );

    ~ChDataDriverTTR() {}

    /// Initialize the driver system.
    virtual void Initialize(size_t num_posts) override;

    /// Update the driver system at the specified time.
    /// The driver inputs are obtained through linear interpolation between the provided data points.
    virtual void Synchronize(double time) override;

    virtual std::string GetInfoMessage() const override { return "Data driver inputs"; }

  private:
    static bool compare(const Entry& a, const Entry& b) { return a.m_time < b.m_time; }

    std::string m_filename;
    bool m_sorted;
    std::vector<Entry> m_data;
};

/// @} vehicle_tracked_test_rig

}  // end namespace vehicle
}  // end namespace chrono

#endif
