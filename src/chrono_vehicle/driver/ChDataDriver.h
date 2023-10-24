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
//   time steering throttle braking
// It is assumed that the time values are unique.
// If the time values are not sorted, this must be specified at construction.
// Driver inputs at intermediate times are obtained through linear interpolation.
//
// =============================================================================

#ifndef CH_DATADRIVER_H
#define CH_DATADRIVER_H

#include <string>
#include <vector>

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChDriver.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_driver
/// @{

/// Driver inputs from data file.
/// A driver model based on user inputs provided as time series. If provided as a
/// text file, each line in the file must contain 4 values:
///   time steering throttle braking clutch
/// It is assumed that the time values are unique.
/// If the time values are not sorted, this must be specified at construction.
/// Driver inputs at intermediate times are obtained through linear interpolation.
class CH_VEHICLE_API ChDataDriver : public ChDriver {
  public:
    /// Definition of driver inputs at a given time.
    struct Entry {
        Entry() {}
        Entry(double time, double steering, double throttle, double braking, double clutch = 0)
            : m_time(time), m_steering(steering), m_throttle(throttle), m_braking(braking), m_clutch(clutch) {}
        double m_time;
        double m_steering;
        double m_throttle;
        double m_braking;
        double m_clutch;
    };

    /// Construct using data from the specified file.
    ChDataDriver(ChVehicle& vehicle,           ///< associated vehicle
                 const std::string& filename,  ///< name of data file
                 bool sorted = true            ///< indicate whether entries are sorted by time stamps
    );

    /// Construct using data from the specified data entries.
    ChDataDriver(ChVehicle& vehicle,              ///< associated vehicle
                 const std::vector<Entry>& data,  ///< vector of data entries
                 bool sorted = true               ///< indicate whether entries are sorted by time stamps
    );

    ~ChDataDriver() {}

    /// Update the driver system at the specified time.
    /// The driver inputs are obtained through linear interpolation between the
    /// provided data points.
    virtual void Synchronize(double time) override;

  private:
    static bool compare(const Entry& a, const Entry& b) { return a.m_time < b.m_time; }

    std::vector<Entry> m_data;
};

/// @} vehicle_driver

}  // end namespace vehicle
}  // end namespace chrono

#endif
