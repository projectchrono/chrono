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

#include "subsys/ChApiSubsys.h"
#include "subsys/ChDriver.h"

namespace chrono {

class CH_SUBSYS_API ChDataDriver : public ChDriver
{
public:
  struct Entry {
    Entry() {}
    Entry(double time, double steering, double throttle, double braking)
      : m_time(time), m_steering(steering), m_throttle(throttle), m_braking(braking)
    {}
    double m_time;
    double m_steering;
    double m_throttle;
    double m_braking;
  };

  ChDataDriver(const std::string& filename,
               bool               sorted = true);
  ChDataDriver(const std::vector<Entry>& data,
               bool                      sorted = true);
  ~ChDataDriver() {}

  virtual void Update(double time);

private:

  static bool compare(const Entry& a, const Entry& b) { return a.m_time < b.m_time; }

  std::vector<Entry> m_data;
};


} // end namespace hmmwv9


#endif