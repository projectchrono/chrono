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
// Simple driveline model template using data from file (JSON format).
//
// =============================================================================

#ifndef SIMPLE_DRIVELINE_H
#define SIMPLE_DRIVELINE_H

#include "subsys/ChApiSubsys.h"
#include "subsys/driveline/ChSimpleDriveline.h"

#include "rapidjson/document.h"

namespace chrono {


class CH_SUBSYS_API SimpleDriveline : public ChSimpleDriveline
{
public:

  SimpleDriveline(const std::string& filename);
  SimpleDriveline(const rapidjson::Document& d);
  ~SimpleDriveline() {}

  virtual double GetFrontTorqueFraction() const      { return m_front_torque_frac; }
  virtual double GetFrontDifferentialMaxBias() const { return m_front_diff_bias; }
  virtual double GetRearDifferentialMaxBias() const  { return m_rear_diff_bias; }

private:
  void Create(const rapidjson::Document& d);

  double m_front_torque_frac;
  double m_front_diff_bias;
  double m_rear_diff_bias;
};


} // end namespace chrono


#endif
