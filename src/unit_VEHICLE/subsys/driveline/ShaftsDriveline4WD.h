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
// 4WD driveline model template based on ChShaft objects using data from file
// (JSON format).
//
// =============================================================================

#ifndef SHAFTS_DRIVELINE_4WD_H
#define SHAFTS_DRIVELINE_4WD_H

#include "subsys/ChApiSubsys.h"
#include "subsys/driveline/ChShaftsDriveline4WD.h"

#include "rapidjson/document.h"

namespace chrono {


class CH_SUBSYS_API ShaftsDriveline4WD : public ChShaftsDriveline4WD
{
public:

  ShaftsDriveline4WD(const std::string& filename);
  ShaftsDriveline4WD(const rapidjson::Document& d);
  ~ShaftsDriveline4WD() {}

  virtual double GetCentralDifferentialBoxInertia() const { return m_central_differentialbox_inertia; }
  virtual double GetFrontDifferentialBoxInertia() const { return m_front_differentialbox_inertia; }
  virtual double GetRearDifferentialBoxInertia() const { return m_rear_differentialbox_inertia; }
  virtual double GetDriveshaftInertia() const { return m_driveshaft_inertia; }
  virtual double GetToFrontDiffShaftInertia() const { return m_frontshaft_inertia; }
  virtual double GetToRearDiffShaftInertia() const { return m_rearshaft_inertia; }

  virtual double GetCentralDifferentialRatio() const { return m_central_differential_ratio; }
  virtual double GetFrontDifferentialRatio() const { return m_front_differential_ratio; }
  virtual double GetRearDifferentialRatio() const { return m_rear_differential_ratio; }
  virtual double GetFrontConicalGearRatio() const { return m_front_conicalgear_ratio; }
  virtual double GetRearConicalGearRatio() const { return m_rear_conicalgear_ratio; }

private:
  void Create(const rapidjson::Document& d);

  // Shaft inertias.
  double  m_central_differentialbox_inertia;
  double  m_front_differentialbox_inertia;
  double  m_rear_differentialbox_inertia;
  double  m_driveshaft_inertia;
  double  m_frontshaft_inertia;
  double  m_rearshaft_inertia;

  // Gear ratios.
  double  m_central_differential_ratio;
  double  m_front_differential_ratio;
  double  m_rear_differential_ratio;
  double  m_front_conicalgear_ratio;
  double  m_rear_conicalgear_ratio;
};


} // end namespace chrono


#endif
