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
// HMMWV 4WD driveline model based on ChShaft objects.
//
// =============================================================================

#ifndef HMMWV_DRIVELINE_4WD_H
#define HMMWV_DRIVELINE_4WD_H

#include "subsys/driveline/ChShaftsDriveline4WD.h"

namespace hmmwv {

class HMMWV_Driveline4WD : public chrono::ChShaftsDriveline4WD
{
public:

  HMMWV_Driveline4WD();
  ~HMMWV_Driveline4WD() {}

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

  // Shaft inertias.
  static const double  m_central_differentialbox_inertia;
  static const double  m_front_differentialbox_inertia;
  static const double  m_rear_differentialbox_inertia;
  static const double  m_driveshaft_inertia;
  static const double  m_frontshaft_inertia;
  static const double  m_rearshaft_inertia;

  // Gear ratios.
  static const double  m_central_differential_ratio;
  static const double  m_front_differential_ratio;
  static const double  m_rear_differential_ratio;
  static const double  m_front_conicalgear_ratio;
  static const double  m_rear_conicalgear_ratio;
};


} // end namespace hmmwv


#endif
