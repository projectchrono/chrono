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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================
//
// HMMWV 2WD driveline model based on ChShaft objects.
//
// =============================================================================

#ifndef HMMWV_DRIVELINE_2WD_H
#define HMMWV_DRIVELINE_2WD_H

#include "subsys/driveline/ChShaftsDriveline2WD.h"
#include "subsys/ChVehicle.h"


namespace hmmwv {

// Forward reference
class chrono::ChVehicle;


class HMMWV_Driveline2WD : public chrono::ChShaftsDriveline2WD {
public:

  HMMWV_Driveline2WD(chrono::ChVehicle* car);

  ~HMMWV_Driveline2WD() {}

  virtual double GetDriveshaftInertia() const      { return m_driveshaft_inertia; }
  virtual double GetDifferentialBoxInertia() const { return m_differentialbox_inertia; }

  virtual double GetConicalGearRatio() const       { return m_conicalgear_ratio; }
  virtual double GetDifferentialRatio() const      { return m_differential_ratio; }

private:

  // Shaft inertias.
  static const double  m_driveshaft_inertia;
  static const double  m_differentialbox_inertia;

  // Gear ratios.
  static const double  m_conicalgear_ratio;
  static const double  m_differential_ratio;

};


} // end namespace hmmwv


#endif
