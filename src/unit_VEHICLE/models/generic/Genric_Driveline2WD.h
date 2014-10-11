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
// Generic 2WD driveline model based on ChShaft objects.
//
// =============================================================================

#ifndef GENERIC_DRIVELINE_2WD_H
#define GENERIC_DRIVELINE_2WD_H

#include "subsys/driveline/ChShaftsDriveline2WD.h"

class Generic_Driveline2WD : public chrono::ChShaftsDriveline2WD
{
public:

  Generic_Driveline2WD() : ChShaftsDriveline2WD()
  {
    SetMotorBlockDirection(chrono::ChVector<>(1, 0, 0));
    SetAxleDirection(chrono::ChVector<>(0, 1, 0));
  }

  ~Generic_Driveline2WD() {}

  virtual double GetDriveshaftInertia() const      { return 0.5; }
  virtual double GetDifferentialBoxInertia() const { return 0.6; }

  virtual double GetConicalGearRatio() const       { return -0.2; }
  virtual double GetDifferentialRatio() const      { return -1; }
};


#endif
