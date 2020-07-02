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
// Generic 2WD driveline model based on ChShaft objects.
//
// =============================================================================

#ifndef ACV_DRIVELINE_2WD_H
#define ACV_DRIVELINE_2WD_H

#include "chrono_vehicle/wheeled_vehicle/driveline/ChShaftsDriveline2WD.h"

class ACV_Driveline2WD : public chrono::vehicle::ChShaftsDriveline2WD {
  public:
    ACV_Driveline2WD(const std::string& name);
    ~ACV_Driveline2WD() {}

    virtual double GetDriveshaftInertia() const override { return m_driveshaft_inertia; }
    virtual double GetDifferentialBoxInertia() const override { return m_differentialbox_inertia; }

    virtual double GetConicalGearRatio() const override { return m_conicalgear_ratio; }
    virtual double GetDifferentialRatio() const override { return m_differential_ratio; }

    virtual double GetAxleDifferentialLockingLimit() const override { return m_axle_differential_locking_limit; }

  private:
    static const double m_driveshaft_inertia;
    static const double m_differentialbox_inertia;
    static const double m_conicalgear_ratio;
    static const double m_differential_ratio;
    static const double m_axle_differential_locking_limit;
};


#endif
