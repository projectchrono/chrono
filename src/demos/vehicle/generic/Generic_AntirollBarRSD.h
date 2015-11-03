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
// Authors: Holger Haut
// =============================================================================
//
// Generic antirollbar RSD model.
//
// =============================================================================

#ifndef GENERIC_ANTIROLLBAR_RSD_H
#define GENERIC_ANTIROLLBAR_RSD_H

#include "chrono_vehicle/wheeled_vehicle/antirollbar/ChAntirollBarRSD.h"

class Generic_AntirollBarRSD : public chrono::vehicle::ChAntirollBarRSD {
  public:
    Generic_AntirollBarRSD(const std::string& name) : chrono::vehicle::ChAntirollBarRSD(name) {}
    ~Generic_AntirollBarRSD() {}

    virtual double getArmMass() const override { return 1.0; }
    virtual chrono::ChVector<> getArmInertia() override { return chrono::ChVector<>(1, 1, 1); }

    virtual double getArmLength() const override { return 0.70; }
    virtual double getArmWidth() const override { return 0.25; }
    virtual double getDroplinkHeight() const override { return -0.20; }
    virtual double getArmRadius() const override { return 0.02; }

    virtual double getSpringCoefficient() const override { return 100000.0; }
    virtual double getDampingCoefficient() const override { return 20000.0; }
};

#endif
