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
// Authors: Radu Serban, Alessandro Tasora
// =============================================================================
//
// Articulated rack-pinion steering model.
//
// =============================================================================

#ifndef ARTICULATED_RACKPINION_H
#define ARTICULATED_RACKPINION_H

#include "subsys/steering/ChRackPinion.h"

class Articulated_RackPinion : public chrono::ChRackPinion
{
public:

  Articulated_RackPinion(const std::string& name) : ChRackPinion(name) {}
  ~Articulated_RackPinion() {}

  virtual double GetSteeringLinkMass() const                 { return 9.0; }
  virtual chrono::ChVector<> GetSteeringLinkInertia() const  { return chrono::ChVector<>(1, 1, 1); }
  virtual double GetSteeringLinkCOM() const                  { return 0.0; }
  virtual double GetSteeringLinkRadius() const               { return 0.03; }
  virtual double GetSteeringLinkLength() const               { return 0.896; }

  virtual double GetPinionRadius() const                     { return 0.1; }

  virtual double GetMaxAngle() const                         { return 1.87; }
};


#endif
