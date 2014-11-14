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
// Authors: Alessandro Tasora
// =============================================================================
//
//
// =============================================================================

#ifndef ARTICULATED_FUNCDRIVER_H
#define ARTICULATED_FUNCDRIVER_H

#include "subsys/ChDriver.h"

class Articulated_FuncDriver : public chrono::ChDriver
{
public:

  Articulated_FuncDriver() {}
  ~Articulated_FuncDriver() {}

  virtual void Update(double time)
  {
    if (time < 0.5)
      m_throttle = 0;
    else if (time < 1.5)
      m_throttle = 0.4 * (time - 0.5);
    else
      m_throttle = 0.4;

    if (time < 4)
      m_steering = 0;
    else if (time < 6)
      m_steering = 0.25 * (time - 4);
    else if (time < 10)
      m_steering = -0.25 * (time - 6) + 0.5;
    else
      m_steering = -0.5;
  }
};


#endif
