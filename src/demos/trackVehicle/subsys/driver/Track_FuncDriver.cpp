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
// Authors: Justin Madsen
// =============================================================================

#include "Track_FuncDriver.h"

using namespace chrono;

namespace chrono {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Track_FuncDriver::Update(double time)
{
  if (time < 1.0)
    m_throttle[0] = 0;
  else if (time < 3.0)
    m_throttle[0] = 0.4 * (time - 1.0);
  else
    m_throttle[0] = 0.9;

}


} // end namespace chrono
