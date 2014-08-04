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
//
// =============================================================================

#ifndef HMMWV9_FUNCDRIVER_H
#define HMMWV9_FUNCDRIVER_H

#include "ChDriver.h"

namespace hmmwv9 {

class HMMWV9_FuncDriver : public chrono::ChDriver
{
public:

  HMMWV9_FuncDriver() {}
  ~HMMWV9_FuncDriver() {}

  virtual void Update(double time);

private:

};


} // end namespace hmmwv9


#endif