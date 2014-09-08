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
// Base class for all steering subsystems
//
// =============================================================================

#ifndef CH_STEERING_H
#define CH_STEERING_H

#include <string>

#include "core/ChShared.h"
#include "physics/ChSystem.h"

#include "subsys/ChApiSubsys.h"

namespace chrono {


class CH_SUBSYS_API ChSteering : public ChShared
{
public:

  ChSteering(const std::string& name);

  virtual ~ChSteering() {}

  const std::string& GetName() const { return m_name; }
  void SetName(const std::string& name) { m_name = name; }

  virtual void Initialize(ChSharedPtr<ChBody>  chassis,
                          const ChVector<>&    location) = 0;

  virtual void ApplySteering(double displ) = 0;

protected:
  std::string               m_name;
};


} // end namespace chrono


#endif
