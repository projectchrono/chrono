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
// Base class for a tire.
// A tire subsystem is a force element. It is passed position and velocity
// information of the wheel body and it produces ground reaction forces and
// moments to be applied to the wheel body.
//
// =============================================================================

#ifndef CH_TIRE_H
#define CH_TIRE_H

#include "core/ChShared.h"
#include "core/ChVector.h"
#include "core/ChQuaternion.h"

#include "subsys/ChApiSubsys.h"
#include "subsys/ChVehicle.h"
#include "subsys/ChTerrain.h"

namespace chrono {


class CH_SUBSYS_API ChTire : public ChShared {
public:
  ChTire(const ChTerrain& terrain);
  virtual ~ChTire() {}

  virtual void Update(double              time,
                      const ChBodyState&  wheel_state) {}

  virtual void Advance(double step) {}

  virtual ChTireForce GetTireForce() const = 0;

protected:
  bool  disc_terrain_contact(const ChVector<>& disc_center,
                             const ChVector<>& disc_normal,
                             double            disc_radius,
                             ChVector<>&       ptD,
                             ChVector<>&       ptT,
                             ChVector<>&       normal,
                             double&           depth);

  const ChTerrain&  m_terrain;

};


} // end namespace chrono


#endif
