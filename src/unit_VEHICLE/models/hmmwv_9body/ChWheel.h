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
// Authors: Radu Serban, Justin Madsen
// =============================================================================
//
// Base class for a vehicle wheel.
// A wheel subsystem does not own a body. Instead, when attached to a suspension
// subsystem, the wheel's mass properties are used to update those of the
// spindle body owned by the suspension.
// A concrete wheel subsystem can optionally carry its own visualization assets
// and/or contact geometry (which are associated with the suspension's spindle
// body).
// =============================================================================

#ifndef CH_WHEEL_H
#define CH_WHEEL_H


#include "core/ChShared.h"
#include "core/ChVector.h"
#include "physics/ChBody.h"


namespace chrono {


class ChWheel : public ChShared {
public:
  ChWheel() {}
  virtual ~ChWheel() {}

  virtual double getMass() const = 0;
  virtual const ChVector<>& getInertia() = 0;

protected:
  virtual void OnInitialize(ChSharedBodyPtr body) {}

  friend class ChDoubleWishboneReduced;
};


} // end namespace chrono


#endif
