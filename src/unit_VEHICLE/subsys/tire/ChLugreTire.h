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
// Authors: Radu Serban, Aki Mikkola
// =============================================================================
//
// Template for LuGre tire model
//
// =============================================================================

#ifndef CH_LUGRETIRE_H
#define CH_LUGRETIRE_H

#include "physics/ChBody.h"

#include "subsys/ChTire.h"
#include "subsys/ChTerrain.h"

namespace chrono {

class CH_SUBSYS_API ChLugreTire : public ChTire {
public:
  ChLugreTire(const ChTerrain& terrain);
  virtual ~ChLugreTire() {}

  virtual ChTireForce GetTireForce() const;

  virtual void Update(double              time,
                      const ChBodyState&  wheel_state);
  virtual void Advance(double step);

  void Initialize(ChSharedBodyPtr wheel);

protected:
  virtual int getNumDiscs() const = 0;
  virtual double getRadius() const = 0;
  virtual double getWidth() const = 0;

private:
  void findContact();

};


} // end namespace chrono


#endif
