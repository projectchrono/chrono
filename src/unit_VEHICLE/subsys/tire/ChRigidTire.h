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
// Generic rigid tire
//
// =============================================================================

#ifndef CH_RIGIDTIRE_H
#define CH_RIGIDTIRE_H

#include "physics/ChBody.h"

#include "subsys/ChTire.h"
#include "subsys/ChTerrain.h"

namespace chrono {

///
/// Rigid tire model.
/// This tire is modeled as a rigid cylinder.  Requires a terrain system that
/// supports rigid contact with friction.
///
class CH_SUBSYS_API ChRigidTire : public ChTire
{
public:

  ChRigidTire(
    const ChTerrain& terrain   ///< [in] reference to the terrain system
    );

  virtual ~ChRigidTire() {}

  /// Get the tire force and moment.
  /// For a rigid tire, the tire forces are automatically applied to the
  /// associated wheel (through Chrono's frictional contact system). The values
  /// returned here are never used.
  virtual ChTireForce GetTireForce() const;

  /// Initialize this tire system.
  /// This function creates the tire contact shape and attaches it to the 
  /// associated wheel body.
  void Initialize(
    ChSharedBodyPtr wheel  ///< handle to the associated wheel body
    );

protected:

  /// Return the coefficient of friction for the tire material.
  virtual float getFrictionCoefficient() const = 0;

  /// Return the tire radius.
  virtual double getRadius() const = 0;

  /// Return the tire width.
  virtual double getWidth() const = 0;
};


} // end namespace chrono


#endif
