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
// Simple rigid terrain...
//
// =============================================================================

#include "physics/ChBody.h"
#include "assets/ChColorAsset.h"

#include "utils/ChUtilsCreators.h"

#include "HMMWV9_RigidTerrain.h"

using namespace chrono;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
HMMWV9_RigidTerrain::HMMWV9_RigidTerrain(ChSystem&  system,
                                         double     height,
                                         double     sizeX,
                                         double     sizeY,
                                         double     mu)
{
  ChSharedBodyPtr ground = ChSharedBodyPtr(new ChBody);

  ground->SetIdentifier(-1);
  ground->SetName("ground");
  ground->SetPos(ChVector<>(0, 0, height));
  ground->SetRot(ChQuaternion<>(1, 0, 0, 0));
  ground->SetBodyFixed(true);
  ground->SetCollide(true);

  double hDepth = 10;

  ground->GetCollisionModel()->ClearModel();
  utils::AddBoxGeometry(ground.get_ptr(), 
    ChVector<>(sizeX / 2, sizeY / 2, hDepth),
    ChVector<>(0, 0, -hDepth));
  ground->GetCollisionModel()->BuildModel();

  ChSharedPtr<ChColorAsset> groundColor(new ChColorAsset);
  groundColor->SetColor(ChColor(0.4, 0.4, 0.6));
  ground->AddAsset(groundColor);

  system.AddBody(ground);
}

