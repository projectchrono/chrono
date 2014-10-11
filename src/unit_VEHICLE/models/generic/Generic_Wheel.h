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
// Generic wheel subsystem
//
// =============================================================================

#ifndef GENERIC_WHEEL_H
#define GENERIC_WHEEL_H

#include "assets/ChCylinderShape.h"
#include "assets/ChTexture.h"

#include "subsys/ChWheel.h"
#include "utils/ChUtilsData.h"
#include "models/ModelDefs.h"

class Generic_Wheel : public chrono::ChWheel
{
public:

  Generic_Wheel(VisualizationType  visType) : m_visType(visType);
  ~Generic_Wheel() {}

  virtual double GetMass() const { return 45.4; }
  virtual const chrono::ChVector<>& GetInertia() const { return chrono::ChVector<>(0.113, 0.113, 0.113); }

  virtual void Initialize(chrono::ChSharedBodyPtr spindle)
  {
    if (m_visType == PRIMITIVES) {
      double radius = 0.47;
      double width = 0.25;
      chrono::ChSharedPtr<chrono::ChCylinderShape> cyl(new chrono::ChCylinderShape);
      cyl->GetCylinderGeometry().rad = radius;
      cyl->GetCylinderGeometry().p1 = ChVector<>(0, width / 2, 0);
      cyl->GetCylinderGeometry().p2 = ChVector<>(0, -width / 2, 0);
      spindle->AddAsset(cyl);

      chrono::ChSharedPtr<chrono::ChTexture> tex(new chrono::ChTexture);
      tex->SetTextureFilename(chrono::GetChronoDataFile("bluwhite.png"));
      spindle->AddAsset(tex);
    }
  }

private:

  VisualizationType  m_visType;
};


#endif
