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

#include "chrono/physics/ChGlobal.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChTexture.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheel.h"

class Generic_Wheel : public chrono::vehicle::ChWheel {
  public:
    Generic_Wheel(chrono::vehicle::VisualizationType visType) : m_visType(visType) {}
    ~Generic_Wheel() {}

    virtual double GetMass() const override { return 45.4; }
    virtual chrono::ChVector<> GetInertia() const override { return chrono::ChVector<>(0.113, 0.113, 0.113); }

    virtual void Initialize(std::shared_ptr<chrono::ChBody> spindle) override {
        // First, invoke the base class method
        chrono::vehicle::ChWheel::Initialize(spindle);

        // Attach visualization
        if (m_visType == chrono::vehicle::PRIMITIVES) {
            double radius = 0.47;
            double width = 0.25;
            auto cyl = std::make_shared<chrono::ChCylinderShape>();
            cyl->GetCylinderGeometry().rad = radius;
            cyl->GetCylinderGeometry().p1 = chrono::ChVector<>(0, width / 2, 0);
            cyl->GetCylinderGeometry().p2 = chrono::ChVector<>(0, -width / 2, 0);
            spindle->AddAsset(cyl);

            auto tex = std::make_shared<chrono::ChTexture>();
            tex->SetTextureFilename(chrono::GetChronoDataFile("bluwhite.png"));
            spindle->AddAsset(tex);
        }
    }

  private:
    chrono::vehicle::VisualizationType m_visType;
};

#endif
