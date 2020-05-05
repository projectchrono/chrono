// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// Semitractor 28t wheel subsystem (3.7t per tire)
//
// =============================================================================

#ifndef SEMITRACTOR_WHEEL_H
#define SEMITRACTOR_WHEEL_H

#include "chrono/assets/ChTriangleMeshShape.h"

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheel.h"

#include "chrono_models/ChApiModels.h"

class SemiTractor_wheel : public chrono::vehicle::ChWheel {
  public:
    SemiTractor_wheel(const std::string& name);
    ~SemiTractor_wheel() {}

    virtual double GetMass() const override { return m_mass; }
    virtual chrono::ChVector<> GetInertia() const override { return m_inertia; }
    virtual double GetRadius() const override { return m_radius; }
    virtual double GetWidth() const override { return m_width; }

    virtual void AddVisualizationAssets(chrono::vehicle::VisualizationType vis) override;
    virtual void RemoveVisualizationAssets() override;

  protected:
    static const std::string m_meshFile;
    std::shared_ptr<chrono::ChTriangleMeshShape> m_trimesh_shape;

    static const double m_radius;
    static const double m_width;
    static const double m_mass;
    static const chrono::ChVector<> m_inertia;
};

#endif
