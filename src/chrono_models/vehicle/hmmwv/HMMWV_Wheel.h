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
// HMMWV wheel subsystem
//
// =============================================================================

#ifndef HMMWV_WHEEL_H
#define HMMWV_WHEEL_H

#include "chrono/assets/ChTriangleMeshShape.h"

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheel.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

class CH_MODELS_API HMMWV_Wheel : public ChWheel {
  public:
    HMMWV_Wheel(const std::string& name);
    ~HMMWV_Wheel() {}

    virtual double GetMass() const override { return m_mass; }
    virtual ChVector<> GetInertia() const override { return m_inertia; }
    virtual double GetRadius() const override { return m_radius; }
    virtual double GetWidth() const override { return m_width; }

    virtual void AddVisualizationAssets(VisualizationType vis) override;
    virtual void RemoveVisualizationAssets() override;

  protected:
    virtual std::string GetMeshName() const = 0;
    virtual std::string GetMeshFile() const = 0;

    std::shared_ptr<ChTriangleMeshShape> m_trimesh_shape;

    static const double m_radius;
    static const double m_width;
    static const double m_mass;
    static const ChVector<> m_inertia;
};

class CH_MODELS_API HMMWV_WheelLeft : public HMMWV_Wheel {
  public:
    HMMWV_WheelLeft(const std::string& name);
    ~HMMWV_WheelLeft() {}

    virtual std::string GetMeshName() const override { return m_meshName; }
    virtual std::string GetMeshFile() const override { return GetDataFile(m_meshFile); }

  private:
    static const std::string m_meshName;
    static const std::string m_meshFile;
};

class CH_MODELS_API HMMWV_WheelRight : public HMMWV_Wheel {
  public:
    HMMWV_WheelRight(const std::string& name);
    ~HMMWV_WheelRight() {}

    virtual std::string GetMeshName() const override { return m_meshName; }
    virtual std::string GetMeshFile() const override { return GetDataFile(m_meshFile); }

  private:
    static const std::string m_meshName;
    static const std::string m_meshFile;
};

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono

#endif
