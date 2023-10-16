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
// Authors: Radu Serban, Justin Madsen, Jayne Henry
// =============================================================================
//
// ARTcar wheel subsystem
//
// =============================================================================

#ifndef ARTCAR_WHEEL_H
#define ARTCAR_WHEEL_H

#include "chrono_models/ChApiModels.h"

#include "chrono/assets/ChTriangleMeshShape.h"

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheel.h"

using namespace chrono;
using namespace chrono::vehicle;

namespace chrono {
namespace vehicle {
namespace artcar {

/// @addtogroup vehicle_models_artcar
/// @{

/// ARTcar wheel base class.
class CH_MODELS_API ARTcar_Wheel : public chrono::vehicle::ChWheel {
  public:
    ARTcar_Wheel(const std::string& name);
    ~ARTcar_Wheel() {}

    virtual double GetWheelMass() const override { return m_mass; }
    virtual const ChVector<>& GetWheelInertia() const override { return m_inertia; }
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

/// ARTcar left wheel (front or rear).
class CH_MODELS_API ARTcar_WheelLeft : public ARTcar_Wheel {
  public:
    ARTcar_WheelLeft(const std::string& name);
    ~ARTcar_WheelLeft() {}

    virtual std::string GetMeshName() const override { return m_meshName; }
    virtual std::string GetMeshFile() const override { return GetDataFile(m_meshFile); }

  private:
    static const std::string m_meshName;
    static const std::string m_meshFile;
};

/// ARTcar right wheel (front or rear).
class CH_MODELS_API ARTcar_WheelRight : public ARTcar_Wheel {
  public:
    ARTcar_WheelRight(const std::string& name);
    ~ARTcar_WheelRight() {}

    virtual std::string GetMeshName() const override { return m_meshName; }
    virtual std::string GetMeshFile() const override { return GetDataFile(m_meshFile); }

  private:
    static const std::string m_meshName;
    static const std::string m_meshFile;
};

/// @} vehicle_models_artcar

}  // end namespace artcar
}  // namespace vehicle
}  // namespace chrono

#endif
