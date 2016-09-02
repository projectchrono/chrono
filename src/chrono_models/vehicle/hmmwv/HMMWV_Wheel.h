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

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheel.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

class CH_MODELS_API HMMWV_Wheel : public ChWheel {
  public:
    HMMWV_Wheel(VisualizationType visType);
    ~HMMWV_Wheel() {}

    virtual double GetMass() const override { return m_mass; }
    virtual ChVector<> GetInertia() const override { return m_inertia; }

    virtual void Initialize(std::shared_ptr<ChBody> spindle) override;

    virtual std::string getMeshName() const = 0;
    virtual std::string getMeshFile() const = 0;

    virtual void ExportMeshPovray(const std::string& out_dir) = 0;

  private:
    VisualizationType m_visType;

    static const double m_radius;
    static const double m_width;
    static const double m_mass;
    static const ChVector<> m_inertia;
};

class CH_MODELS_API HMMWV_WheelLeft : public HMMWV_Wheel {
  public:
    HMMWV_WheelLeft(VisualizationType visType);
    ~HMMWV_WheelLeft() {}

    virtual std::string getMeshName() const override { return m_meshName; }
    virtual std::string getMeshFile() const override { return GetDataFile(m_meshFile); }

    virtual void ExportMeshPovray(const std::string& out_dir) override;

  private:
    static const std::string m_meshName;
    static const std::string m_meshFile;
};

class CH_MODELS_API HMMWV_WheelRight : public HMMWV_Wheel {
  public:
    HMMWV_WheelRight(VisualizationType visType);
    ~HMMWV_WheelRight() {}

    virtual std::string getMeshName() const override { return m_meshName; }
    virtual std::string getMeshFile() const override { return GetDataFile(m_meshFile); }

    virtual void ExportMeshPovray(const std::string& out_dir) override;

  private:
    static const std::string m_meshName;
    static const std::string m_meshFile;
};

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono

#endif
