// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Rainer Gericke, Radu Serban
// =============================================================================
//
// Template for a Magic Formula tire model
//
// ChPac02 is based on the Pacejka 2002 formulae as written in
// Hans B. Pacejka's "Tire and Vehicle Dynamics" Third Edition, Elsevier 2012
// ISBN: 978-0-08-097016-5
//
// This implementation is a subset of the commercial product MFtire:
//  - only steady state force/torque calculations
//  - uncombined (use_mode = 3)
//  - combined (use_mode = 4) via Pacejka method
//  - parametration is given by a TIR file (Tiem Orbit Format,
//    ADAMS/Car compatible)
//  - unit conversion is implemented but only tested for SI units
//  - optional inflation pressure dependency is implemented, but not tested
//  - this implementation could be validated for the FED-Alpha vehicle and rsp.
//    tire data sets against KRC test results from a Nato CDT
//
// This derived class reads parameters from a JSON parameter file
//  - input can be redirected from a TIR file
//  - input parameters can be set directly (only SI units!)
// =============================================================================

#ifndef PAC02_TIRE_H
#define PAC02_TIRE_H

#include "chrono_vehicle/wheeled_vehicle/tire/ChPac02Tire.h"
#include "chrono_vehicle/ChApiVehicle.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_tire
/// @{

/// PAC89 tire model from JSON file.
class CH_VEHICLE_API Pac02Tire : public ChPac02Tire {
  public:
    Pac02Tire(const std::string& filename);
    Pac02Tire(const rapidjson::Document& d);
    ~Pac02Tire() {}

    virtual double GetTireMass() const override { return m_mass; }
    virtual ChVector<> GetTireInertia() const override { return m_inertia; }

    virtual double GetVisualizationWidth() const override { return m_visualization_width; }

    virtual void SetMFParams() override;

    virtual void AddVisualizationAssets(VisualizationType vis) override;
    virtual void RemoveVisualizationAssets() override final;

  private:
    virtual void Create(const rapidjson::Document& d) override;

    double m_mass;
    ChVector<> m_inertia;
    std::string m_tir_file;
    bool m_has_mesh;

    double m_visualization_width;
    std::string m_meshFile_left;
    std::string m_meshFile_right;
    std::shared_ptr<ChVisualShapeTriangleMesh> m_trimesh_shape;
};

/// @} vehicle_wheeled_tire

}  // namespace vehicle
}  // end namespace chrono

#endif
