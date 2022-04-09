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
// Authors: Radu Serban, Michael Taylor
// =============================================================================
//
// JSON PAC89 tire subsystem
//
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

    virtual double GetNormalStiffnessForce(double depth) const override {
        if (m_has_vert_table) {
            if (m_has_bott_table) {
                return m_vert_map.Get_y(depth) + m_bott_map.Get_y(depth);
            } else {
                return m_vert_map.Get_y(depth);
            }
        } else {
            if (m_has_bott_table) {
                return m_PacCoeff.Cz * depth + m_bott_map.Get_y(depth);
            } else {
                return m_PacCoeff.Cz * depth;
            }
        }
    }

    virtual double GetNormalDampingForce(double depth, double velocity) const override {
        return m_PacCoeff.Kz * velocity;
    }

    virtual double GetTireMass() const override { return m_mass; }
    virtual ChVector<> GetTireInertia() const override { return m_inertia; }
    virtual void SetPac02Params() override {}

    virtual double GetVisualizationWidth() const override { return m_visualization_width; }

    virtual void AddVisualizationAssets(VisualizationType vis) override;
    virtual void RemoveVisualizationAssets() override final;

  private:
    virtual void Create(const rapidjson::Document& d) override;

    double m_mass;
    ChVector<> m_inertia;
    bool m_has_mesh;

    // tire vertical stiffness given as lookup table
    bool m_has_vert_table;
    ChFunction_Recorder m_vert_map;

    // tire bottoming stiffness given as lookup table (rim impact)
    bool m_has_bott_table;
    ChFunction_Recorder m_bott_map;

    double m_visualization_width;
    std::string m_meshFile_left;
    std::string m_meshFile_right;
    std::shared_ptr<ChTriangleMeshShape> m_trimesh_shape;
};

/// @} vehicle_wheeled_tire

}  // namespace vehicle
}  // end namespace chrono

#endif
