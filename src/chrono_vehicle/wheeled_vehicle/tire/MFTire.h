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
// Authors: Radu Serban, Michael Taylor, Rainer Gericke, Marvin Struijk
// =============================================================================
//
// MFTire 6.2-5.2 constructed with data from file (JSON format).
//
// =============================================================================

#ifndef MF_TIRE_H
#define MF_TIRE_H

#include "chrono_vehicle/wheeled_vehicle/tire/ChMFTire.h"
#include "chrono_vehicle/ChApiVehicle.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_tire
/// @{

/// PAC89 tire model from JSON file.
class CH_VEHICLE_API MFTire : public ChMFTire {
  public:
    MFTire(const std::string& filename);
    MFTire(const rapidjson::Document& d);
    ~MFTire() {}

    virtual double GetNormalStiffnessForce(double depth) const override { return 0.0; }

    virtual double GetNormalDampingForce(double depth, double velocity) const override { return 0.0; }

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
    // bool m_has_vert_table;
    // ChFunction_Recorder m_vert_map;

    // tire bottoming stiffness given as lookup table (rim impact)
    // bool m_has_bott_table;
    // ChFunction_Recorder m_bott_map;

    double m_visualization_width;
    std::string m_meshFile_left;
    std::string m_meshFile_right;
    std::shared_ptr<ChTriangleMeshShape> m_trimesh_shape;
};

/// @} vehicle_wheeled_tire

}  // namespace vehicle
}  // end namespace chrono

#endif
