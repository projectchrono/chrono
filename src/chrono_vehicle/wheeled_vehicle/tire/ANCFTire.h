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
// Authors: Radu Serban
// =============================================================================
//
// ANCF tire constructed with data from file (JSON format).
//
// =============================================================================

#ifndef ANCF_TIRE_H
#define ANCF_TIRE_H

#include "chrono/fea/ChElementShellANCF_3423.h"
#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChANCFTire.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_tire
/// @{

/// ANCF tire constructed with data from file (JSON format).
class CH_VEHICLE_API ANCFTire : public ChANCFTire {
  public:
    ANCFTire(const std::string& filename);
    ANCFTire(const rapidjson::Document& d);
    ~ANCFTire() {}

    /// Get the tire radius.
    virtual double GetRadius() const override { return m_tire_radius; }
    /// Get the rim radius (inner tire radius).
    virtual double GetRimRadius() const override { return m_rim_radius; }
    /// Get the tire width.
    virtual double GetWidth() const override { return m_rim_width; }
    /// Get the default tire pressure.
    virtual double GetDefaultPressure() const override { return m_default_pressure; }

    /// Return list of nodes connected to the rim.
    virtual std::vector<std::shared_ptr<fea::ChNodeFEAbase>> GetConnectedNodes() const override;

    /// Create the FEA nodes and elements.
    /// The wheel rotational axis is assumed to be the Y axis.
    virtual void CreateMesh(const ChFrameMoving<>& wheel_frame,  ///< frame of associated wheel
                            VehicleSide side                     ///< left/right vehicle side
                            ) override;

  private:
    void ProcessJSON(const rapidjson::Document& d);
    virtual void CreateContactMaterial() override;

    double m_tire_radius;
    double m_rim_radius;
    double m_rim_width;

    int m_div_circumference;
    int m_div_width;

    double m_alpha;
    double m_default_pressure;

    std::vector<std::shared_ptr<fea::ChMaterialShellANCF>> m_materials;

    unsigned int m_num_elements_bead;
    unsigned int m_num_layers_bead;
    std::vector<double> m_layer_thickness_bead;
    std::vector<double> m_ply_angle_bead;
    std::vector<int> m_material_id_bead;

    unsigned int m_num_elements_sidewall;
    unsigned int m_num_layers_sidewall;
    std::vector<double> m_layer_thickness_sidewall;
    std::vector<double> m_ply_angle_sidewall;
    std::vector<int> m_material_id_sidewall;

    unsigned int m_num_elements_tread;
    unsigned int m_num_layers_tread;
    std::vector<double> m_layer_thickness_tread;
    std::vector<double> m_ply_angle_tread;
    std::vector<int> m_material_id_tread;

    unsigned int m_num_points;
    std::vector<double> m_profile_t;
    std::vector<double> m_profile_x;
    std::vector<double> m_profile_y;

    ChContactMaterialData m_mat_info;
};

/// @} vehicle_wheeled_tire

}  // end namespace vehicle
}  // end namespace chrono

#endif
