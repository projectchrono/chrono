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
// HMMWV ANCF tire subsystem
//
// =============================================================================

#ifndef HMMWV_ANCF_TIRE_H
#define HMMWV_ANCF_TIRE_H

#include "chrono/fea/ChElementShellANCF_3423.h"
#include "chrono_models/ChApiModels.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChANCFTire.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

/// @addtogroup vehicle_models_hmmwv
/// @{

/// Deformable tire model for the HMMWV vehicle (using ANCF shell FEA elements)
class CH_MODELS_API HMMWV_ANCFTire : public ChANCFTire {
  public:
    enum class ElementType {
        ANCF_4,  ///< 4-node ANCF shell element
        ANCF_8   ///< 8-node ANCF shell element
    };

    HMMWV_ANCFTire(const std::string& name, ElementType element_type = ElementType::ANCF_4);
    ~HMMWV_ANCFTire() {}

    /// Get the tire radius.
    virtual double GetRadius() const override { return m_tire_radius; }
    /// Get the rim radius (inner tire radius).
    virtual double GetRimRadius() const override { return m_rim_radius; }
    /// Get the tire width.
    virtual double GetWidth() const override { return m_rim_width; }
    /// Get the default tire pressure.
    virtual double GetDefaultPressure() const override { return m_default_pressure; }

    /// Return list of nodes connected to the rim.
    virtual std::vector<std::shared_ptr<fea::ChNodeFEAbase>> GetConnectedNodes() const override { return m_rim_nodes; }

    /// Create the FEA nodes and elements.
    /// The wheel rotational axis is assumed to be the Y axis.
    virtual void CreateMesh(const ChFrameMoving<>& wheel_frame,  ///< frame of associated wheel
                            VehicleSide side                     ///< left/right vehicle side
                            ) override;

  private:
    virtual void CreateContactMaterial() override;

    ElementType m_element_type;

    static const double m_tire_radius;
    static const double m_rim_radius;
    static const double m_rim_width;

    static const int m_div_circumference;
    int m_div_width;

    static const double m_alpha;
    static const double m_default_pressure;

    static const double m_rho_0;
    static const ChVector<> m_E_0;
    static const ChVector<> m_nu_0;
    static const ChVector<> m_G_0;
    static const double m_rho_1;
    static const ChVector<> m_E_1;
    static const ChVector<> m_nu_1;
    static const ChVector<> m_G_1;
    static const double m_rho_2;
    static const ChVector<> m_E_2;
    static const ChVector<> m_nu_2;
    static const ChVector<> m_G_2;
    std::vector<std::shared_ptr<fea::ChMaterialShellANCF>> m_materials;

    static const int m_num_elements_b;
    static const int m_num_layers_b;
    static const std::vector<double> m_layer_thickness_b;
    static const std::vector<double> m_ply_angle_b;
    static const std::vector<int> m_material_id_b;

    static const int m_num_elements_s;
    static const int m_num_layers_s;
    static const std::vector<double> m_layer_thickness_s;
    static const std::vector<double> m_ply_angle_s;
    static const std::vector<int> m_material_id_s;

    static const int m_num_elements_t;
    static const int m_num_layers_t;
    static const std::vector<double> m_layer_thickness_t;
    static const std::vector<double> m_ply_angle_t;
    static const std::vector<int> m_material_id_t;

    static const float m_friction;
    static const float m_restitution;
    static const float m_Young;
    static const float m_Poisson;
    static const float m_kn;
    static const float m_gn;
    static const float m_kt;
    static const float m_gt;

    static const unsigned int m_num_points;
    static const double m_profile[71][3];
    std::vector<double> m_profile_t;
    std::vector<double> m_profile_x;
    std::vector<double> m_profile_y;

    std::vector<std::shared_ptr<fea::ChNodeFEAbase>> m_rim_nodes;
};

/// @} vehicle_models_hmmwv

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono

#endif
