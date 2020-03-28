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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================
//
// HMMWV Reissner-shell tire subsystem
//
// =============================================================================

#ifndef HMMWV_REISSNER_TIRE_H
#define HMMWV_REISSNER_TIRE_H

#include "chrono/fea/ChElementShellReissner4.h"
#include "chrono_models/ChApiModels.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChReissnerTire.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

/// @addtogroup vehicle_models_hmmwv
/// @{

/// Deformable tire model for the HMMWV vehicle (using Reissner shell FEA elements)
class CH_MODELS_API HMMWV_ReissnerTire : public ChReissnerTire {
  public:
    HMMWV_ReissnerTire(const std::string& name);
    ~HMMWV_ReissnerTire() {}

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
    virtual void CreateContactMaterial() override;

    static const double m_tire_radius;
    static const double m_rim_radius;
    static const double m_rim_width;

    static const int m_div_circumference;
    int m_div_width;

    static const double m_alpha;
    static const double m_default_pressure;

    static const double m_rho_0;
    static const ChVector<> m_E_0; // Ex Ey (Ez not used)
    static const double m_nu_0;    // nu for xy shear
    static const ChVector<> m_G_0; // Gxy Gxz Gyz
    static const double m_rho_1;   
    static const ChVector<> m_E_1; // Ex Ey (Ez not used)
    static const double m_nu_1;    // nu for xy shear
    static const ChVector<> m_G_1; // Gxy Gxz Gyz
    static const double m_rho_2;
    static const ChVector<> m_E_2; // Ex Ey (Ez not used)
    static const double m_nu_2;    // nu for xy shear
    static const ChVector<> m_G_2; // Gxy Gxz Gyz
    std::vector<std::shared_ptr<fea::ChMaterialShellReissnerOrthotropic>> m_materials;

    static const unsigned int m_num_elements_bead;
    static const unsigned int m_num_layers_bead;
    static const std::vector<double> m_layer_thickness_bead;
    static const std::vector<double> m_ply_angle_bead;
    static const std::vector<int> m_material_id_bead;

    static const unsigned int m_num_elements_sidewall;
    static const unsigned int m_num_layers_sidewall;
    static const std::vector<double> m_layer_thickness_sidewall;
    static const std::vector<double> m_ply_angle_sidewall;
    static const std::vector<int> m_material_id_sidewall;

    static const unsigned int m_num_elements_tread;
    static const unsigned int m_num_layers_tread;
    static const std::vector<double> m_layer_thickness_tread;
    static const std::vector<double> m_ply_angle_tread;
    static const std::vector<int> m_material_id_tread;

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
};

/// @} vehicle_models_hmmwv

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono

#endif
