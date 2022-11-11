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

#include "chrono/core/ChCubicSpline.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_ANCFTire.h"

using namespace chrono::fea;

namespace chrono {
namespace vehicle {
namespace hmmwv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double HMMWV_ANCFTire::m_tire_radius = 0.4673;
const double HMMWV_ANCFTire::m_rim_radius = 0.2683;
const double HMMWV_ANCFTire::m_rim_width = 0.254;

const double HMMWV_ANCFTire::m_alpha = 0.005;
const double HMMWV_ANCFTire::m_default_pressure = 200e3;

const double HMMWV_ANCFTire::m_rho_0 = 0.1e4;
const ChVector<> HMMWV_ANCFTire::m_E_0(0.756e10, 0.474e8, 0.474e8);
const ChVector<> HMMWV_ANCFTire::m_nu_0(0.45, 0.45, 0.45);
const ChVector<> HMMWV_ANCFTire::m_G_0(0.1634e8, 0.1634e8, 0.1634e8);
const double HMMWV_ANCFTire::m_rho_1 = 0.2639e4;
const ChVector<> HMMWV_ANCFTire::m_E_1(0.18e12, 0.474e8, 0.474e8);
const ChVector<> HMMWV_ANCFTire::m_nu_1(0.45, 0.45, 0.45);
const ChVector<> HMMWV_ANCFTire::m_G_1(0.1634e8, 0.1634e8, 0.1634e8);
const double HMMWV_ANCFTire::m_rho_2 = 0.11e4;
const ChVector<> HMMWV_ANCFTire::m_E_2(0.474e8, 0.474e8, 0.474e8);
const ChVector<> HMMWV_ANCFTire::m_nu_2(0.45, 0.45, 0.45);
const ChVector<> HMMWV_ANCFTire::m_G_2(0.1634e8, 0.1634e8, 0.1634e8);

const int HMMWV_ANCFTire::m_num_elements_b = 2;
const int HMMWV_ANCFTire::m_num_layers_b = 3;
const std::vector<double> HMMWV_ANCFTire::m_layer_thickness_b{{0.5e-03, 0.5e-02, 0.5e-03}};
const std::vector<double> HMMWV_ANCFTire::m_ply_angle_b{{90, 0, 90}};
const std::vector<int> HMMWV_ANCFTire::m_material_id_b{{0, 2, 0}};

const int HMMWV_ANCFTire::m_num_elements_s = 4;
const int HMMWV_ANCFTire::m_num_layers_s = 3;
const std::vector<double> HMMWV_ANCFTire::m_layer_thickness_s{{0.5e-03, 0.1e-03, 0.5e-03}};
const std::vector<double> HMMWV_ANCFTire::m_ply_angle_s{{90, 0, 90}};
const std::vector<int> HMMWV_ANCFTire::m_material_id_s{{0, 2, 0}};

const int HMMWV_ANCFTire::m_num_elements_t = 6;
const int HMMWV_ANCFTire::m_num_layers_t = 4;
const std::vector<double> HMMWV_ANCFTire::m_layer_thickness_t{{0.1e-02, 0.3e-03, 0.3e-03, 0.5e-03}};
const std::vector<double> HMMWV_ANCFTire::m_ply_angle_t{{0, -20, 20, 90}};
const std::vector<int> HMMWV_ANCFTire::m_material_id_t{{2, 1, 1, 0}};

const int HMMWV_ANCFTire::m_div_circumference = 90;

const float HMMWV_ANCFTire::m_friction = 0.9f;
const float HMMWV_ANCFTire::m_restitution = 0.1f;
const float HMMWV_ANCFTire::m_Young = 2.0e6f;
const float HMMWV_ANCFTire::m_Poisson = 0.3f;
const float HMMWV_ANCFTire::m_kn = 2.0e6f;
const float HMMWV_ANCFTire::m_gn = 1.3e1f;
const float HMMWV_ANCFTire::m_kt = 1.0e6f;
const float HMMWV_ANCFTire::m_gt = 0;

const unsigned int HMMWV_ANCFTire::m_num_points = 71;
const double HMMWV_ANCFTire::m_profile[71][3] = {
    {0.000000E+00, 0.000000E+00, -1.150000E-01}, {1.428571E-02, 1.166670E-02, -1.164180E-01},
    {2.857143E-02, 2.333330E-02, -1.192300E-01}, {4.285714E-02, 3.500000E-02, -1.230200E-01},
    {5.714286E-02, 4.666670E-02, -1.273710E-01}, {7.142857E-02, 5.833330E-02, -1.318700E-01},
    {8.571429E-02, 7.000000E-02, -1.361330E-01}, {1.000000E-01, 8.166670E-02, -1.399910E-01},
    {1.142857E-01, 9.333330E-02, -1.433510E-01}, {1.285714E-01, 1.050000E-01, -1.461240E-01},
    {1.428571E-01, 1.166670E-01, -1.482160E-01}, {1.571429E-01, 1.283330E-01, -1.495390E-01},
    {1.714286E-01, 1.400000E-01, -1.500000E-01}, {1.857143E-01, 1.475000E-01, -1.486380E-01},
    {2.000000E-01, 1.550000E-01, -1.457860E-01}, {2.142857E-01, 1.625000E-01, -1.419760E-01},
    {2.285714E-01, 1.700000E-01, -1.360000E-01}, {2.428571E-01, 1.768970E-01, -1.288420E-01},
    {2.571429E-01, 1.831090E-01, -1.216840E-01}, {2.714286E-01, 1.883940E-01, -1.145260E-01},
    {2.857143E-01, 1.925100E-01, -1.073680E-01}, {3.000000E-01, 1.953230E-01, -1.002110E-01},
    {3.142857E-01, 1.970380E-01, -9.305260E-02}, {3.285714E-01, 1.979260E-01, -8.589470E-02},
    {3.428571E-01, 1.982580E-01, -7.873680E-02}, {3.571429E-01, 1.983020E-01, -7.157890E-02},
    {3.714286E-01, 1.983090E-01, -6.442110E-02}, {3.857143E-01, 1.983540E-01, -5.726320E-02},
    {4.000000E-01, 1.984290E-01, -5.010530E-02}, {4.142857E-01, 1.985240E-01, -4.294740E-02},
    {4.285714E-01, 1.986300E-01, -3.578950E-02}, {4.428571E-01, 1.987380E-01, -2.863160E-02},
    {4.571429E-01, 1.988390E-01, -2.147370E-02}, {4.714286E-01, 1.989220E-01, -1.431580E-02},
    {4.857143E-01, 1.989790E-01, -7.157890E-03}, {5.000000E-01, 1.990000E-01, 0.000000E+00},
    {5.142857E-01, 1.989790E-01, 7.157890E-03},  {5.285714E-01, 1.989220E-01, 1.431580E-02},
    {5.428571E-01, 1.988390E-01, 2.147370E-02},  {5.571429E-01, 1.987380E-01, 2.863160E-02},
    {5.714286E-01, 1.986300E-01, 3.578950E-02},  {5.857143E-01, 1.985240E-01, 4.294740E-02},
    {6.000000E-01, 1.984290E-01, 5.010530E-02},  {6.142857E-01, 1.983540E-01, 5.726320E-02},
    {6.285714E-01, 1.983090E-01, 6.442110E-02},  {6.428571E-01, 1.983020E-01, 7.157890E-02},
    {6.571429E-01, 1.982580E-01, 7.873680E-02},  {6.714286E-01, 1.979260E-01, 8.589470E-02},
    {6.857143E-01, 1.970380E-01, 9.305260E-02},  {7.000000E-01, 1.953230E-01, 1.002110E-01},
    {7.142857E-01, 1.925100E-01, 1.073680E-01},  {7.285714E-01, 1.883940E-01, 1.145260E-01},
    {7.428571E-01, 1.831090E-01, 1.216840E-01},  {7.571429E-01, 1.768970E-01, 1.288420E-01},
    {7.714286E-01, 1.700000E-01, 1.360000E-01},  {7.857143E-01, 1.625000E-01, 1.419760E-01},
    {8.000000E-01, 1.550000E-01, 1.457860E-01},  {8.142857E-01, 1.475000E-01, 1.486380E-01},
    {8.285714E-01, 1.400000E-01, 1.500000E-01},  {8.428571E-01, 1.283330E-01, 1.495390E-01},
    {8.571429E-01, 1.166670E-01, 1.482160E-01},  {8.714286E-01, 1.050000E-01, 1.461240E-01},
    {8.857143E-01, 9.333330E-02, 1.433510E-01},  {9.000000E-01, 8.166670E-02, 1.399910E-01},
    {9.142857E-01, 7.000000E-02, 1.361330E-01},  {9.285714E-01, 5.833330E-02, 1.318700E-01},
    {9.428571E-01, 4.666670E-02, 1.273710E-01},  {9.571429E-01, 3.500000E-02, 1.230200E-01},
    {9.714286E-01, 2.333330E-02, 1.192300E-01},  {9.857143E-01, 1.166670E-02, 1.164180E-01},
    {1.000000E+00, 0.000000E+00, 1.150000E-01}};

// -----------------------------------------------------------------------------

HMMWV_ANCFTire::HMMWV_ANCFTire(const std::string& name, ElementType element_type)
    : ChANCFTire(name), m_element_type(element_type) {
    m_div_width = 2 * (m_num_elements_b + m_num_elements_s + m_num_elements_t);

    // Create the vector of orthotropic layer materials
    m_materials.resize(3);
    m_materials[0] = chrono_types::make_shared<ChMaterialShellANCF>(m_rho_0, m_E_0, m_nu_0, m_G_0);
    m_materials[1] = chrono_types::make_shared<ChMaterialShellANCF>(m_rho_1, m_E_1, m_nu_1, m_G_1);
    m_materials[2] = chrono_types::make_shared<ChMaterialShellANCF>(m_rho_2, m_E_2, m_nu_2, m_G_2);

    // Set the profile
    m_profile_t.resize(m_num_points);
    m_profile_x.resize(m_num_points);
    m_profile_y.resize(m_num_points);
    for (unsigned int i = 0; i < m_num_points; i++) {
        m_profile_t[i] = m_profile[i][0];
        m_profile_x[i] = m_profile[i][1];
        m_profile_y[i] = m_profile[i][2];
    }
}

void HMMWV_ANCFTire::CreateMesh(const ChFrameMoving<>& wheel_frame, VehicleSide side) {
    std::vector<std::shared_ptr<fea::ChMaterialShellANCF>> mat_b;
    std::vector<std::shared_ptr<fea::ChMaterialShellANCF>> mat_s;
    std::vector<std::shared_ptr<fea::ChMaterialShellANCF>> mat_t;

    for (int i = 0; i < m_num_layers_b; i++)
        mat_b.push_back(m_materials[m_material_id_b[i]]);
    for (int i = 0; i < m_num_layers_s; i++)
        mat_s.push_back(m_materials[m_material_id_s[i]]);
    for (int i = 0; i < m_num_layers_t; i++)
        mat_t.push_back(m_materials[m_material_id_t[i]]);

    switch (m_element_type) {
        case ElementType::ANCF_4:
            m_rim_nodes = CreateMeshANCF4(                                                      //
                {m_profile_t, m_profile_x, m_profile_y},                                        //
                {m_num_elements_b, m_num_layers_b, m_layer_thickness_b, m_ply_angle_b, mat_b},  //
                {m_num_elements_s, m_num_layers_s, m_layer_thickness_s, m_ply_angle_s, mat_s},  //
                {m_num_elements_t, m_num_layers_t, m_layer_thickness_t, m_ply_angle_t, mat_t},  //
                m_div_circumference,                                                            //
                m_rim_radius,                                                                   //
                m_alpha,                                                                        //
                m_mesh,                                                                         //
                wheel_frame                                                                     //
            );
            break;
        case ElementType::ANCF_8:
            m_rim_nodes = CreateMeshANCF8(                                                          //
                {m_profile_t, m_profile_x, m_profile_y},                                            //
                {m_num_elements_b / 2, m_num_layers_b, m_layer_thickness_b, m_ply_angle_b, mat_b},  //
                {m_num_elements_s / 2, m_num_layers_s, m_layer_thickness_s, m_ply_angle_s, mat_s},  //
                {m_num_elements_t / 2, m_num_layers_t, m_layer_thickness_t, m_ply_angle_t, mat_t},  //
                m_div_circumference / 2,                                                            //
                m_rim_radius,                                                                       //
                m_alpha,                                                                            //
                m_mesh,                                                                             //
                wheel_frame                                                                         //
            );
            break;
    }
}

void HMMWV_ANCFTire::CreateContactMaterial() {
    m_contact_mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    m_contact_mat->SetFriction(m_friction);
    m_contact_mat->SetRestitution(m_restitution);
    m_contact_mat->SetYoungModulus(m_Young);
    m_contact_mat->SetPoissonRatio(m_Poisson);
    m_contact_mat->SetKn(m_kn);
    m_contact_mat->SetGn(m_gn);
    m_contact_mat->SetKt(m_kt);
    m_contact_mat->SetGt(m_gt);
}

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono
