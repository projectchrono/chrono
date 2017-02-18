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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================
//
// HMMWV Reissner-shell tire subsystem
//
// =============================================================================

#include "chrono/core/ChCubicSpline.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_ReissnerTire.h"

using namespace chrono::fea;

namespace chrono {
namespace vehicle {
namespace hmmwv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double HMMWV_ReissnerTire::m_tire_radius = 0.4673;
const double HMMWV_ReissnerTire::m_rim_radius = 0.2683;
const double HMMWV_ReissnerTire::m_rim_width = 0.254;

const double HMMWV_ReissnerTire::m_alpha = 0.005;
const double HMMWV_ReissnerTire::m_default_pressure = 200e3;

const double HMMWV_ReissnerTire::m_rho_0 = 0.1e4;
const ChVector<> HMMWV_ReissnerTire::m_E_0(0.756e10, 0.474e8, 0.474e8);
const double HMMWV_ReissnerTire::m_nu_0 = 0.45;
const ChVector<> HMMWV_ReissnerTire::m_G_0(0.1634e8, 0.1634e8, 0.1634e8);
const double HMMWV_ReissnerTire::m_rho_1 = 0.2639e4;
const ChVector<> HMMWV_ReissnerTire::m_E_1(0.18e12, 0.474e8, 0.474e8);
const double HMMWV_ReissnerTire::m_nu_1 = 0.45;
const ChVector<> HMMWV_ReissnerTire::m_G_1(0.1634e8, 0.1634e8, 0.1634e8);
const double HMMWV_ReissnerTire::m_rho_2 = 0.11e4;
const ChVector<> HMMWV_ReissnerTire::m_E_2(0.474e8, 0.474e8, 0.474e8);
const double HMMWV_ReissnerTire::m_nu_2 = 0.45;
const ChVector<> HMMWV_ReissnerTire::m_G_2(0.1634e8, 0.1634e8, 0.1634e8);

const unsigned int HMMWV_ReissnerTire::m_num_elements_bead = 2;
const unsigned int HMMWV_ReissnerTire::m_num_layers_bead = 3;
const std::vector<double> HMMWV_ReissnerTire::m_layer_thickness_bead{{0.5e-03, 0.5e-02, 0.5e-03}};
const std::vector<double> HMMWV_ReissnerTire::m_ply_angle_bead{{90, 0, 90}};
const std::vector<int> HMMWV_ReissnerTire::m_material_id_bead{{0, 2, 0}};

const unsigned int HMMWV_ReissnerTire::m_num_elements_sidewall = 4;
const unsigned int HMMWV_ReissnerTire::m_num_layers_sidewall = 3;
const std::vector<double> HMMWV_ReissnerTire::m_layer_thickness_sidewall{{0.5e-03, 0.1e-03, 0.5e-03}};
const std::vector<double> HMMWV_ReissnerTire::m_ply_angle_sidewall{{90, 0, 90}};
const std::vector<int> HMMWV_ReissnerTire::m_material_id_sidewall{{0, 2, 0}};

const unsigned int HMMWV_ReissnerTire::m_num_elements_tread = 6;
const unsigned int HMMWV_ReissnerTire::m_num_layers_tread = 4;
const std::vector<double> HMMWV_ReissnerTire::m_layer_thickness_tread{{0.1e-02, 0.3e-03, 0.3e-03, 0.5e-03}};
const std::vector<double> HMMWV_ReissnerTire::m_ply_angle_tread{{0, -20, 20, 90}};
const std::vector<int> HMMWV_ReissnerTire::m_material_id_tread{{2, 1, 1, 0}};

const int HMMWV_ReissnerTire::m_div_circumference = 90;

const float HMMWV_ReissnerTire::m_friction = 0.9f;
const float HMMWV_ReissnerTire::m_restiturion = 0.1f;
const float HMMWV_ReissnerTire::m_Young = 2.0e6f;
const float HMMWV_ReissnerTire::m_Poisson = 0.3f;
const float HMMWV_ReissnerTire::m_kn = 2.0e6f;
const float HMMWV_ReissnerTire::m_gn = 1.3e1f;
const float HMMWV_ReissnerTire::m_kt = 1.0e6f;
const float HMMWV_ReissnerTire::m_gt = 0;

const unsigned int HMMWV_ReissnerTire::m_num_points = 71;
const double HMMWV_ReissnerTire::m_profile[71][3] = {
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

HMMWV_ReissnerTire::HMMWV_ReissnerTire(const std::string& name) : ChReissnerTire(name) {
    m_div_width = 2 * (m_num_elements_bead + m_num_elements_sidewall + m_num_elements_tread);

    // Set contact material properties
    SetContactFrictionCoefficient(m_friction);
    SetContactRestitutionCoefficient(m_restitution);
    SetContactMaterialProperties(m_Young, m_Poisson);
    SetContactMaterialCoefficients(m_kn, m_gn, m_kt, m_gt);

    // Create the vector of orthotropic layer materials
    // Initialize with (density, E_x, E_y,nu_xy, Gxy, Gxz, Gyz)
    m_materials.resize(3);
    m_materials[0] = std::make_shared<ChMaterialShellReissnerOrthotropic>(m_rho_0, m_E_0.x(), m_E_0.y(), m_nu_0, m_G_0.x(), m_G_0.y(), m_G_0.z());
    m_materials[1] = std::make_shared<ChMaterialShellReissnerOrthotropic>(m_rho_1, m_E_1.x(), m_E_1.y(), m_nu_1, m_G_1.x(), m_G_1.y(), m_G_1.z());
    m_materials[2] = std::make_shared<ChMaterialShellReissnerOrthotropic>(m_rho_2, m_E_2.x(), m_E_2.y(), m_nu_2, m_G_2.x(), m_G_2.y(), m_G_2.z());

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

void HMMWV_ReissnerTire::CreateMesh(const ChFrameMoving<>& wheel_frame, VehicleSide side) {
    // Create piece-wise cubic spline approximation of the tire profile.
    //   x - radial direction
    //   y - transversal direction
    ChCubicSpline splineX(m_profile_t, m_profile_x);
    ChCubicSpline splineY(m_profile_t, m_profile_y);

    // Create the mesh nodes.
    // The nodes are first created in the wheel local frame, assuming Y as the tire axis,
    // and are then transformed to the global frame.
    for (int i = 0; i < m_div_circumference; i++) {
        double phi = (CH_C_2PI * i) / m_div_circumference;
        ChVector<> nrm(-std::sin(phi), 0, std::cos(phi));

        for (int j = 0; j <= m_div_width; j++) {
            double t_prf = double(j) / m_div_width;
            double x_prf, xp_prf, xpp_prf;
            double y_prf, yp_prf, ypp_prf;
            splineX.Evaluate(t_prf, x_prf, xp_prf, xpp_prf);
            splineY.Evaluate(t_prf, y_prf, yp_prf, ypp_prf);

            // Node position with respect to rim center
            double x = (m_rim_radius + x_prf) * std::cos(phi);
            double y = y_prf;
            double z = (m_rim_radius + x_prf) * std::sin(phi);
            // Node position in global frame (actual coordinate values)
            ChVector<> loc = wheel_frame.TransformPointLocalToParent(ChVector<>(x, y, z));

            // Node direction
            ChVector<> tan_prf(std::cos(phi) * xp_prf, yp_prf, std::sin(phi) * xp_prf);
            ChVector<> nrm_prf = Vcross(tan_prf, nrm).GetNormalized();
            ChVector<> dir = wheel_frame.TransformDirectionLocalToParent(nrm_prf);
            ChMatrix33<> mrot; mrot.Set_A_Xdir(tan_prf,nrm_prf);
            auto node = std::make_shared<ChNodeFEAxyzrot>(ChFrame<>(loc, mrot));

            // Node velocity
            ChVector<> vel = wheel_frame.PointSpeedLocalToParent(ChVector<>(x, y, z));
            node->SetPos_dt(vel);
            node->SetMass(0);
            m_mesh->AddNode(node);
        }
    }

    // Create the Reissner shell elements
    for (int i = 0; i < m_div_circumference; i++) {
        for (int j = 0; j < m_div_width; j++) {
            // Adjacent nodes
            int inode0, inode1, inode2, inode3;
            inode1 = j + i * (m_div_width + 1);
            inode2 = j + 1 + i * (m_div_width + 1);
            if (i == m_div_circumference - 1) {
                inode0 = j;
                inode3 = j + 1;
            } else {
                inode0 = j + (i + 1) * (m_div_width + 1);
                inode3 = j + 1 + (i + 1) * (m_div_width + 1);
            }

            auto node0 = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(m_mesh->GetNode(inode0));
            auto node1 = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(m_mesh->GetNode(inode1));
            auto node2 = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(m_mesh->GetNode(inode2));
            auto node3 = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(m_mesh->GetNode(inode3));

            // Create the element and set its nodes.
            auto element = std::make_shared<ChElementShellReissner4>();
            element->SetNodes(node0, node1, node2, node3);

            // Element dimensions
            double len_circumference =
                0.5 * ((node1->GetPos() - node0->GetPos()).Length() + (node3->GetPos() - node2->GetPos()).Length());
            double len_width = (node2->GetPos() - node0->GetPos()).Length();

            // Figure out the section for this element
            int b1 = m_num_elements_bead;
            int b2 = m_div_width - m_num_elements_bead;
            int s1 = b1 + m_num_elements_sidewall;
            int s2 = b2 - m_num_elements_sidewall;
            if (j < b1 || j >= b2) {
                // Bead section
                for (unsigned int im = 0; im < m_num_layers_bead; im++) {
                    element->AddLayer(m_layer_thickness_bead[im], CH_C_DEG_TO_RAD * m_ply_angle_bead[im],
                                      m_materials[m_material_id_bead[im]]);
                }
            } else if (j < s1 || j >= s2) {
                // Sidewall section
                for (unsigned int im = 0; im < m_num_layers_sidewall; im++) {
                    element->AddLayer(m_layer_thickness_sidewall[im], CH_C_DEG_TO_RAD * m_ply_angle_sidewall[im],
                                      m_materials[m_material_id_sidewall[im]]);
                }
            } else {
                // Tread section
                for (unsigned int im = 0; im < m_num_layers_tread; im++) {
                    element->AddLayer(m_layer_thickness_tread[im], CH_C_DEG_TO_RAD * m_ply_angle_tread[im],
                                      m_materials[m_material_id_tread[im]]);
                }
            }

            // Set other element properties
            element->SetAlphaDamp(m_alpha);

            // Add element to mesh
            m_mesh->AddElement(element);
        }
    }

    // Switch on automatic gravity
    m_mesh->SetAutomaticGravity(true);
}

std::vector<std::shared_ptr<fea::ChNodeFEAbase>> HMMWV_ReissnerTire::GetConnectedNodes() const {
    std::vector<std::shared_ptr<fea::ChNodeFEAbase>> nodes;

    for (int i = 0; i < m_div_circumference; i++) {
        for (int j = 0; j <= m_div_width; j++) {
            int index = j + i * (m_div_width + 1);
            if (index % (m_div_width + 1) == 0) {
                nodes.push_back(std::dynamic_pointer_cast<fea::ChNodeFEAbase>(m_mesh->GetNode(index)));
                nodes.push_back(std::dynamic_pointer_cast<fea::ChNodeFEAbase>(m_mesh->GetNode(index + m_div_width)));
            }
        }
    }

    return nodes;
}

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono
