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
// Authors: Radu Serban
// =============================================================================
//
// ANCF tire constructed with data from file (JSON format).
//
// =============================================================================

#include "chrono/core/ChCubicSpline.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ANCFTire.h"

#include "chrono_thirdparty/rapidjson/filereadstream.h"

using namespace chrono::fea;
using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// These utility functions return a ChVector and a ChQuaternion, respectively,
// from the specified JSON array.
// -----------------------------------------------------------------------------
static ChVector<> loadVector(const Value& a) {
    assert(a.IsArray());
    assert(a.Size() == 3);
    return ChVector<>(a[0u].GetDouble(), a[1u].GetDouble(), a[2u].GetDouble());
}

static ChQuaternion<> loadQuaternion(const Value& a) {
    assert(a.IsArray());
    assert(a.Size() == 4);
    return ChQuaternion<>(a[0u].GetDouble(), a[1u].GetDouble(), a[2u].GetDouble(), a[3u].GetDouble());
}

// -----------------------------------------------------------------------------
// Constructors for ANCFTire
// -----------------------------------------------------------------------------
ANCFTire::ANCFTire(const std::string& filename) : ChANCFTire("") {
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

    ProcessJSON(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

ANCFTire::ANCFTire(const rapidjson::Document& d) : ChANCFTire("") {
    ProcessJSON(d);
}

// -----------------------------------------------------------------------------
// Process the specified JSON document and load tire specification
// -----------------------------------------------------------------------------
void ANCFTire::ProcessJSON(const rapidjson::Document& d) {
    // Read top-level data
    assert(d.HasMember("Type"));
    assert(d.HasMember("Template"));
    assert(d.HasMember("Name"));

    SetName(d["Name"].GetString());
    
    // Read geometric dimensions
    m_tire_radius = d["Tire Radius"].GetDouble();
    m_rim_radius = d["Rim Radius"].GetDouble();
    m_rim_width = d["Rim Width"].GetDouble();

    // Read contact material data
    assert(d.HasMember("Contact Material"));

    float mu = d["Contact Material"]["Coefficient of Friction"].GetFloat();
    float cr = d["Contact Material"]["Coefficient of Restitution"].GetFloat();

    SetContactFrictionCoefficient(mu);
    SetContactRestitutionCoefficient(cr);

    if (d["Contact Material"].HasMember("Properties")) {
        float ym = d["Contact Material"]["Properties"]["Young Modulus"].GetFloat();
        float pr = d["Contact Material"]["Properties"]["Poisson Ratio"].GetFloat();
        SetContactMaterialProperties(ym, pr);
    }
    if (d["Contact Material"].HasMember("Coefficients")) {
        float kn = d["Contact Material"]["Coefficients"]["Normal Stiffness"].GetFloat();
        float gn = d["Contact Material"]["Coefficients"]["Normal Damping"].GetFloat();
        float kt = d["Contact Material"]["Coefficients"]["Tangential Stiffness"].GetFloat();
        float gt = d["Contact Material"]["Coefficients"]["Tangential Damping"].GetFloat();
        SetContactMaterialCoefficients(kn, gn, kt, gt);
    }

    // Read the list of materials (note that order is important)
    int num_materials = d["Materials"].Size();
    m_materials.resize(num_materials);
    for (int i = 0; i < num_materials; i++) {
        std::string type = d["Materials"][i]["Type"].GetString();
        if (type.compare("Isotropic") == 0) {
            double rho = d["Materials"][i]["Density"].GetDouble();
            double E = d["Materials"][i]["E"].GetDouble();
            double nu = d["Materials"][i]["nu"].GetDouble();
            m_materials[i] = std::make_shared<ChMaterialShellANCF>(rho, E, nu);
        } else if (type.compare("Orthotropic") == 0) {
            double rho = d["Materials"][i]["Density"].GetDouble();
            ChVector<> E = loadVector(d["Materials"][i]["E"]);
            ChVector<> nu = loadVector(d["Materials"][i]["nu"]);
            ChVector<> G = loadVector(d["Materials"][i]["G"]);
            m_materials[i] = std::make_shared<ChMaterialShellANCF>(rho, E, nu, G);
        }
    }

    // Structural damping
    m_alpha = d["Structural Damping Coefficient"].GetDouble();

    // Default tire pressure
    m_default_pressure = d["Default Pressure"].GetDouble();

    // Read layer information for the Bead Section
    m_num_layers_bead = d["Bead Section"]["Layer Thickness"].Size();
    assert(d["Bead Section"]["Ply Angle"].Size() == m_num_layers_bead);
    assert(d["Bead Section"]["Material ID"].Size() == m_num_layers_bead);
    for (unsigned int i = 0; i < m_num_layers_bead; i++) {
        double thickness = d["Bead Section"]["Layer Thickness"][i].GetDouble();
        double angle = d["Bead Section"]["Ply Angle"][i].GetDouble();
        int id = d["Bead Section"]["Material ID"][i].GetInt();
        assert(id >= 0 && id < num_materials);
        m_layer_thickness_bead.push_back(thickness);
        m_ply_angle_bead.push_back(angle);
        m_material_id_bead.push_back(id);
    }
    m_num_elements_bead = d["Bead Section"]["Number Elements"].GetInt();

    // Read layer information for the Sidewall Section
    m_num_layers_sidewall = d["Sidewall Section"]["Layer Thickness"].Size();
    assert(d["Sidewall Section"]["Ply Angle"].Size() == m_num_layers_sidewall);
    assert(d["Sidewall Section"]["Material ID"].Size() == m_num_layers_sidewall);
    for (unsigned int i = 0; i < m_num_layers_sidewall; i++) {
        double thickness = d["Sidewall Section"]["Layer Thickness"][i].GetDouble();
        double angle = d["Sidewall Section"]["Ply Angle"][i].GetDouble();
        int id = d["Sidewall Section"]["Material ID"][i].GetInt();
        assert(id >= 0 && id < num_materials);
        m_layer_thickness_sidewall.push_back(thickness);
        m_ply_angle_sidewall.push_back(angle);
        m_material_id_sidewall.push_back(id);
    }
    m_num_elements_sidewall = d["Sidewall Section"]["Number Elements"].GetInt();

    // Read layer information for the Tread Section
    m_num_layers_tread = d["Tread Section"]["Layer Thickness"].Size();
    assert(d["Tread Section"]["Ply Angle"].Size() == m_num_layers_tread);
    assert(d["Tread Section"]["Material ID"].Size() == m_num_layers_tread);
    for (unsigned int i = 0; i < m_num_layers_tread; i++) {
        double thickness = d["Tread Section"]["Layer Thickness"][i].GetDouble();
        double angle = d["Tread Section"]["Ply Angle"][i].GetDouble();
        int id = d["Tread Section"]["Material ID"][i].GetInt();
        assert(id >= 0 && id < num_materials);
        m_layer_thickness_tread.push_back(thickness);
        m_ply_angle_tread.push_back(angle);
        m_material_id_tread.push_back(id);
    }
    m_num_elements_tread = d["Tread Section"]["Number Elements"].GetInt();

    // Number of elements in the two orthogonal directions
    m_div_circumference = d["Number Elements Circumference"].GetInt();
    m_div_width = 2 * (m_num_elements_bead + m_num_elements_sidewall + m_num_elements_tread);

    // Read profile specification
    m_num_points = d["Profile"].Size();
    m_profile_t.resize(m_num_points);
    m_profile_x.resize(m_num_points);
    m_profile_y.resize(m_num_points);
    for (unsigned int i = 0; i < m_num_points; i++) {
        m_profile_t[i] = d["Profile"][i][0u].GetDouble();
        m_profile_x[i] = d["Profile"][i][1u].GetDouble();
        m_profile_y[i] = d["Profile"][i][2u].GetDouble();
    }
}

// -----------------------------------------------------------------------------
// Create the FEA mesh
// -----------------------------------------------------------------------------
void ANCFTire::CreateMesh(const ChFrameMoving<>& wheel_frame, VehicleSide side) {
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

            auto node = std::make_shared<ChNodeFEAxyzD>(loc, dir);

            // Node velocity
            ChVector<> vel = wheel_frame.PointSpeedLocalToParent(ChVector<>(x, y, z));
            node->SetPos_dt(vel);
            node->SetMass(0);
            m_mesh->AddNode(node);
        }
    }

    // Create the ANCF shell elements
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

            auto node0 = std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_mesh->GetNode(inode0));
            auto node1 = std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_mesh->GetNode(inode1));
            auto node2 = std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_mesh->GetNode(inode2));
            auto node3 = std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_mesh->GetNode(inode3));

            // Create the element and set its nodes.
            auto element = std::make_shared<ChElementShellANCF>();
            element->SetNodes(node0, node1, node2, node3);

            // Element dimensions
            double len_circumference =
                0.5 * ((node1->GetPos() - node0->GetPos()).Length() + (node3->GetPos() - node2->GetPos()).Length());
            double len_width = (node2->GetPos() - node0->GetPos()).Length();

            element->SetDimensions(len_circumference, len_width);

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
            element->SetGravityOn(true);

            // Add element to mesh
            m_mesh->AddElement(element);
        }
    }

    // Switch off automatic gravity
    m_mesh->SetAutomaticGravity(false);
}

std::vector<std::shared_ptr<fea::ChNodeFEAbase>> ANCFTire::GetConnectedNodes() const {
    std::vector<std::shared_ptr<fea::ChNodeFEAbase>> nodes;

    for (int i = 0; i < m_div_circumference; i++) {
        for (int j = 0; j <= m_div_width; j++) {
            int index = j + i * (m_div_width + 1);
            if (index % (m_div_width + 1) == 0) {
                nodes.push_back(std::dynamic_pointer_cast<ChNodeFEAbase>(m_mesh->GetNode(index)));
                nodes.push_back(std::dynamic_pointer_cast<ChNodeFEAbase>(m_mesh->GetNode(index + m_div_width)));
            }
        }
    }

    return nodes;
}

}  // end namespace vehicle
}  // end namespace chrono
