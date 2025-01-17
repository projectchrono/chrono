// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
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
// HMMWV ANCF-4 single-layer tire.
//
// =============================================================================

#include "chrono/core/ChCubicSpline.h"
#include "chrono_models/vehicle/hmmwv/tire/HMMWV_ANCF4LumpedTire.h"

using namespace chrono::fea;

namespace chrono {
namespace vehicle {
namespace hmmwv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double HMMWV_ANCF4LumpedTire::m_tire_radius = 0.4673;
const double HMMWV_ANCF4LumpedTire::m_rim_radius = 0.2683;
const double HMMWV_ANCF4LumpedTire::m_rim_width = 0.254;

const double HMMWV_ANCF4LumpedTire::m_alpha = 0.15;
const double HMMWV_ANCF4LumpedTire::m_default_pressure = 300e3;

const unsigned int HMMWV_ANCF4LumpedTire::m_num_points = 71;
const double HMMWV_ANCF4LumpedTire::m_profile[71][3] = {
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

HMMWV_ANCF4LumpedTire::HMMWV_ANCF4LumpedTire(const std::string& name)
    : ChANCFTire(name), m_num_div_circumference(40), m_num_elements_b(1), m_num_elements_s(2), m_num_elements_t(2) {
    // Create the contact material (default properties)
    m_contact_mat = chrono_types::make_shared<ChContactMaterialSMC>();
}

void HMMWV_ANCF4LumpedTire::SetMeshResolution(int num_div_circumference,
                                              int num_elements_b,
                                              int num_elements_s,
                                              int num_elements_t) {
    m_num_div_circumference = num_div_circumference;
    m_num_elements_b = num_elements_b;
    m_num_elements_s = num_elements_s;
    m_num_elements_t = num_elements_t;
}

void HMMWV_ANCF4LumpedTire::SetContactMaterial(std::shared_ptr<ChContactMaterialSMC> mat) {
    m_contact_mat = mat;
}

void HMMWV_ANCF4LumpedTire::CreateMesh(const ChFrameMoving<>& wheel_frame, VehicleSide side) {
    // Set the profile
    std::vector<double> profile_t(m_num_points);
    std::vector<double> profile_x(m_num_points);
    std::vector<double> profile_y(m_num_points);
    for (unsigned int i = 0; i < m_num_points; i++) {
        profile_t[i] = m_profile[i][0];
        profile_x[i] = m_profile[i][1];
        profile_y[i] = m_profile[i][2];
    }

    auto material_b =
        chrono_types::make_shared<ChMaterialShellANCF>(0.108333E+04,                                             //
                                                       ChVector3d(0.0569311E+09, 1.2995000E+09, 0.0569311E+09),  //
                                                       ChVector3d(0.0196939E+00, 0.7415553E+00, 0.4500000E+00),  //
                                                       ChVector3d(1.6344800E+07, 1.6344800E+07, 1.6344800E+07));
    auto material_s =
        chrono_types::make_shared<ChMaterialShellANCF>(0.10090909E+04,                                           //
                                                       ChVector3d(0.0484797E+09, 6.8770364E+09, 0.0484797E+09),  //
                                                       ChVector3d(0.0031508E+00, 0.4830288E+00, 0.4500000E+00),  //
                                                       ChVector3d(1.6344800E+07, 1.6344800E+07, 1.6344800E+07));
    auto material_t =
        chrono_types::make_shared<ChMaterialShellANCF>(0.11515900E+04,                                           //
                                                       ChVector3d(0.7522762E+09, 1.8424304E+09, 0.0819833E+09),  //
                                                       ChVector3d(0.0139820E+00, 0.6983265E+00, 0.6514890E+00),  //
                                                       ChVector3d(5.3971520E+09, 0.0163448E+09, 0.0163448E+09));

    m_rim_nodes = CreateMeshANCF4(                           //
        {profile_t, profile_x, profile_y},                   //
        {m_num_elements_b, 1, {9e-3}, {0}, {material_b}},    //
        {m_num_elements_s, 1, {3.1e-3}, {0}, {material_s}},  //
        {m_num_elements_t, 1, {8.1e-3}, {0}, {material_t}},  //
        m_num_div_circumference,                             //
        m_rim_radius,                                        //
        m_alpha,                                             //
        m_mesh,                                              //
        wheel_frame                                          //
    );
}

}  // namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono
