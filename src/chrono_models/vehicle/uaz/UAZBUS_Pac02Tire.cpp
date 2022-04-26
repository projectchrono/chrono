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
// Authors: Radu Serban, Michael Taylor, Rainer Gericke
// =============================================================================
//
// HMMWV PAC02 tire subsystem
//
// Coefficents were pulled from the Adams/Tire help - Adams 2017.1.
// https://simcompanion.mscsoftware.com/infocenter/index?page=content&id=DOC11293&cat=2017.1_ADAMS_DOCS&actp=LIST
//
// =============================================================================

#include <cmath>
#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_models/vehicle/uaz/UAZBUS_Pac02Tire.h"

namespace chrono {
namespace vehicle {
namespace uaz {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double UAZBUS_Pac02Tire::m_mass = 16.0;
const ChVector<> UAZBUS_Pac02Tire::m_inertia(1.02121, 1.82824, 1.02121);

const std::string UAZBUS_Pac02Tire::m_meshFile = "uaz/uaz_tire.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
UAZBUS_Pac02Tire::UAZBUS_Pac02Tire(const std::string& name) : ChPac02Tire(name), m_use_vert_map(false) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void UAZBUS_Pac02Tire::SetPac02Params() {  // begin of variables set up
    m_use_mode = 3;
    m_measured_side = LEFT;
    m_allow_mirroring = true;
    m_PacCoeff.R0 = 0.372;
    m_PacCoeff.width = 0.228;
    m_PacCoeff.aspect_ratio = 0.75;
    m_PacCoeff.rim_radius = 0.203;
    m_PacCoeff.rim_width = 0.1524;
    m_PacCoeff.Cz = 332737;
    m_PacCoeff.Kz = 2307.3;
    m_PacCoeff.FzNomin = 4700;
    // begin scaling factors set up
    m_PacScal.lfz0 = 2.35772;
    m_PacScal.lcx = 1;
    m_PacScal.lmux = 1;
    m_PacScal.lex = 1;
    m_PacScal.lkx = 1;
    m_PacScal.lhx = 1;
    m_PacScal.lvx = 1;
    m_PacScal.lgax = 1;
    m_PacScal.lcy = 1;
    m_PacScal.lmuy = 1;
    m_PacScal.ley = 1;
    m_PacScal.lky = 1;
    m_PacScal.lhy = 1;
    m_PacScal.lvy = 1;
    m_PacScal.lgay = 1;
    m_PacScal.ltr = 1.3191;
    m_PacScal.lres = 1;
    m_PacScal.lgaz = 1;
    m_PacScal.lxal = 1;
    m_PacScal.lyka = 1;
    m_PacScal.lvyka = 1;
    m_PacScal.ls = 1;
    m_PacScal.lsgkp = 1;
    m_PacScal.lsgal = 1;
    m_PacScal.lgyr = 1;
    m_PacScal.lmx = 1;
    m_PacScal.lvmx = 1;
    m_PacScal.lmy = 1;
    // setting longitidunal coefficients
    m_PacCoeff.pcx1 = 1.3178;
    m_PacCoeff.pdx1 = 1.0455;
    m_PacCoeff.pdx2 = 0.063954;
    m_PacCoeff.pdx3 = 0;
    m_PacCoeff.pex1 = 0.15798;
    m_PacCoeff.pex2 = 0.41141;
    m_PacCoeff.pex3 = 0.1487;
    m_PacCoeff.pex4 = 3.0004;
    m_PacCoeff.pkx1 = 23.181;
    m_PacCoeff.pkx2 = -0.037391;
    m_PacCoeff.pkx3 = 0.80348;
    m_PacCoeff.phx1 = -0.00058264;
    m_PacCoeff.phx2 = -0.0037992;
    m_PacCoeff.pvx1 = 0.045118;
    m_PacCoeff.pvx2 = 0.058244;
    m_PacCoeff.rbx1 = 13.276;
    m_PacCoeff.rbx2 = -13.778;
    m_PacCoeff.rcx1 = 1;
    m_PacCoeff.rex1 = 0;
    m_PacCoeff.rex2 = 0;
    m_PacCoeff.rhx1 = 0;
    // setting lateral coefficients
    m_PacCoeff.pcy1 = 1.2676;
    m_PacCoeff.pdy1 = 0.90031;
    m_PacCoeff.pdy2 = -0.16748;
    m_PacCoeff.pdy3 = -0.43989;
    m_PacCoeff.pey1 = -0.3442;
    m_PacCoeff.pey2 = -0.10763;
    m_PacCoeff.pey3 = 0.11513;
    m_PacCoeff.pey4 = -6.9663;
    m_PacCoeff.pky1 = -25.714;
    m_PacCoeff.pky2 = 3.2658;
    m_PacCoeff.pky3 = -0.0054467;
    m_PacCoeff.phy1 = 0.0031111;
    m_PacCoeff.phy2 = 2.1666e-05;
    m_PacCoeff.phy3 = 0.036592;
    m_PacCoeff.pvy1 = 0.0064945;
    m_PacCoeff.pvy2 = -0.0052059;
    m_PacCoeff.pvy3 = 0.013713;
    m_PacCoeff.pvy4 = -0.0092737;
    m_PacCoeff.rby1 = 7.1433;
    m_PacCoeff.rby2 = 9.1916;
    m_PacCoeff.rby3 = -0.027856;
    m_PacCoeff.rby1 = 1;
    m_PacCoeff.rey1 = 0;
    m_PacCoeff.rey2 = 0;
    m_PacCoeff.rhy1 = 0;
    m_PacCoeff.rhy2 = 0;
    m_PacCoeff.rvy1 = 0;
    m_PacCoeff.rvy2 = 0;
    m_PacCoeff.rvy3 = 0;
    m_PacCoeff.rvy4 = 0;
    m_PacCoeff.rvy5 = 1.9;
    m_PacCoeff.rvy6 = 0;
    // setting alignment coefficients
    m_PacCoeff.qbz1 = 5.6008;
    m_PacCoeff.qbz2 = -1.9968;
    m_PacCoeff.qbz3 = -0.58685;
    m_PacCoeff.qbz4 = -0.20922;
    m_PacCoeff.qbz5 = 0.2973;
    m_PacCoeff.qbz9 = 3.2333;
    m_PacCoeff.qbz10 = 0;
    m_PacCoeff.qcz1 = 1.0913;
    m_PacCoeff.qdz1 = 0.082536;
    m_PacCoeff.qdz2 = -0.011631;
    m_PacCoeff.qdz3 = -0.18704;
    m_PacCoeff.qdz4 = 0.18698;
    m_PacCoeff.qdz6 = 0.00071228;
    m_PacCoeff.qdz7 = 0.0010419;
    m_PacCoeff.qdz8 = -0.11886;
    m_PacCoeff.qdz9 = -0.011967;
    m_PacCoeff.qez1 = -35.25;
    m_PacCoeff.qez2 = -34.746;
    m_PacCoeff.qez3 = 0;
    m_PacCoeff.qez4 = 0.62393;
    m_PacCoeff.qez5 = -2.6405;
    m_PacCoeff.qhz1 = 0.0023279;
    m_PacCoeff.qhz2 = -0.0010156;
    m_PacCoeff.qhz3 = 0.030508;
    m_PacCoeff.qhz4 = 0.058344;
    m_PacCoeff.ssz1 = 0.0097546;
    m_PacCoeff.ssz2 = 0.0043624;
    m_PacCoeff.ssz3 = 0;
    m_PacCoeff.ssz4 = 0;
    // setting overturning coefficients
    m_PacCoeff.qsx1 = 0;
    m_PacCoeff.qsx2 = 0;
    m_PacCoeff.qsx3 = 0;
    // setting rolling coefficients
    m_PacCoeff.qsy1 = 0.01;
    m_PacCoeff.qsy2 = 0;
    m_PacCoeff.qsy3 = 0;
    m_PacCoeff.qsy4 = 0;
}

double UAZBUS_Pac02Tire::GetNormalStiffnessForce(double depth) const {
    if (m_use_vert_map)
        return m_vert_map.Get_y(depth);
    else
        return depth * m_PacCoeff.Cz;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void UAZBUS_Pac02Tire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile,   // left side
                                               m_meshFile);  // right side
    } else {
        ChPac02Tire::AddVisualizationAssets(vis);
    }
}

void UAZBUS_Pac02Tire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChPac02Tire::RemoveVisualizationAssets();
}

}  // namespace uaz
}  // end namespace vehicle
}  // end namespace chrono
