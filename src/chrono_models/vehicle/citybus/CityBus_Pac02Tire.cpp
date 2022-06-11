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
// CityBus PAC02 tire subsystem
//
// Coefficents were pulled from the Adams/Tire help - Adams 2017.1.
// https://simcompanion.mscsoftware.com/infocenter/index?page=content&id=DOC11293&cat=2017.1_ADAMS_DOCS&actp=LIST
//
// =============================================================================

#include <cmath>
#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_models/vehicle/citybus/CityBus_Pac02Tire.h"

namespace chrono {
namespace vehicle {
namespace citybus {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double CityBus_Pac02Tire::m_mass = 70.7;
const ChVector<> CityBus_Pac02Tire::m_inertia(9.04687, 16.4688, 9.04687);

const std::string CityBus_Pac02Tire::m_meshFile = "citybus/CityBusTire.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
CityBus_Pac02Tire::CityBus_Pac02Tire(const std::string& name) : ChPac02Tire(name), m_use_vert_map(false) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void CityBus_Pac02Tire::SetPac02Params() {
    // begin of variables set up
    m_use_mode = 3;
    m_allow_mirroring = false;
    m_PacCoeff.R0 = 0.54485;
    m_PacCoeff.width = 0.305;
    m_PacCoeff.aspect_ratio = 0.85;
    m_PacCoeff.rim_radius = 0.28575;
    m_PacCoeff.rim_width = 0.2286;
    m_PacCoeff.Cz = 998304;
    m_PacCoeff.Kz = 8401.19;
    m_PacCoeff.FzNomin = 29912;
    // begin scaling factors set up
    m_PacScal.lfz0 = 1.21951;
    m_PacScal.lcx = 1;
    m_PacScal.lmux = 1;
    m_PacScal.lex = 1;
    m_PacScal.lkx = 1;
    m_PacScal.lhx = 1;
    m_PacScal.lvx = 1;
    m_PacScal.lcy = 1;
    m_PacScal.lmuy = 1;
    m_PacScal.ley = 1;
    m_PacScal.lky = 1;
    m_PacScal.lhy = 1;
    m_PacScal.lvy = 1;
    m_PacScal.lgay = 1;
    m_PacScal.ltr = 1.06387;
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
    m_PacScal.lmy = 1;
    // setting longitidunal coefficients
    m_PacCoeff.pcx1 = 1.4;
    m_PacCoeff.pdx1 = 0.84003;
    m_PacCoeff.pdx2 = -0.065962;
    m_PacCoeff.pex1 = -4.5309;
    m_PacCoeff.pex2 = -3.0987;
    m_PacCoeff.pex3 = 0.20647;
    m_PacCoeff.pex4 = 0;
    m_PacCoeff.pkx1 = 6.3425;
    m_PacCoeff.pkx2 = -1.9878e-05;
    m_PacCoeff.pkx3 = -0.16666;
    m_PacCoeff.phx1 = 0;
    m_PacCoeff.phx2 = 0;
    m_PacCoeff.pvx1 = -0;
    m_PacCoeff.pvx2 = 0;
    m_PacCoeff.rbx1 = 10;
    m_PacCoeff.rbx2 = 6;
    m_PacCoeff.rcx1 = 1;
    m_PacCoeff.rhx1 = 0;
    // setting lateral coefficients
    m_PacCoeff.pcy1 = 0.54764;
    m_PacCoeff.pdy1 = -1.1188;
    m_PacCoeff.pdy2 = 0.072812;
    m_PacCoeff.pdy3 = -1.7244;
    m_PacCoeff.pey1 = 0.056372;
    m_PacCoeff.pey2 = -0.065607;
    m_PacCoeff.pey3 = -0.28765;
    m_PacCoeff.pey4 = 63.843;
    m_PacCoeff.pky1 = -9.5432;
    m_PacCoeff.pky2 = 2.4559;
    m_PacCoeff.pky3 = 0.62823;
    m_PacCoeff.phy1 = 0.0035499;
    m_PacCoeff.phy2 = 0.0045166;
    m_PacCoeff.phy3 = -0.035468;
    m_PacCoeff.pvy1 = 0.0031041;
    m_PacCoeff.pvy2 = 0.009559;
    m_PacCoeff.pvy3 = -0.13882;
    m_PacCoeff.pvy4 = -0.25693;
    m_PacCoeff.rby1 = 0;
    m_PacCoeff.rby2 = 0;
    m_PacCoeff.rby3 = 0;
    m_PacCoeff.rby1 = 0;
    m_PacCoeff.rhy1 = 0;
    m_PacCoeff.rvy1 = 0;
    m_PacCoeff.rvy2 = 0;
    m_PacCoeff.rvy3 = 0;
    m_PacCoeff.rvy4 = 0;
    m_PacCoeff.rvy5 = 0;
    m_PacCoeff.rvy6 = 0;
    // setting alignment coefficients
    m_PacCoeff.qbz1 = 8.5499;
    m_PacCoeff.qbz2 = -2.0123;
    m_PacCoeff.qbz3 = -9.7502;
    m_PacCoeff.qbz4 = 0.40886;
    m_PacCoeff.qbz5 = -0.75474;
    m_PacCoeff.qbz9 = 0.49999;
    m_PacCoeff.qcz1 = 1.4;
    m_PacCoeff.qdz1 = 0.080379;
    m_PacCoeff.qdz2 = -0.02931;
    m_PacCoeff.qdz3 = -0.032073;
    m_PacCoeff.qdz4 = 0.29184;
    m_PacCoeff.qdz6 = -0.0025776;
    m_PacCoeff.qdz7 = -0.0014791;
    m_PacCoeff.qdz8 = -0.020474;
    m_PacCoeff.qdz9 = 0.0044162;
    m_PacCoeff.qez1 = -0.017913;
    m_PacCoeff.qez2 = -0.73133;
    m_PacCoeff.qez3 = -4.7227;
    m_PacCoeff.qez4 = 0.32329;
    m_PacCoeff.qez5 = 2.5289;
    m_PacCoeff.qhz1 = -0.0011513;
    m_PacCoeff.qhz2 = -0.0057439;
    m_PacCoeff.qhz3 = 0.12163;
    m_PacCoeff.qhz4 = 0.10576;
    m_PacCoeff.ssz1 = 0;
    m_PacCoeff.ssz2 = 0;
    m_PacCoeff.ssz3 = 0;
    m_PacCoeff.ssz4 = 0;
    // setting overturning coefficients
    m_PacCoeff.qsx1 = 0;
    m_PacCoeff.qsx2 = 0;
    m_PacCoeff.qsx3 = 0;
    // setting rolling coefficients
    m_PacCoeff.qsy1 = 0.015;
    m_PacCoeff.qsy2 = 0;
}

double CityBus_Pac02Tire::GetNormalStiffnessForce(double depth) const {
    if (m_use_vert_map)
        return m_vert_map.Get_y(depth);
    else
        return depth * m_PacCoeff.Cz;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void CityBus_Pac02Tire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile,   // left side
                                               m_meshFile);  // right side
    } else {
        ChPac02Tire::AddVisualizationAssets(vis);
    }
}

void CityBus_Pac02Tire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChPac02Tire::RemoveVisualizationAssets();
}

}  // namespace citybus
}  // end namespace vehicle
}  // end namespace chrono
