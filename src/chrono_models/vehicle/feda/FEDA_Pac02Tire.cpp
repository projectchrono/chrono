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
// FEDA PAC02 tire subsystem
//
// Coefficents were pulled from the Adams/Tire help - Adams 2017.1.
// https://simcompanion.mscsoftware.com/infocenter/index?page=content&id=DOC11293&cat=2017.1_ADAMS_DOCS&actp=LIST
//
// =============================================================================

#include <cmath>
#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_models/vehicle/feda/FEDA_Pac02Tire.h"

namespace chrono {
namespace vehicle {
namespace feda {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double FEDA_Pac02Tire::m_mass = 55.4;
const ChVector<> FEDA_Pac02Tire::m_inertia(6.39, 11.31, 6.39);

const std::string FEDA_Pac02Tire::m_meshFile_left = "feda/meshes/feda_tire_fine.obj";
const std::string FEDA_Pac02Tire::m_meshFile_right = "feda/meshes/feda_tire_fine.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
FEDA_Pac02Tire::FEDA_Pac02Tire(const std::string& name, unsigned int pressure_level)
    : ChPac02Tire(name), m_use_vert_map(false), m_tire_inflation_pressure_level(pressure_level) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void FEDA_Pac02Tire::SetPac02Params() {
    switch (m_tire_inflation_pressure_level) {
        case 1:
            GetLog() << "Tire Inflation Pressure set to 40 psi\n";
            SetParametersLevel1();
            break;
        default:
        case 2:
            GetLog() << "Tire Inflation Pressure set to 60 psi\n";
            SetParametersLevel2();
            break;
        case 3:
            GetLog() << "Tire Inflation Pressure set to 70 psi\n";
            SetParametersLevel3();
            break;
        case 4:
            GetLog() << "Tire Inflation Pressure set to 95 psi\n";
            SetParametersLevel4();
            break;
    }
}

void FEDA_Pac02Tire::SetParametersLevel1() {
    m_use_mode = 3;
    m_allow_mirroring = false;
    m_PacCoeff.R0 = 0.4987;
    m_PacCoeff.width = 0.335;
    m_PacCoeff.aspect_ratio = 0.65;
    m_PacCoeff.rim_radius = 0.2858;
    m_PacCoeff.rim_width = 0.2286;
    m_PacCoeff.Cz = 406520;
    m_PacCoeff.Kz = 637.589;
    m_PacCoeff.FzNomin = 16929;
    // begin scaling factors set up
    m_PacScal.lfz0 = 1;
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
    m_PacScal.ltr = 1;
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
    m_PacCoeff.pdx1 = 0.98412;
    m_PacCoeff.pdx2 = -0.043414;
    m_PacCoeff.pex1 = -6.9271;
    m_PacCoeff.pex2 = -4.498;
    m_PacCoeff.pex3 = 0.014378;
    m_PacCoeff.pex4 = 0;
    m_PacCoeff.pkx1 = 8.5834;
    m_PacCoeff.pkx2 = 0.00010235;
    m_PacCoeff.pkx3 = -0.066015;
    m_PacCoeff.phx1 = 0;
    m_PacCoeff.phx2 = 0;
    m_PacCoeff.pvx1 = -0;
    m_PacCoeff.pvx2 = 0;
    m_PacCoeff.rbx1 = 10;
    m_PacCoeff.rbx2 = 6;
    m_PacCoeff.rcx1 = 1;
    m_PacCoeff.rhx1 = 0;
    // setting lateral coefficients
    m_PacCoeff.pcy1 = 1.5328;
    m_PacCoeff.pdy1 = -0.70977;
    m_PacCoeff.pdy2 = 0.14161;
    m_PacCoeff.pdy3 = -4.4676;
    m_PacCoeff.pey1 = 0.085482;
    m_PacCoeff.pey2 = -0.041758;
    m_PacCoeff.pey3 = 0.83929;
    m_PacCoeff.pey4 = 99.846;
    m_PacCoeff.pky1 = -14.584;
    m_PacCoeff.pky2 = 2.1214;
    m_PacCoeff.pky3 = 0.69802;
    m_PacCoeff.phy1 = 0.0047162;
    m_PacCoeff.phy2 = 9.7415e-05;
    m_PacCoeff.phy3 = -0.042423;
    m_PacCoeff.pvy1 = 0.01392;
    m_PacCoeff.pvy2 = -0.011621;
    m_PacCoeff.pvy3 = -0.25674;
    m_PacCoeff.pvy4 = -0.032037;
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
    m_PacCoeff.qbz1 = 13.04;
    m_PacCoeff.qbz2 = -4.2651;
    m_PacCoeff.qbz3 = -10.824;
    m_PacCoeff.qbz4 = 0.75764;
    m_PacCoeff.qbz5 = -1.3346;
    m_PacCoeff.qbz9 = 0.50001;
    m_PacCoeff.qcz1 = 1.4;
    m_PacCoeff.qdz1 = 0.077735;
    m_PacCoeff.qdz2 = -0.022013;
    m_PacCoeff.qdz3 = -0.030204;
    m_PacCoeff.qdz4 = 0.28203;
    m_PacCoeff.qdz6 = -0.0012986;
    m_PacCoeff.qdz7 = -3.6748e-05;
    m_PacCoeff.qdz8 = -0.010173;
    m_PacCoeff.qdz9 = 0.03622;
    m_PacCoeff.qez1 = -0.28175;
    m_PacCoeff.qez2 = -0.37831;
    m_PacCoeff.qez3 = -1.2116;
    m_PacCoeff.qez4 = 0.69676;
    m_PacCoeff.qez5 = 17.861;
    m_PacCoeff.qhz1 = 0.0027781;
    m_PacCoeff.qhz2 = -0.0070641;
    m_PacCoeff.qhz3 = 0.054028;
    m_PacCoeff.qhz4 = 0.2104;
    m_PacCoeff.ssz1 = 0;
    m_PacCoeff.ssz2 = 0;
    m_PacCoeff.ssz3 = 0;
    m_PacCoeff.ssz4 = 0;
    // setting overturning coefficients
    m_PacCoeff.qsx1 = 0;
    m_PacCoeff.qsx2 = 0;
    m_PacCoeff.qsx3 = 0;
    // setting rolling coefficients
    m_PacCoeff.qsy1 = 0;
    m_PacCoeff.qsy2 = 0;

    // setting vertical table
    m_use_vert_map = true;
    m_vert_map.AddPoint(0, 0);
    m_vert_map.AddPoint(0.005, 1505.41);
    m_vert_map.AddPoint(0.01, 3154);
    m_vert_map.AddPoint(0.015, 4931.89);
    m_vert_map.AddPoint(0.02, 6825.2);
    m_vert_map.AddPoint(0.025, 8820.06);
    m_vert_map.AddPoint(0.03, 10902.6);
    m_vert_map.AddPoint(0.035, 13058.9);
    m_vert_map.AddPoint(0.04, 15275.2);
    m_vert_map.AddPoint(0.045, 17537.5);
    m_vert_map.AddPoint(0.05, 19832);
    m_vert_map.AddPoint(0.055, 22144.8);
    m_vert_map.AddPoint(0.06, 24462);
    m_vert_map.AddPoint(0.065, 26769.8);
    m_vert_map.AddPoint(0.07, 29054.2);
    m_vert_map.AddPoint(0.075, 31301.4);
    m_vert_map.AddPoint(0.08, 33497.6);
    m_vert_map.AddPoint(0.085, 35628.8);
    m_vert_map.AddPoint(0.09, 37681.2);
    m_vert_map.AddPoint(0.095, 39640.9);
    m_vert_map.AddPoint(0.1, 41494);

    // setting bottoming table
    m_use_bott_map = true;
    m_bott_map.AddPoint(0, 0);
    m_bott_map.AddPoint(0.10546, 0);
    m_bott_map.AddPoint(0.30546, 563080);
}

void FEDA_Pac02Tire::SetParametersLevel2() {
    // begin of variables set up
    m_use_mode = 3;
    m_allow_mirroring = false;
    m_PacCoeff.R0 = 0.4987;
    m_PacCoeff.width = 0.335;
    m_PacCoeff.aspect_ratio = 0.65;
    m_PacCoeff.rim_radius = 0.2858;
    m_PacCoeff.rim_width = 0.2286;
    m_PacCoeff.Cz = 565190;
    m_PacCoeff.Kz = 751.791;
    m_PacCoeff.FzNomin = 21674;
    // begin scaling factors set up
    m_PacScal.lfz0 = 1;
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
    m_PacScal.ltr = 1;
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
    m_PacCoeff.pdx1 = 0.93385;
    m_PacCoeff.pdx2 = -0.043779;
    m_PacCoeff.pex1 = -5.8966;
    m_PacCoeff.pex2 = -7.0247;
    m_PacCoeff.pex3 = -0.21695;
    m_PacCoeff.pex4 = 0;
    m_PacCoeff.pkx1 = 7.5991;
    m_PacCoeff.pkx2 = 2.0158e-05;
    m_PacCoeff.pkx3 = -0.11869;
    m_PacCoeff.phx1 = 0;
    m_PacCoeff.phx2 = 0;
    m_PacCoeff.pvx1 = -0;
    m_PacCoeff.pvx2 = 0;
    m_PacCoeff.rbx1 = 10;
    m_PacCoeff.rbx2 = 6;
    m_PacCoeff.rcx1 = 1;
    m_PacCoeff.rhx1 = 0;
    // setting lateral coefficients
    m_PacCoeff.pcy1 = 1.2742;
    m_PacCoeff.pdy1 = -0.73151;
    m_PacCoeff.pdy2 = 0.10076;
    m_PacCoeff.pdy3 = -1.6121;
    m_PacCoeff.pey1 = 0.069355;
    m_PacCoeff.pey2 = -0.045834;
    m_PacCoeff.pey3 = 0.23519;
    m_PacCoeff.pey4 = 89.965;
    m_PacCoeff.pky1 = -12.265;
    m_PacCoeff.pky2 = 2.3291;
    m_PacCoeff.pky3 = 0.39846;
    m_PacCoeff.phy1 = 0.0041814;
    m_PacCoeff.phy2 = 0.0019571;
    m_PacCoeff.phy3 = -0.038875;
    m_PacCoeff.pvy1 = 0.0078979;
    m_PacCoeff.pvy2 = -0.0033858;
    m_PacCoeff.pvy3 = -0.21044;
    m_PacCoeff.pvy4 = -0.13928;
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
    m_PacCoeff.qbz1 = 10.231;
    m_PacCoeff.qbz2 = -2.8746;
    m_PacCoeff.qbz3 = -9.9609;
    m_PacCoeff.qbz4 = 0.58091;
    m_PacCoeff.qbz5 = -0.52975;
    m_PacCoeff.qbz9 = 0.5;
    m_PacCoeff.qcz1 = 1.4;
    m_PacCoeff.qdz1 = 0.079179;
    m_PacCoeff.qdz2 = -0.024616;
    m_PacCoeff.qdz3 = -0.031977;
    m_PacCoeff.qdz4 = 0.1399;
    m_PacCoeff.qdz6 = -0.0022134;
    m_PacCoeff.qdz7 = -0.0010696;
    m_PacCoeff.qdz8 = -0.017916;
    m_PacCoeff.qdz9 = 0.023003;
    m_PacCoeff.qez1 = -0.20626;
    m_PacCoeff.qez2 = -0.58411;
    m_PacCoeff.qez3 = -3.2451;
    m_PacCoeff.qez4 = 0.45327;
    m_PacCoeff.qez5 = 7.8689;
    m_PacCoeff.qhz1 = 0.0012666;
    m_PacCoeff.qhz2 = -0.0069367;
    m_PacCoeff.qhz3 = 0.090016;
    m_PacCoeff.qhz4 = 0.1671;
    m_PacCoeff.ssz1 = 0;
    m_PacCoeff.ssz2 = 0;
    m_PacCoeff.ssz3 = 0;
    m_PacCoeff.ssz4 = 0;
    // setting overturning coefficients
    m_PacCoeff.qsx1 = 0;
    m_PacCoeff.qsx2 = 0;
    m_PacCoeff.qsx3 = 0;
    // setting rolling coefficients
    m_PacCoeff.qsy1 = 0;
    m_PacCoeff.qsy2 = 0;

    // setting vertical table
    m_use_vert_map = true;
    m_vert_map.AddPoint(0, 0);
    m_vert_map.AddPoint(0.005, 2004.06);
    m_vert_map.AddPoint(0.01, 4242.26);
    m_vert_map.AddPoint(0.015, 6688.46);
    m_vert_map.AddPoint(0.02, 9316.51);
    m_vert_map.AddPoint(0.025, 12100.2);
    m_vert_map.AddPoint(0.03, 15013.5);
    m_vert_map.AddPoint(0.035, 18030.2);
    m_vert_map.AddPoint(0.04, 21124.2);
    m_vert_map.AddPoint(0.045, 24269.2);
    m_vert_map.AddPoint(0.05, 27439.2);
    m_vert_map.AddPoint(0.055, 30607.9);
    m_vert_map.AddPoint(0.06, 33749.4);
    m_vert_map.AddPoint(0.065, 36837.3);
    m_vert_map.AddPoint(0.07, 39845.5);
    m_vert_map.AddPoint(0.075, 42748);
    m_vert_map.AddPoint(0.08, 45518.5);
    m_vert_map.AddPoint(0.085, 48130.9);
    m_vert_map.AddPoint(0.09, 50559.1);
    m_vert_map.AddPoint(0.095, 52776.8);
    m_vert_map.AddPoint(0.1, 54758);

    // setting bottoming table
    m_use_bott_map = true;
    m_bott_map.AddPoint(0, 0);
    m_bott_map.AddPoint(0.10546, 0);
    m_bott_map.AddPoint(0.30546, 563080);
}

void FEDA_Pac02Tire::SetParametersLevel3() {
    // begin of variables set up
    m_use_mode = 3;
    m_allow_mirroring = false;
    m_PacCoeff.R0 = 0.4987;
    m_PacCoeff.width = 0.335;
    m_PacCoeff.aspect_ratio = 0.65;
    m_PacCoeff.rim_radius = 0.2858;
    m_PacCoeff.rim_width = 0.2286;
    m_PacCoeff.Cz = 644520;
    m_PacCoeff.Kz = 802.82;
    m_PacCoeff.FzNomin = 24046;
    // begin scaling factors set up
    m_PacScal.lfz0 = 1;
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
    m_PacScal.ltr = 1;
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
    m_PacCoeff.pdx1 = 0.90872;
    m_PacCoeff.pdx2 = -0.043961;
    m_PacCoeff.pex1 = -5.3813;
    m_PacCoeff.pex2 = -8.288;
    m_PacCoeff.pex3 = -0.33261;
    m_PacCoeff.pex4 = 0;
    m_PacCoeff.pkx1 = 7.107;
    m_PacCoeff.pkx2 = -2.0938e-05;
    m_PacCoeff.pkx3 = -0.14503;
    m_PacCoeff.phx1 = 0;
    m_PacCoeff.phx2 = 0;
    m_PacCoeff.pvx1 = -0;
    m_PacCoeff.pvx2 = 0;
    m_PacCoeff.rbx1 = 10;
    m_PacCoeff.rbx2 = 6;
    m_PacCoeff.rcx1 = 1;
    m_PacCoeff.rhx1 = 0;
    // setting lateral coefficients
    m_PacCoeff.pcy1 = 1.1449;
    m_PacCoeff.pdy1 = -0.74238;
    m_PacCoeff.pdy2 = 0.080332;
    m_PacCoeff.pdy3 = -0.18433;
    m_PacCoeff.pey1 = 0.061291;
    m_PacCoeff.pey2 = -0.047872;
    m_PacCoeff.pey3 = -0.066861;
    m_PacCoeff.pey4 = 85.025;
    m_PacCoeff.pky1 = -11.106;
    m_PacCoeff.pky2 = 2.4329;
    m_PacCoeff.pky3 = 0.24868;
    m_PacCoeff.phy1 = 0.003914;
    m_PacCoeff.phy2 = 0.002887;
    m_PacCoeff.phy3 = -0.037101;
    m_PacCoeff.pvy1 = 0.0048596;
    m_PacCoeff.pvy2 = 0.00073183;
    m_PacCoeff.pvy3 = -0.18729;
    m_PacCoeff.pvy4 = -0.1929;
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
    m_PacCoeff.qbz1 = 8.8264;
    m_PacCoeff.qbz2 = -2.1794;
    m_PacCoeff.qbz3 = -9.5293;
    m_PacCoeff.qbz4 = 0.49255;
    m_PacCoeff.qbz5 = -0.12732;
    m_PacCoeff.qbz9 = 0.49999;
    m_PacCoeff.qcz1 = 1.4;
    m_PacCoeff.qdz1 = 0.079901;
    m_PacCoeff.qdz2 = -0.025918;
    m_PacCoeff.qdz3 = -0.032863;
    m_PacCoeff.qdz4 = 0.068831;
    m_PacCoeff.qdz6 = -0.0026708;
    m_PacCoeff.qdz7 = -0.0015861;
    m_PacCoeff.qdz8 = -0.021788;
    m_PacCoeff.qdz9 = 0.016395;
    m_PacCoeff.qez1 = -0.16852;
    m_PacCoeff.qez2 = -0.68701;
    m_PacCoeff.qez3 = -4.2618;
    m_PacCoeff.qez4 = 0.33152;
    m_PacCoeff.qez5 = 2.8728;
    m_PacCoeff.qhz1 = 0.0005109;
    m_PacCoeff.qhz2 = -0.006873;
    m_PacCoeff.qhz3 = 0.10801;
    m_PacCoeff.qhz4 = 0.14545;
    m_PacCoeff.ssz1 = 0;
    m_PacCoeff.ssz2 = 0;
    m_PacCoeff.ssz3 = 0;
    m_PacCoeff.ssz4 = 0;
    // setting overturning coefficients
    m_PacCoeff.qsx1 = 0;
    m_PacCoeff.qsx2 = 0;
    m_PacCoeff.qsx3 = 0;
    // setting rolling coefficients
    m_PacCoeff.qsy1 = 0;
    m_PacCoeff.qsy2 = 0;

    m_use_vert_map = false;

    // setting bottoming table
    m_use_bott_map = true;
    m_bott_map.AddPoint(0, 0);
    m_bott_map.AddPoint(0.10546, 0);
    m_bott_map.AddPoint(0.30546, 563080);
}

void FEDA_Pac02Tire::SetParametersLevel4() {
    // begin of variables set up
    m_use_mode = 3;
    m_allow_mirroring = false;
    m_PacCoeff.R0 = 0.499;
    m_PacCoeff.width = 0.335;
    m_PacCoeff.aspect_ratio = 0.65;
    m_PacCoeff.rim_radius = 0.2858;
    m_PacCoeff.rim_width = 0.2286;
    m_PacCoeff.Cz = 848550;
    m_PacCoeff.Kz = 921.168;
    m_PacCoeff.FzNomin = 29912;
    // begin scaling factors set up
    m_PacScal.lfz0 = 1;
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
    m_PacScal.ltr = 1;
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
    m_PacCoeff.qsy1 = 0;
    m_PacCoeff.qsy2 = 0;

    m_use_vert_map = false;

    // setting bottoming table
    m_use_bott_map = true;
    m_bott_map.AddPoint(0, 0);
    m_bott_map.AddPoint(0.10546, 0);
    m_bott_map.AddPoint(0.30546, 563080);
}

double FEDA_Pac02Tire::GetNormalStiffnessForce(double depth) const {
    if (m_use_vert_map)
        if (m_use_bott_map) {
            return m_vert_map.Get_y(depth) + m_bott_map.Get_y(depth);
        } else {
            return m_vert_map.Get_y(depth);
        }
    else if (m_use_bott_map) {
        return depth * m_PacCoeff.Cz + m_bott_map.Get_y(depth);
    } else {
        return depth * m_PacCoeff.Cz;
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void FEDA_Pac02Tire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile_left,    // left side
                                               m_meshFile_right);  // right side
    } else {
        ChPac02Tire::AddVisualizationAssets(vis);
    }
}

void FEDA_Pac02Tire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChPac02Tire::RemoveVisualizationAssets();
}

}  // namespace feda
}  // end namespace vehicle
}  // end namespace chrono
