// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2015 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Michael Taylor, Rainer Gericke, Marvin Struijk
// =============================================================================
//
// MFTire 6.2-5.2 constructed with data from file (JSON format).
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/wheeled_vehicle/tire/MFTire.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
MFTire::MFTire(const std::string& filename) : ChMFTire(""), m_mass(0), m_has_mesh(false) {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

MFTire::MFTire(const rapidjson::Document& d) : ChMFTire(""), m_mass(0), m_has_mesh(false) {
    Create(d);
}

void MFTire::Create(const rapidjson::Document& d) {
    ChPart::Create(d);
    m_mass = d["Mass"].GetDouble();
    m_inertia = ReadVectorJSON(d["Inertia"]);
    if (d.HasMember("Friction")) {
        m_mu0 = d["Friction"].GetDouble();
    }
    if (d.HasMember("MDI_HEADER")) {
        if (d["MDI_HEADER"].HasMember("FILE_TYPE")) {
            // par.FILE_TYPE = d["MDI_HEADER"]["FILE_TYPE"].GetString();
        }
        if (d["MDI_HEADER"].HasMember("FILE_VERSION")) {
            par.FILE_VERSION = d["MDI_HEADER"]["FILE_VERSION"].GetDouble();
        }
        if (d["MDI_HEADER"].HasMember("FILE_FORMAT")) {
            // par.FILE_FORMAT = d["MDI_HEADER"]["FILE_FORMAT"].GetString();
        }
    }
    /*
    if (d.HasMember("UNITS")) {
        if (d["UNITS"].HasMember("LENGTH")) {
            par.LENGTH = d["UNITS"]["LENGTH"].GetString();
        }
        if (d["UNITS"].HasMember("FORCE")) {
            par.FORCE = d["UNITS"]["FORCE"].GetString();
        }
        if (d["UNITS"].HasMember("ANGLE")) {
            par.ANGLE = d["UNITS"]["ANGLE"].GetString();
        }
        if (d["UNITS"].HasMember("MASS")) {
            // TODO: Split these out per category
            par.MASS = d["UNITS"]["MASS"].GetString();
        }
        if (d["UNITS"].HasMember("TIME")) {
            par.TIME = d["UNITS"]["TIME"].GetString();
        }
    }
    */
    if (d.HasMember("MODEL")) {
        if (d["MODEL"].HasMember("FITTYP")) {
            par.FITTYP = d["MODEL"]["FITTYP"].GetInt();
        }
        if (d["MODEL"].HasMember("TYRESIDE")) {
            std::string temp = d["MODEL"]["TYRESIDE"].GetString();
            for (auto& c : temp)
                c = toupper(c);
            par.TYRESIDE = temp == "LEFT" ? LEFT : RIGHT;
        }
        if (d["MODEL"].HasMember("LONGVL")) {
            par.LONGVL = d["MODEL"]["LONGVL"].GetDouble();
        }
        if (d["MODEL"].HasMember("VXLOW")) {
            par.VXLOW = d["MODEL"]["VXLOW"].GetDouble();
        }
        if (d["MODEL"].HasMember("ROAD_INCREMENT")) {
            par.ROAD_INCREMENT = d["MODEL"]["ROAD_INCREMENT"].GetDouble();
        }
        if (d["MODEL"].HasMember("ROAD_DIRECTION")) {
            par.ROAD_DIRECTION = d["MODEL"]["ROAD_DIRECTION"].GetDouble();
        }
        if (d["MODEL"].HasMember("PROPERTY_FILE_FORMAT")) {
            // par.PROPERTY_FILE_FORMAT = d["MODEL"]["PROPERTY_FILE_FORMAT"].GetString();
        }
        if (d["MODEL"].HasMember("USER_SUB_ID")) {
            par.USER_SUB_ID = d["MODEL"]["USER_SUB_ID"].GetDouble();
        }
        if (d["MODEL"].HasMember("N_TIRE_STATES")) {
            par.N_TIRE_STATES = d["MODEL"]["N_TIRE_STATES"].GetDouble();
        }
        if (d["MODEL"].HasMember("USE_MODE")) {
            par.USE_MODE = d["MODEL"]["USE_MODE"].GetDouble();
        }
        if (d["MODEL"].HasMember("HMAX_LOCAL")) {
            par.HMAX_LOCAL = d["MODEL"]["HMAX_LOCAL"].GetDouble();
        }
        if (d["MODEL"].HasMember("TIME_SWITCH_INTEG")) {
            par.TIME_SWITCH_INTEG = d["MODEL"]["TIME_SWITCH_INTEG"].GetDouble();
        }
    }
    if (d.HasMember("DIMENSION")) {
        if (d["DIMENSION"].HasMember("UNLOADED_RADIUS")) {
            par.UNLOADED_RADIUS = d["DIMENSION"]["UNLOADED_RADIUS"].GetDouble();
        }
        if (d["DIMENSION"].HasMember("WIDTH")) {
            par.WIDTH = d["DIMENSION"]["WIDTH"].GetDouble();
        }
        if (d["DIMENSION"].HasMember("ASPECT_RATIO")) {
            par.ASPECT_RATIO = d["DIMENSION"]["ASPECT_RATIO"].GetDouble();
        }
        if (d["DIMENSION"].HasMember("RIM_RADIUS")) {
            par.RIM_RADIUS = d["DIMENSION"]["RIM_RADIUS"].GetDouble();
        }
        if (d["DIMENSION"].HasMember("RIM_WIDTH")) {
            par.RIM_WIDTH = d["DIMENSION"]["RIM_WIDTH"].GetDouble();
        }
    }
    if (d.HasMember("VERTICAL")) {
        if (d["VERTICAL"].HasMember("FNOMIN")) {
            par.FNOMIN = d["VERTICAL"]["FNOMIN"].GetDouble();
        }
        if (d["VERTICAL"].HasMember("VERTICAL_STIFFNESS")) {
            par.VERTICAL_STIFFNESS = d["VERTICAL"]["VERTICAL_STIFFNESS"].GetDouble();
        }
        if (d["VERTICAL"].HasMember("VERTICAL_DAMPING")) {
            par.VERTICAL_DAMPING = d["VERTICAL"]["VERTICAL_DAMPING"].GetDouble();
        }
        if (d["VERTICAL"].HasMember("MC_CONTOUR_A")) {
            par.MC_CONTOUR_A = d["VERTICAL"]["MC_CONTOUR_A"].GetDouble();
        }
        if (d["VERTICAL"].HasMember("MC_CONTOUR_B")) {
            par.MC_CONTOUR_B = d["VERTICAL"]["MC_CONTOUR_B"].GetDouble();
        }
        if (d["VERTICAL"].HasMember("BREFF")) {
            par.BREFF = d["VERTICAL"]["BREFF"].GetDouble();
        }
        if (d["VERTICAL"].HasMember("DREFF")) {
            par.DREFF = d["VERTICAL"]["DREFF"].GetDouble();
        }
        if (d["VERTICAL"].HasMember("FREFF")) {
            par.FREFF = d["VERTICAL"]["FREFF"].GetDouble();
        }
        if (d["VERTICAL"].HasMember("Q_RE0")) {
            par.Q_RE0 = d["VERTICAL"]["Q_RE0"].GetDouble();
        }
        if (d["VERTICAL"].HasMember("Q_V1")) {
            par.Q_V1 = d["VERTICAL"]["Q_V1"].GetDouble();
        }
        if (d["VERTICAL"].HasMember("Q_V2")) {
            par.Q_V2 = d["VERTICAL"]["Q_V2"].GetDouble();
        }
        if (d["VERTICAL"].HasMember("Q_FZ2")) {
            par.Q_FZ2 = d["VERTICAL"]["Q_FZ2"].GetDouble();
        }
        if (d["VERTICAL"].HasMember("Q_FCX")) {
            par.Q_FCX = d["VERTICAL"]["Q_FCX"].GetDouble();
        }
        if (d["VERTICAL"].HasMember("Q_FCY")) {
            par.Q_FCY = d["VERTICAL"]["Q_FCY"].GetDouble();
        }
        if (d["VERTICAL"].HasMember("Q_CAM")) {
            par.Q_CAM = d["VERTICAL"]["Q_CAM"].GetDouble();
        }
        if (d["VERTICAL"].HasMember("PFZ1")) {
            par.PFZ1 = d["VERTICAL"]["PFZ1"].GetDouble();
        }
        if (d["VERTICAL"].HasMember("BOTTOM_OFFST")) {
            par.BOTTOM_OFFST = d["VERTICAL"]["BOTTOM_OFFST"].GetDouble();
        }
        if (d["VERTICAL"].HasMember("BOTTOM_STIFF")) {
            par.BOTTOM_STIFF = d["VERTICAL"]["BOTTOM_STIFF"].GetDouble();
        }
    }
    if (d.HasMember("LONG_SLIP_RANGE")) {
        if (d["LONG_SLIP_RANGE"].HasMember("KPUMIN")) {
            par.KPUMIN = d["LONG_SLIP_RANGE"]["KPUMIN"].GetDouble();
        }
        if (d["LONG_SLIP_RANGE"].HasMember("KPUMAX")) {
            par.KPUMAX = d["LONG_SLIP_RANGE"]["KPUMAX"].GetDouble();
        }
    }
    if (d.HasMember("SLIP_ANGLE_RANGE")) {
        if (d["SLIP_ANGLE_RANGE"].HasMember("ALPMIN")) {
            par.ALPMIN = d["SLIP_ANGLE_RANGE"]["ALPMIN"].GetDouble();
        }
        if (d["SLIP_ANGLE_RANGE"].HasMember("ALPMAX")) {
            par.ALPMAX = d["SLIP_ANGLE_RANGE"]["ALPMAX"].GetDouble();
        }
    }
    if (d.HasMember("INCLINATION_ANGLE_RANGE")) {
        if (d["INCLINATION_ANGLE_RANGE"].HasMember("CAMMIN")) {
            par.CAMMIN = d["INCLINATION_ANGLE_RANGE"]["CAMMIN"].GetDouble();
        }
        if (d["INCLINATION_ANGLE_RANGE"].HasMember("CAMMAX")) {
            par.CAMMAX = d["INCLINATION_ANGLE_RANGE"]["CAMMAX"].GetDouble();
        }
    }
    if (d.HasMember("VERTICAL_FORCE_RANGE")) {
        if (d["VERTICAL_FORCE_RANGE"].HasMember("FZMIN")) {
            par.FZMIN = d["VERTICAL_FORCE_RANGE"]["FZMIN"].GetDouble();
        }
        if (d["VERTICAL_FORCE_RANGE"].HasMember("FZMAX")) {
            par.FZMAX = d["VERTICAL_FORCE_RANGE"]["FZMAX"].GetDouble();
        }
    }
    if (d.HasMember("SCALING_COEFFICIENTS")) {
        if (d["SCALING_COEFFICIENTS"].HasMember("LFZO")) {
            par.LFZO = d["SCALING_COEFFICIENTS"]["LFZO"].GetDouble();
        }
        if (d["SCALING_COEFFICIENTS"].HasMember("LCX")) {
            par.LCX = d["SCALING_COEFFICIENTS"]["LCX"].GetDouble();
        }
        if (d["SCALING_COEFFICIENTS"].HasMember("LMUX")) {
            par.LMUX = d["SCALING_COEFFICIENTS"]["LMUX"].GetDouble();
        }
        if (d["SCALING_COEFFICIENTS"].HasMember("LEX")) {
            par.LEX = d["SCALING_COEFFICIENTS"]["LEX"].GetDouble();
        }
        if (d["SCALING_COEFFICIENTS"].HasMember("LKX")) {
            par.LKX = d["SCALING_COEFFICIENTS"]["LKX"].GetDouble();
        }
        if (d["SCALING_COEFFICIENTS"].HasMember("LHX")) {
            par.LHX = d["SCALING_COEFFICIENTS"]["LHX"].GetDouble();
        }
        if (d["SCALING_COEFFICIENTS"].HasMember("LVX")) {
            par.LVX = d["SCALING_COEFFICIENTS"]["LVX"].GetDouble();
        }
        if (d["SCALING_COEFFICIENTS"].HasMember("LCY")) {
            par.LCY = d["SCALING_COEFFICIENTS"]["LCY"].GetDouble();
        }
        if (d["SCALING_COEFFICIENTS"].HasMember("LMUY")) {
            par.LMUY = d["SCALING_COEFFICIENTS"]["LMUY"].GetDouble();
        }
        if (d["SCALING_COEFFICIENTS"].HasMember("LEY")) {
            par.LEY = d["SCALING_COEFFICIENTS"]["LEY"].GetDouble();
        }
        if (d["SCALING_COEFFICIENTS"].HasMember("LKY")) {
            par.LKY = d["SCALING_COEFFICIENTS"]["LKY"].GetDouble();
        }
        if (d["SCALING_COEFFICIENTS"].HasMember("LHY")) {
            par.LHY = d["SCALING_COEFFICIENTS"]["LHY"].GetDouble();
        }
        if (d["SCALING_COEFFICIENTS"].HasMember("LVY")) {
            par.LVY = d["SCALING_COEFFICIENTS"]["LVY"].GetDouble();
        }
        if (d["SCALING_COEFFICIENTS"].HasMember("LTR")) {
            par.LTR = d["SCALING_COEFFICIENTS"]["LTR"].GetDouble();
        }
        if (d["SCALING_COEFFICIENTS"].HasMember("LRES")) {
            par.LRES = d["SCALING_COEFFICIENTS"]["LRES"].GetDouble();
        }
        if (d["SCALING_COEFFICIENTS"].HasMember("LXAL")) {
            par.LXAL = d["SCALING_COEFFICIENTS"]["LXAL"].GetDouble();
        }
        if (d["SCALING_COEFFICIENTS"].HasMember("LYKA")) {
            par.LYKA = d["SCALING_COEFFICIENTS"]["LYKA"].GetDouble();
        }
        if (d["SCALING_COEFFICIENTS"].HasMember("LVYKA")) {
            par.LVYKA = d["SCALING_COEFFICIENTS"]["LVYKA"].GetDouble();
        }
        if (d["SCALING_COEFFICIENTS"].HasMember("LS")) {
            par.LS = d["SCALING_COEFFICIENTS"]["LS"].GetDouble();
        }
        if (d["SCALING_COEFFICIENTS"].HasMember("LKYC")) {
            par.LKYC = d["SCALING_COEFFICIENTS"]["LKYC"].GetDouble();
        }
        if (d["SCALING_COEFFICIENTS"].HasMember("LKZC")) {
            par.LKZC = d["SCALING_COEFFICIENTS"]["LKZC"].GetDouble();
        }
        if (d["SCALING_COEFFICIENTS"].HasMember("LVMX")) {
            par.LVMX = d["SCALING_COEFFICIENTS"]["LVMX"].GetDouble();
        }
        if (d["SCALING_COEFFICIENTS"].HasMember("LMX")) {
            par.LMX = d["SCALING_COEFFICIENTS"]["LMX"].GetDouble();
        }
        if (d["SCALING_COEFFICIENTS"].HasMember("LMY")) {
            par.LMY = d["SCALING_COEFFICIENTS"]["LMY"].GetDouble();
        }
        if (d["SCALING_COEFFICIENTS"].HasMember("LMP")) {
            par.LMP = d["SCALING_COEFFICIENTS"]["LMP"].GetDouble();
        }
    }
    if (d.HasMember("LONGITUDINAL_COEFFICIENTS")) {
        if (d["LONGITUDINAL_COEFFICIENTS"].HasMember("PCX1")) {
            par.PCX1 = d["LONGITUDINAL_COEFFICIENTS"]["PCX1"].GetDouble();
        }
        if (d["LONGITUDINAL_COEFFICIENTS"].HasMember("PDX1")) {
            par.PDX1 = d["LONGITUDINAL_COEFFICIENTS"]["PDX1"].GetDouble();
        }
        if (d["LONGITUDINAL_COEFFICIENTS"].HasMember("PDX2")) {
            par.PDX2 = d["LONGITUDINAL_COEFFICIENTS"]["PDX2"].GetDouble();
        }
        if (d["LONGITUDINAL_COEFFICIENTS"].HasMember("PDX3")) {
            par.PDX3 = d["LONGITUDINAL_COEFFICIENTS"]["PDX3"].GetDouble();
        }
        if (d["LONGITUDINAL_COEFFICIENTS"].HasMember("PEX1")) {
            par.PEX1 = d["LONGITUDINAL_COEFFICIENTS"]["PEX1"].GetDouble();
        }
        if (d["LONGITUDINAL_COEFFICIENTS"].HasMember("PEX2")) {
            par.PEX2 = d["LONGITUDINAL_COEFFICIENTS"]["PEX2"].GetDouble();
        }
        if (d["LONGITUDINAL_COEFFICIENTS"].HasMember("PEX3")) {
            par.PEX3 = d["LONGITUDINAL_COEFFICIENTS"]["PEX3"].GetDouble();
        }
        if (d["LONGITUDINAL_COEFFICIENTS"].HasMember("PEX4")) {
            par.PEX4 = d["LONGITUDINAL_COEFFICIENTS"]["PEX4"].GetDouble();
        }
        if (d["LONGITUDINAL_COEFFICIENTS"].HasMember("PKX1")) {
            par.PKX1 = d["LONGITUDINAL_COEFFICIENTS"]["PKX1"].GetDouble();
        }
        if (d["LONGITUDINAL_COEFFICIENTS"].HasMember("PKX2")) {
            par.PKX2 = d["LONGITUDINAL_COEFFICIENTS"]["PKX2"].GetDouble();
        }
        if (d["LONGITUDINAL_COEFFICIENTS"].HasMember("PKX3")) {
            par.PKX3 = d["LONGITUDINAL_COEFFICIENTS"]["PKX3"].GetDouble();
        }
        if (d["LONGITUDINAL_COEFFICIENTS"].HasMember("PHX1")) {
            par.PHX1 = d["LONGITUDINAL_COEFFICIENTS"]["PHX1"].GetDouble();
        }
        if (d["LONGITUDINAL_COEFFICIENTS"].HasMember("PHX2")) {
            par.PHX2 = d["LONGITUDINAL_COEFFICIENTS"]["PHX2"].GetDouble();
        }
        if (d["LONGITUDINAL_COEFFICIENTS"].HasMember("PVX1")) {
            par.PVX1 = d["LONGITUDINAL_COEFFICIENTS"]["PVX1"].GetDouble();
        }
        if (d["LONGITUDINAL_COEFFICIENTS"].HasMember("PVX2")) {
            par.PVX2 = d["LONGITUDINAL_COEFFICIENTS"]["PVX2"].GetDouble();
        }
        if (d["LONGITUDINAL_COEFFICIENTS"].HasMember("PPX1")) {
            par.PPX1 = d["LONGITUDINAL_COEFFICIENTS"]["PPX1"].GetDouble();
        }
        if (d["LONGITUDINAL_COEFFICIENTS"].HasMember("PPX2")) {
            par.PPX2 = d["LONGITUDINAL_COEFFICIENTS"]["PPX2"].GetDouble();
        }
        if (d["LONGITUDINAL_COEFFICIENTS"].HasMember("PPX3")) {
            par.PPX3 = d["LONGITUDINAL_COEFFICIENTS"]["PPX3"].GetDouble();
        }
        if (d["LONGITUDINAL_COEFFICIENTS"].HasMember("PPX4")) {
            par.PPX4 = d["LONGITUDINAL_COEFFICIENTS"]["PPX4"].GetDouble();
        }
        if (d["LONGITUDINAL_COEFFICIENTS"].HasMember("RBX1")) {
            par.RBX1 = d["LONGITUDINAL_COEFFICIENTS"]["RBX1"].GetDouble();
        }
        if (d["LONGITUDINAL_COEFFICIENTS"].HasMember("RBX2")) {
            par.RBX2 = d["LONGITUDINAL_COEFFICIENTS"]["RBX2"].GetDouble();
        }
        if (d["LONGITUDINAL_COEFFICIENTS"].HasMember("RBX3")) {
            par.RBX3 = d["LONGITUDINAL_COEFFICIENTS"]["RBX3"].GetDouble();
        }
        if (d["LONGITUDINAL_COEFFICIENTS"].HasMember("RCX1")) {
            par.RCX1 = d["LONGITUDINAL_COEFFICIENTS"]["RCX1"].GetDouble();
        }
        if (d["LONGITUDINAL_COEFFICIENTS"].HasMember("REX1")) {
            par.REX1 = d["LONGITUDINAL_COEFFICIENTS"]["REX1"].GetDouble();
        }
        if (d["LONGITUDINAL_COEFFICIENTS"].HasMember("REX2")) {
            par.REX2 = d["LONGITUDINAL_COEFFICIENTS"]["REX2"].GetDouble();
        }
        if (d["LONGITUDINAL_COEFFICIENTS"].HasMember("RHX1")) {
            par.RHX1 = d["LONGITUDINAL_COEFFICIENTS"]["RHX1"].GetDouble();
        }
    }
    if (d.HasMember("OVERTURNING_COEFFICIENTS")) {
        if (d["OVERTURNING_COEFFICIENTS"].HasMember("QSX1")) {
            par.QSX1 = d["OVERTURNING_COEFFICIENTS"]["QSX1"].GetDouble();
        }
        if (d["OVERTURNING_COEFFICIENTS"].HasMember("QSX2")) {
            par.QSX2 = d["OVERTURNING_COEFFICIENTS"]["QSX2"].GetDouble();
        }
        if (d["OVERTURNING_COEFFICIENTS"].HasMember("QSX3")) {
            par.QSX3 = d["OVERTURNING_COEFFICIENTS"]["QSX3"].GetDouble();
        }
        if (d["OVERTURNING_COEFFICIENTS"].HasMember("QSX4")) {
            par.QSX4 = d["OVERTURNING_COEFFICIENTS"]["QSX4"].GetDouble();
        }
        if (d["OVERTURNING_COEFFICIENTS"].HasMember("QSX5")) {
            par.QSX5 = d["OVERTURNING_COEFFICIENTS"]["QSX5"].GetDouble();
        }
        if (d["OVERTURNING_COEFFICIENTS"].HasMember("QSX6")) {
            par.QSX6 = d["OVERTURNING_COEFFICIENTS"]["QSX6"].GetDouble();
        }
        if (d["OVERTURNING_COEFFICIENTS"].HasMember("QSX7")) {
            par.QSX7 = d["OVERTURNING_COEFFICIENTS"]["QSX7"].GetDouble();
        }
        if (d["OVERTURNING_COEFFICIENTS"].HasMember("QSX8")) {
            par.QSX8 = d["OVERTURNING_COEFFICIENTS"]["QSX8"].GetDouble();
        }
        if (d["OVERTURNING_COEFFICIENTS"].HasMember("QSX9")) {
            par.QSX9 = d["OVERTURNING_COEFFICIENTS"]["QSX9"].GetDouble();
        }
        if (d["OVERTURNING_COEFFICIENTS"].HasMember("QSX10")) {
            par.QSX10 = d["OVERTURNING_COEFFICIENTS"]["QSX10"].GetDouble();
        }
        if (d["OVERTURNING_COEFFICIENTS"].HasMember("QSX11")) {
            par.QSX11 = d["OVERTURNING_COEFFICIENTS"]["QSX11"].GetDouble();
        }
        if (d["OVERTURNING_COEFFICIENTS"].HasMember("QSX12")) {
            par.QSX12 = d["OVERTURNING_COEFFICIENTS"]["QSX12"].GetDouble();
        }
        if (d["OVERTURNING_COEFFICIENTS"].HasMember("QSX13")) {
            par.QSX13 = d["OVERTURNING_COEFFICIENTS"]["QSX13"].GetDouble();
        }
        if (d["OVERTURNING_COEFFICIENTS"].HasMember("QSX14")) {
            par.QSX14 = d["OVERTURNING_COEFFICIENTS"]["QSX14"].GetDouble();
        }
        if (d["OVERTURNING_COEFFICIENTS"].HasMember("PPMX1")) {
            par.PPMX1 = d["OVERTURNING_COEFFICIENTS"]["PPMX1"].GetDouble();
        }
    }
    if (d.HasMember("LATERAL_COEFFICIENTS")) {
        if (d["LATERAL_COEFFICIENTS"].HasMember("PCY1")) {
            par.PCY1 = d["LATERAL_COEFFICIENTS"]["PCY1"].GetDouble();
        }
        if (d["LATERAL_COEFFICIENTS"].HasMember("PDY1")) {
            par.PDY1 = d["LATERAL_COEFFICIENTS"]["PDY1"].GetDouble();
        }
        if (d["LATERAL_COEFFICIENTS"].HasMember("PDY2")) {
            par.PDY2 = d["LATERAL_COEFFICIENTS"]["PDY2"].GetDouble();
        }
        if (d["LATERAL_COEFFICIENTS"].HasMember("PDY3")) {
            par.PDY3 = d["LATERAL_COEFFICIENTS"]["PDY3"].GetDouble();
        }
        if (d["LATERAL_COEFFICIENTS"].HasMember("PEY1")) {
            par.PEY1 = d["LATERAL_COEFFICIENTS"]["PEY1"].GetDouble();
        }
        if (d["LATERAL_COEFFICIENTS"].HasMember("PEY2")) {
            par.PEY2 = d["LATERAL_COEFFICIENTS"]["PEY2"].GetDouble();
        }
        if (d["LATERAL_COEFFICIENTS"].HasMember("PEY3")) {
            par.PEY3 = d["LATERAL_COEFFICIENTS"]["PEY3"].GetDouble();
        }
        if (d["LATERAL_COEFFICIENTS"].HasMember("PEY4")) {
            par.PEY4 = d["LATERAL_COEFFICIENTS"]["PEY4"].GetDouble();
        }
        if (d["LATERAL_COEFFICIENTS"].HasMember("PEY5")) {
            par.PEY5 = d["LATERAL_COEFFICIENTS"]["PEY5"].GetDouble();
        }
        if (d["LATERAL_COEFFICIENTS"].HasMember("PKY1")) {
            par.PKY1 = d["LATERAL_COEFFICIENTS"]["PKY1"].GetDouble();
        }
        if (d["LATERAL_COEFFICIENTS"].HasMember("PKY2")) {
            par.PKY2 = d["LATERAL_COEFFICIENTS"]["PKY2"].GetDouble();
        }
        if (d["LATERAL_COEFFICIENTS"].HasMember("PKY3")) {
            par.PKY3 = d["LATERAL_COEFFICIENTS"]["PKY3"].GetDouble();
        }
        if (d["LATERAL_COEFFICIENTS"].HasMember("PKY4")) {
            par.PKY4 = d["LATERAL_COEFFICIENTS"]["PKY4"].GetDouble();
        }
        if (d["LATERAL_COEFFICIENTS"].HasMember("PKY5")) {
            par.PKY5 = d["LATERAL_COEFFICIENTS"]["PKY5"].GetDouble();
        }
        if (d["LATERAL_COEFFICIENTS"].HasMember("PKY6")) {
            par.PKY6 = d["LATERAL_COEFFICIENTS"]["PKY6"].GetDouble();
        }
        if (d["LATERAL_COEFFICIENTS"].HasMember("PKY7")) {
            par.PKY7 = d["LATERAL_COEFFICIENTS"]["PKY7"].GetDouble();
        }
        if (d["LATERAL_COEFFICIENTS"].HasMember("PHY1")) {
            par.PHY1 = d["LATERAL_COEFFICIENTS"]["PHY1"].GetDouble();
        }
        if (d["LATERAL_COEFFICIENTS"].HasMember("PHY2")) {
            par.PHY2 = d["LATERAL_COEFFICIENTS"]["PHY2"].GetDouble();
        }
        if (d["LATERAL_COEFFICIENTS"].HasMember("PVY1")) {
            par.PVY1 = d["LATERAL_COEFFICIENTS"]["PVY1"].GetDouble();
        }
        if (d["LATERAL_COEFFICIENTS"].HasMember("PVY2")) {
            par.PVY2 = d["LATERAL_COEFFICIENTS"]["PVY2"].GetDouble();
        }
        if (d["LATERAL_COEFFICIENTS"].HasMember("PVY3")) {
            par.PVY3 = d["LATERAL_COEFFICIENTS"]["PVY3"].GetDouble();
        }
        if (d["LATERAL_COEFFICIENTS"].HasMember("PVY4")) {
            par.PVY4 = d["LATERAL_COEFFICIENTS"]["PVY4"].GetDouble();
        }
        if (d["LATERAL_COEFFICIENTS"].HasMember("PPY1")) {
            par.PPY1 = d["LATERAL_COEFFICIENTS"]["PPY1"].GetDouble();
        }
        if (d["LATERAL_COEFFICIENTS"].HasMember("PPY2")) {
            par.PPY2 = d["LATERAL_COEFFICIENTS"]["PPY2"].GetDouble();
        }
        if (d["LATERAL_COEFFICIENTS"].HasMember("PPY3")) {
            par.PPY3 = d["LATERAL_COEFFICIENTS"]["PPY3"].GetDouble();
        }
        if (d["LATERAL_COEFFICIENTS"].HasMember("PPY4")) {
            par.PPY4 = d["LATERAL_COEFFICIENTS"]["PPY4"].GetDouble();
        }
        if (d["LATERAL_COEFFICIENTS"].HasMember("PPY5")) {
            par.PPY5 = d["LATERAL_COEFFICIENTS"]["PPY5"].GetDouble();
        }
        if (d["LATERAL_COEFFICIENTS"].HasMember("RBY1")) {
            par.RBY1 = d["LATERAL_COEFFICIENTS"]["RBY1"].GetDouble();
        }
        if (d["LATERAL_COEFFICIENTS"].HasMember("RBY2")) {
            par.RBY2 = d["LATERAL_COEFFICIENTS"]["RBY2"].GetDouble();
        }
        if (d["LATERAL_COEFFICIENTS"].HasMember("RBY3")) {
            par.RBY3 = d["LATERAL_COEFFICIENTS"]["RBY3"].GetDouble();
        }
        if (d["LATERAL_COEFFICIENTS"].HasMember("RBY4")) {
            par.RBY4 = d["LATERAL_COEFFICIENTS"]["RBY4"].GetDouble();
        }
        if (d["LATERAL_COEFFICIENTS"].HasMember("RCY1")) {
            par.RCY1 = d["LATERAL_COEFFICIENTS"]["RCY1"].GetDouble();
        }
        if (d["LATERAL_COEFFICIENTS"].HasMember("REY1")) {
            par.REY1 = d["LATERAL_COEFFICIENTS"]["REY1"].GetDouble();
        }
        if (d["LATERAL_COEFFICIENTS"].HasMember("REY2")) {
            par.REY2 = d["LATERAL_COEFFICIENTS"]["REY2"].GetDouble();
        }
        if (d["LATERAL_COEFFICIENTS"].HasMember("RHY1")) {
            par.RHY1 = d["LATERAL_COEFFICIENTS"]["RHY1"].GetDouble();
        }
        if (d["LATERAL_COEFFICIENTS"].HasMember("RHY2")) {
            par.RHY2 = d["LATERAL_COEFFICIENTS"]["RHY2"].GetDouble();
        }
        if (d["LATERAL_COEFFICIENTS"].HasMember("RVY1")) {
            par.RVY1 = d["LATERAL_COEFFICIENTS"]["RVY1"].GetDouble();
        }
        if (d["LATERAL_COEFFICIENTS"].HasMember("RVY2")) {
            par.RVY2 = d["LATERAL_COEFFICIENTS"]["RVY2"].GetDouble();
        }
        if (d["LATERAL_COEFFICIENTS"].HasMember("RVY3")) {
            par.RVY3 = d["LATERAL_COEFFICIENTS"]["RVY3"].GetDouble();
        }
        if (d["LATERAL_COEFFICIENTS"].HasMember("RVY4")) {
            par.RVY4 = d["LATERAL_COEFFICIENTS"]["RVY4"].GetDouble();
        }
        if (d["LATERAL_COEFFICIENTS"].HasMember("RVY5")) {
            par.RVY5 = d["LATERAL_COEFFICIENTS"]["RVY5"].GetDouble();
        }
        if (d["LATERAL_COEFFICIENTS"].HasMember("RVY6")) {
            par.RVY6 = d["LATERAL_COEFFICIENTS"]["RVY6"].GetDouble();
        }
    }
    if (d.HasMember("ROLLING_COEFFICIENTS")) {
        if (d["ROLLING_COEFFICIENTS"].HasMember("QSY1")) {
            par.QSY1 = d["ROLLING_COEFFICIENTS"]["QSY1"].GetDouble();
        }
        if (d["ROLLING_COEFFICIENTS"].HasMember("QSY2")) {
            par.QSY2 = d["ROLLING_COEFFICIENTS"]["QSY2"].GetDouble();
        }
        if (d["ROLLING_COEFFICIENTS"].HasMember("QSY3")) {
            par.QSY3 = d["ROLLING_COEFFICIENTS"]["QSY3"].GetDouble();
        }
        if (d["ROLLING_COEFFICIENTS"].HasMember("QSY4")) {
            par.QSY4 = d["ROLLING_COEFFICIENTS"]["QSY4"].GetDouble();
        }
        if (d["ROLLING_COEFFICIENTS"].HasMember("QSY5")) {
            par.QSY5 = d["ROLLING_COEFFICIENTS"]["QSY5"].GetDouble();
        }
        if (d["ROLLING_COEFFICIENTS"].HasMember("QSY6")) {
            par.QSY6 = d["ROLLING_COEFFICIENTS"]["QSY6"].GetDouble();
        }
        if (d["ROLLING_COEFFICIENTS"].HasMember("QSY7")) {
            par.QSY7 = d["ROLLING_COEFFICIENTS"]["QSY7"].GetDouble();
        }
        if (d["ROLLING_COEFFICIENTS"].HasMember("QSY8")) {
            par.QSY8 = d["ROLLING_COEFFICIENTS"]["QSY8"].GetDouble();
        }
    }
    if (d.HasMember("ALIGNING_COEFFICIENTS")) {
        if (d["ALIGNING_COEFFICIENTS"].HasMember("QBZ1")) {
            par.QBZ1 = d["ALIGNING_COEFFICIENTS"]["QBZ1"].GetDouble();
        }
        if (d["ALIGNING_COEFFICIENTS"].HasMember("QBZ2")) {
            par.QBZ2 = d["ALIGNING_COEFFICIENTS"]["QBZ2"].GetDouble();
        }
        if (d["ALIGNING_COEFFICIENTS"].HasMember("QBZ3")) {
            par.QBZ3 = d["ALIGNING_COEFFICIENTS"]["QBZ3"].GetDouble();
        }
        if (d["ALIGNING_COEFFICIENTS"].HasMember("QBZ4")) {
            par.QBZ4 = d["ALIGNING_COEFFICIENTS"]["QBZ4"].GetDouble();
        }
        if (d["ALIGNING_COEFFICIENTS"].HasMember("QBZ5")) {
            par.QBZ5 = d["ALIGNING_COEFFICIENTS"]["QBZ5"].GetDouble();
        }
        if (d["ALIGNING_COEFFICIENTS"].HasMember("QBZ9")) {
            par.QBZ9 = d["ALIGNING_COEFFICIENTS"]["QBZ9"].GetDouble();
        }
        if (d["ALIGNING_COEFFICIENTS"].HasMember("QBZ10")) {
            par.QBZ10 = d["ALIGNING_COEFFICIENTS"]["QBZ10"].GetDouble();
        }
        if (d["ALIGNING_COEFFICIENTS"].HasMember("QCZ1")) {
            par.QCZ1 = d["ALIGNING_COEFFICIENTS"]["QCZ1"].GetDouble();
        }
        if (d["ALIGNING_COEFFICIENTS"].HasMember("QDZ1")) {
            par.QDZ1 = d["ALIGNING_COEFFICIENTS"]["QDZ1"].GetDouble();
        }
        if (d["ALIGNING_COEFFICIENTS"].HasMember("QDZ2")) {
            par.QDZ2 = d["ALIGNING_COEFFICIENTS"]["QDZ2"].GetDouble();
        }
        if (d["ALIGNING_COEFFICIENTS"].HasMember("QDZ3")) {
            par.QDZ3 = d["ALIGNING_COEFFICIENTS"]["QDZ3"].GetDouble();
        }
        if (d["ALIGNING_COEFFICIENTS"].HasMember("QDZ4")) {
            par.QDZ4 = d["ALIGNING_COEFFICIENTS"]["QDZ4"].GetDouble();
        }
        if (d["ALIGNING_COEFFICIENTS"].HasMember("QDZ6")) {
            par.QDZ6 = d["ALIGNING_COEFFICIENTS"]["QDZ6"].GetDouble();
        }
        if (d["ALIGNING_COEFFICIENTS"].HasMember("QDZ7")) {
            par.QDZ7 = d["ALIGNING_COEFFICIENTS"]["QDZ7"].GetDouble();
        }
        if (d["ALIGNING_COEFFICIENTS"].HasMember("QDZ8")) {
            par.QDZ8 = d["ALIGNING_COEFFICIENTS"]["QDZ8"].GetDouble();
        }
        if (d["ALIGNING_COEFFICIENTS"].HasMember("QDZ9")) {
            par.QDZ9 = d["ALIGNING_COEFFICIENTS"]["QDZ9"].GetDouble();
        }
        if (d["ALIGNING_COEFFICIENTS"].HasMember("QDZ10")) {
            par.QDZ10 = d["ALIGNING_COEFFICIENTS"]["QDZ10"].GetDouble();
        }
        if (d["ALIGNING_COEFFICIENTS"].HasMember("QDZ11")) {
            par.QDZ11 = d["ALIGNING_COEFFICIENTS"]["QDZ11"].GetDouble();
        }
        if (d["ALIGNING_COEFFICIENTS"].HasMember("QEZ1")) {
            par.QEZ1 = d["ALIGNING_COEFFICIENTS"]["QEZ1"].GetDouble();
        }
        if (d["ALIGNING_COEFFICIENTS"].HasMember("QEZ2")) {
            par.QEZ2 = d["ALIGNING_COEFFICIENTS"]["QEZ2"].GetDouble();
        }
        if (d["ALIGNING_COEFFICIENTS"].HasMember("QEZ3")) {
            par.QEZ3 = d["ALIGNING_COEFFICIENTS"]["QEZ3"].GetDouble();
        }
        if (d["ALIGNING_COEFFICIENTS"].HasMember("QEZ4")) {
            par.QEZ4 = d["ALIGNING_COEFFICIENTS"]["QEZ4"].GetDouble();
        }
        if (d["ALIGNING_COEFFICIENTS"].HasMember("QEZ5")) {
            par.QEZ5 = d["ALIGNING_COEFFICIENTS"]["QEZ5"].GetDouble();
        }
        if (d["ALIGNING_COEFFICIENTS"].HasMember("QHZ1")) {
            par.QHZ1 = d["ALIGNING_COEFFICIENTS"]["QHZ1"].GetDouble();
        }
        if (d["ALIGNING_COEFFICIENTS"].HasMember("QHZ2")) {
            par.QHZ2 = d["ALIGNING_COEFFICIENTS"]["QHZ2"].GetDouble();
        }
        if (d["ALIGNING_COEFFICIENTS"].HasMember("QHZ3")) {
            par.QHZ3 = d["ALIGNING_COEFFICIENTS"]["QHZ3"].GetDouble();
        }
        if (d["ALIGNING_COEFFICIENTS"].HasMember("QHZ4")) {
            par.QHZ4 = d["ALIGNING_COEFFICIENTS"]["QHZ4"].GetDouble();
        }
        if (d["ALIGNING_COEFFICIENTS"].HasMember("PPZ1")) {
            par.PPZ1 = d["ALIGNING_COEFFICIENTS"]["PPZ1"].GetDouble();
        }
        if (d["ALIGNING_COEFFICIENTS"].HasMember("PPZ2")) {
            par.PPZ2 = d["ALIGNING_COEFFICIENTS"]["PPZ2"].GetDouble();
        }
        if (d["ALIGNING_COEFFICIENTS"].HasMember("SSZ1")) {
            par.SSZ1 = d["ALIGNING_COEFFICIENTS"]["SSZ1"].GetDouble();
        }
        if (d["ALIGNING_COEFFICIENTS"].HasMember("SSZ2")) {
            par.SSZ2 = d["ALIGNING_COEFFICIENTS"]["SSZ2"].GetDouble();
        }
        if (d["ALIGNING_COEFFICIENTS"].HasMember("SSZ3")) {
            par.SSZ3 = d["ALIGNING_COEFFICIENTS"]["SSZ3"].GetDouble();
        }
        if (d["ALIGNING_COEFFICIENTS"].HasMember("SSZ4")) {
            par.SSZ4 = d["ALIGNING_COEFFICIENTS"]["SSZ4"].GetDouble();
        }
    }
    if (d.HasMember("OPERATING_CONDITIONS")) {
        if (d["OPERATING_CONDITIONS"].HasMember("INFLPRES")) {
            par.INFLPRES = d["OPERATING_CONDITIONS"]["INFLPRES"].GetDouble();
        }
        if (d["OPERATING_CONDITIONS"].HasMember("NOMPRES")) {
            par.NOMPRES = d["OPERATING_CONDITIONS"]["NOMPRES"].GetDouble();
        }
    }
    if (d.HasMember("INERTIA")) {
        if (d["INERTIA"].HasMember("MASS")) {
            par.MASS1 = d["INERTIA"]["MASS"].GetDouble();
        }
        if (d["INERTIA"].HasMember("IXX")) {
            par.IXX = d["INERTIA"]["IXX"].GetDouble();
        }
        if (d["INERTIA"].HasMember("IYY")) {
            par.IYY = d["INERTIA"]["IYY"].GetDouble();
        }
        if (d["INERTIA"].HasMember("BELT_MASS")) {
            par.BELT_MASS = d["INERTIA"]["BELT_MASS"].GetDouble();
        }
        if (d["INERTIA"].HasMember("BELT_IXX")) {
            par.BELT_IXX = d["INERTIA"]["BELT_IXX"].GetDouble();
        }
        if (d["INERTIA"].HasMember("BELT_IYY")) {
            par.BELT_IYY = d["INERTIA"]["BELT_IYY"].GetDouble();
        }
        if (d["INERTIA"].HasMember("GRAVITY")) {
            par.GRAVITY = d["INERTIA"]["GRAVITY"].GetDouble();
        }
    }
    if (d.HasMember("STRUCTURAL")) {
        if (d["STRUCTURAL"].HasMember("LONGITUDINAL_STIFFNESS")) {
            par.LONGITUDINAL_STIFFNESS = d["STRUCTURAL"]["LONGITUDINAL_STIFFNESS"].GetDouble();
        }
        if (d["STRUCTURAL"].HasMember("LATERAL_STIFFNESS")) {
            par.LATERAL_STIFFNESS = d["STRUCTURAL"]["LATERAL_STIFFNESS"].GetDouble();
        }
        if (d["STRUCTURAL"].HasMember("YAW_STIFFNESS")) {
            par.YAW_STIFFNESS = d["STRUCTURAL"]["YAW_STIFFNESS"].GetDouble();
        }
        if (d["STRUCTURAL"].HasMember("FREQ_LONG")) {
            par.FREQ_LONG = d["STRUCTURAL"]["FREQ_LONG"].GetDouble();
        }
        if (d["STRUCTURAL"].HasMember("FREQ_LAT")) {
            par.FREQ_LAT = d["STRUCTURAL"]["FREQ_LAT"].GetDouble();
        }
        if (d["STRUCTURAL"].HasMember("FREQ_YAW")) {
            par.FREQ_YAW = d["STRUCTURAL"]["FREQ_YAW"].GetDouble();
        }
        if (d["STRUCTURAL"].HasMember("FREQ_WINDUP")) {
            par.FREQ_WINDUP = d["STRUCTURAL"]["FREQ_WINDUP"].GetDouble();
        }
        if (d["STRUCTURAL"].HasMember("DAMP_LONG")) {
            par.DAMP_LONG = d["STRUCTURAL"]["DAMP_LONG"].GetDouble();
        }
        if (d["STRUCTURAL"].HasMember("DAMP_LAT")) {
            par.DAMP_LAT = d["STRUCTURAL"]["DAMP_LAT"].GetDouble();
        }
        if (d["STRUCTURAL"].HasMember("DAMP_YAW")) {
            par.DAMP_YAW = d["STRUCTURAL"]["DAMP_YAW"].GetDouble();
        }
        if (d["STRUCTURAL"].HasMember("DAMP_WINDUP")) {
            par.DAMP_WINDUP = d["STRUCTURAL"]["DAMP_WINDUP"].GetDouble();
        }
        if (d["STRUCTURAL"].HasMember("DAMP_RESIDUAL")) {
            par.DAMP_RESIDUAL = d["STRUCTURAL"]["DAMP_RESIDUAL"].GetDouble();
        }
        if (d["STRUCTURAL"].HasMember("DAMP_VLOW")) {
            par.DAMP_VLOW = d["STRUCTURAL"]["DAMP_VLOW"].GetDouble();
        }
        if (d["STRUCTURAL"].HasMember("Q_BVX")) {
            par.Q_BVX = d["STRUCTURAL"]["Q_BVX"].GetDouble();
        }
        if (d["STRUCTURAL"].HasMember("Q_BVT")) {
            par.Q_BVT = d["STRUCTURAL"]["Q_BVT"].GetDouble();
        }
        if (d["STRUCTURAL"].HasMember("PCFX1")) {
            par.PCFX1 = d["STRUCTURAL"]["PCFX1"].GetDouble();
        }
        if (d["STRUCTURAL"].HasMember("PCFX2")) {
            par.PCFX2 = d["STRUCTURAL"]["PCFX2"].GetDouble();
        }
        if (d["STRUCTURAL"].HasMember("PCFX3")) {
            par.PCFX3 = d["STRUCTURAL"]["PCFX3"].GetDouble();
        }
        if (d["STRUCTURAL"].HasMember("PCFY1")) {
            par.PCFY1 = d["STRUCTURAL"]["PCFY1"].GetDouble();
        }
        if (d["STRUCTURAL"].HasMember("PCFY2")) {
            par.PCFY2 = d["STRUCTURAL"]["PCFY2"].GetDouble();
        }
        if (d["STRUCTURAL"].HasMember("PCFY3")) {
            par.PCFY3 = d["STRUCTURAL"]["PCFY3"].GetDouble();
        }
        if (d["STRUCTURAL"].HasMember("PCMZ1")) {
            par.PCMZ1 = d["STRUCTURAL"]["PCMZ1"].GetDouble();
        }
    }
    if (d.HasMember("CONTACT_PATCH")) {
        if (d["CONTACT_PATCH"].HasMember("Q_RA1")) {
            par.Q_RA1 = d["CONTACT_PATCH"]["Q_RA1"].GetDouble();
        }
        if (d["CONTACT_PATCH"].HasMember("Q_RA2")) {
            par.Q_RA2 = d["CONTACT_PATCH"]["Q_RA2"].GetDouble();
        }
        if (d["CONTACT_PATCH"].HasMember("Q_RB1")) {
            par.Q_RB1 = d["CONTACT_PATCH"]["Q_RB1"].GetDouble();
        }
        if (d["CONTACT_PATCH"].HasMember("Q_RB2")) {
            par.Q_RB2 = d["CONTACT_PATCH"]["Q_RB2"].GetDouble();
        }
        if (d["CONTACT_PATCH"].HasMember("ELLIPS_SHIFT")) {
            par.ELLIPS_SHIFT = d["CONTACT_PATCH"]["ELLIPS_SHIFT"].GetDouble();
        }
        if (d["CONTACT_PATCH"].HasMember("ELLIPS_LENGTH")) {
            par.ELLIPS_LENGTH = d["CONTACT_PATCH"]["ELLIPS_LENGTH"].GetDouble();
        }
        if (d["CONTACT_PATCH"].HasMember("ELLIPS_HEIGHT")) {
            par.ELLIPS_HEIGHT = d["CONTACT_PATCH"]["ELLIPS_HEIGHT"].GetDouble();
        }
        if (d["CONTACT_PATCH"].HasMember("ELLIPS_ORDER")) {
            par.ELLIPS_ORDER = d["CONTACT_PATCH"]["ELLIPS_ORDER"].GetDouble();
        }
        if (d["CONTACT_PATCH"].HasMember("ELLIPS_MAX_STEP")) {
            par.ELLIPS_MAX_STEP = d["CONTACT_PATCH"]["ELLIPS_MAX_STEP"].GetDouble();
        }
        if (d["CONTACT_PATCH"].HasMember("ELLIPS_NWIDTH")) {
            par.ELLIPS_NWIDTH = d["CONTACT_PATCH"]["ELLIPS_NWIDTH"].GetDouble();
        }
        if (d["CONTACT_PATCH"].HasMember("ELLIPS_NLENGTH")) {
            par.ELLIPS_NLENGTH = d["CONTACT_PATCH"]["ELLIPS_NLENGTH"].GetDouble();
        }
    }
    if (d.HasMember("INFLATION_PRESSURE_RANGE")) {
        if (d["INFLATION_PRESSURE_RANGE"].HasMember("PRESMIN")) {
            par.PRESMIN = d["INFLATION_PRESSURE_RANGE"]["PRESMIN"].GetDouble();
        }
        if (d["INFLATION_PRESSURE_RANGE"].HasMember("PRESMAX")) {
            par.PRESMAX = d["INFLATION_PRESSURE_RANGE"]["PRESMAX"].GetDouble();
        }
    }
    if (d.HasMember("TURNSLIP_COEFFICIENTS")) {
        if (d["TURNSLIP_COEFFICIENTS"].HasMember("PDXP1")) {
            par.PDXP1 = d["TURNSLIP_COEFFICIENTS"]["PDXP1"].GetDouble();
        }
        if (d["TURNSLIP_COEFFICIENTS"].HasMember("PDXP2")) {
            par.PDXP2 = d["TURNSLIP_COEFFICIENTS"]["PDXP2"].GetDouble();
        }
        if (d["TURNSLIP_COEFFICIENTS"].HasMember("PDXP3")) {
            par.PDXP3 = d["TURNSLIP_COEFFICIENTS"]["PDXP3"].GetDouble();
        }
        if (d["TURNSLIP_COEFFICIENTS"].HasMember("PKYP1")) {
            par.PKYP1 = d["TURNSLIP_COEFFICIENTS"]["PKYP1"].GetDouble();
        }
        if (d["TURNSLIP_COEFFICIENTS"].HasMember("PDYP1")) {
            par.PDYP1 = d["TURNSLIP_COEFFICIENTS"]["PDYP1"].GetDouble();
        }
        if (d["TURNSLIP_COEFFICIENTS"].HasMember("PDYP2")) {
            par.PDYP2 = d["TURNSLIP_COEFFICIENTS"]["PDYP2"].GetDouble();
        }
        if (d["TURNSLIP_COEFFICIENTS"].HasMember("PDYP3")) {
            par.PDYP3 = d["TURNSLIP_COEFFICIENTS"]["PDYP3"].GetDouble();
        }
        if (d["TURNSLIP_COEFFICIENTS"].HasMember("PDYP4")) {
            par.PDYP4 = d["TURNSLIP_COEFFICIENTS"]["PDYP4"].GetDouble();
        }
        if (d["TURNSLIP_COEFFICIENTS"].HasMember("PHYP1")) {
            par.PHYP1 = d["TURNSLIP_COEFFICIENTS"]["PHYP1"].GetDouble();
        }
        if (d["TURNSLIP_COEFFICIENTS"].HasMember("PHYP2")) {
            par.PHYP2 = d["TURNSLIP_COEFFICIENTS"]["PHYP2"].GetDouble();
        }
        if (d["TURNSLIP_COEFFICIENTS"].HasMember("PHYP3")) {
            par.PHYP3 = d["TURNSLIP_COEFFICIENTS"]["PHYP3"].GetDouble();
        }
        if (d["TURNSLIP_COEFFICIENTS"].HasMember("PHYP4")) {
            par.PHYP4 = d["TURNSLIP_COEFFICIENTS"]["PHYP4"].GetDouble();
        }
        if (d["TURNSLIP_COEFFICIENTS"].HasMember("PECP1")) {
            par.PECP1 = d["TURNSLIP_COEFFICIENTS"]["PECP1"].GetDouble();
        }
        if (d["TURNSLIP_COEFFICIENTS"].HasMember("PECP2")) {
            par.PECP2 = d["TURNSLIP_COEFFICIENTS"]["PECP2"].GetDouble();
        }
        if (d["TURNSLIP_COEFFICIENTS"].HasMember("QDTP1")) {
            par.QDTP1 = d["TURNSLIP_COEFFICIENTS"]["QDTP1"].GetDouble();
        }
        if (d["TURNSLIP_COEFFICIENTS"].HasMember("QCRP1")) {
            par.QCRP1 = d["TURNSLIP_COEFFICIENTS"]["QCRP1"].GetDouble();
        }
        if (d["TURNSLIP_COEFFICIENTS"].HasMember("QCRP2")) {
            par.QCRP2 = d["TURNSLIP_COEFFICIENTS"]["QCRP2"].GetDouble();
        }
        if (d["TURNSLIP_COEFFICIENTS"].HasMember("QBRP1")) {
            par.QBRP1 = d["TURNSLIP_COEFFICIENTS"]["QBRP1"].GetDouble();
        }
        if (d["TURNSLIP_COEFFICIENTS"].HasMember("QDRP1")) {
            par.QDRP1 = d["TURNSLIP_COEFFICIENTS"]["QDRP1"].GetDouble();
        }
    }
}

void MFTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH && m_has_mesh) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile_left,    // left side
                                               m_meshFile_right);  // right side
    } else {
        ChMFTire::AddVisualizationAssets(vis);
    }
}

void MFTire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChMFTire::RemoveVisualizationAssets();
}

}  // namespace vehicle
}  // namespace chrono
