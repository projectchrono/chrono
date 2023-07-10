// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Rainer Gericke, Radu Serban
// =============================================================================
//
// Template for a Magic Formula tire model
//
// ChPac02 is based on the Pacejka 2002 formulae as written in
// Hans B. Pacejka's "Tire and Vehicle Dynamics" Third Edition, Elsevier 2012
// ISBN: 978-0-08-097016-5
//
// This implementation is a subset of the commercial product MFtire:
//  - only steady state force/torque calculations
//  - uncombined (use_mode = 3)
//  - combined (use_mode = 4) via Pacejka method
//  - parametration is given by a TIR file (Tiem Orbit Format,
//    ADAMS/Car compatible)
//  - unit conversion is implemented but only tested for SI units
//  - optional inflation pressure dependency is implemented, but not tested
//  - this implementation could be validated for the FED-Alpha vehicle and rsp.
//    tire data sets against KRC test results from a Nato CDT
//
// This derived class reads parameters from a JSON parameter file
//  - input can be redirected from a TIR file
//  - input parameters can set directly (only SI units!)
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/wheeled_vehicle/tire/Pac02Tire.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

Pac02Tire::Pac02Tire(const std::string& filename) : ChPac02Tire(""), m_mass(0), m_has_mesh(false) {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

Pac02Tire::Pac02Tire(const rapidjson::Document& d) : ChPac02Tire(""), m_mass(0), m_has_mesh(false) {
    Create(d);
}

void Pac02Tire::Create(const rapidjson::Document& d) {
    // Invoke base class method
    ChPart::Create(d);

    if (d.HasMember("Mass")) {
        m_mass = d["Mass"].GetDouble();
    } else {
        GetLog() << "Fatal: Mass not set!\n";
        exit(9);
    }
    if (d.HasMember("Inertia")) {
        m_inertia = ReadVectorJSON(d["Inertia"]);
    } else {
        GetLog() << "Fatal: Inertia not set!\n";
        exit(9);
    }
    if (d.HasMember("Coefficient of Friction")) {
        m_mu0 = d["Coefficient of Friction"].GetDouble();
    } else {
        GetLog() << "Fatal: Friction not set!\n";
        exit(9);
    }

    // Check if TIR specification file provided
    if (d.HasMember("TIR Specification File")) {
        m_tir_file = d["TIR Specification File"].GetString();
    } else {
        // Tire parameters explicitly specified in JSON file
        if (d.HasMember("Use Mode")) {
            m_use_mode = d["Use Mode"].GetInt();
        }
        if (d.HasMember("Tire Side")) {
            std::string tside = d["Tire Side"].GetString();
            if (tside.compare("left") == 0 || tside.compare("unknown") == 0) {
                m_measured_side = LEFT;
            } else {
                m_measured_side = RIGHT;
            }
        }
        if (d.HasMember("Dimension")) {
            m_par.UNLOADED_RADIUS = d["Dimension"]["Unloaded Radius"].GetDouble();
            m_par.WIDTH = d["Dimension"]["Width"].GetDouble();
            m_par.ASPECT_RATIO = d["Dimension"]["Aspect Ratio"].GetDouble();
            m_par.RIM_RADIUS = d["Dimension"]["Rim Radius"].GetDouble();
            m_par.RIM_WIDTH = d["Dimension"]["Rim Width"].GetDouble();
        } else {
            GetLog() << "Fatal: Dimension not set!\n";
            exit(9);
        }
        if (d.HasMember("Vertical")) {
            m_par.VERTICAL_STIFFNESS = d["Vertical"]["Vertical Stiffness"].GetDouble();
            m_par.VERTICAL_DAMPING = d["Vertical"]["Vertical Damping"].GetDouble();
            m_par.FNOMIN = d["Vertical"]["Nominal Wheel Load"].GetDouble();
            m_par.QFZ1 = m_par.VERTICAL_STIFFNESS * m_par.UNLOADED_RADIUS / m_par.FNOMIN;
            m_par.QFZ2 = 0;
        } else {
            GetLog() << "Fatal: Vertical not set!\n";
            exit(9);
        }
        if (d.HasMember("Scaling Factors")) {
            if (d["Scaling Factors"].HasMember("LFZO"))
                m_par.LFZO = d["Scaling Factors"]["LFZO"].GetDouble();
            if (d["Scaling Factors"].HasMember("LCX"))
                m_par.LCX = d["Scaling Factors"]["LCX"].GetDouble();
            if (d["Scaling Factors"].HasMember("LMUX"))
                m_par.LMUX = d["Scaling Factors"]["LMUX"].GetDouble();
            if (d["Scaling Factors"].HasMember("LEX"))
                m_par.LEX = d["Scaling Factors"]["LEX"].GetDouble();
            if (d["Scaling Factors"].HasMember("LKX"))
                m_par.LKX = d["Scaling Factors"]["LKX"].GetDouble();
            if (d["Scaling Factors"].HasMember("LHX"))
                m_par.LHX = d["Scaling Factors"]["LHX"].GetDouble();
            if (d["Scaling Factors"].HasMember("LVX"))
                m_par.LVX = d["Scaling Factors"]["LVX"].GetDouble();
            if (d["Scaling Factors"].HasMember("LGAX"))
                m_par.LGAX = d["Scaling Factors"]["LGAX"].GetDouble();
            if (d["Scaling Factors"].HasMember("LCY"))
                m_par.LCY = d["Scaling Factors"]["LCY"].GetDouble();
            if (d["Scaling Factors"].HasMember("LMUY"))
                m_par.LMUY = d["Scaling Factors"]["LMUY"].GetDouble();
            if (d["Scaling Factors"].HasMember("LEY"))
                m_par.LEY = d["Scaling Factors"]["LEY"].GetDouble();
            if (d["Scaling Factors"].HasMember("LKY"))
                m_par.LKY = d["Scaling Factors"]["LKY"].GetDouble();
            if (d["Scaling Factors"].HasMember("LHY"))
                m_par.LHY = d["Scaling Factors"]["LHY"].GetDouble();
            if (d["Scaling Factors"].HasMember("LVY"))
                m_par.LVY = d["Scaling Factors"]["LVY"].GetDouble();
            if (d["Scaling Factors"].HasMember("LGAY"))
                m_par.LGAY = d["Scaling Factors"]["LGAY"].GetDouble();
            if (d["Scaling Factors"].HasMember("LTR"))
                m_par.LTR = d["Scaling Factors"]["LTR"].GetDouble();
            if (d["Scaling Factors"].HasMember("LRES"))
                m_par.LRES = d["Scaling Factors"]["LRES"].GetDouble();
            if (d["Scaling Factors"].HasMember("LGAZ"))
                m_par.LGAZ = d["Scaling Factors"]["LGAZ"].GetDouble();
            if (d["Scaling Factors"].HasMember("LXAL"))
                m_par.LXAL = d["Scaling Factors"]["LXAL"].GetDouble();
            if (d["Scaling Factors"].HasMember("LYKA"))
                m_par.LYKA = d["Scaling Factors"]["LYKA"].GetDouble();
            if (d["Scaling Factors"].HasMember("LVYKA"))
                m_par.LVYKA = d["Scaling Factors"]["LVYKA"].GetDouble();
            if (d["Scaling Factors"].HasMember("LS"))
                m_par.LS = d["Scaling Factors"]["LS"].GetDouble();
            if (d["Scaling Factors"].HasMember("LSGKP"))
                m_par.LSGKP = d["Scaling Factors"]["LSGKP"].GetDouble();
            if (d["Scaling Factors"].HasMember("LSGAL"))
                m_par.LSGAL = d["Scaling Factors"]["LSGAL"].GetDouble();
            if (d["Scaling Factors"].HasMember("LGYR"))
                m_par.LGYR = d["Scaling Factors"]["LGYR"].GetDouble();
            if (d["Scaling Factors"].HasMember("LVMX"))
                m_par.LVMX = d["Scaling Factors"]["LVMX"].GetDouble();
            if (d["Scaling Factors"].HasMember("LMY"))
                m_par.LMY = d["Scaling Factors"]["LMY"].GetDouble();
            if (d["Scaling Factors"].HasMember("LIP"))
                m_par.LIP = d["Scaling Factors"]["LIP"].GetDouble();
            if (d["Scaling Factors"].HasMember("LKYG"))
                m_par.LKYG = d["Scaling Factors"]["LKYG"].GetDouble();
            if (d["Scaling Factors"].HasMember("LCZ"))
                m_par.LCZ = d["Scaling Factors"]["LCZ"].GetDouble();
        } else {
            GetLog() << "Missing Scaling Factors are set to 1!\n";
        }
        if (d.HasMember("Longitudinal Coefficients")) {
            if (d["Longitudinal Coefficients"].HasMember("PCX1"))
                m_par.PCX1 = d["Longitudinal Coefficients"]["PCX1"].GetDouble();
            if (d["Longitudinal Coefficients"].HasMember("PDX1"))
                m_par.PDX1 = d["Longitudinal Coefficients"]["PDX1"].GetDouble();
            if (d["Longitudinal Coefficients"].HasMember("PDX2"))
                m_par.PDX2 = d["Longitudinal Coefficients"]["PDX2"].GetDouble();
            if (d["Longitudinal Coefficients"].HasMember("PDX3"))
                m_par.PDX3 = d["Longitudinal Coefficients"]["PDX3"].GetDouble();
            if (d["Longitudinal Coefficients"].HasMember("PEX1"))
                m_par.PEX1 = d["Longitudinal Coefficients"]["PEX1"].GetDouble();
            if (d["Longitudinal Coefficients"].HasMember("PEX2"))
                m_par.PEX2 = d["Longitudinal Coefficients"]["PEX2"].GetDouble();
            if (d["Longitudinal Coefficients"].HasMember("PEX3"))
                m_par.PEX3 = d["Longitudinal Coefficients"]["PEX3"].GetDouble();
            if (d["Longitudinal Coefficients"].HasMember("PEX4"))
                m_par.PEX4 = d["Longitudinal Coefficients"]["PEX4"].GetDouble();
            if (d["Longitudinal Coefficients"].HasMember("PKX1"))
                m_par.PKX1 = d["Longitudinal Coefficients"]["PKX1"].GetDouble();
            if (d["Longitudinal Coefficients"].HasMember("PKX2"))
                m_par.PKX2 = d["Longitudinal Coefficients"]["PKX2"].GetDouble();
            if (d["Longitudinal Coefficients"].HasMember("PKX3"))
                m_par.PKX3 = d["Longitudinal Coefficients"]["PKX3"].GetDouble();
            if (d["Longitudinal Coefficients"].HasMember("PHX1"))
                m_par.PHX1 = d["Longitudinal Coefficients"]["PHX1"].GetDouble();
            if (d["Longitudinal Coefficients"].HasMember("PHX2"))
                m_par.PHX2 = d["Longitudinal Coefficients"]["PHX2"].GetDouble();
            if (d["Longitudinal Coefficients"].HasMember("PVX1"))
                m_par.PVX1 = d["Longitudinal Coefficients"]["PVX1"].GetDouble();
            if (d["Longitudinal Coefficients"].HasMember("PVX2"))
                m_par.PVX2 = d["Longitudinal Coefficients"]["PVX2"].GetDouble();
            if (d["Longitudinal Coefficients"].HasMember("RBX1"))
                m_par.RBX1 = d["Longitudinal Coefficients"]["RBX1"].GetDouble();
            if (d["Longitudinal Coefficients"].HasMember("RBX2"))
                m_par.RBX2 = d["Longitudinal Coefficients"]["RBX2"].GetDouble();
            if (d["Longitudinal Coefficients"].HasMember("RCX1"))
                m_par.RCX1 = d["Longitudinal Coefficients"]["RCX1"].GetDouble();
            if (d["Longitudinal Coefficients"].HasMember("REX1"))
                m_par.REX1 = d["Longitudinal Coefficients"]["REX1"].GetDouble();
            if (d["Longitudinal Coefficients"].HasMember("REX2"))
                m_par.REX2 = d["Longitudinal Coefficients"]["REX2"].GetDouble();
            if (d["Longitudinal Coefficients"].HasMember("RHX1"))
                m_par.RHX1 = d["Longitudinal Coefficients"]["RHX1"].GetDouble();
            if (d["Longitudinal Coefficients"].HasMember("PTX1"))
                m_par.PTX1 = d["Longitudinal Coefficients"]["PTX1"].GetDouble();
            if (d["Longitudinal Coefficients"].HasMember("PTX2"))
                m_par.PTX2 = d["Longitudinal Coefficients"]["PTX2"].GetDouble();
            if (d["Longitudinal Coefficients"].HasMember("PTX3"))
                m_par.PTX3 = d["Longitudinal Coefficients"]["PTX3"].GetDouble();
            if (d["Longitudinal Coefficients"].HasMember("PPX1"))
                m_par.PPX1 = d["Longitudinal Coefficients"]["PPX1"].GetDouble();
            if (d["Longitudinal Coefficients"].HasMember("PPX2"))
                m_par.PPX2 = d["Longitudinal Coefficients"]["PPX2"].GetDouble();
            if (d["Longitudinal Coefficients"].HasMember("PPX3"))
                m_par.PPX3 = d["Longitudinal Coefficients"]["PPX3"].GetDouble();
            if (d["Longitudinal Coefficients"].HasMember("PPX4"))
                m_par.PPX4 = d["Longitudinal Coefficients"]["PPX4"].GetDouble();
        } else {
            GetLog() << "Fatal: Longitudinal Coefficients not set!\n";
            exit(9);
        }
        if (d.HasMember("Lateral Coefficients")) {
            if (d["Lateral Coefficients"].HasMember("PCY1"))
                m_par.PCY1 = d["Lateral Coefficients"]["PCY1"].GetDouble();
            if (d["Lateral Coefficients"].HasMember("PDY1"))
                m_par.PDY1 = d["Lateral Coefficients"]["PDY1"].GetDouble();
            if (d["Lateral Coefficients"].HasMember("PDY2"))
                m_par.PDY2 = d["Lateral Coefficients"]["PDY2"].GetDouble();
            if (d["Lateral Coefficients"].HasMember("PDY3"))
                m_par.PDY3 = d["Lateral Coefficients"]["PDY3"].GetDouble();
            if (d["Lateral Coefficients"].HasMember("PEY1"))
                m_par.PEY1 = d["Lateral Coefficients"]["PEY1"].GetDouble();
            if (d["Lateral Coefficients"].HasMember("PEY2"))
                m_par.PEY2 = d["Lateral Coefficients"]["PEY2"].GetDouble();
            if (d["Lateral Coefficients"].HasMember("PEY3"))
                m_par.PEY3 = d["Lateral Coefficients"]["PEY3"].GetDouble();
            if (d["Lateral Coefficients"].HasMember("PEY4"))
                m_par.PEY4 = d["Lateral Coefficients"]["PEY4"].GetDouble();
            if (d["Lateral Coefficients"].HasMember("PKY1"))
                m_par.PKY1 = d["Lateral Coefficients"]["PKY1"].GetDouble();
            if (d["Lateral Coefficients"].HasMember("PKY2"))
                m_par.PKY2 = d["Lateral Coefficients"]["PKY2"].GetDouble();
            if (d["Lateral Coefficients"].HasMember("PKY3"))
                m_par.PKY3 = d["Lateral Coefficients"]["PKY3"].GetDouble();
            if (d["Lateral Coefficients"].HasMember("PHY1"))
                m_par.PHY1 = d["Lateral Coefficients"]["PHY1"].GetDouble();
            if (d["Lateral Coefficients"].HasMember("PHY2"))
                m_par.PHY2 = d["Lateral Coefficients"]["PHY2"].GetDouble();
            if (d["Lateral Coefficients"].HasMember("PHY3"))
                m_par.PHY3 = d["Lateral Coefficients"]["PHY3"].GetDouble();
            if (d["Lateral Coefficients"].HasMember("PVY1"))
                m_par.PVY1 = d["Lateral Coefficients"]["PVY1"].GetDouble();
            if (d["Lateral Coefficients"].HasMember("PVY2"))
                m_par.PVY2 = d["Lateral Coefficients"]["PVY2"].GetDouble();
            if (d["Lateral Coefficients"].HasMember("PVY3"))
                m_par.PVY3 = d["Lateral Coefficients"]["PVY3"].GetDouble();
            if (d["Lateral Coefficients"].HasMember("PVY4"))
                m_par.PVY4 = d["Lateral Coefficients"]["PVY4"].GetDouble();
            if (d["Lateral Coefficients"].HasMember("RBY1"))
                m_par.RBY1 = d["Lateral Coefficients"]["RBY1"].GetDouble();
            if (d["Lateral Coefficients"].HasMember("RBY2"))
                m_par.RBY2 = d["Lateral Coefficients"]["RBY2"].GetDouble();
            if (d["Lateral Coefficients"].HasMember("RBY3"))
                m_par.RBY3 = d["Lateral Coefficients"]["RBY3"].GetDouble();
            if (d["Lateral Coefficients"].HasMember("RCY1"))
                m_par.RCY1 = d["Lateral Coefficients"]["RCY1"].GetDouble();
            if (d["Lateral Coefficients"].HasMember("REY1"))
                m_par.REY1 = d["Lateral Coefficients"]["REY1"].GetDouble();
            if (d["Lateral Coefficients"].HasMember("REY2"))
                m_par.REY2 = d["Lateral Coefficients"]["REY2"].GetDouble();
            if (d["Lateral Coefficients"].HasMember("RHY1"))
                m_par.RHY1 = d["Lateral Coefficients"]["RHY1"].GetDouble();
            if (d["Lateral Coefficients"].HasMember("RHY2"))
                m_par.RHY2 = d["Lateral Coefficients"]["RHY2"].GetDouble();
            if (d["Lateral Coefficients"].HasMember("RVY1"))
                m_par.RVY1 = d["Lateral Coefficients"]["RVY1"].GetDouble();
            if (d["Lateral Coefficients"].HasMember("RVY2"))
                m_par.RVY2 = d["Lateral Coefficients"]["RVY2"].GetDouble();
            if (d["Lateral Coefficients"].HasMember("RVY3"))
                m_par.RVY3 = d["Lateral Coefficients"]["RVY3"].GetDouble();
            if (d["Lateral Coefficients"].HasMember("RVY4"))
                m_par.RVY4 = d["Lateral Coefficients"]["RVY4"].GetDouble();
            if (d["Lateral Coefficients"].HasMember("RVY5"))
                m_par.RVY5 = d["Lateral Coefficients"]["RVY5"].GetDouble();
            if (d["Lateral Coefficients"].HasMember("RVY6"))
                m_par.RVY6 = d["Lateral Coefficients"]["RVY6"].GetDouble();
            if (d["Lateral Coefficients"].HasMember("PTY1"))
                m_par.PTY1 = d["Lateral Coefficients"]["PTY1"].GetDouble();
            if (d["Lateral Coefficients"].HasMember("PTY2"))
                m_par.PTY2 = d["Lateral Coefficients"]["PTY2"].GetDouble();
            if (d["Lateral Coefficients"].HasMember("PPY1"))
                m_par.PPY1 = d["Lateral Coefficients"]["PPY1"].GetDouble();
            if (d["Lateral Coefficients"].HasMember("PPY2"))
                m_par.PPY2 = d["Lateral Coefficients"]["PPY2"].GetDouble();
            if (d["Lateral Coefficients"].HasMember("PPY3"))
                m_par.PPY3 = d["Lateral Coefficients"]["PPY3"].GetDouble();
            if (d["Lateral Coefficients"].HasMember("PPY4"))
                m_par.PPY4 = d["Lateral Coefficients"]["PPY4"].GetDouble();
        } else {
            GetLog() << "Fatal: Lateral Coefficients not set!\n";
            exit(9);
        }
        if (d.HasMember("Aligning Coefficients")) {
            if (d["Aligning Coefficients"].HasMember("QBZ1"))
                m_par.QBZ1 = d["Aligning Coefficients"]["QBZ1"].GetDouble();
            if (d["Aligning Coefficients"].HasMember("QBZ2"))
                m_par.QBZ2 = d["Aligning Coefficients"]["QBZ2"].GetDouble();
            if (d["Aligning Coefficients"].HasMember("QBZ3"))
                m_par.QBZ3 = d["Aligning Coefficients"]["QBZ3"].GetDouble();
            if (d["Aligning Coefficients"].HasMember("QBZ4"))
                m_par.QBZ4 = d["Aligning Coefficients"]["QBZ4"].GetDouble();
            if (d["Aligning Coefficients"].HasMember("QBZ5"))
                m_par.QBZ5 = d["Aligning Coefficients"]["QBZ5"].GetDouble();
            if (d["Aligning Coefficients"].HasMember("QBZ9"))
                m_par.QBZ9 = d["Aligning Coefficients"]["QBZ9"].GetDouble();
            if (d["Aligning Coefficients"].HasMember("QBZ10"))
                m_par.QBZ10 = d["Aligning Coefficients"]["QBZ10"].GetDouble();
            if (d["Aligning Coefficients"].HasMember("QCZ1"))
                m_par.QCZ1 = d["Aligning Coefficients"]["QCZ1"].GetDouble();
            if (d["Aligning Coefficients"].HasMember("QDZ1"))
                m_par.QDZ1 = d["Aligning Coefficients"]["QDZ1"].GetDouble();
            if (d["Aligning Coefficients"].HasMember("QDZ2"))
                m_par.QDZ2 = d["Aligning Coefficients"]["QDZ2"].GetDouble();
            if (d["Aligning Coefficients"].HasMember("QDZ3"))
                m_par.QDZ3 = d["Aligning Coefficients"]["QDZ3"].GetDouble();
            if (d["Aligning Coefficients"].HasMember("QDZ4"))
                m_par.QDZ4 = d["Aligning Coefficients"]["QDZ4"].GetDouble();
            if (d["Aligning Coefficients"].HasMember("QDZ6"))
                m_par.QDZ6 = d["Aligning Coefficients"]["QDZ6"].GetDouble();
            if (d["Aligning Coefficients"].HasMember("QDZ7"))
                m_par.QDZ7 = d["Aligning Coefficients"]["QDZ7"].GetDouble();
            if (d["Aligning Coefficients"].HasMember("QDZ8"))
                m_par.QDZ8 = d["Aligning Coefficients"]["QDZ8"].GetDouble();
            if (d["Aligning Coefficients"].HasMember("QDZ9"))
                m_par.QDZ9 = d["Aligning Coefficients"]["QDZ9"].GetDouble();
            if (d["Aligning Coefficients"].HasMember("QEZ1"))
                m_par.QEZ1 = d["Aligning Coefficients"]["QEZ1"].GetDouble();
            if (d["Aligning Coefficients"].HasMember("QEZ2"))
                m_par.QEZ2 = d["Aligning Coefficients"]["QEZ2"].GetDouble();
            if (d["Aligning Coefficients"].HasMember("QEZ3"))
                m_par.QEZ3 = d["Aligning Coefficients"]["QEZ3"].GetDouble();
            if (d["Aligning Coefficients"].HasMember("QEZ4"))
                m_par.QEZ4 = d["Aligning Coefficients"]["QEZ4"].GetDouble();
            if (d["Aligning Coefficients"].HasMember("QEZ5"))
                m_par.QEZ5 = d["Aligning Coefficients"]["QEZ5"].GetDouble();
            if (d["Aligning Coefficients"].HasMember("QHZ1"))
                m_par.QHZ1 = d["Aligning Coefficients"]["QHZ1"].GetDouble();
            if (d["Aligning Coefficients"].HasMember("QHZ2"))
                m_par.QHZ2 = d["Aligning Coefficients"]["QHZ2"].GetDouble();
            if (d["Aligning Coefficients"].HasMember("QHZ3"))
                m_par.QHZ3 = d["Aligning Coefficients"]["QHZ3"].GetDouble();
            if (d["Aligning Coefficients"].HasMember("QHZ4"))
                m_par.QHZ4 = d["Aligning Coefficients"]["QHZ4"].GetDouble();
            if (d["Aligning Coefficients"].HasMember("QPZ1"))
                m_par.QPZ1 = d["Aligning Coefficients"]["QPZ1"].GetDouble();
            if (d["Aligning Coefficients"].HasMember("QPZ2"))
                m_par.QPZ2 = d["Aligning Coefficients"]["QPZ2"].GetDouble();
            if (d["Aligning Coefficients"].HasMember("SSZ1"))
                m_par.SSZ1 = d["Aligning Coefficients"]["SSZ1"].GetDouble();
            if (d["Aligning Coefficients"].HasMember("SSZ2"))
                m_par.SSZ2 = d["Aligning Coefficients"]["SSZ2"].GetDouble();
            if (d["Aligning Coefficients"].HasMember("SSZ3"))
                m_par.SSZ3 = d["Aligning Coefficients"]["SSZ3"].GetDouble();
            if (d["Aligning Coefficients"].HasMember("SSZ4"))
                m_par.SSZ4 = d["Aligning Coefficients"]["SSZ4"].GetDouble();
            if (d["Aligning Coefficients"].HasMember("QTZ1"))
                m_par.QTZ1 = d["Aligning Coefficients"]["QTZ1"].GetDouble();
            if (d["Aligning Coefficients"].HasMember("MBELT"))
                m_par.MBELT = d["Aligning Coefficients"]["MBELT"].GetDouble();
        } else {
            GetLog() << "Fatal: Aligning Coefficients not set!\n";
            exit(9);
        }
        if (d.HasMember("Overturning Coefficients")) {
            if (d["Overturning Coefficients"].HasMember("QSX1"))
                m_par.QSX1 = d["Overturning Coefficients"]["QSX1"].GetDouble();
            if (d["Overturning Coefficients"].HasMember("QSX2"))
                m_par.QSX2 = d["Overturning Coefficients"]["QSX2"].GetDouble();
            if (d["Overturning Coefficients"].HasMember("QSX3"))
                m_par.QSX3 = d["Overturning Coefficients"]["QSX3"].GetDouble();
            if (d["Overturning Coefficients"].HasMember("QSX4"))
                m_par.QSX4 = d["Overturning Coefficients"]["QSX4"].GetDouble();
            if (d["Overturning Coefficients"].HasMember("QSX5"))
                m_par.QSX5 = d["Overturning Coefficients"]["QSX5"].GetDouble();
            if (d["Overturning Coefficients"].HasMember("QSX6"))
                m_par.QSX6 = d["Overturning Coefficients"]["QSX6"].GetDouble();
            if (d["Overturning Coefficients"].HasMember("QSX7"))
                m_par.QSX7 = d["Overturning Coefficients"]["QSX7"].GetDouble();
            if (d["Overturning Coefficients"].HasMember("QSX8"))
                m_par.QSX8 = d["Overturning Coefficients"]["QSX8"].GetDouble();
            if (d["Overturning Coefficients"].HasMember("QSX9"))
                m_par.QSX9 = d["Overturning Coefficients"]["QSX9"].GetDouble();
            if (d["Overturning Coefficients"].HasMember("QSX10"))
                m_par.QSX10 = d["Overturning Coefficients"]["QSX10"].GetDouble();
            if (d["Overturning Coefficients"].HasMember("QSX11"))
                m_par.QSX11 = d["Overturning Coefficients"]["QSX11"].GetDouble();
            if (d["Overturning Coefficients"].HasMember("QPX1"))
                m_par.QPX1 = d["Overturning Coefficients"]["QPX1"].GetDouble();
        } else {
            GetLog() << "Fatal: Overturning Coefficients not set!\n";
            exit(9);
        }
        if (d.HasMember("Rolling Coefficients")) {
            if (d["Rolling Coefficients"].HasMember("QSY1"))
                m_par.QSY1 = d["Rolling Coefficients"]["QSY1"].GetDouble();
            if (d["Rolling Coefficients"].HasMember("QSY2"))
                m_par.QSY2 = d["Rolling Coefficients"]["QSY2"].GetDouble();
            if (d["Rolling Coefficients"].HasMember("QSY3"))
                m_par.QSY3 = d["Rolling Coefficients"]["QSY3"].GetDouble();
            if (d["Rolling Coefficients"].HasMember("QSY4"))
                m_par.QSY4 = d["Rolling Coefficients"]["QSY4"].GetDouble();
            if (d["Rolling Coefficients"].HasMember("QSY5"))
                m_par.QSY5 = d["Rolling Coefficients"]["QSY5"].GetDouble();
            if (d["Rolling Coefficients"].HasMember("QSY6"))
                m_par.QSY6 = d["Rolling Coefficients"]["QSY6"].GetDouble();
            if (d["Rolling Coefficients"].HasMember("QSY7"))
                m_par.QSY7 = d["Rolling Coefficients"]["QSY7"].GetDouble();
            if (d["Rolling Coefficients"].HasMember("QSY8"))
                m_par.QSY8 = d["Rolling Coefficients"]["QSY8"].GetDouble();
        } else {
            GetLog() << "Fatal: Rolling Coefficients not set!\n";
            exit(9);
        }

        if (!m_tire_conditions_found) {
            // direct setting of inflation pressure is actually unsupported!
            // set all pressure dependence parameters to zero to prevent erratic force calculations
            m_par.PPX1 = 0;
            m_par.PPX2 = 0;
            m_par.PPX3 = 0;
            m_par.PPX4 = 0;
            m_par.PPY1 = 0;
            m_par.PPY2 = 0;
            m_par.PPY3 = 0;
            m_par.PPY4 = 0;
            m_par.QSY1 = 0;
            m_par.QSY2 = 0;
            m_par.QSY8 = 0;
            m_par.QPFZ1 = 0;
        }

        //// TODO
    }

    m_visualization_width = ChPac02Tire::GetVisualizationWidth();

    if (d.HasMember("Visualization")) {
        if (d["Visualization"].HasMember("Mesh Filename Left") && d["Visualization"].HasMember("Mesh Filename Right")) {
            m_meshFile_left = d["Visualization"]["Mesh Filename Left"].GetString();
            m_meshFile_right = d["Visualization"]["Mesh Filename Right"].GetString();
            m_has_mesh = true;
        }

        if (d["Visualization"].HasMember("Width")) {
            m_visualization_width = d["Visualization"]["Width"].GetDouble();
        }
    }
}

void Pac02Tire::SetMFParams() {
    if (!m_tir_file.empty()) {
        SetMFParamsByFile(vehicle::GetDataFile(m_tir_file));
    } else {
        //// TODO
    }
}

void Pac02Tire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH && m_has_mesh) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile_left,    // left side
                                               m_meshFile_right);  // right side
    } else {
        ChPac02Tire::AddVisualizationAssets(vis);
    }
}

void Pac02Tire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChPac02Tire::RemoveVisualizationAssets();
}

}  // namespace vehicle
}  // namespace chrono
