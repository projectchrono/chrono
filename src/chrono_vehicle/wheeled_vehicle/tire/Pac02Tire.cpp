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
// Authors: Radu Serban, Michael Taylor, Rainer Gericke
// =============================================================================
//
// Pac89 tire constructed with data from file (JSON format).
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/wheeled_vehicle/tire/Pac02Tire.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Pac02Tire::Pac02Tire(const std::string& filename)
    : ChPac02Tire(""), m_mass(0), m_has_mesh(false), m_has_vert_table(false), m_has_bott_table(false) {
    Document d; ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

Pac02Tire::Pac02Tire(const rapidjson::Document& d)
    : ChPac02Tire(""), m_mass(0), m_has_mesh(false), m_has_vert_table(false), m_has_bott_table(false) {
    Create(d);
}

void Pac02Tire::Create(const rapidjson::Document& d) {  // Invoke base class method.
    ChPart::Create(d);

    m_mass = d["Mass"].GetDouble();
    m_inertia = ReadVectorJSON(d["Inertia"]);
    if (d.HasMember("Use Mode")) {
        // Default value = 3
        m_use_mode = d["Use Mode"].GetInt();
    }
    if (d.HasMember("Use Friction Ellipsis")) {
        // Default value = true
        m_use_friction_ellipsis = d["Use Friction Ellipsis"].GetBool();
    }
    if (d.HasMember("Coefficient of Friction")) {
        // Default value = 0.8
        m_PacCoeff.mu0 = d["Coefficient of Friction"].GetDouble();
    }
    if (d.HasMember("Tire Side")) {
        // Default value = LEFT
        std::string tSide = d["Tire Side"].GetString();
        if (tSide.compare("left") == 0) {
            m_measured_side = LEFT;
            m_allow_mirroring = true;
        }
        if (tSide.compare("right") == 0) {
            m_measured_side = RIGHT;
            m_allow_mirroring = true;
        }
    }
    if (d.HasMember("Dimension")) {
        m_PacCoeff.R0 = d["Dimension"]["Unloaded Radius"].GetDouble();
        m_PacCoeff.width = d["Dimension"]["Width"].GetDouble();
        m_PacCoeff.aspect_ratio = d["Dimension"]["Aspect Ratio"].GetDouble();
        m_PacCoeff.rim_radius = d["Dimension"]["Rim Radius"].GetDouble();
        m_PacCoeff.rim_width = d["Dimension"]["Rim Width"].GetDouble();
    } else {
        GetLog() << "Couldn't find mandatory block 'Dimension' in JSON file.\n";
    }

    if (d.HasMember("Vertical")) {
        m_PacCoeff.Cz = d["Vertical"]["Vertical Stiffness"].GetDouble();
        m_PacCoeff.Kz = d["Vertical"]["Vertical Damping"].GetDouble();
        m_PacCoeff.FzNomin = d["Vertical"]["Nominal Wheel Load"].GetDouble();
        if (d["Vertical"].HasMember("Vertical Curve Data")) {
            int num_points = d["Vertical"]["Vertical Curve Data"].Size();
            for (int i = 0; i < num_points; i++) {
                m_vert_map.AddPoint(d["Vertical"]["Vertical Curve Data"][i][0u].GetDouble(),
                                    d["Vertical"]["Vertical Curve Data"][i][1u].GetDouble());
            }
            m_has_vert_table = true;
        }
        if (d["Vertical"].HasMember("Bottoming Curve Data")) {
            int num_points = d["Vertical"]["Bottoming Curve Data"].Size();
            for (int i = 0; i < num_points; i++) {
                m_vert_map.AddPoint(d["Vertical"]["Bottoming Curve Data"][i][0u].GetDouble(),
                                    d["Vertical"]["Bottoming Curve Data"][i][1u].GetDouble());
            }
            m_has_bott_table = true;
        }
    }
    if (d.HasMember("Scaling Factors")) {
        if (d["Scaling Factors"].HasMember("lfz0")) {
            m_PacScal.lfz0 = d["Scaling Factors"]["lfz0"].GetDouble();
        }
        if (d["Scaling Factors"].HasMember("ltr")) {
            m_PacScal.ltr = d["Scaling Factors"]["ltr"].GetDouble();
        }
        if (d["Scaling Factors"].HasMember("lcx")) {
            m_PacScal.lcx = d["Scaling Factors"]["lcx"].GetDouble();
        }
        if (d["Scaling Factors"].HasMember("lmux")) {
            m_PacScal.lmux = d["Scaling Factors"]["lmux"].GetDouble();
        }
        if (d["Scaling Factors"].HasMember("lex")) {
            m_PacScal.lex = d["Scaling Factors"]["lex"].GetDouble();
        }
        if (d["Scaling Factors"].HasMember("lkx")) {
            m_PacScal.lkx = d["Scaling Factors"]["lkx"].GetDouble();
        }
        if (d["Scaling Factors"].HasMember("lhx")) {
            m_PacScal.lhx = d["Scaling Factors"]["lhx"].GetDouble();
        }
        if (d["Scaling Factors"].HasMember("lvx")) {
            m_PacScal.lvx = d["Scaling Factors"]["lvx"].GetDouble();
        }
        if (d["Scaling Factors"].HasMember("lgax")) {
            m_PacScal.lgax = d["Scaling Factors"]["lgax"].GetDouble();
        }
        if (d["Scaling Factors"].HasMember("lcy")) {
            m_PacScal.lcy = d["Scaling Factors"]["lcy"].GetDouble();
        }
        if (d["Scaling Factors"].HasMember("lmuy")) {
            m_PacScal.lmuy = d["Scaling Factors"]["lmuy"].GetDouble();
        }
        if (d["Scaling Factors"].HasMember("ley")) {
            m_PacScal.ley = d["Scaling Factors"]["ley"].GetDouble();
        }
        if (d["Scaling Factors"].HasMember("lky")) {
            m_PacScal.lky = d["Scaling Factors"]["lky"].GetDouble();
        }
        if (d["Scaling Factors"].HasMember("lhy")) {
            m_PacScal.lhy = d["Scaling Factors"]["lhy"].GetDouble();
        }
        if (d["Scaling Factors"].HasMember("lvy")) {
            m_PacScal.lvy = d["Scaling Factors"]["lvy"].GetDouble();
        }
        if (d["Scaling Factors"].HasMember("lgay")) {
            m_PacScal.lgay = d["Scaling Factors"]["lgay"].GetDouble();
        }
        if (d["Scaling Factors"].HasMember("lres")) {
            m_PacScal.lres = d["Scaling Factors"]["lres"].GetDouble();
        }
        if (d["Scaling Factors"].HasMember("lgaz")) {
            m_PacScal.lgaz = d["Scaling Factors"]["lgaz"].GetDouble();
        }
        if (d["Scaling Factors"].HasMember("lxal")) {
            m_PacScal.lxal = d["Scaling Factors"]["lxal"].GetDouble();
        }
        if (d["Scaling Factors"].HasMember("lyka")) {
            m_PacScal.lyka = d["Scaling Factors"]["lyka"].GetDouble();
        }
        if (d["Scaling Factors"].HasMember("lvyka")) {
            m_PacScal.lvyka = d["Scaling Factors"]["lvyka"].GetDouble();
        }
        if (d["Scaling Factors"].HasMember("ls")) {
            m_PacScal.ls = d["Scaling Factors"]["ls"].GetDouble();
        }
        if (d["Scaling Factors"].HasMember("lsgkp")) {
            m_PacScal.lsgkp = d["Scaling Factors"]["lsgkp"].GetDouble();
        }
        if (d["Scaling Factors"].HasMember("lsgal")) {
            m_PacScal.lsgal = d["Scaling Factors"]["lsgal"].GetDouble();
        }
        if (d["Scaling Factors"].HasMember("lgyr")) {
            m_PacScal.lgyr = d["Scaling Factors"]["lgyr"].GetDouble();
        }
        if (d["Scaling Factors"].HasMember("lmx")) {
            m_PacScal.lmx = d["Scaling Factors"]["lmx"].GetDouble();
        }
        if (d["Scaling Factors"].HasMember("lvmx")) {
            m_PacScal.lvmx = d["Scaling Factors"]["lvmx"].GetDouble();
        }
        if (d["Scaling Factors"].HasMember("lmy")) {
            m_PacScal.lmy = d["Scaling Factors"]["lmy"].GetDouble();
        }
    } else {
        GetLog() << "All scaling factors have been set to 1.0\n";
    }
    if (d.HasMember("Lateral Coefficients")) {
        if (d["Lateral Coefficients"].HasMember("pcy1")) {
            m_PacCoeff.pcy1 = d["Lateral Coefficients"]["pcy1"].GetDouble();
        }
        if (d["Lateral Coefficients"].HasMember("pdy1")) {
            m_PacCoeff.pdy1 = d["Lateral Coefficients"]["pdy1"].GetDouble();
        }
        if (d["Lateral Coefficients"].HasMember("pdy2")) {
            m_PacCoeff.pdy2 = d["Lateral Coefficients"]["pdy2"].GetDouble();
        }
        if (d["Lateral Coefficients"].HasMember("pdy3")) {
            m_PacCoeff.pdy3 = d["Lateral Coefficients"]["pdy3"].GetDouble();
        }
        if (d["Lateral Coefficients"].HasMember("pey1")) {
            m_PacCoeff.pey1 = d["Lateral Coefficients"]["pey1"].GetDouble();
        }
        if (d["Lateral Coefficients"].HasMember("pey2")) {
            m_PacCoeff.pey2 = d["Lateral Coefficients"]["pey2"].GetDouble();
        }
        if (d["Lateral Coefficients"].HasMember("pey3")) {
            m_PacCoeff.pey3 = d["Lateral Coefficients"]["pey3"].GetDouble();
        }
        if (d["Lateral Coefficients"].HasMember("pey4")) {
            m_PacCoeff.pey4 = d["Lateral Coefficients"]["pey4"].GetDouble();
        }
        if (d["Lateral Coefficients"].HasMember("pky1")) {
            m_PacCoeff.pky1 = d["Lateral Coefficients"]["pky1"].GetDouble();
        }
        if (d["Lateral Coefficients"].HasMember("pky2")) {
            m_PacCoeff.pky2 = d["Lateral Coefficients"]["pky2"].GetDouble();
        }
        if (d["Lateral Coefficients"].HasMember("pky3")) {
            m_PacCoeff.pky3 = d["Lateral Coefficients"]["pky3"].GetDouble();
        }
        if (d["Lateral Coefficients"].HasMember("phy1")) {
            m_PacCoeff.phy1 = d["Lateral Coefficients"]["phy1"].GetDouble();
        }
        if (d["Lateral Coefficients"].HasMember("phy2")) {
            m_PacCoeff.phy2 = d["Lateral Coefficients"]["phy2"].GetDouble();
        }
        if (d["Lateral Coefficients"].HasMember("phy3")) {
            m_PacCoeff.phy3 = d["Lateral Coefficients"]["phy3"].GetDouble();
        }
        if (d["Lateral Coefficients"].HasMember("pvy1")) {
            m_PacCoeff.pvy1 = d["Lateral Coefficients"]["pvy1"].GetDouble();
        }
        if (d["Lateral Coefficients"].HasMember("pvy2")) {
            m_PacCoeff.pvy2 = d["Lateral Coefficients"]["pvy2"].GetDouble();
        }
        if (d["Lateral Coefficients"].HasMember("pvy3")) {
            m_PacCoeff.pvy3 = d["Lateral Coefficients"]["pvy3"].GetDouble();
        }
        if (d["Lateral Coefficients"].HasMember("pvy4")) {
            m_PacCoeff.pvy4 = d["Lateral Coefficients"]["pvy4"].GetDouble();
        }
        if (d["Lateral Coefficients"].HasMember("rby1")) {
            m_PacCoeff.rby1 = d["Lateral Coefficients"]["rby1"].GetDouble();
        }
        if (d["Lateral Coefficients"].HasMember("rby2")) {
            m_PacCoeff.rby2 = d["Lateral Coefficients"]["rby2"].GetDouble();
        }
        if (d["Lateral Coefficients"].HasMember("rby3")) {
            m_PacCoeff.rby3 = d["Lateral Coefficients"]["rby3"].GetDouble();
        }
        if (d["Lateral Coefficients"].HasMember("rcy1")) {
            m_PacCoeff.rcy1 = d["Lateral Coefficients"]["rcy1"].GetDouble();
        }
        if (d["Lateral Coefficients"].HasMember("rey1")) {
            m_PacCoeff.rey1 = d["Lateral Coefficients"]["rey1"].GetDouble();
        }
        if (d["Lateral Coefficients"].HasMember("rey2")) {
            m_PacCoeff.rey2 = d["Lateral Coefficients"]["rey2"].GetDouble();
        }
        if (d["Lateral Coefficients"].HasMember("rhy1")) {
            m_PacCoeff.rhy1 = d["Lateral Coefficients"]["rhy1"].GetDouble();
        }
        if (d["Lateral Coefficients"].HasMember("rhy2")) {
            m_PacCoeff.rhy2 = d["Lateral Coefficients"]["rhy2"].GetDouble();
        }
        if (d["Lateral Coefficients"].HasMember("rvy1")) {
            m_PacCoeff.rvy1 = d["Lateral Coefficients"]["rvy1"].GetDouble();
        }
        if (d["Lateral Coefficients"].HasMember("rvy2")) {
            m_PacCoeff.rvy2 = d["Lateral Coefficients"]["rvy2"].GetDouble();
        }
        if (d["Lateral Coefficients"].HasMember("rvy3")) {
            m_PacCoeff.rvy3 = d["Lateral Coefficients"]["rvy3"].GetDouble();
        }
        if (d["Lateral Coefficients"].HasMember("rvy4")) {
            m_PacCoeff.rvy4 = d["Lateral Coefficients"]["rvy4"].GetDouble();
        }
        if (d["Lateral Coefficients"].HasMember("rvy5")) {
            m_PacCoeff.rvy5 = d["Lateral Coefficients"]["rvy5"].GetDouble();
        }
        if (d["Lateral Coefficients"].HasMember("rvy6")) {
            m_PacCoeff.rvy6 = d["Lateral Coefficients"]["rvy6"].GetDouble();
        }
        if (d["Lateral Coefficients"].HasMember("pty1")) {
            m_PacCoeff.pty1 = d["Lateral Coefficients"]["pty1"].GetDouble();
        }
        if (d["Lateral Coefficients"].HasMember("pty2")) {
            m_PacCoeff.pty2 = d["Lateral Coefficients"]["pty2"].GetDouble();
        }
    }
    if (d.HasMember("Longitudinal Coefficients")) {
        if (d["Longitudinal Coefficients"].HasMember("pcx1")) {
            m_PacCoeff.pcx1 = d["Longitudinal Coefficients"]["pcx1"].GetDouble();
        }
        if (d["Longitudinal Coefficients"].HasMember("pdx1")) {
            m_PacCoeff.pdx1 = d["Longitudinal Coefficients"]["pdx1"].GetDouble();
        }
        if (d["Longitudinal Coefficients"].HasMember("pdx2")) {
            m_PacCoeff.pdx2 = d["Longitudinal Coefficients"]["pdx2"].GetDouble();
        }
        if (d["Longitudinal Coefficients"].HasMember("pdx3")) {
            m_PacCoeff.pdx3 = d["Longitudinal Coefficients"]["pdx3"].GetDouble();
        }
        if (d["Longitudinal Coefficients"].HasMember("pex1")) {
            m_PacCoeff.pex1 = d["Longitudinal Coefficients"]["pex1"].GetDouble();
        }
        if (d["Longitudinal Coefficients"].HasMember("pex2")) {
            m_PacCoeff.pex2 = d["Longitudinal Coefficients"]["pex2"].GetDouble();
        }
        if (d["Longitudinal Coefficients"].HasMember("pex3")) {
            m_PacCoeff.pex3 = d["Longitudinal Coefficients"]["pex3"].GetDouble();
        }
        if (d["Longitudinal Coefficients"].HasMember("pex4")) {
            m_PacCoeff.pex4 = d["Longitudinal Coefficients"]["pex4"].GetDouble();
        }
        if (d["Longitudinal Coefficients"].HasMember("pkx1")) {
            m_PacCoeff.pkx1 = d["Longitudinal Coefficients"]["pkx1"].GetDouble();
        }
        if (d["Longitudinal Coefficients"].HasMember("pkx2")) {
            m_PacCoeff.pkx2 = d["Longitudinal Coefficients"]["pkx2"].GetDouble();
        }
        if (d["Longitudinal Coefficients"].HasMember("pkx3")) {
            m_PacCoeff.pkx3 = d["Longitudinal Coefficients"]["pkx3"].GetDouble();
        }
        if (d["Longitudinal Coefficients"].HasMember("phx1")) {
            m_PacCoeff.phx1 = d["Longitudinal Coefficients"]["phx1"].GetDouble();
        }
        if (d["Longitudinal Coefficients"].HasMember("phx2")) {
            m_PacCoeff.phx2 = d["Longitudinal Coefficients"]["phx2"].GetDouble();
        }
        if (d["Longitudinal Coefficients"].HasMember("pvx1")) {
            m_PacCoeff.pvx1 = d["Longitudinal Coefficients"]["pvx1"].GetDouble();
        }
        if (d["Longitudinal Coefficients"].HasMember("pvx2")) {
            m_PacCoeff.pvx2 = d["Longitudinal Coefficients"]["pvx2"].GetDouble();
        }
        if (d["Longitudinal Coefficients"].HasMember("rbx1")) {
            m_PacCoeff.rbx1 = d["Longitudinal Coefficients"]["rbx1"].GetDouble();
        }
        if (d["Longitudinal Coefficients"].HasMember("rbx2")) {
            m_PacCoeff.rbx2 = d["Longitudinal Coefficients"]["rbx2"].GetDouble();
        }
        if (d["Longitudinal Coefficients"].HasMember("rcx1")) {
            m_PacCoeff.rcx1 = d["Longitudinal Coefficients"]["rcx1"].GetDouble();
        }
        if (d["Longitudinal Coefficients"].HasMember("rex1")) {
            m_PacCoeff.rex1 = d["Longitudinal Coefficients"]["rex1"].GetDouble();
        }
        if (d["Longitudinal Coefficients"].HasMember("rex2")) {
            m_PacCoeff.rex2 = d["Longitudinal Coefficients"]["rex2"].GetDouble();
        }
        if (d["Longitudinal Coefficients"].HasMember("rhx1")) {
            m_PacCoeff.rhx1 = d["Longitudinal Coefficients"]["rhx1"].GetDouble();
        }
        if (d["Longitudinal Coefficients"].HasMember("ptx1")) {
            m_PacCoeff.ptx1 = d["Longitudinal Coefficients"]["ptx1"].GetDouble();
        }
        if (d["Longitudinal Coefficients"].HasMember("ptx2")) {
            m_PacCoeff.ptx2 = d["Longitudinal Coefficients"]["ptx2"].GetDouble();
        }
        if (d["Longitudinal Coefficients"].HasMember("ptx3")) {
            m_PacCoeff.ptx3 = d["Longitudinal Coefficients"]["ptx3"].GetDouble();
        }
    }

    if (d.HasMember("Aligning Coefficients")) {
        if (d["Aligning Coefficients"].HasMember("qbz1")) {
            m_PacCoeff.qbz1 = d["Aligning Coefficients"]["qbz1"].GetDouble();
        }
        if (d["Aligning Coefficients"].HasMember("qbz2")) {
            m_PacCoeff.qbz2 = d["Aligning Coefficients"]["qbz2"].GetDouble();
        }
        if (d["Aligning Coefficients"].HasMember("qbz3")) {
            m_PacCoeff.qbz3 = d["Aligning Coefficients"]["qbz3"].GetDouble();
        }
        if (d["Aligning Coefficients"].HasMember("qbz4")) {
            m_PacCoeff.qbz4 = d["Aligning Coefficients"]["qbz4"].GetDouble();
        }
        if (d["Aligning Coefficients"].HasMember("qbz5")) {
            m_PacCoeff.qbz5 = d["Aligning Coefficients"]["qbz5"].GetDouble();
        }
        if (d["Aligning Coefficients"].HasMember("qbz6")) {
            m_PacCoeff.qbz6 = d["Aligning Coefficients"]["qbz6"].GetDouble();
        }
        if (d["Aligning Coefficients"].HasMember("qbz9")) {
            m_PacCoeff.qbz9 = d["Aligning Coefficients"]["qbz9"].GetDouble();
        }
        if (d["Aligning Coefficients"].HasMember("qbz10")) {
            m_PacCoeff.qbz10 = d["Aligning Coefficients"]["qbz10"].GetDouble();
        }
        if (d["Aligning Coefficients"].HasMember("qcz1")) {
            m_PacCoeff.qcz1 = d["Aligning Coefficients"]["qcz1"].GetDouble();
        }
        if (d["Aligning Coefficients"].HasMember("qdz1")) {
            m_PacCoeff.qdz1 = d["Aligning Coefficients"]["qdz1"].GetDouble();
        }
        if (d["Aligning Coefficients"].HasMember("qdz2")) {
            m_PacCoeff.qdz2 = d["Aligning Coefficients"]["qdz2"].GetDouble();
        }
        if (d["Aligning Coefficients"].HasMember("qdz3")) {
            m_PacCoeff.qdz3 = d["Aligning Coefficients"]["qdz3"].GetDouble();
        }
        if (d["Aligning Coefficients"].HasMember("qdz4")) {
            m_PacCoeff.qdz4 = d["Aligning Coefficients"]["qdz4"].GetDouble();
        }
        if (d["Aligning Coefficients"].HasMember("qdz5")) {
            m_PacCoeff.qdz5 = d["Aligning Coefficients"]["qdz5"].GetDouble();
        }
        if (d["Aligning Coefficients"].HasMember("qdz6")) {
            m_PacCoeff.qdz6 = d["Aligning Coefficients"]["qdz6"].GetDouble();
        }
        if (d["Aligning Coefficients"].HasMember("qdz7")) {
            m_PacCoeff.qdz7 = d["Aligning Coefficients"]["qdz7"].GetDouble();
        }
        if (d["Aligning Coefficients"].HasMember("qdz8")) {
            m_PacCoeff.qdz8 = d["Aligning Coefficients"]["qdz8"].GetDouble();
        }
        if (d["Aligning Coefficients"].HasMember("qdz9")) {
            m_PacCoeff.qdz9 = d["Aligning Coefficients"]["qdz9"].GetDouble();
        }
        if (d["Aligning Coefficients"].HasMember("qez1")) {
            m_PacCoeff.qez1 = d["Aligning Coefficients"]["qez1"].GetDouble();
        }
        if (d["Aligning Coefficients"].HasMember("qez2")) {
            m_PacCoeff.qez2 = d["Aligning Coefficients"]["qez2"].GetDouble();
        }
        if (d["Aligning Coefficients"].HasMember("qez3")) {
            m_PacCoeff.qez3 = d["Aligning Coefficients"]["qez3"].GetDouble();
        }
        if (d["Aligning Coefficients"].HasMember("qez4")) {
            m_PacCoeff.qez4 = d["Aligning Coefficients"]["qez4"].GetDouble();
        }
        if (d["Aligning Coefficients"].HasMember("qez5")) {
            m_PacCoeff.qez5 = d["Aligning Coefficients"]["qez5"].GetDouble();
        }
        if (d["Aligning Coefficients"].HasMember("qhz1")) {
            m_PacCoeff.qhz1 = d["Aligning Coefficients"]["qhz1"].GetDouble();
        }
        if (d["Aligning Coefficients"].HasMember("qhz2")) {
            m_PacCoeff.qhz2 = d["Aligning Coefficients"]["qhz2"].GetDouble();
        }
        if (d["Aligning Coefficients"].HasMember("qhz3")) {
            m_PacCoeff.qhz3 = d["Aligning Coefficients"]["qhz3"].GetDouble();
        }
        if (d["Aligning Coefficients"].HasMember("qhz4")) {
            m_PacCoeff.qhz4 = d["Aligning Coefficients"]["qhz4"].GetDouble();
        }
        if (d["Aligning Coefficients"].HasMember("ssz1")) {
            m_PacCoeff.ssz1 = d["Aligning Coefficients"]["ssz1"].GetDouble();
        }
        if (d["Aligning Coefficients"].HasMember("ssz2")) {
            m_PacCoeff.ssz2 = d["Aligning Coefficients"]["ssz2"].GetDouble();
        }
        if (d["Aligning Coefficients"].HasMember("ssz3")) {
            m_PacCoeff.ssz3 = d["Aligning Coefficients"]["ssz3"].GetDouble();
        }
        if (d["Aligning Coefficients"].HasMember("ssz4")) {
            m_PacCoeff.ssz4 = d["Aligning Coefficients"]["ssz4"].GetDouble();
        }
        if (d["Aligning Coefficients"].HasMember("qtz1")) {
            m_PacCoeff.qtz1 = d["Aligning Coefficients"]["qtz1"].GetDouble();
        }
        if (d["Aligning Coefficients"].HasMember("mbelt")) {
            m_PacCoeff.mbelt = d["Aligning Coefficients"]["mbelt"].GetDouble();
        }
    }
    if (d.HasMember("Rolling Coefficients")) {
        // m_PacCoeff.A0 = d["Lateral Coefficients"]["a0"].GetDouble();
        if (d["Rolling Coefficients"].HasMember("qsy1")) {
            m_PacCoeff.qsy1 = d["Rolling Coefficients"]["qsy1"].GetDouble();
        }
        if (d["Rolling Coefficients"].HasMember("qsy2")) {
            m_PacCoeff.qsy2 = d["Rolling Coefficients"]["qsy2"].GetDouble();
        }
        if (d["Rolling Coefficients"].HasMember("qsy3")) {
            m_PacCoeff.qsy3 = d["Rolling Coefficients"]["qsy3"].GetDouble();
        }
        if (d["Rolling Coefficients"].HasMember("qsy4")) {
            m_PacCoeff.qsy4 = d["Rolling Coefficients"]["qsy4"].GetDouble();
        }
        if (d["Rolling Coefficients"].HasMember("qyx5")) {
            m_PacCoeff.qsy5 = d["Rolling Coefficients"]["qsy5"].GetDouble();
        }
        if (d["Rolling Coefficients"].HasMember("qyx6")) {
            m_PacCoeff.qsy6 = d["Rolling Coefficients"]["qsy6"].GetDouble();
        }
        if (d["Rolling Coefficients"].HasMember("qsy7")) {
            m_PacCoeff.qsy7 = d["Rolling Coefficients"]["qsy7"].GetDouble();
        }
        if (d["Rolling Coefficients"].HasMember("qsy8")) {
            m_PacCoeff.qsy1 = d["Rolling Coefficients"]["qsy8"].GetDouble();
        }
    }
    if (d.HasMember("Overturning Coefficients")) {
        if (d["Overturning Coefficients"].HasMember("qsx1")) {
            m_PacCoeff.qsx1 = d["Overturning Coefficients"]["qsx1"].GetDouble();
        }
        if (d["Overturning Coefficients"].HasMember("qsx2")) {
            m_PacCoeff.qsx2 = d["Overturning Coefficients"]["qsx2"].GetDouble();
        }
        if (d["Overturning Coefficients"].HasMember("qsx3")) {
            m_PacCoeff.qsx3 = d["Overturning Coefficients"]["qsx3"].GetDouble();
        }
        if (d["Overturning Coefficients"].HasMember("qsx4")) {
            m_PacCoeff.qsx4 = d["Overturning Coefficients"]["qsx4"].GetDouble();
        }
        if (d["Overturning Coefficients"].HasMember("qsx5")) {
            m_PacCoeff.qsx5 = d["Overturning Coefficients"]["qsx5"].GetDouble();
        }
        if (d["Overturning Coefficients"].HasMember("qsx6")) {
            m_PacCoeff.qsx6 = d["Overturning Coefficients"]["qsx6"].GetDouble();
        }
        if (d["Overturning Coefficients"].HasMember("qsx7")) {
            m_PacCoeff.qsx7 = d["Overturning Coefficients"]["qsx7"].GetDouble();
        }
        if (d["Overturning Coefficients"].HasMember("qsx8")) {
            m_PacCoeff.qsx8 = d["Overturning Coefficients"]["qsx8"].GetDouble();
        }
        if (d["Overturning Coefficients"].HasMember("qsx9")) {
            m_PacCoeff.qsx9 = d["Overturning Coefficients"]["qsx9"].GetDouble();
        }
        if (d["Overturning Coefficients"].HasMember("qsx10")) {
            m_PacCoeff.qsx10 = d["Overturning Coefficients"]["qsx10"].GetDouble();
        }
        if (d["Overturning Coefficients"].HasMember("qsx11")) {
            m_PacCoeff.qsx11 = d["Overturning Coefficients"]["qsx11"].GetDouble();
        }
    }

    m_visualization_width = m_PacCoeff.width;
    // Check how to visualize this tire.
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
