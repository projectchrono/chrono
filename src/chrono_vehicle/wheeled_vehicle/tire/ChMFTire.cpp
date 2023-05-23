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
// Template for a tire model based on the Pacejka 2002 Tire Model
//
// =============================================================================
// =============================================================================
// STILL UNDERDEVELOPMENT
// =============================================================================
// =============================================================================

#include <cstdio>
#include <cstdlib>
#include <cctype>
#include <cstring>

#include <algorithm>
#include <cmath>

#include "chrono/core/ChGlobal.h"
#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_vehicle/wheeled_vehicle/tire/ChMFTire.h"

namespace chrono {
namespace vehicle {

ChMFTire::ChMFTire(const std::string& name)
    : ChForceElementTire(name),
      m_gamma_limit(3.0 * CH_C_DEG_TO_RAD),
      m_use_friction_ellipsis(true),
      m_mu_road(0),
      m_Shf(0),
      m_measured_side(LEFT),
      m_allow_mirroring(false),
      m_use_mode(0) {
    m_tireforce.force = ChVector<>(0, 0, 0);
    m_tireforce.point = ChVector<>(0, 0, 0);
    m_tireforce.moment = ChVector<>(0, 0, 0);
}

// -----------------------------------------------------------------------------

void ChMFTire::SetMFParamsByFile(std::string& tirFileName) {
    std::string dataFile = vehicle::GetDataFile(tirFileName);

    FILE* fp = fopen(dataFile.c_str(), "r+");
    if (fp == NULL) {
        GetLog() << "TIR File not found <" << dataFile << ">!\n";
        exit(1);
    }
    LoadSectionUnits(fp);
    LoadSectionModel(fp);
    LoadSectionDimension(fp);
    LoadSectionVertical(fp);
    LoadSectionScaling(fp);
    LoadSectionLongitudinal(fp);
    LoadSectionOverturning(fp);
    LoadSectionLateral(fp);
    LoadSectionRolling(fp);
    LoadSectionAligning(fp);
    fclose(fp);
}

bool ChMFTire::FindSectionStart(std::string sectName, FILE* fp) {
    bool ret = false;
    rewind(fp);
    while (!feof(fp)) {
        char line[201];
        fgets(line, 200, fp);  // buffer one line
        // remove leading white space
        size_t l = strlen(line);
        size_t ipos = 0;
        while (isblank(line[ipos])) {
            if (ipos < l)
                ipos++;
        }
        std::string sbuf(line + ipos);
        // don't process pure comment lines
        if (sbuf.front() == '!' || sbuf.front() == '$')
            continue;
        // section name contained in line?
        size_t npos = sbuf.find(sectName);
        if (npos != std::string::npos) {
            ret = true;
            break;
        }
    }
    return ret;
}

void ChMFTire::LoadSectionUnits(FILE* fp) {
    bool ok = FindSectionStart("[UNITS]", fp);
    if (!ok) {
        GetLog() << "Desired section [UNITS] not found.\n";
        return;
    }
    while (!feof(fp)) {
        char line[201];
        fgets(line, 200, fp);  // buffer one line
        // remove leading white space
        size_t l = strlen(line);
        size_t ipos = 0;
        while (isblank(line[ipos])) {
            if (ipos < l)
                ipos++;
        }
        std::string sbuf(line + ipos);
        // skip pure comment lines
        if (sbuf.front() == '!' || sbuf.front() == '$')
            continue;
        // leave, since a new section is reached
        if (sbuf.front() == '[')
            break;
        // this should be a data line
        // there can be a trailing comment
        size_t trpos = sbuf.find_first_of("$");
        if (trpos != std::string::npos) {
            sbuf = sbuf.substr(0, trpos - 1);
        }
        // GetLog() << sbuf;
        std::string skey, sval;
        size_t eqpos = sbuf.find_first_of("=");
        if (eqpos != std::string::npos) {
            skey = sbuf.substr(0, eqpos - 1);
        }
        size_t bpos = skey.find_first_of(' ');
        if (bpos != std::string::npos) {
            skey = skey.substr(0, bpos);
        }
        sval = sbuf.substr(eqpos + 1);
        size_t a1pos = sval.find_first_of("'");
        size_t a2pos = sval.find_last_of("'");
        if (a1pos == std::string::npos) {
            // unprocessable input
            continue;
        }
        sval = sval.substr(a1pos + 1, a2pos - a1pos - 1);
        // change letters to upper case
        std::transform(sval.begin(), sval.end(), sval.begin(), ::toupper);
        // GetLog() << "Key=" << skey << "  Val=" << sval << "\n";
        if (skey.compare("LENGTH") == 0) {
            // GetLog() << skey << "\n";
            if (sval.compare("METER") == 0) {
                m_par.u_length = 1.0;
            } else if (sval.compare("MM") == 0) {
                m_par.u_length = 0.001;
            } else if (sval.compare("CM") == 0) {
                m_par.u_length = 0.01;
            } else if (sval.compare("KM") == 0) {
                m_par.u_length = 1000.0;
            } else if (sval.compare("MILE") == 0) {
                m_par.u_length = 1609.35;
            } else if (sval.compare("FOOT") == 0) {
                m_par.u_length = 0.3048;
            } else if (sval.compare("IN") == 0) {
                m_par.u_length = 0.0254;
            } else {
                GetLog() << "No unit conversion for " << skey << "=" << sval << "\n";
            }
        } else if (skey.compare("TIME") == 0) {
            // GetLog() << skey << "\n";
            if (sval.compare("MILLI") == 0) {
                m_par.u_time = 0.001;
            } else if (sval.compare("SEC") == 0 || sval.compare("SECOND") == 0) {
                m_par.u_time = 1.0;
            } else if (sval.compare("MIN") == 0) {
                m_par.u_time = 60.0;
            } else if (sval.compare("HOUR") == 0) {
                m_par.u_time = 3600;
            } else {
                GetLog() << "No unit conversion for " << skey << "=" << sval << "\n";
            }
        } else if (skey.compare("ANGLE") == 0) {
            // GetLog() << skey << "\n";
            if (sval.compare("DEG") == 0) {
                m_par.u_angle = 0.0174532925;
            } else if (sval.compare("RAD") == 0 || sval.compare("RADIAN") == 0 || sval.compare("RADIANS") == 0) {
                m_par.u_angle = 1.0;
            } else {
                GetLog() << "No unit conversion for " << skey << "=" << sval << "\n";
            }
        } else if (skey.compare("MASS") == 0) {
            // GetLog() << skey << "\n";
            if (sval.compare("KG") == 0) {
                m_par.u_mass = 1.0;
            } else if (sval.compare("GRAM") == 0) {
                m_par.u_mass = 0.001;
            } else if (sval.compare("POUND_MASS") == 0) {
                m_par.u_mass = 0.45359237;
            } else if (sval.compare("KPOUND_MASS") == 0) {
                m_par.u_mass = 0.45359237 / 1000.0;
            } else if (sval.compare("SLUG") == 0) {
                m_par.u_mass = 14.593902937;
            } else if (sval.compare("OUNCE_MASS") == 0) {
                m_par.u_mass = 0.0283495231;
            } else {
                GetLog() << "No unit conversion for " << skey << "=" << sval << "\n";
            }
        } else if (skey.compare("FORCE") == 0) {
            // GetLog() << skey << "\n";
            if (sval.compare("N") == 0 || sval.compare("NEWTON") == 0) {
                m_par.u_force = 1.0;
            } else if (sval.compare("KN") == 0 || sval.compare("KNEWTON") == 0) {
                m_par.u_force = 0.001;
            } else if (sval.compare("POUND_FORCE") == 0) {
                m_par.u_force = 4.4482216153;
            } else if (sval.compare("KPOUND_FORCE") == 0) {
                m_par.u_force = 4.4482216153 / 1000.0;
            } else if (sval.compare("DYNE") == 0) {
                m_par.u_force = 0.00001;
            } else if (sval.compare("OUNCE_FORCE") == 0) {
                m_par.u_force = 0.278013851;
            } else if (sval.compare("KG_FORCE") == 0) {
                m_par.u_force = 9.80665;
            } else {
                GetLog() << "No unit conversion for " << skey << "=" << sval << "\n";
            }
        }
    }
    m_par.u_speed = m_par.u_length / m_par.u_time;
    m_par.u_inertia = m_par.u_mass * m_par.u_length * m_par.u_length;
    m_par.u_stiffness = m_par.u_force / m_par.u_length;
    m_par.u_damping = m_par.u_force / m_par.u_speed;
}

void ChMFTire::LoadSectionModel(FILE* fp) {
    bool ok = FindSectionStart("[MODEL]", fp);
    if (!ok) {
        GetLog() << "Desired section [MODEL] not found.\n";
        return;
    }
    while (!feof(fp)) {
        char line[201];
        fgets(line, 200, fp);  // buffer one line
        // remove leading white space
        size_t l = strlen(line);
        size_t ipos = 0;
        while (isblank(line[ipos])) {
            if (ipos < l)
                ipos++;
        }
        std::string sbuf(line + ipos);
        // skip pure comment lines
        if (sbuf.front() == '!' || sbuf.front() == '$')
            continue;
        // leave, since a new section is reached
        if (sbuf.front() == '[')
            break;
        // this should be a data line
        // there can be a trailing comment
        size_t trpos = sbuf.find_first_of("$");
        if (trpos != std::string::npos) {
            sbuf = sbuf.substr(0, trpos - 1);
        }
        // GetLog() << sbuf << "\n";
        // not all entries are of numerical type!
        size_t eqpos = sbuf.find_first_of("=");
        if (eqpos == std::string::npos)
            continue;
        std::string skey, sval;
        skey = sbuf.substr(0, eqpos);
        sval = sbuf.substr(eqpos + 1);
        size_t sppos = skey.find_first_of(" ");
        if (sppos != std::string::npos) {
            skey = skey.substr(0, sppos);
        }
        if (skey.compare("PROPERTY_FILE_FORMAT") == 0) {
            size_t a1pos = sval.find_first_of("'");
            size_t a2pos = sval.find_last_of("'");
            sval = sval.substr(a1pos, a2pos - a1pos + 1);
            // GetLog() << ">>Key=" << skey << "|" << sval << "|\n";
            if (sval.compare("'PAC2002'") != 0 && sval.compare("'MF_05'") != 0) {
                GetLog() << "FATAL: unknown file format " << sval << ".\n";
                exit(41);
            }
        }
        if (skey.compare("TYRESIDE") == 0) {
            size_t a1pos = sval.find_first_of("'");
            size_t a2pos = sval.find_last_of("'");
            sval = sval.substr(a1pos, a2pos - a1pos + 1);
            // GetLog() << ">>Key=" << skey << "|" << sval << "|\n";
            if (sval.compare("'LEFT'") != 0 || sval.compare("'UNKNOWN'") == 0) {
                m_measured_side = LEFT;
            } else {
                m_measured_side = RIGHT;
            }
        }
        if (skey.compare("USE_MODE") == 0) {
            m_par.USE_MODE = stoi(sval);
        }
        if (skey.compare("FITTYP") == 0) {
            m_par.FITTYP = stoi(sval);
        }
        if (skey.compare("VXLOW") == 0) {
            m_par.VXLOW = stod(sval);
        }
        if (skey.compare("LONGVL") == 0) {
            m_par.LONGVL = stod(sval);
        }
    }
}

void ChMFTire::LoadSectionDimension(FILE* fp) {
    bool ok = FindSectionStart("[DIMENSION]", fp);
    if (!ok) {
        GetLog() << "Desired section [DIMENSION] not found.\n";
        return;
    }
    while (!feof(fp)) {
        char line[201];
        fgets(line, 200, fp);  // buffer one line
        // remove leading white space
        size_t l = strlen(line);
        size_t ipos = 0;
        while (isblank(line[ipos])) {
            if (ipos < l)
                ipos++;
        }
        std::string sbuf(line + ipos);
        // skip pure comment lines
        if (sbuf.front() == '!' || sbuf.front() == '$')
            continue;
        // leave, since a new section is reached
        if (sbuf.front() == '[')
            break;
        // this should be a data line
        // there can be a trailing comment
        size_t trpos = sbuf.find_first_of("$");
        if (trpos != std::string::npos) {
            sbuf = sbuf.substr(0, trpos - 1);
        }
        // GetLog() << sbuf << "\n";
        // not all entries are of numerical type!
        size_t eqpos = sbuf.find_first_of("=");
        if (eqpos == std::string::npos)
            continue;
        std::string skey, sval;
        skey = sbuf.substr(0, eqpos);
        sval = sbuf.substr(eqpos + 1);
        size_t sppos = skey.find_first_of(" ");
        if (sppos != std::string::npos) {
            skey = skey.substr(0, sppos);
        }
        // GetLog() << ">>Key=" << skey << "|" << sval << "\n";
        if (skey.compare("UNLOADED_RADIUS") == 0) {
            m_par.UNLOADED_RADIUS = m_par.u_length * stod(sval);
            GetLog() << "R0 = " << m_par.UNLOADED_RADIUS << " m\n";
        }
        if (skey.compare("WIDTH") == 0) {
            m_par.WIDTH = m_par.u_length * stod(sval);
            GetLog() << "width = " << m_par.WIDTH << " m\n";
        }
        if (skey.compare("ASPECT_RATIO") == 0) {
            m_par.ASPECT_RATIO = stod(sval);
            GetLog() << "ratio = " << m_par.ASPECT_RATIO << "\n";
        }
        if (skey.compare("RIM_RADIUS") == 0) {
            m_par.RIM_RADIUS = m_par.u_length * stod(sval);
            GetLog() << "rimrad = " << m_par.RIM_RADIUS << " m\n";
        }
        if (skey.compare("RIM_WIDTH") == 0) {
            m_par.RIM_WIDTH = m_par.u_length * stod(sval);
            GetLog() << "rimw = " << m_par.RIM_WIDTH << " m\n";
        }
    }
}

void ChMFTire::LoadSectionVertical(FILE* fp) {
    bool ok = FindSectionStart("[VERTICAL]", fp);
    if (!ok) {
        GetLog() << "Desired section [VERTICAL] not found.\n";
        return;
    }
    while (!feof(fp)) {
        char line[201];
        fgets(line, 200, fp);  // buffer one line
        // remove leading white space
        size_t l = strlen(line);
        size_t ipos = 0;
        while (isblank(line[ipos])) {
            if (ipos < l)
                ipos++;
        }
        std::string sbuf(line + ipos);
        // skip pure comment lines
        if (sbuf.front() == '!' || sbuf.front() == '$')
            continue;
        // leave, since a new section is reached
        if (sbuf.front() == '[')
            break;
        // this should be a data line
        // there can be a trailing comment
        size_t trpos = sbuf.find_first_of("$");
        if (trpos != std::string::npos) {
            sbuf = sbuf.substr(0, trpos - 1);
        }
        // GetLog() << sbuf << "\n";
        // not all entries are of numerical type!
        size_t eqpos = sbuf.find_first_of("=");
        if (eqpos == std::string::npos)
            continue;
        std::string skey, sval;
        skey = sbuf.substr(0, eqpos);
        sval = sbuf.substr(eqpos + 1);
        size_t sppos = skey.find_first_of(" ");
        if (sppos != std::string::npos) {
            skey = skey.substr(0, sppos);
        }
        // GetLog() << ">>Key=" << skey << "|" << sval << "\n";
        if (skey.compare("VERTICAL_STIFFNESS") == 0) {
            m_par.VERTICAL_STIFFNESS = m_par.u_stiffness * stod(sval);
            GetLog() << "Cz = " << m_par.VERTICAL_STIFFNESS << " N/m\n";
        }
        if (skey.compare("VERTICAL_DAMPING") == 0) {
            m_par.VERTICAL_DAMPING = m_par.u_damping * stod(sval);
            GetLog() << "Kz = " << m_par.VERTICAL_DAMPING << " Ns/m\n";
        }
        if (skey.compare("BREFF") == 0) {
            m_par.BREFF = stod(sval);
            GetLog() << "BREFF = " << m_par.BREFF << "\n";
        }
        if (skey.compare("DREFF") == 0) {
            m_par.DREFF = stod(sval);
            GetLog() << "DREFF = " << m_par.DREFF << " \n";
        }
        if (skey.compare("FREFF") == 0) {
            m_par.FREFF = stod(sval);
            GetLog() << "FREFF = " << m_par.FREFF << " \n";
        }
        if (skey.compare("FNOMIN") == 0) {
            m_par.FNOMIN = m_par.u_force * stod(sval);
            GetLog() << "FNOMIN = " << m_par.FNOMIN << " N\n";
        }
    }
}

void ChMFTire::LoadSectionScaling(FILE* fp) {
    bool ok = FindSectionStart("[SCALING_COEFFICIENTS]", fp);
    if (!ok) {
        GetLog() << "Desired section [SCALING_COEFFICIENTS] not found.\n";
        return;
    }
    while (!feof(fp)) {
        char line[201];
        fgets(line, 200, fp);  // buffer one line
        // remove leading white space
        size_t l = strlen(line);
        size_t ipos = 0;
        while (isblank(line[ipos])) {
            if (ipos < l)
                ipos++;
        }
        std::string sbuf(line + ipos);
        // skip pure comment lines
        if (sbuf.front() == '!' || sbuf.front() == '$')
            continue;
        // leave, since a new section is reached
        if (sbuf.front() == '[')
            break;
        // this should be a data line
        // there can be a trailing comment
        size_t trpos = sbuf.find_first_of("$");
        if (trpos != std::string::npos) {
            sbuf = sbuf.substr(0, trpos - 1);
        }
        // GetLog() << sbuf << "\n";
        // not all entries are of numerical type!
        size_t eqpos = sbuf.find_first_of("=");
        if (eqpos == std::string::npos)
            continue;
        std::string skey, sval;
        skey = sbuf.substr(0, eqpos);
        sval = sbuf.substr(eqpos + 1);
        size_t sppos = skey.find_first_of(" ");
        if (sppos != std::string::npos) {
            skey = skey.substr(0, sppos);
        }
        // GetLog() << ">>Key=" << skey << "|" << sval << "\n";
        if (skey.compare("LFZO") == 0) {
            m_par.LFZO = stod(sval);
            GetLog() << "LFZO = " << m_par.LFZO << "\n";
        }
        if (skey.compare("LCX") == 0) {
            m_par.LCX = stod(sval);
            GetLog() << "LCX = " << m_par.LCX << "\n";
        }
        if (skey.compare("LMUX") == 0) {
            m_par.LMUX = stod(sval);
            GetLog() << "LMUX = " << m_par.LMUX << "\n";
        }
        if (skey.compare("LEX") == 0) {
            m_par.LEX = stod(sval);
            GetLog() << "LEX = " << m_par.LEX << "\n";
        }
        if (skey.compare("LKX") == 0) {
            m_par.LKX = stod(sval);
            GetLog() << "LKX = " << m_par.LKX << "\n";
        }
        if (skey.compare("LHX") == 0) {
            m_par.LHX = stod(sval);
            GetLog() << "LHX = " << m_par.LHX << "\n";
        }
        if (skey.compare("LVX") == 0) {
            m_par.LVX = stod(sval);
            GetLog() << "LVX = " << m_par.LVX << "\n";
        }
        if (skey.compare("LCY") == 0) {
            m_par.LCY = stod(sval);
            GetLog() << "LCY = " << m_par.LCY << "\n";
        }
        if (skey.compare("LMUY") == 0) {
            m_par.LMUY = stod(sval);
            GetLog() << "LMUY = " << m_par.LMUY << "\n";
        }
        if (skey.compare("LEY") == 0) {
            m_par.LEY = stod(sval);
            GetLog() << "LEY = " << m_par.LEY << "\n";
        }
        if (skey.compare("LKY") == 0) {
            m_par.LKY = stod(sval);
            GetLog() << "LKY = " << m_par.LKY << "\n";
        }
        if (skey.compare("LHY") == 0) {
            m_par.LHY = stod(sval);
            GetLog() << "LHY = " << m_par.LHY << "\n";
        }
        if (skey.compare("LVY") == 0) {
            m_par.LVY = stod(sval);
            GetLog() << "LVY = " << m_par.LVY << "\n";
        }
        if (skey.compare("LGAY") == 0) {
            m_par.LGAY = stod(sval);
            GetLog() << "LGAY = " << m_par.LGAY << "\n";
        }
        if (skey.compare("LTR") == 0) {
            m_par.LTR = stod(sval);
            GetLog() << "LTR = " << m_par.LTR << "\n";
        }
        if (skey.compare("LRES") == 0) {
            m_par.LRES = stod(sval);
            GetLog() << "LRES = " << m_par.LRES << "\n";
        }
        if (skey.compare("LGAZ") == 0) {
            m_par.LGAZ = stod(sval);
            GetLog() << "LGAZ = " << m_par.LGAZ << "\n";
        }
        if (skey.compare("LXAL") == 0) {
            m_par.LXAL = stod(sval);
            GetLog() << "LXAL = " << m_par.LXAL << "\n";
        }
        if (skey.compare("LYKA") == 0) {
            m_par.LYKA = stod(sval);
            GetLog() << "LYKA = " << m_par.LYKA << "\n";
        }
        if (skey.compare("LVYKA") == 0) {
            m_par.LVYKA = stod(sval);
            GetLog() << "LVYKA = " << m_par.LVYKA << "\n";
        }
        if (skey.compare("LS") == 0) {
            m_par.LS = stod(sval);
            GetLog() << "LS = " << m_par.LS << "\n";
        }
        if (skey.compare("LSGKP") == 0) {
            m_par.LSGKP = stod(sval);
            GetLog() << "LSGKP = " << m_par.LSGKP << "\n";
        }
        if (skey.compare("LSGAL") == 0) {
            m_par.LSGAL = stod(sval);
            GetLog() << "LSGAL = " << m_par.LSGAL << "\n";
        }
        if (skey.compare("LGYR") == 0) {
            m_par.LGYR = stod(sval);
            GetLog() << "LGYR = " << m_par.LGYR << "\n";
        }
        if (skey.compare("LMX") == 0) {
            m_par.LMX = stod(sval);
            GetLog() << "LMX = " << m_par.LMX << "\n";
        }
        if (skey.compare("LMY") == 0) {
            m_par.LMY = stod(sval);
            GetLog() << "LMY = " << m_par.LMY << "\n";
        }
    }
}

void ChMFTire::LoadSectionLongitudinal(FILE* fp) {
    bool ok = FindSectionStart("[LONGITUDINAL_COEFFICIENTS]", fp);
    if (!ok) {
        GetLog() << "Desired section [LONGITUDINAL_COEFFICIENTS] not found.\n";
        return;
    }
    while (!feof(fp)) {
        char line[201];
        fgets(line, 200, fp);  // buffer one line
        // remove leading white space
        size_t l = strlen(line);
        size_t ipos = 0;
        while (isblank(line[ipos])) {
            if (ipos < l)
                ipos++;
        }
        std::string sbuf(line + ipos);
        // skip pure comment lines
        if (sbuf.front() == '!' || sbuf.front() == '$')
            continue;
        // leave, since a new section is reached
        if (sbuf.front() == '[')
            break;
        // this should be a data line
        // there can be a trailing comment
        size_t trpos = sbuf.find_first_of("$");
        if (trpos != std::string::npos) {
            sbuf = sbuf.substr(0, trpos - 1);
        }
        // GetLog() << sbuf << "\n";
        // not all entries are of numerical type!
        size_t eqpos = sbuf.find_first_of("=");
        if (eqpos == std::string::npos)
            continue;
        std::string skey, sval;
        skey = sbuf.substr(0, eqpos);
        sval = sbuf.substr(eqpos + 1);
        size_t sppos = skey.find_first_of(" ");
        if (sppos != std::string::npos) {
            skey = skey.substr(0, sppos);
        }
        // GetLog() << ">>Key=" << skey << "|" << sval << "\n";
        if (skey.compare("PCX1") == 0) {
            m_par.PCX1 = stod(sval);
            GetLog() << "PCX1 = " << m_par.PCX1 << "\n";
        }
        if (skey.compare("PDX1") == 0) {
            m_par.PDX1 = stod(sval);
            GetLog() << "PDX1 = " << m_par.PDX1 << "\n";
        }
        if (skey.compare("PDX2") == 0) {
            m_par.PDX2 = stod(sval);
            GetLog() << "PDX2 = " << m_par.PDX2 << "\n";
        }
        if (skey.compare("PEX1") == 0) {
            m_par.PEX1 = stod(sval);
            GetLog() << "PEX1 = " << m_par.PEX1 << "\n";
        }
        if (skey.compare("PEX2") == 0) {
            m_par.PEX2 = stod(sval);
            GetLog() << "PEX2 = " << m_par.PEX2 << "\n";
        }
        if (skey.compare("PEX3") == 0) {
            m_par.PEX3 = stod(sval);
            GetLog() << "PEX3 = " << m_par.PEX3 << "\n";
        }
        if (skey.compare("PEX4") == 0) {
            m_par.PEX4 = stod(sval);
            GetLog() << "PEX4 = " << m_par.PEX4 << "\n";
        }
        if (skey.compare("PKX1") == 0) {
            m_par.PKX1 = stod(sval);
            GetLog() << "PKX1 = " << m_par.PKX1 << "\n";
        }
        if (skey.compare("PKX2") == 0) {
            m_par.PKX2 = stod(sval);
            GetLog() << "PKX2 = " << m_par.PKX2 << "\n";
        }
        if (skey.compare("PKX3") == 0) {
            m_par.PKX3 = stod(sval);
            GetLog() << "PKX3 = " << m_par.PKX3 << "\n";
        }
        if (skey.compare("PHX1") == 0) {
            m_par.PHX1 = stod(sval);
            GetLog() << "PHX1 = " << m_par.PHX1 << "\n";
        }
        if (skey.compare("PHX2") == 0) {
            m_par.PHX2 = stod(sval);
            GetLog() << "PHX2 = " << m_par.PHX2 << "\n";
        }
        if (skey.compare("PVX1") == 0) {
            m_par.PVX1 = stod(sval);
            GetLog() << "PVX1 = " << m_par.PVX1 << "\n";
        }
        if (skey.compare("PVX2") == 0) {
            m_par.PVX2 = stod(sval);
            GetLog() << "PVX2 = " << m_par.PVX2 << "\n";
        }
        if (skey.compare("RBX1") == 0) {
            m_par.RBX1 = stod(sval);
            GetLog() << "RBX1 = " << m_par.RBX1 << "\n";
        }
        if (skey.compare("RBX2") == 0) {
            m_par.RBX2 = stod(sval);
            GetLog() << "RBX2 = " << m_par.RBX2 << "\n";
        }
        if (skey.compare("RCX1") == 0) {
            m_par.RCX1 = stod(sval);
            GetLog() << "RCX1 = " << m_par.RCX1 << "\n";
        }
        if (skey.compare("RHX1") == 0) {
            m_par.RHX1 = stod(sval);
            GetLog() << "RHX1 = " << m_par.RHX1 << "\n";
        }
        if (skey.compare("PTX1") == 0) {
            m_par.PTX1 = stod(sval);
            GetLog() << "PTX1 = " << m_par.PTX1 << "\n";
        }
        if (skey.compare("PTX2") == 0) {
            m_par.PTX2 = stod(sval);
            GetLog() << "PTX2 = " << m_par.PTX2 << "\n";
        }
        if (skey.compare("PTX3") == 0) {
            m_par.PTX3 = stod(sval);
            GetLog() << "PTX3 = " << m_par.PTX3 << "\n";
        }
    }
}

void ChMFTire::LoadSectionOverturning(FILE* fp) {
    bool ok = FindSectionStart("[OVERTURNING_COEFFICIENTS]", fp);
    if (!ok) {
        GetLog() << "Desired section [OVERTURNING_COEFFICIENTS] not found.\n";
        return;
    }
    while (!feof(fp)) {
        char line[201];
        fgets(line, 200, fp);  // buffer one line
        // remove leading white space
        size_t l = strlen(line);
        size_t ipos = 0;
        while (isblank(line[ipos])) {
            if (ipos < l)
                ipos++;
        }
        std::string sbuf(line + ipos);
        // skip pure comment lines
        if (sbuf.front() == '!' || sbuf.front() == '$')
            continue;
        // leave, since a new section is reached
        if (sbuf.front() == '[')
            break;
        // this should be a data line
        // there can be a trailing comment
        size_t trpos = sbuf.find_first_of("$");
        if (trpos != std::string::npos) {
            sbuf = sbuf.substr(0, trpos - 1);
        }
        // GetLog() << sbuf << "\n";
        // not all entries are of numerical type!
        size_t eqpos = sbuf.find_first_of("=");
        if (eqpos == std::string::npos)
            continue;
        std::string skey, sval;
        skey = sbuf.substr(0, eqpos);
        sval = sbuf.substr(eqpos + 1);
        size_t sppos = skey.find_first_of(" ");
        if (sppos != std::string::npos) {
            skey = skey.substr(0, sppos);
        }
        // GetLog() << ">>Key=" << skey << "|" << sval << "\n";
        if (skey.compare("QSX1") == 0) {
            m_par.QSX1 = stod(sval);
            GetLog() << "QSX1 = " << m_par.QSX1 << "\n";
        }
        if (skey.compare("QSX2") == 0) {
            m_par.QSX2 = stod(sval);
            GetLog() << "QSX2 = " << m_par.QSX2 << "\n";
        }
        if (skey.compare("QSX3") == 0) {
            m_par.QSX3 = stod(sval);
            GetLog() << "QSX3 = " << m_par.QSX3 << "\n";
        }
    }
}

void ChMFTire::LoadSectionLateral(FILE* fp) {
    bool ok = FindSectionStart("[LATERAL_COEFFICIENTS]", fp);
    if (!ok) {
        GetLog() << "Desired section [LATERAL_COEFFICIENTS] not found.\n";
        return;
    }
    while (!feof(fp)) {
        char line[201];
        fgets(line, 200, fp);  // buffer one line
        // remove leading white space
        size_t l = strlen(line);
        size_t ipos = 0;
        while (isblank(line[ipos])) {
            if (ipos < l)
                ipos++;
        }
        std::string sbuf(line + ipos);
        // skip pure comment lines
        if (sbuf.front() == '!' || sbuf.front() == '$')
            continue;
        // leave, since a new section is reached
        if (sbuf.front() == '[')
            break;
        // this should be a data line
        // there can be a trailing comment
        size_t trpos = sbuf.find_first_of("$");
        if (trpos != std::string::npos) {
            sbuf = sbuf.substr(0, trpos - 1);
        }
        // GetLog() << sbuf << "\n";
        // not all entries are of numerical type!
        size_t eqpos = sbuf.find_first_of("=");
        if (eqpos == std::string::npos)
            continue;
        std::string skey, sval;
        skey = sbuf.substr(0, eqpos);
        sval = sbuf.substr(eqpos + 1);
        size_t sppos = skey.find_first_of(" ");
        if (sppos != std::string::npos) {
            skey = skey.substr(0, sppos);
        }
        // GetLog() << ">>Key=" << skey << "|" << sval << "\n";
        if (skey.compare("PCY1") == 0) {
            m_par.PCY1 = stod(sval);
            GetLog() << "PCY1 = " << m_par.PCY1 << "\n";
        }
        if (skey.compare("PDY1") == 0) {
            m_par.PDY1 = stod(sval);
            GetLog() << "PDY1 = " << m_par.PDY1 << "\n";
        }
        if (skey.compare("PDY2") == 0) {
            m_par.PDY2 = stod(sval);
            GetLog() << "PDY2 = " << m_par.PDY2 << "\n";
        }
        if (skey.compare("PDY3") == 0) {
            m_par.PDY3 = stod(sval);
            GetLog() << "PDY3 = " << m_par.PDY3 << "\n";
        }
        if (skey.compare("PEY1") == 0) {
            m_par.PEY1 = stod(sval);
            GetLog() << "PEY1 = " << m_par.PEY1 << "\n";
        }
        if (skey.compare("PEY2") == 0) {
            m_par.PEY2 = stod(sval);
            GetLog() << "PEY2 = " << m_par.PEY2 << "\n";
        }
        if (skey.compare("PEY3") == 0) {
            m_par.PEY3 = stod(sval);
            GetLog() << "PEY3 = " << m_par.PEY3 << "\n";
        }
        if (skey.compare("PEY4") == 0) {
            m_par.PEY4 = stod(sval);
            GetLog() << "PEY4 = " << m_par.PEY4 << "\n";
        }
        if (skey.compare("PKY1") == 0) {
            m_par.PKY1 = stod(sval);
            GetLog() << "PKY1 = " << m_par.PKY1 << "\n";
        }
        if (skey.compare("PKY2") == 0) {
            m_par.PKY2 = stod(sval);
            GetLog() << "PKY2 = " << m_par.PKY2 << "\n";
        }
        if (skey.compare("PKY3") == 0) {
            m_par.PKY3 = stod(sval);
            GetLog() << "PKY3 = " << m_par.PKY3 << "\n";
        }
        if (skey.compare("PHY1") == 0) {
            m_par.PHY1 = stod(sval);
            GetLog() << "PHY1 = " << m_par.PHY1 << "\n";
        }
        if (skey.compare("PHY2") == 0) {
            m_par.PHY2 = stod(sval);
            GetLog() << "PHY2 = " << m_par.PHY2 << "\n";
        }
        if (skey.compare("PHY3") == 0) {
            m_par.PHY3 = stod(sval);
            GetLog() << "PHY3 = " << m_par.PHY3 << "\n";
        }
        if (skey.compare("PVY1") == 0) {
            m_par.PVY1 = stod(sval);
            GetLog() << "PVY1 = " << m_par.PVY1 << "\n";
        }
        if (skey.compare("PVY2") == 0) {
            m_par.PVY2 = stod(sval);
            GetLog() << "PVY2 = " << m_par.PVY2 << "\n";
        }
        if (skey.compare("PVY3") == 0) {
            m_par.PVY3 = stod(sval);
            GetLog() << "PVY3 = " << m_par.PVY3 << "\n";
        }
        if (skey.compare("PVY4") == 0) {
            m_par.PVY4 = stod(sval);
            GetLog() << "PVY4 = " << m_par.PVY4 << "\n";
        }
        if (skey.compare("RBY1") == 0) {
            m_par.RBY1 = stod(sval);
            GetLog() << "RBY1 = " << m_par.RBY1 << "\n";
        }
        if (skey.compare("RBY2") == 0) {
            m_par.RBY2 = stod(sval);
            GetLog() << "RBY2 = " << m_par.RBY2 << "\n";
        }
        if (skey.compare("RBY3") == 0) {
            m_par.RBY3 = stod(sval);
            GetLog() << "RBY3 = " << m_par.RBY3 << "\n";
        }
        if (skey.compare("RCY1") == 0) {
            m_par.RCY1 = stod(sval);
            GetLog() << "RCY1 = " << m_par.RCY1 << "\n";
        }
        if (skey.compare("RHY1") == 0) {
            m_par.RHY1 = stod(sval);
            GetLog() << "RHY1 = " << m_par.RHY1 << "\n";
        }
        if (skey.compare("RVY1") == 0) {
            m_par.RVY1 = stod(sval);
            GetLog() << "RVY1 = " << m_par.RVY1 << "\n";
        }
        if (skey.compare("RVY2") == 0) {
            m_par.RVY2 = stod(sval);
            GetLog() << "RVY2 = " << m_par.RVY2 << "\n";
        }
        if (skey.compare("RVY3") == 0) {
            m_par.RVY3 = stod(sval);
            GetLog() << "RVY3 = " << m_par.RVY3 << "\n";
        }
        if (skey.compare("RVY4") == 0) {
            m_par.RVY4 = stod(sval);
            GetLog() << "RVY4 = " << m_par.RVY4 << "\n";
        }
        if (skey.compare("RVY5") == 0) {
            m_par.RVY5 = stod(sval);
            GetLog() << "RVY5 = " << m_par.RVY5 << "\n";
        }
        if (skey.compare("RVY6") == 0) {
            m_par.RVY6 = stod(sval);
            GetLog() << "RVY6 = " << m_par.RVY6 << "\n";
        }
        if (skey.compare("PTY1") == 0) {
            m_par.PTY1 = stod(sval);
            GetLog() << "PTY1 = " << m_par.PTY1 << "\n";
        }
        if (skey.compare("PTY2") == 0) {
            m_par.PTY2 = stod(sval);
            GetLog() << "PTY2 = " << m_par.PTY2 << "\n";
        }
    }
}

void ChMFTire::LoadSectionRolling(FILE* fp) {
    bool ok = FindSectionStart("[ROLLING_COEFFICIENTS]", fp);
    if (!ok) {
        GetLog() << "Desired section [ROLLING_COEFFICIENTS] not found.\n";
        return;
    }
    while (!feof(fp)) {
        char line[201];
        fgets(line, 200, fp);  // buffer one line
        // remove leading white space
        size_t l = strlen(line);
        size_t ipos = 0;
        while (isblank(line[ipos])) {
            if (ipos < l)
                ipos++;
        }
        std::string sbuf(line + ipos);
        // skip pure comment lines
        if (sbuf.front() == '!' || sbuf.front() == '$')
            continue;
        // leave, since a new section is reached
        if (sbuf.front() == '[')
            break;
        // this should be a data line
        // there can be a trailing comment
        size_t trpos = sbuf.find_first_of("$");
        if (trpos != std::string::npos) {
            sbuf = sbuf.substr(0, trpos - 1);
        }
        // GetLog() << sbuf << "\n";
        // not all entries are of numerical type!
        size_t eqpos = sbuf.find_first_of("=");
        if (eqpos == std::string::npos)
            continue;
        std::string skey, sval;
        skey = sbuf.substr(0, eqpos);
        sval = sbuf.substr(eqpos + 1);
        size_t sppos = skey.find_first_of(" ");
        if (sppos != std::string::npos) {
            skey = skey.substr(0, sppos);
        }
        // GetLog() << ">>Key=" << skey << "|" << sval << "\n";
        if (skey.compare("QSY1") == 0) {
            m_par.QSY1 = stod(sval);
            GetLog() << "QSY1 = " << m_par.QSY1 << "\n";
        }
        if (skey.compare("QSY2") == 0) {
            m_par.QSY2 = stod(sval);
            GetLog() << "QSY2 = " << m_par.QSY2 << "\n";
        }
    }
}

void ChMFTire::LoadSectionAligning(FILE* fp) {
    bool ok = FindSectionStart("[ALIGNING_COEFFICIENTS]", fp);
    if (!ok) {
        GetLog() << "Desired section [ALIGNING_COEFFICIENTS] not found.\n";
        return;
    }
    while (!feof(fp)) {
        char line[201];
        fgets(line, 200, fp);  // buffer one line
        // remove leading white space
        size_t l = strlen(line);
        size_t ipos = 0;
        while (isblank(line[ipos])) {
            if (ipos < l)
                ipos++;
        }
        std::string sbuf(line + ipos);
        // skip pure comment lines
        if (sbuf.front() == '!' || sbuf.front() == '$')
            continue;
        // leave, since a new section is reached
        if (sbuf.front() == '[')
            break;
        // this should be a data line
        // there can be a trailing comment
        size_t trpos = sbuf.find_first_of("$");
        if (trpos != std::string::npos) {
            sbuf = sbuf.substr(0, trpos - 1);
        }
        // GetLog() << sbuf << "\n";
        // not all entries are of numerical type!
        size_t eqpos = sbuf.find_first_of("=");
        if (eqpos == std::string::npos)
            continue;
        std::string skey, sval;
        skey = sbuf.substr(0, eqpos);
        sval = sbuf.substr(eqpos + 1);
        size_t sppos = skey.find_first_of(" ");
        if (sppos != std::string::npos) {
            skey = skey.substr(0, sppos);
        }
        // GetLog() << ">>Key=" << skey << "|" << sval << "\n";
        if (skey.compare("QBZ1") == 0) {
            m_par.QBZ1 = stod(sval);
            GetLog() << "QBZ1 = " << m_par.QBZ1 << "\n";
        }
        if (skey.compare("QBZ2") == 0) {
            m_par.QBZ2 = stod(sval);
            GetLog() << "QBZ2 = " << m_par.QBZ2 << "\n";
        }
        if (skey.compare("QBZ3") == 0) {
            m_par.QBZ3 = stod(sval);
            GetLog() << "QBZ3 = " << m_par.QBZ3 << "\n";
        }
        if (skey.compare("QBZ4") == 0) {
            m_par.QBZ4 = stod(sval);
            GetLog() << "QBZ4 = " << m_par.QBZ4 << "\n";
        }
        if (skey.compare("QBZ5") == 0) {
            m_par.QBZ5 = stod(sval);
            GetLog() << "QBZ5 = " << m_par.QBZ5 << "\n";
        }
        if (skey.compare("QBZ9") == 0) {
            m_par.QBZ9 = stod(sval);
            GetLog() << "QBZ9 = " << m_par.QBZ9 << "\n";
        }
        if (skey.compare("QCZ1") == 0) {
            m_par.QCZ1 = stod(sval);
            GetLog() << "QCZ1 = " << m_par.QCZ1 << "\n";
        }
        if (skey.compare("QDZ1") == 0) {
            m_par.QDZ1 = stod(sval);
            GetLog() << "QDZ1 = " << m_par.QDZ1 << "\n";
        }
        if (skey.compare("QDZ2") == 0) {
            m_par.QDZ2 = stod(sval);
            GetLog() << "QDZ2 = " << m_par.QDZ2 << "\n";
        }
        if (skey.compare("QDZ3") == 0) {
            m_par.QDZ3 = stod(sval);
            GetLog() << "QDZ3 = " << m_par.QDZ3 << "\n";
        }
        if (skey.compare("QDZ4") == 0) {
            m_par.QDZ4 = stod(sval);
            GetLog() << "QDZ4 = " << m_par.QDZ4 << "\n";
        }
        if (skey.compare("QDZ6") == 0) {
            m_par.QDZ6 = stod(sval);
            GetLog() << "QDZ6 = " << m_par.QDZ6 << "\n";
        }
        if (skey.compare("QDZ7") == 0) {
            m_par.QDZ7 = stod(sval);
            GetLog() << "QDZ7 = " << m_par.QDZ7 << "\n";
        }
        if (skey.compare("QDZ8") == 0) {
            m_par.QDZ8 = stod(sval);
            GetLog() << "QDZ8 = " << m_par.QDZ8 << "\n";
        }
        if (skey.compare("QDZ9") == 0) {
            m_par.QDZ9 = stod(sval);
            GetLog() << "QDZ9 = " << m_par.QDZ9 << "\n";
        }
        if (skey.compare("QEZ1") == 0) {
            m_par.QEZ1 = stod(sval);
            GetLog() << "QEZ1 = " << m_par.QEZ1 << "\n";
        }
        if (skey.compare("QEZ2") == 0) {
            m_par.QEZ2 = stod(sval);
            GetLog() << "QEZ2 = " << m_par.QEZ2 << "\n";
        }
        if (skey.compare("QEZ3") == 0) {
            m_par.QEZ3 = stod(sval);
            GetLog() << "QEZ3 = " << m_par.QEZ3 << "\n";
        }
        if (skey.compare("QEZ4") == 0) {
            m_par.QEZ4 = stod(sval);
            GetLog() << "QEZ4 = " << m_par.QEZ4 << "\n";
        }
        if (skey.compare("QEZ5") == 0) {
            m_par.QEZ5 = stod(sval);
            GetLog() << "QEZ5 = " << m_par.QEZ5 << "\n";
        }
        if (skey.compare("QHZ1") == 0) {
            m_par.QHZ1 = stod(sval);
            GetLog() << "QHZ1 = " << m_par.QHZ1 << "\n";
        }
        if (skey.compare("QHZ2") == 0) {
            m_par.QHZ2 = stod(sval);
            GetLog() << "QHZ2 = " << m_par.QHZ2 << "\n";
        }
        if (skey.compare("QHZ3") == 0) {
            m_par.QHZ3 = stod(sval);
            GetLog() << "QHZ3 = " << m_par.QHZ3 << "\n";
        }
        if (skey.compare("QHZ4") == 0) {
            m_par.QHZ4 = stod(sval);
            GetLog() << "QHZ4 = " << m_par.QHZ4 << "\n";
        }
        if (skey.compare("SSZ1") == 0) {
            m_par.SSZ1 = stod(sval);
            GetLog() << "SSZ1 = " << m_par.SSZ1 << "\n";
        }
        if (skey.compare("SSZ2") == 0) {
            m_par.SSZ2 = stod(sval);
            GetLog() << "SSZ2 = " << m_par.SSZ2 << "\n";
        }
        if (skey.compare("SSZ3") == 0) {
            m_par.SSZ3 = stod(sval);
            GetLog() << "SSZ3 = " << m_par.SSZ3 << "\n";
        }
        if (skey.compare("SSZ4") == 0) {
            m_par.SSZ4 = stod(sval);
            GetLog() << "SSZ4 = " << m_par.SSZ4 << "\n";
        }
        if (skey.compare("QTZ1") == 0) {
            m_par.QTZ1 = stod(sval);
            GetLog() << "QTZ1 = " << m_par.QTZ1 << "\n";
        }
        if (skey.compare("MBELT") == 0) {
            m_par.MBELT = stod(sval);
            GetLog() << "MBELT = " << m_par.MBELT << "\n";
        }
    }
}

void ChMFTire::Initialize(std::shared_ptr<ChWheel> wheel) {
    ChTire::Initialize(wheel);

    SetMFParams();
    // Build the lookup table for penetration depth as function of intersection area
    // (used only with the ChTire::ENVELOPE method for terrain-tire collision detection)
    ConstructAreaDepthTable(m_par.UNLOADED_RADIUS, m_areaDep);

    // all parameters are known now pepare mirroring
    if (m_allow_mirroring) {
        if (wheel->GetSide() != m_measured_side) {
            // we flip the sign of some parameters to compensate asymmetry
            m_par.RHX1 *= -1.0;
            m_par.QSX1 *= -1.0;
            m_par.PEY3 *= -1.0;
            m_par.PHY1 *= -1.0;
            m_par.PHY2 *= -1.0;
            m_par.PVY1 *= -1.0;
            m_par.PVY2 *= -1.0;
            m_par.RBY3 *= -1.0;
            m_par.RVY1 *= -1.0;
            m_par.RVY2 *= -1.0;
            m_par.QBZ4 *= -1.0;
            m_par.QDZ3 *= -1.0;
            m_par.QDZ6 *= -1.0;
            m_par.QDZ7 *= -1.0;
            m_par.QEZ4 *= -1.0;
            m_par.QHZ1 *= -1.0;
            m_par.QHZ2 *= -1.0;
            m_par.SSZ1 *= -1.0;
            if (m_measured_side == LEFT) {
                GetLog() << "Tire is measured as left tire but mounted on the right vehicle side -> mirroring.\n";
            } else {
                GetLog() << "Tire is measured as right tire but mounted on the lleft vehicle side -> mirroring.\n";
            }
        }
    }

    // Initialize contact patch state variables to 0
    m_data.normal_force = 0;
    m_states.R_eff = m_par.UNLOADED_RADIUS;
    m_states.kappa = 0;
    m_states.alpha = 0;
    m_states.gamma = 0;
    m_states.vx = 0;
    m_states.vsx = 0;
    m_states.vsy = 0;
    m_states.omega = 0;
    m_states.disc_normal = ChVector<>(0, 0, 0);
}

void ChMFTire::Synchronize(double time, const ChTerrain& terrain) {
    WheelState wheel_state = m_wheel->GetState();

    // Extract the wheel normal (expressed in global frame)
    ChMatrix33<> A(wheel_state.rot);
    ChVector<> disc_normal = A.Get_A_Yaxis();

    // Assuming the tire is a disc, check contact with terrain
    float mu;
    m_data.in_contact =
        DiscTerrainCollision(m_collision_type, terrain, wheel_state.pos, disc_normal, m_par.UNLOADED_RADIUS,
                             m_par.WIDTH, m_areaDep, m_data.frame, m_data.depth, mu);
    ChClampValue(mu, 0.1f, 1.0f);
    m_mu_road = mu;

    // Calculate tire kinematics
    CalculateKinematics(wheel_state, m_data.frame);

    if (m_data.in_contact) {
        // Wheel velocity in the ISO-C Frame
        ChVector<> vel = wheel_state.lin_vel;
        m_data.vel = m_data.frame.TransformDirectionParentToLocal(vel);

        // Generate normal contact force (recall, all forces are reduced to the wheel
        // center). If the resulting force is negative, the disc is moving away from
        // the terrain so fast that no contact force is generated.
        // The sign of the velocity term in the damping function is negative since
        // a positive velocity means a decreasing depth, not an increasing depth
        double Fn_mag = GetNormalStiffnessForce(m_data.depth) + GetNormalDampingForce(m_data.depth, -m_data.vel.z());

        if (Fn_mag < 0) {
            Fn_mag = 0;
            m_data.in_contact = false;  // Skip Force and moment calculations when the normal force = 0
        }

        m_data.normal_force = Fn_mag;
        m_states.gamma = CH_C_PI_2 - std::acos(m_states.disc_normal.z());
        // R_eff is a Rill estimation, not Pacejka. Advantage: it works well with speed = zero.
        m_states.R_eff = (2.0 * m_par.UNLOADED_RADIUS + (m_par.UNLOADED_RADIUS - m_data.depth)) / 3.0;
        m_states.vx = std::abs(m_data.vel.x());
        m_states.vsx = m_data.vel.x() - wheel_state.omega * m_states.R_eff;
        m_states.vsy = -m_data.vel.y();
        // prevent singularity for kappa, when vx == 0
        const double epsilon = 0.1;
        m_states.kappa = -m_states.vsx / (m_states.vx + epsilon);
        m_states.alpha = std::atan2(m_states.vsy, m_states.vx + epsilon);
        m_states.gamma = CH_C_PI_2 - std::acos(m_states.disc_normal.z());
        m_states.omega = wheel_state.omega;
        m_states.disc_normal = disc_normal;
        // Ensure that kappa stays between -1 & 1
        ChClampValue(m_states.kappa, -1.0, 1.0);
        // Ensure that alpha stays between -pi()/2 & pi()/2 (a little less to prevent tan from going to infinity)
        ChClampValue(m_states.alpha, -CH_C_PI_2 + 0.001, CH_C_PI_2 - 0.001);
        // Clamp |gamma| to specified value: Limit due to tire testing, avoids erratic extrapolation. m_gamma_limit is
        // in rad too.
        ChClampValue(m_states.gamma, -m_gamma_limit, m_gamma_limit);
    } else {
        // Reset all states if the tire comes off the ground.
        m_data.normal_force = 0;
        m_states.R_eff = m_par.UNLOADED_RADIUS;
        m_states.kappa = 0;
        m_states.alpha = 0;
        m_states.gamma = 0;
        m_states.vx = 0;
        m_states.vsx = 0;
        m_states.vsy = 0;
        m_states.omega = 0;
        m_states.disc_normal = ChVector<>(0, 0, 0);
    }
}

void ChMFTire::Advance(double step) {
    // Set tire forces to zero.
    m_tireforce.force = ChVector<>(0, 0, 0);
    m_tireforce.moment = ChVector<>(0, 0, 0);

    // Return now if no contact.
    if (!m_data.in_contact)
        return;

    // Calculate the new force and moment values (normal force and moment have already been accounted for in
    // Synchronize()).
    // See reference for details on the calculations.
    double Fx = 0;
    double Fy = 0;
    double Fz = m_data.normal_force;
    double Mx = 0;
    double My = 0;
    double Mz = 0;

    switch (m_use_mode) {
        case 0:
            // vertical spring & damper mode
            break;
        case 1:
            // steady state pure longitudinal slip
            break;
        case 2:
            // steady state pure lateral slip
            break;
        case 3:
            // steady state pure lateral slip uncombined
            break;
        case 4:
            // steady state combined slip
            break;
    }

    // Compile the force and moment vectors so that they can be
    // transformed into the global coordinate system.
    // Convert from SAE to ISO Coordinates at the contact patch.
    m_tireforce.force = ChVector<>(Fx, -Fy, m_data.normal_force);
    m_tireforce.moment = ChVector<>(Mx, -My, -Mz);
}

// -----------------------------------------------------------------------------

void ChMFTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    m_cyl_shape =
        ChVehicleGeometry::AddVisualizationCylinder(m_wheel->GetSpindle(),                                        //
                                                    ChVector<>(0, GetOffset() + GetVisualizationWidth() / 2, 0),  //
                                                    ChVector<>(0, GetOffset() - GetVisualizationWidth() / 2, 0),  //
                                                    GetRadius());
    m_cyl_shape->SetTexture(GetChronoDataFile("textures/greenwhite.png"));
}

void ChMFTire::RemoveVisualizationAssets() {
    // Make sure we only remove the assets added by ChMFTire::AddVisualizationAssets.
    // This is important for the ChTire object because a wheel may add its own assets to the same body (the
    // spindle/wheel).
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_cyl_shape);
}

// -----------------------------------------------------------------------------

}  // end namespace vehicle
}  // namespace chrono
