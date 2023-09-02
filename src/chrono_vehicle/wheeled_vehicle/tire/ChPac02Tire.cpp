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
// Authors: Rainer Gericke
// =============================================================================
//
// Template for a Magic Formula tire model
//
// ChPac02 is based on the Pacejka 2002 formulae as written in
// Hans B. Pacejka's "Tire and Vehicle Dynamics" Third Edition, Elsevier 2012
// ISBN: 978-0-08-097016-5
//
// This implementation is a small subset of the commercial product MFtire:
//  - only steady state force/torque calculations
//  - uncombined (use_mode = 3)
//  - combined (use_mode = 4) via Friction Ellipsis (default) or Pacejka method
//  - parametration is given by a TIR file (Tiem Orbit Format,
//    ADAMS/Car compatible)
//  - unit conversion is implemented but only tested for SI units
//  - optional inflation pressure dependency is implemented, but not tested
//  - this implementation could be validated for the FED-Alpha vehicle and rsp.
//    tire data sets against KRC test results from a Nato CDT
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

#include "chrono_vehicle/wheeled_vehicle/tire/ChPac02Tire.h"

namespace chrono {
namespace vehicle {

ChPac02Tire::ChPac02Tire(const std::string& name)
    : ChForceElementTire(name),
      m_gamma_limit(3.0 * CH_C_DEG_TO_RAD),
      m_mu0(0.8),
      m_measured_side(LEFT),
      m_allow_mirroring(false),
      m_use_mode(0),
      m_vcoulomb(1.0),
      m_frblend_begin(1.0),
      m_frblend_end(3.0) {
    m_tireforce.force = ChVector<>(0, 0, 0);
    m_tireforce.point = ChVector<>(0, 0, 0);
    m_tireforce.moment = ChVector<>(0, 0, 0);
}

double ChPac02Tire::GetNormalStiffnessForce(double depth) const {
    double R0 = m_par.UNLOADED_RADIUS;
    double gamma = m_states.gamma;
    double dpi = m_states.dpi;
    double Fc = (1.0) *
                (m_par.QFZ1 * depth / R0 + m_par.QFZ2 * pow(depth / R0, 2) + m_par.QFZ3 * pow(gamma, 2) * depth / R0) *
                (1.0 + m_par.QPFZ1 * dpi) * m_par.LCZ * m_par.FNOMIN;
    double Fb = 0;
    if (m_bottoming_table_found)
        Fb = m_bott_map.Get_y(depth);
    return Fc + Fb;
}

double ChPac02Tire::GetNormalDampingForce(double depth, double velocity) const {
    double Fd = m_par.VERTICAL_DAMPING * velocity;
    return Fd;
}

void ChPac02Tire::CombinedCoulombForces(double& fx, double& fy, double fz) {
    ChVector2<> F;
    F.x() = tanh(-2.0 * m_states.vsx / m_vcoulomb) * fz * m_states.mu_scale;
    F.y() = tanh(-2.0 * m_states.vsy / m_vcoulomb) * fz * m_states.mu_scale;
    if (F.Length() > fz * m_states.mu_scale) {
        F.Normalize();
        F *= fz * m_states.mu_scale;
    }
    fx = F.x();
    fy = F.y();
}

void ChPac02Tire::CalcFxyMz(double& Fx, double& Fy, double& Mz, double kappa, double alpha, double Fz, double gamma) {
    // steady state calculation
    double Fx0 = 0;  // longitudinal steady stae force
    double Cx = m_par.PCX1 * m_par.LCX;
    double Shx = (m_par.PHX1 + m_par.PHX2 * m_states.dfz0) * m_par.LHX;
    double Svx = Fz * (m_par.PVX1 + m_par.PVX2 * m_states.dfz0) * m_par.LVX * m_par.LMUX;
    double kappa_x = kappa + Shx;
    double gamma_x = gamma * m_par.LGAX;
    double Ex = (m_par.PEX1 + m_par.PEX2 * m_states.dfz0 + m_par.PEX3 * pow(m_states.dfz0, 2)) *
                (1.0 - m_par.PEX4 * ChSignum(kappa_x)) * m_par.LEX;
    if (Ex > 1.0)
        Ex = 1.0;
    double mu_x = std::abs(m_states.mu_scale * (m_par.PDX1 + m_par.PDX2 * m_states.dfz0) *
                           (1.0 + m_par.PPX3 * m_states.dpi + m_par.PPX4 * pow(m_states.dpi, 2)) *
                           (1.0 - m_par.PDX3 * pow(gamma_x, 2)) * m_par.LMUX);
    double Dx = mu_x * Fz;
    double Kx = Fz * (m_par.PKX1 + m_par.PKX2 * m_states.dfz0) * exp(m_par.PKX3 * m_states.dfz0) *
                (1.0 + m_par.PPX1 * m_states.dpi + m_par.PPX2 * pow(m_states.dpi, 2)) * m_par.LKX;
    double Bx = Kx / (Cx * Dx + 0.1);
    double X1 = Bx * kappa_x;
    ChClampValue(X1, -CH_C_PI_2 + 0.01, CH_C_PI_2 - 0.01);
    // Fx0 = Dx * sin(Cx * atan(Bx * kappa_x - Ex * (Bx * kappa_x - atan(Bx * kappa_x)))) + Svx;
    Fx0 = Dx * sin(Cx * atan(X1 - Ex * (X1 - atan(X1)))) + Svx;

    double Fy0 = 0;  // lateral steady state force
    double gamma_y = gamma * m_par.LGAY;
    double Cy = m_par.PCY1 * m_par.LCY;
    double Ky0 = m_par.PKY1 * m_par.FNOMIN * (1 + m_par.PPY1 * m_states.dpi) *
                 sin(2.0 * atan(Fz / (m_par.PKY2 * m_states.Fz0_prime * (1.0 + m_par.PPY2 * m_states.dpi)))) *
                 m_par.LFZO * m_par.LMUY;
    double Ky = Ky0 * (1.0 - m_par.PKY3 * fabs(gamma_y));
    double Shy = (m_par.PHY1 + m_par.PHY2 * m_states.dfz0) * m_par.LHY + m_par.PHY3 * gamma_y * m_par.LKYG;
    double alpha_y = alpha + Shy;
    double Svy = Fz *
                 ((m_par.PVY1 + m_par.PVY2 * m_states.dfz0) * m_par.LVY +
                  (m_par.PVY3 + m_par.PVY4 * m_states.dfz0) * gamma_y * m_par.LKYG) *
                 m_par.LMUY;
    double Ey = (m_par.PEY1 + m_par.PEY2 * m_states.dfz0) *
                (1.0 - (m_par.PEY3 + m_par.PEY4 * gamma_y) * ChSignum(alpha_y)) * m_par.LEY;
    if (Ey > 1.0)
        Ey = 1.0;
    double mu_y = std::abs(m_states.mu_scale * (m_par.PDY1 + m_par.PDY2 * m_states.dfz0) *
                           (1.0 + m_par.PPY3 * m_states.dpi + m_par.PPY4 * pow(m_states.dpi, 2)) *
                           (1.0 + m_par.PDY3 * pow(gamma_y, 2)) * m_par.LMUY);
    double Dy = mu_y * Fz;
    double By = Ky / (Cy * Dy + 0.1);
    double Y1 = By * alpha_y;
    ChClampValue(Y1, -CH_C_PI_2 + 0.01, CH_C_PI_2 - 0.01);
    // Fy0 = Dy * sin(Cy * atan(By * alpha_y - Ey * (By * alpha_y - atan(By * alpha_y)))) + Svy;
    Fy0 = Dy * sin(Cy * atan(Y1 - Ey * (Y1 - atan(Y1)))) + Svy;

    // not Pacejka: grip saturation longitudinal ------------------------
    m_states.grip_sat_x = std::abs((Fx0 - Svx) / Fz) / std::abs(Dx / Fz);
    ChClampValue(m_states.grip_sat_x, 0.0, 1.0);
    //-------------------------------------------------------------------
    // not Pacejka: grip saturation lateral -----------------------------
    m_states.grip_sat_y = std::abs((Fy0 - Svy) / Fz) / std::abs(Dy / Fz);
    ChClampValue(m_states.grip_sat_y, 0.0, 1.0);
    //-------------------------------------------------------------------

    double R0 = m_par.UNLOADED_RADIUS;
    // steady state alignment torque / pneumatic trail
    double gamma_z = gamma * m_par.LGAZ;
    double Sht = m_par.QHZ1 + m_par.QHZ2 * m_states.dfz0 + (m_par.QHZ3 + m_par.QHZ4 * m_states.dfz0) * gamma_z;
    double Shf = Shy + Svy / Ky;
    double alpha_r = alpha + Shf;
    double alpha_t = alpha + Sht;
    double Ct = m_par.QCZ1;
    double Bt = std::abs((m_par.QBZ1 + m_par.QBZ2 * m_states.dfz0 + m_par.QBZ3 * pow(m_states.dfz0, 2)) *
                         (1.0 + m_par.QBZ4 * gamma_z + m_par.QBZ5 * std::abs(gamma_z)) * m_par.LKY / m_par.LMUY);
    double Et = (m_par.QEZ1 + m_par.QEZ2 * m_states.dfz0 + m_par.QEZ3 * pow(m_states.dfz0, 2)) *
                (1.0 + (m_par.QEZ4 + m_par.QEZ5 * gamma_z) * ((2.0 / CH_C_PI) * atan(Bt * Ct * alpha_t)));
    if (Et > 1.0)
        Et = 1.0;
    double Dt = Fz * (m_par.QDZ1 + m_par.QDZ2 * m_states.dfz0) * (1.0 - m_par.QPZ1 * m_states.dpi) *
                (1.0 + m_par.QDZ3 * gamma_z + m_par.QDZ4 * pow(gamma_z, 2)) * R0 / m_states.Fz0_prime * m_par.LTR;
    // trail, uncombined forces
    double t = Dt * (cos(Ct * atan(Bt * alpha_t - Et * (Bt * alpha_t - atan(Bt * alpha_t))))) * cos(alpha);
    // residual moment
    double Br = (m_par.QBZ9 * m_par.LKY / m_par.LMUY + m_par.QBZ10 * By * Cy);
    double Dr = Fz *
                ((m_par.QDZ6 + m_par.QDZ7 * m_states.dfz0) * m_par.LRES +
                 (m_par.QDZ8 + m_par.QDZ9 * m_states.dfz0) * (1.0 + m_par.QPZ2 * m_states.dpi) * gamma_z) *
                R0 * m_par.LMUY;
    // residual moment, uncombined forces
    const double Cr = 1.0;
    double Mzr = Dr * cos(Cr * atan(Br * alpha_r)) * cos(alpha);

    switch (m_use_mode) {
        case 1:
            // Fx only
            Fx = Fx0;
            Fy = 0;
            Mz = 0;
            break;
        case 2: {
            // Fy and Mz only
            Fx = 0;
            Fy = Fy0;
            Mz = -t * Fy0 + Mzr;
        } break;
        case 3: {
            // uncombined Fx, Fy, Mz calculation
            Fx = Fx0;
            Fy = Fy0;
            Mz = -t * Fy0 + Mzr;
        } break;
        case 4: {
            // combined Fx, Fy, Mz calculation
            if (m_use_friction_ellipsis) {
                // combining without rsp. Pacejka coefficients, ADAMs
                double kappa_c = kappa + Shx + Svx / Kx;
                double alpha_c = alpha + Shy + Svy / Ky;
                double alpha_s = sin(alpha_c);
                double beta = acos(std::abs(kappa_c) / hypot(kappa_c, alpha_s));
                double mu_x_act = std::abs((Fx0 - Svx) / Fz);
                double mu_y_act = std::abs((Fy0 - Svy) / Fz);
                double mu_x_max = Dx / Fz;
                double mu_y_max = Dy / Fz;
                double mu_x_c = 1.0 / hypot(1.0 / mu_x_act, tan(beta) / mu_y_max);
                double mu_y_c = tan(beta) / hypot(1.0 / mu_x_max, tan(beta) / mu_y_act);
                Fx = Fx0 * mu_x_c / mu_x_act;
                Fy = Fy0 * mu_y_c / mu_y_act;
                Mz = -t * Fy + Mzr;
            } else {
                // use rsp. Pacejka coefficients for combining
                double Shxa = m_par.RHX1;
                double Cxa = m_par.RCX1;
                double alpha_s = alpha + Shxa;
                double Exa = m_par.REX1 + m_par.REX2 * m_states.dfz0;
                if (Exa > 1.0)
                    Exa = 1.0;
                double Bxa = std::abs(m_par.RBX1 * cos(atan(m_par.RBX2 * kappa)) * m_par.LXAL);
                ////double Dxa = Fx0 / cos(Cxa * atan(Bxa * Shxa - Exa * (Bxa * Shxa - atan(Bxa * Shxa))));
                double Gxa = cos(Cxa * atan(Bxa * alpha_s - Exa * (Bxa * alpha_s - atan(Bxa * alpha_s)))) /
                             cos(Cxa * atan(Bxa * Shxa - Exa * (Bxa * Shxa - atan(Bxa * Shxa))));
                Fx = Fx0 * Gxa;

                double Shyk = m_par.RHY1 + m_par.RHY2 * m_states.dfz0;
                double kappa_s = kappa + Shyk;
                double Cyk = m_par.RCY1;
                double Eyk = m_par.REY1 + m_par.REY2 * m_states.dfz0;
                if (Eyk > 1.0)
                    Eyk = 1.0;
                double Byk = m_par.RBY1 * cos(atan(m_par.RBY2 * (alpha - m_par.RBY3))) * m_par.LYKA;
                double Dvyk = mu_y * Fz * (m_par.RVY1 + m_par.RVY2 * m_states.dfz0 + m_par.RVY3 * gamma) *
                              cos(atan(m_par.RVY4 * alpha));
                double Svyk = Dvyk * sin(m_par.RVY5 * atan(m_par.RVY6 * kappa)) * m_par.LVYKA;
                double Gyk = cos(Cyk * atan(Byk * kappa_s - Eyk * (Byk * kappa_s - atan(Byk * kappa_s)))) /
                             cos(Cyk * atan(Byk * Shyk - Eyk * (Byk * Shyk - atan(Byk * Shyk))));
                Fy = Fy0 * Gyk + Svyk;

                double alpha_teq = atan(sqrt(pow(tan(alpha_t), 2) + pow(Kx / Ky, 2) * pow(kappa, 2))) * ChSignum(kappa);
                double alpha_req =
                    atan(sqrt(pow(tan(alpha_r), 2) + pow(Kx / Ky, 2) * pow(kappa, 2))) * ChSignum(alpha_r);
                // trail combined forces
                double tc =
                    Dt * (cos(Ct * atan(Bt * alpha_teq - Et * (Bt * alpha_teq - atan(Bt * alpha_teq))))) * cos(alpha);
                // residual moment
                double s = (m_par.SSZ1 + m_par.SSZ2 * Fy / m_states.Fz0_prime +
                            (m_par.SSZ3 + m_par.SSZ4 * m_states.dfz0) * gamma) *
                           R0 * m_par.LS;
                // residual moment, combined forces
                double Mzrc = Dr * cos(Cr * atan(Br * alpha_req)) * cos(alpha);
                double Fy_prime = Fy - Svyk;
                Mz = -tc * Fy_prime + Mzrc + s * Fx;
            }
        } break;
    }
}

double ChPac02Tire::CalcSigmaK(double Fz) {
    double R0 = m_par.UNLOADED_RADIUS;
    return Fz * (m_par.PTX1 + m_par.PTX2 * m_states.dfz0) * exp(m_par.PTX3 * m_states.dfz0) *
           (R0 / m_states.Fz0_prime) * m_par.LSGKP;
}

double ChPac02Tire::CalcSigmaA(double Fz) {
    double R0 = m_par.UNLOADED_RADIUS;
    return m_par.PTY1 * sin(2.0 * atan(Fz / (m_par.PTY2 * m_par.FNOMIN * m_par.LFZO))) *
           (1.0 - m_par.PKY3 * fabs(m_states.gamma)) * (R0 * m_par.LFZO) * m_par.LSGAL;
}

double ChPac02Tire::CalcMx(double Fy, double Fz, double gamma) {
    double Mx =
        m_par.UNLOADED_RADIUS * Fz *
        (m_par.QSX3 * Fy / m_states.Fz0_prime +
         m_par.QSX4 * cos(m_par.QSX5 * atan(pow(Fz / m_states.Fz0_prime, 2))) *
             sin(m_par.QSX7 * gamma + m_par.QSX8 * atan(m_par.QSX9 * Fy / m_states.Fz0_prime)) +
         (m_par.QSX10 * atan(m_par.QSX11 * Fz / m_states.Fz0_prime) - m_par.QSX2 * (1.0 + m_par.QPX1 * m_states.dpi)) *
             gamma +
         m_par.QSX1 * m_par.LVMX) *
        m_par.LMX;
    return Mx;
}

double ChPac02Tire::CalcMy(double Fx, double Fz, double gamma) {
    // in most cases only QSY1 is used.
    double V0 = sqrt(m_g * m_par.UNLOADED_RADIUS);
    double My = Fz * m_par.UNLOADED_RADIUS *
                (m_par.QSY1 + m_par.QSY2 * Fx / m_par.FNOMIN + m_par.QSY3 * fabs(m_states.vx / V0) +
                 m_par.QSY4 * pow(m_states.vx / V0, 4) + m_par.QSY5 * pow(gamma, 2) +
                 m_par.QSY6 * pow(gamma, 2) * Fz / m_par.FNOMIN) *
                (pow(Fz / m_par.FNOMIN, m_par.QSY7) * pow(m_par.IP / m_par.IP_NOM, m_par.QSY8)) * m_par.LMY;
    return My;
}

// -----------------------------------------------------------------------------

void ChPac02Tire::SetMFParamsByFile(const std::string& tirFileName) {
    FILE* fp = fopen(tirFileName.c_str(), "r+");
    if (fp == NULL) {
        GetLog() << "TIR File not found <" << tirFileName << ">!\n";
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
    LoadSectionConditions(fp);
    LoadVerticalTable(fp);
    LoadBottomingTable(fp);

    if (!m_vertical_table_found) {
        // set linear stiffness funct parameters
        m_par.QFZ1 = m_par.VERTICAL_STIFFNESS * m_par.UNLOADED_RADIUS / m_par.FNOMIN;
        m_par.QFZ2 = 0;
    }

    if (!m_tire_conditions_found) {
        // set all pressure dependence parameters to zero
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

    fclose(fp);
}

bool ChPac02Tire::FindSectionStart(const std::string& sectName, FILE* fp) {
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

void ChPac02Tire::LoadSectionUnits(FILE* fp) {
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
        } else if (skey.compare("PRESSURE") == 0) {
            if (sval.compare("PASCAL") == 0 || sval.compare("PA") == 0) {
                m_par.u_pressure = 1.0;
            } else if (sval.compare("KPASCAL") == 0 || sval.compare("KPA") == 0) {
                m_par.u_pressure = 1000.0;
            } else if (sval.compare("BAR") == 0) {
                m_par.u_pressure = 1.0e5;
            } else if (sval.compare("PSI") == 0) {
                m_par.u_pressure = 6894.7572932;
            } else if (sval.compare("KSI") == 0) {
                m_par.u_pressure = 6894757.2932;
            }
        }
    }
    m_par.u_speed = m_par.u_length / m_par.u_time;
    m_par.u_inertia = m_par.u_mass * m_par.u_length * m_par.u_length;
    m_par.u_stiffness = m_par.u_force / m_par.u_length;
    m_par.u_damping = m_par.u_force / m_par.u_speed;
}

void ChPac02Tire::LoadSectionModel(FILE* fp) {
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
            if (sval.compare("'LEFT'") == 0 || sval.compare("'UNKNOWN'") == 0) {
                m_measured_side = LEFT;
            } else {
                m_measured_side = RIGHT;
            }
        }
        if (skey.compare("FE_METHOD") == 0) {
            size_t a1pos = sval.find_first_of("'");
            size_t a2pos = sval.find_last_of("'");
            sval = sval.substr(a1pos, a2pos - a1pos + 1);
            // GetLog() << ">>Key=" << skey << "|" << sval << "|\n";
            if (sval.compare("'NO'") == 0 || sval.compare("'no'") == 0) {
                m_use_friction_ellipsis = false;
                GetLog() << "Friction Ellipsis Method switched off, relying on Pac02 parameters!\n";
            }
        }
        if (skey.compare("USE_MODE") == 0) {
            m_par.USE_MODE = stoi(sval);
            switch (m_par.USE_MODE) {
                default:
                case 0:
                    m_use_mode = m_par.USE_MODE;
                    GetLog() << "Only Vertical Force Fz will be calculated!\n";
                    break;
                case 1:
                    m_use_mode = m_par.USE_MODE;
                    GetLog() << "Only Forces Fx and Fz will be calculated!\n";
                    break;
                case 2:
                    m_use_mode = m_par.USE_MODE;
                    GetLog() << "Only Forces Fy and Fz will be calculated!\n";
                    break;
                case 3:
                    m_use_mode = m_par.USE_MODE;
                    GetLog() << "Uncombined Force calculation!\n";
                    break;
                case 4:
                    m_use_mode = m_par.USE_MODE;
                    GetLog() << "Combined Force calculation!\n";
                    break;
            }
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

void ChPac02Tire::LoadSectionDimension(FILE* fp) {
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
        }
        if (skey.compare("WIDTH") == 0) {
            m_par.WIDTH = m_par.u_length * stod(sval);
        }
        if (skey.compare("ASPECT_RATIO") == 0) {
            m_par.ASPECT_RATIO = stod(sval);
        }
        if (skey.compare("RIM_RADIUS") == 0) {
            m_par.RIM_RADIUS = m_par.u_length * stod(sval);
        }
        if (skey.compare("RIM_WIDTH") == 0) {
            m_par.RIM_WIDTH = m_par.u_length * stod(sval);
        }
    }
}

void ChPac02Tire::LoadSectionVertical(FILE* fp) {
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
        }
        if (skey.compare("VERTICAL_DAMPING") == 0) {
            m_par.VERTICAL_DAMPING = m_par.u_damping * stod(sval);
        }
        if (skey.compare("BREFF") == 0) {
            m_par.BREFF = stod(sval);
        }
        if (skey.compare("DREFF") == 0) {
            m_par.DREFF = stod(sval);
        }
        if (skey.compare("FREFF") == 0) {
            m_par.FREFF = stod(sval);
        }
        if (skey.compare("FNOMIN") == 0) {
            m_par.FNOMIN = m_par.u_force * stod(sval);
        }
        if (skey.compare("TIRE_MASS") == 0) {
            m_par.TIRE_MASS = m_par.u_mass * stod(sval);
        }
        if (skey.compare("QFZ1") == 0) {
            m_par.QFZ1 = stod(sval);
        }
        if (skey.compare("QFZ2") == 0) {
            m_par.QFZ2 = stod(sval);
        }
        if (skey.compare("QFZ3") == 0) {
            m_par.QFZ3 = stod(sval);
        }
        if (skey.compare("QPFZ1") == 0) {
            m_par.QPFZ1 = stod(sval);
        }
        if (skey.compare("QV2") == 0) {
            m_par.QV2 = stod(sval);
        }
    }
}

void ChPac02Tire::LoadSectionScaling(FILE* fp) {
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
        }
        if (skey.compare("LCX") == 0) {
            m_par.LCX = stod(sval);
        }
        if (skey.compare("LMUX") == 0) {
            m_par.LMUX = stod(sval);
        }
        if (skey.compare("LEX") == 0) {
            m_par.LEX = stod(sval);
        }
        if (skey.compare("LKX") == 0) {
            m_par.LKX = stod(sval);
        }
        if (skey.compare("LHX") == 0) {
            m_par.LHX = stod(sval);
        }
        if (skey.compare("LVX") == 0) {
            m_par.LVX = stod(sval);
        }
        if (skey.compare("LCY") == 0) {
            m_par.LCY = stod(sval);
        }
        if (skey.compare("LMUY") == 0) {
            m_par.LMUY = stod(sval);
        }
        if (skey.compare("LEY") == 0) {
            m_par.LEY = stod(sval);
        }
        if (skey.compare("LKY") == 0) {
            m_par.LKY = stod(sval);
        }
        if (skey.compare("LHY") == 0) {
            m_par.LHY = stod(sval);
        }
        if (skey.compare("LVY") == 0) {
            m_par.LVY = stod(sval);
        }
        if (skey.compare("LGAY") == 0) {
            m_par.LGAY = stod(sval);
        }
        if (skey.compare("LTR") == 0) {
            m_par.LTR = stod(sval);
        }
        if (skey.compare("LRES") == 0) {
            m_par.LRES = stod(sval);
        }
        if (skey.compare("LGAZ") == 0) {
            m_par.LGAZ = stod(sval);
        }
        if (skey.compare("LXAL") == 0) {
            m_par.LXAL = stod(sval);
        }
        if (skey.compare("LYKA") == 0) {
            m_par.LYKA = stod(sval);
        }
        if (skey.compare("LVYKA") == 0) {
            m_par.LVYKA = stod(sval);
        }
        if (skey.compare("LS") == 0) {
            m_par.LS = stod(sval);
        }
        if (skey.compare("LSGKP") == 0) {
            m_par.LSGKP = stod(sval);
        }
        if (skey.compare("LSGAL") == 0) {
            m_par.LSGAL = stod(sval);
        }
        if (skey.compare("LGYR") == 0) {
            m_par.LGYR = stod(sval);
        }
        if (skey.compare("LMX") == 0) {
            m_par.LMX = stod(sval);
        }
        if (skey.compare("LMY") == 0) {
            m_par.LMY = stod(sval);
        }
        if (skey.compare("LIP") == 0) {
            m_par.LIP = stod(sval);
        }
    }
}

void ChPac02Tire::LoadSectionLongitudinal(FILE* fp) {
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
        }
        if (skey.compare("PDX1") == 0) {
            m_par.PDX1 = stod(sval);
        }
        if (skey.compare("PDX2") == 0) {
            m_par.PDX2 = stod(sval);
        }
        if (skey.compare("PEX1") == 0) {
            m_par.PEX1 = stod(sval);
        }
        if (skey.compare("PEX2") == 0) {
            m_par.PEX2 = stod(sval);
        }
        if (skey.compare("PEX3") == 0) {
            m_par.PEX3 = stod(sval);
        }
        if (skey.compare("PEX4") == 0) {
            m_par.PEX4 = stod(sval);
        }
        if (skey.compare("PKX1") == 0) {
            m_par.PKX1 = stod(sval);
        }
        if (skey.compare("PKX2") == 0) {
            m_par.PKX2 = stod(sval);
        }
        if (skey.compare("PKX3") == 0) {
            m_par.PKX3 = stod(sval);
        }
        if (skey.compare("PHX1") == 0) {
            m_par.PHX1 = stod(sval);
        }
        if (skey.compare("PHX2") == 0) {
            m_par.PHX2 = stod(sval);
        }
        if (skey.compare("PVX1") == 0) {
            m_par.PVX1 = stod(sval);
        }
        if (skey.compare("PVX2") == 0) {
            m_par.PVX2 = stod(sval);
        }
        if (skey.compare("RBX1") == 0) {
            m_par.RBX1 = stod(sval);
        }
        if (skey.compare("RBX2") == 0) {
            m_par.RBX2 = stod(sval);
        }
        if (skey.compare("RCX1") == 0) {
            m_par.RCX1 = stod(sval);
        }
        if (skey.compare("RHX1") == 0) {
            m_par.RHX1 = stod(sval);
        }
        if (skey.compare("PTX1") == 0) {
            m_par.PTX1 = stod(sval);
        }
        if (skey.compare("PTX2") == 0) {
            m_par.PTX2 = stod(sval);
        }
        if (skey.compare("PTX3") == 0) {
            m_par.PTX3 = stod(sval);
        }
        if (skey.compare("PPX1") == 0) {
            m_par.PPX1 = stod(sval);
        }
        if (skey.compare("PPX2") == 0) {
            m_par.PPX2 = stod(sval);
        }
        if (skey.compare("PPX3") == 0) {
            m_par.PPX3 = stod(sval);
        }
        if (skey.compare("PPX4") == 0) {
            m_par.PPX4 = stod(sval);
        }
    }
}

void ChPac02Tire::LoadSectionOverturning(FILE* fp) {
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
        }
        if (skey.compare("QSX2") == 0) {
            m_par.QSX2 = stod(sval);
        }
        if (skey.compare("QSX3") == 0) {
            m_par.QSX3 = stod(sval);
        }
        if (skey.compare("QSX4") == 0) {
            m_par.QSX4 = stod(sval);
        }
        if (skey.compare("QSX5") == 0) {
            m_par.QSX5 = stod(sval);
        }
        if (skey.compare("QSX6") == 0) {
            m_par.QSX6 = stod(sval);
        }
        if (skey.compare("QSX7") == 0) {
            m_par.QSX7 = stod(sval);
        }
        if (skey.compare("QSX8") == 0) {
            m_par.QSX8 = stod(sval);
        }
        if (skey.compare("QSX9") == 0) {
            m_par.QSX9 = stod(sval);
        }
        if (skey.compare("QSX10") == 0) {
            m_par.QSX10 = stod(sval);
        }
        if (skey.compare("QSX11") == 0) {
            m_par.QSX11 = stod(sval);
        }
        if (skey.compare("QPX1") == 0) {
            m_par.QPX1 = stod(sval);
        }
    }
}

void ChPac02Tire::LoadSectionLateral(FILE* fp) {
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
        }
        if (skey.compare("PDY1") == 0) {
            m_par.PDY1 = stod(sval);
        }
        if (skey.compare("PDY2") == 0) {
            m_par.PDY2 = stod(sval);
        }
        if (skey.compare("PDY3") == 0) {
            m_par.PDY3 = stod(sval);
        }
        if (skey.compare("PEY1") == 0) {
            m_par.PEY1 = stod(sval);
        }
        if (skey.compare("PEY2") == 0) {
            m_par.PEY2 = stod(sval);
        }
        if (skey.compare("PEY3") == 0) {
            m_par.PEY3 = stod(sval);
        }
        if (skey.compare("PEY4") == 0) {
            m_par.PEY4 = stod(sval);
        }
        if (skey.compare("PKY1") == 0) {
            m_par.PKY1 = stod(sval);
        }
        if (skey.compare("PKY2") == 0) {
            m_par.PKY2 = stod(sval);
        }
        if (skey.compare("PKY3") == 0) {
            m_par.PKY3 = stod(sval);
        }
        if (skey.compare("PHY1") == 0) {
            m_par.PHY1 = stod(sval);
        }
        if (skey.compare("PHY2") == 0) {
            m_par.PHY2 = stod(sval);
        }
        if (skey.compare("PHY3") == 0) {
            m_par.PHY3 = stod(sval);
        }
        if (skey.compare("PVY1") == 0) {
            m_par.PVY1 = stod(sval);
        }
        if (skey.compare("PVY2") == 0) {
            m_par.PVY2 = stod(sval);
        }
        if (skey.compare("PVY3") == 0) {
            m_par.PVY3 = stod(sval);
        }
        if (skey.compare("PVY4") == 0) {
            m_par.PVY4 = stod(sval);
        }
        if (skey.compare("RBY1") == 0) {
            m_par.RBY1 = stod(sval);
        }
        if (skey.compare("RBY2") == 0) {
            m_par.RBY2 = stod(sval);
        }
        if (skey.compare("RBY3") == 0) {
            m_par.RBY3 = stod(sval);
        }
        if (skey.compare("RCY1") == 0) {
            m_par.RCY1 = stod(sval);
        }
        if (skey.compare("RHY1") == 0) {
            m_par.RHY1 = stod(sval);
        }
        if (skey.compare("RVY1") == 0) {
            m_par.RVY1 = stod(sval);
        }
        if (skey.compare("RVY2") == 0) {
            m_par.RVY2 = stod(sval);
        }
        if (skey.compare("RVY3") == 0) {
            m_par.RVY3 = stod(sval);
        }
        if (skey.compare("RVY4") == 0) {
            m_par.RVY4 = stod(sval);
        }
        if (skey.compare("RVY5") == 0) {
            m_par.RVY5 = stod(sval);
        }
        if (skey.compare("RVY6") == 0) {
            m_par.RVY6 = stod(sval);
        }
        if (skey.compare("PTY1") == 0) {
            m_par.PTY1 = stod(sval);
        }
        if (skey.compare("PTY2") == 0) {
            m_par.PTY2 = stod(sval);
        }
        if (skey.compare("PPY1") == 0) {
            m_par.PPY1 = stod(sval);
        }
        if (skey.compare("PPY2") == 0) {
            m_par.PPY2 = stod(sval);
        }
        if (skey.compare("PPY3") == 0) {
            m_par.PPY3 = stod(sval);
        }
        if (skey.compare("PPY4") == 0) {
            m_par.PPY4 = stod(sval);
        }
    }
}

void ChPac02Tire::LoadSectionRolling(FILE* fp) {
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
            if (m_par.QSY1 <= 0.0)
                m_par.QSY1 = 0.01;  // be sure to have some rolling resistance
        }
        if (skey.compare("QSY2") == 0) {
            m_par.QSY2 = stod(sval);
        }
        if (skey.compare("QSY3") == 0) {
            m_par.QSY3 = stod(sval);
        }
        if (skey.compare("QSY4") == 0) {
            m_par.QSY4 = stod(sval);
        }
        if (skey.compare("QSY5") == 0) {
            m_par.QSY5 = stod(sval);
        }
        if (skey.compare("QSY6") == 0) {
            m_par.QSY6 = stod(sval);
        }
        if (skey.compare("QSY7") == 0) {
            m_par.QSY7 = stod(sval);
        }
        if (skey.compare("QSY8") == 0) {
            m_par.QSY8 = stod(sval);
        }
    }
}

void ChPac02Tire::LoadSectionConditions(FILE* fp) {
    bool ok = FindSectionStart("[TIRE_CONDITIONS]", fp);
    if (!ok) {
        GetLog() << "Desired section [TIRE_CONDITIONS] not found, older Pacejka file version.\n";
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
        bool ip_ok = false;
        if (skey.compare("IP") == 0) {
            m_par.IP = m_par.u_pressure * stod(sval);
            ip_ok = true;
        }
        bool ip_nom_ok = false;
        if (skey.compare("IP_NOM") == 0) {
            m_par.IP_NOM = m_par.u_pressure * stod(sval);
            ip_nom_ok = true;
        }
        m_tire_conditions_found = ip_ok && ip_nom_ok;
    }
}

void ChPac02Tire::LoadVerticalTable(FILE* fp) {
    bool ok = FindSectionStart("[DEFLECTION_LOAD_CURVE]", fp);
    if (!ok) {
        GetLog() << "Desired section [DEFLECTION_LOAD_CURVE] not found, using linear vertical stiffness.\n";
        return;
    }
    std::vector<double> xval, yval;
    while (true) {
        char line[201];
        fgets(line, 200, fp);  // buffer one line
        if (feof(fp))
            break;
        // remove leading white space
        size_t l = strlen(line);
        size_t ipos = 0;
        while (isblank(line[ipos])) {
            if (ipos < l)
                ipos++;
        }
        std::string sbuf(line + ipos);
        // skip pure comment lines
        if (sbuf.front() == '!' || sbuf.front() == '$' || sbuf.front() == '{')
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
        size_t sz;
        double x = m_par.u_length * stod(sbuf, &sz);
        double y = m_par.u_force * stod(sbuf.substr(sz), &sz);
        xval.push_back(x / m_par.UNLOADED_RADIUS);
        yval.push_back(y / m_par.FNOMIN);
    }
    size_t ndata = xval.size();
    Eigen::MatrixXd M(ndata, 2);
    Eigen::VectorXd r(ndata);
    for (size_t i = 0; i < ndata; i++) {
        M(i, 0) = xval[i];
        M(i, 1) = pow(xval[i], 2);
        r(i) = yval[i];
    }
    Eigen::VectorXd x = M.householderQr().solve(r);
    m_par.QFZ1 = x(0);
    m_par.QFZ2 = x(1);
    /*
    GetLog() << "a = " << x(0) << "\n";
    GetLog() << "b = " << x(1) << "\n";
    GetLog() << "Test1 " << (x(0)*xval.back()/4.0 + x(1)*pow(xval.back()/4.0,2))*m_par.FNOMIN << "\n";
    GetLog() << "Test2 " << (x(0)*xval.back()/2.0 + x(1)*pow(xval.back()/2.0,2))*m_par.FNOMIN << "\n";
    GetLog() << "Test3 " << (x(0)*xval.back()*3.0/4.0 + x(1)*pow(xval.back()*3.0/4.0,2))*m_par.FNOMIN << "\n";
    GetLog() << "Test2 " << (x(0)*xval.back() + x(1)*pow(xval.back(),2))*m_par.FNOMIN << "\n";
    double sum = 0.0;
    for(int i=0; i<ndata; i++) {
        double f = (m_par.QFZ1*xval[i] + m_par.QFZ2*pow(xval[i],2));
        double y = yval[i];
        double e = (f-y);
        sum += e*e;
    }
    GetLog() << "SumOfSquares = " << sum/double(ndata) << "\n";
     */
    m_vertical_table_found = true;
}

void ChPac02Tire::LoadBottomingTable(FILE* fp) {
    bool ok = FindSectionStart("[BOTTOMING_CURVE]", fp);
    if (!ok) {
        GetLog() << "Desired section [BOTTOMING_CURVE] not found, no bottoming stiffness set.\n";
        return;
    }
    while (true) {
        char line[201];
        fgets(line, 200, fp);  // buffer one line
        if (feof(fp))
            break;
        // remove leading white space
        size_t l = strlen(line);
        size_t ipos = 0;
        while (isblank(line[ipos])) {
            if (ipos < l)
                ipos++;
        }
        std::string sbuf(line + ipos);
        // skip pure comment lines
        if (sbuf.front() == '!' || sbuf.front() == '$' || sbuf.front() == '{')
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
        size_t sz;
        double x = m_par.u_length * stod(sbuf, &sz);
        double y = m_par.u_force * stod(sbuf.substr(sz), &sz);
        m_bott_map.AddPoint(x, y);
    }
    if (m_bott_map.GetPoints().size() >= 3)
        m_bottoming_table_found = true;
}

void ChPac02Tire::LoadSectionAligning(FILE* fp) {
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
        }
        if (skey.compare("QBZ2") == 0) {
            m_par.QBZ2 = stod(sval);
        }
        if (skey.compare("QBZ3") == 0) {
            m_par.QBZ3 = stod(sval);
        }
        if (skey.compare("QBZ4") == 0) {
            m_par.QBZ4 = stod(sval);
        }
        if (skey.compare("QBZ5") == 0) {
            m_par.QBZ5 = stod(sval);
        }
        if (skey.compare("QBZ9") == 0) {
            m_par.QBZ9 = stod(sval);
        }
        if (skey.compare("QCZ1") == 0) {
            m_par.QCZ1 = stod(sval);
        }
        if (skey.compare("QDZ1") == 0) {
            m_par.QDZ1 = stod(sval);
        }
        if (skey.compare("QDZ2") == 0) {
            m_par.QDZ2 = stod(sval);
        }
        if (skey.compare("QDZ3") == 0) {
            m_par.QDZ3 = stod(sval);
        }
        if (skey.compare("QDZ4") == 0) {
            m_par.QDZ4 = stod(sval);
        }
        if (skey.compare("QDZ6") == 0) {
            m_par.QDZ6 = stod(sval);
        }
        if (skey.compare("QDZ7") == 0) {
            m_par.QDZ7 = stod(sval);
        }
        if (skey.compare("QDZ8") == 0) {
            m_par.QDZ8 = stod(sval);
        }
        if (skey.compare("QDZ9") == 0) {
            m_par.QDZ9 = stod(sval);
        }
        if (skey.compare("QEZ1") == 0) {
            m_par.QEZ1 = stod(sval);
        }
        if (skey.compare("QEZ2") == 0) {
            m_par.QEZ2 = stod(sval);
        }
        if (skey.compare("QEZ3") == 0) {
            m_par.QEZ3 = stod(sval);
        }
        if (skey.compare("QEZ4") == 0) {
            m_par.QEZ4 = stod(sval);
        }
        if (skey.compare("QEZ5") == 0) {
            m_par.QEZ5 = stod(sval);
        }
        if (skey.compare("QHZ1") == 0) {
            m_par.QHZ1 = stod(sval);
        }
        if (skey.compare("QHZ2") == 0) {
            m_par.QHZ2 = stod(sval);
        }
        if (skey.compare("QHZ3") == 0) {
            m_par.QHZ3 = stod(sval);
        }
        if (skey.compare("QHZ4") == 0) {
            m_par.QHZ4 = stod(sval);
        }
        if (skey.compare("SSZ1") == 0) {
            m_par.SSZ1 = stod(sval);
        }
        if (skey.compare("SSZ2") == 0) {
            m_par.SSZ2 = stod(sval);
        }
        if (skey.compare("SSZ3") == 0) {
            m_par.SSZ3 = stod(sval);
        }
        if (skey.compare("SSZ4") == 0) {
            m_par.SSZ4 = stod(sval);
        }
        if (skey.compare("QTZ1") == 0) {
            m_par.QTZ1 = stod(sval);
        }
        if (skey.compare("QPZ1") == 0) {
            m_par.QPZ1 = stod(sval);
        }
        if (skey.compare("QPZ2") == 0) {
            m_par.QPZ2 = stod(sval);
        }
        if (skey.compare("MBELT") == 0) {
            m_par.MBELT = stod(sval);
        }
    }
}

void ChPac02Tire::Initialize(std::shared_ptr<ChWheel> wheel) {
    ChTire::Initialize(wheel);

    m_g = wheel->GetSpindle()->GetSystem()->Get_G_acc().Length();
    
    // Let derived class set the MF tire parameters
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

void ChPac02Tire::Synchronize(double time, const ChTerrain& terrain) {
    WheelState wheel_state = m_wheel->GetState();

    // Extract the wheel normal (expressed in global frame)
    ChMatrix33<> A(wheel_state.rot);
    ChVector<> disc_normal = A.Get_A_Yaxis();

    // Assuming the tire is a disc, check contact with terrain
    float mu_road;
    m_data.in_contact =
        DiscTerrainCollision(m_collision_type, terrain, wheel_state.pos, disc_normal, m_par.UNLOADED_RADIUS,
                             m_par.WIDTH, m_areaDep, m_data.frame, m_data.depth, mu_road);
    ChClampValue(mu_road, 0.1f, 1.0f);

    m_states.mu_scale = mu_road / m_mu0;  // can change with terrain conditions
    m_states.mu_road = mu_road;           // needed for access method

    // Calculate tire kinematics
    CalculateKinematics(wheel_state, m_data.frame);

    m_states.gamma = ChClamp(GetCamberAngle(), -m_gamma_limit * CH_C_DEG_TO_RAD, m_gamma_limit * CH_C_DEG_TO_RAD);

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
        // R_eff is a Rill estimation, not Pacejka. Advantage: it works well with speed = zero.
        m_states.R_eff = (2.0 * m_par.UNLOADED_RADIUS + (m_par.UNLOADED_RADIUS - m_data.depth)) / 3.0;
        m_states.vx = std::abs(m_data.vel.x());
        m_states.vsx = m_data.vel.x() - wheel_state.omega * m_states.R_eff;
        m_states.vsy = -m_data.vel.y();
        // prevent singularity for kappa, when vx == 0
        const double epsilon = 0.1;
        m_states.kappa = -m_states.vsx / (m_states.vx + epsilon);
        m_states.alpha = std::atan2(m_states.vsy, m_states.vx + epsilon);
        m_states.omega = wheel_state.omega;
        m_states.disc_normal = disc_normal;
        m_states.Fz0_prime = m_par.FNOMIN * m_par.LFZO;
        m_states.dfz0 = (Fn_mag - m_states.Fz0_prime) / m_states.Fz0_prime;
        m_states.Pi0_prime = m_par.IP_NOM * m_par.LIP;
        m_states.dpi = (m_par.IP - m_states.Pi0_prime) / m_states.Pi0_prime;
        // Ensure that kappa stays between -1 & 1
        ChClampValue(m_states.kappa, -1.0, 1.0);
        // Ensure that alpha stays between -pi()/2 & pi()/2 (a little less to prevent tan from going to infinity)
        ChClampValue(m_states.alpha, -CH_C_PI_2 + 0.01, CH_C_PI_2 - 0.01);
        // Clamp |gamma| to specified value: Limit due to tire testing, avoids erratic extrapolation. m_gamma_limit is
        // in rad too.
        ChClampValue(m_states.gamma, -m_gamma_limit, m_gamma_limit);
    } else {
        // Reset all states if the tire comes off the ground.
        m_data.normal_force = 0;
        m_states.R_eff = m_par.UNLOADED_RADIUS;
        m_states.grip_sat_x = 0;
        m_states.grip_sat_y = 0;
        m_states.kappa = 0;
        m_states.alpha = 0;
        m_states.gamma = 0;
        m_states.vx = 0;
        m_states.vsx = 0;
        m_states.vsy = 0;
        m_states.omega = 0;
        m_states.Fz0_prime = 0;
        m_states.dfz0 = 0;
        m_states.Pi0_prime = 0;
        m_states.dpi = 1;
        m_states.disc_normal = ChVector<>(0, 0, 0);
    }
}

void ChPac02Tire::Advance(double step) {
    // Set tire forces to zero.
    m_tireforce.force = ChVector<>(0, 0, 0);
    m_tireforce.moment = ChVector<>(0, 0, 0);

    // Return now if no contact.
    if (!m_data.in_contact)
        return;

    // Calculate the new force and moment values (normal force and moment have already been accounted for in
    // Synchronize()).
    // See reference for details on the calculations.
    double Fx0 = 0;  // Fx at zero/small speed
    double Fy0 = 0;  // Fy at zero/small speed
    double Fx = 0;
    double Fy = 0;
    double Fz = m_data.normal_force;
    double Mx = 0;
    double My = 0;
    double Mz = 0;
    double kappa = m_states.kappa;
    double alpha = m_states.alpha;
    double gamma = m_states.gamma;
    double frblend = ChSineStep(std::abs(m_data.vel.x()), m_frblend_begin, 0.0, m_frblend_end, 1.0);

    switch (m_use_mode) {
        case 0:
            // vertical spring & damper mode
            break;
        case 1:
            // steady state pure longitudinal slip
            CalcFxyMz(Fx, Fy, Mz, kappa, 0.0, Fz, gamma);
            My = CalcMy(Fx, Fz, gamma);
            break;
        case 2:
            // steady state pure lateral slip
            CalcFxyMz(Fx, Fy, Mz, 0.0, alpha, Fz, gamma);
            Mx = CalcMx(Fy, Fz, gamma);
            break;
        case 3:
        case 4: {
            // steady state (un)combined slip
            CombinedCoulombForces(Fx0, Fy0, Fz);
            double Fx_ss = 0, Fy_ss = 0;
            CalcFxyMz(Fx_ss, Fy_ss, Mz, kappa, alpha, Fz, gamma);
            Fx = (1.0 - frblend) * Fx0 + frblend * Fx_ss;
            Fy = (1.0 - frblend) * Fy0 + frblend * Fy_ss;
            My = CalcMy(Fx, Fz, gamma);
            Mx = CalcMx(Fy, Fz, gamma);
        } break;
    }

    // Compile the force and moment vectors so that they can be
    // transformed into the global coordinate system.
    // Convert from SAE to ISO Coordinates at the contact patch.
    m_tireforce.force = ChVector<>(Fx, -Fy, m_data.normal_force);
    m_tireforce.moment = ChVector<>(Mx, -My, -Mz);
}

double ChPac02Tire::GetLongitudinalGripSaturation() {
    return m_states.grip_sat_x;
}

double ChPac02Tire::GetLateralGripSaturation() {
    return m_states.grip_sat_y;
}

// -----------------------------------------------------------------------------

}  // end namespace vehicle
}  // namespace chrono
