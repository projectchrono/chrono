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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include <cmath>

#include "chrono/physics/ChLinkLockScrew.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkLockScrew)

ChLinkLockScrew::ChLinkLockScrew() {
    SetThread(0.05);

    // Mask: initialize our LinkMaskLF (lock formulation mask) to X,Y,Z,Rx Ry,
    // (note: the Z lock is not a standard LinkLock z-lock and will be handled as a custom screw constraint
    // z = tau*alpha, later in the updating functions).
    mask.SetLockMask(true, true, true, false, true, true, false);
    BuildLink();
}

ChLinkLockScrew::ChLinkLockScrew(const ChLinkLockScrew& other) : ChLinkLock(other) {
    tau = other.tau;
}

void ChLinkLockScrew::UpdateState() {
    // First, compute everything as it were a normal "revolute" joint, on z axis...
    ChLinkLock::UpdateState();

    // Then, MODIFY the Z part of equations, such that the Z = 0  becomes Z = tau * alpha
    double scr_C, scr_C_dt, scr_C_dtdt;
    double scr_Ct, scr_Qc;
    double coeffa, coeffb, zangle, msign;
    ChMatrixNM<double, 1, 7> scr_Cq1;
    ChMatrixNM<double, 1, 7> scr_Cq2;
    double Crz;

    if (fabs(relM.rot.e0()) < 0.707) {
        Crz = relM.rot.e0();  // cos(alpha/2)
        msign = +1;
        zangle = std::acos(Crz);
        if (relM.rot.e3() < 0) {
            zangle = -zangle;  // a/2 = -acos(Crz);
            msign = -1;
        }
        double mrelz = relM.pos.z();

        scr_C = mrelz - tau * 2.0 * zangle;
        // modulus correction..
        scr_C = scr_C - GetThread() * floor(scr_C / GetThread());
        double shiftedC = scr_C - GetThread() * ceil(scr_C / GetThread());
        if (fabs(scr_C) > fabs(shiftedC))
            scr_C = shiftedC;

        coeffa = +2.0 * tau * msign * 1 / (std::sqrt(1 - std::pow(Crz, 2.0)));
        coeffb = +2.0 * tau * msign * Crz / (std::pow((1 - std::pow(Crz, 2)), 3.0 / 2.0));

        scr_C_dt = relM_dt.pos.z() + relM_dt.rot.e0() * coeffa;
        scr_C_dtdt = relM_dtdt.pos.z() + relM_dt.rot.e0() * coeffb + relM_dtdt.rot.e0() * coeffa;
        scr_Ct = Ct_temp.pos.z() + coeffa * Ct_temp.rot.e0();
        scr_Qc = Q_c_temp(2) + coeffa * Q_c_temp(3) - relM_dt.rot.e0() * coeffb;
        scr_Cq1.setZero();
        scr_Cq2.setZero();
        scr_Cq1.block(0, 3, 1, 4) = coeffa * Cq1_temp.block(3, 3, 1, 4);
        scr_Cq2.block(0, 3, 1, 4) = coeffa * Cq2_temp.block(3, 3, 1, 4);
    } else {
        Crz = relM.rot.e3();  // Zz*sin(alpha/2)
        msign = +1;
        zangle = std::asin(Crz);
        if (relM.rot.e0() < 0) {
            zangle = CH_PI - zangle;
            msign = -1;
        }
        double mrelz = relM.pos.z();  // fmod (relM.pos.z() , (tau * 2 * CH_PI));

        scr_C = mrelz - tau * 2.0 * zangle;
        // modulus correction..
        scr_C = scr_C - GetThread() * floor(scr_C / GetThread());
        double shiftedC = scr_C - GetThread() * ceil(scr_C / GetThread());
        if (fabs(scr_C) > fabs(shiftedC))
            scr_C = shiftedC;

        coeffa = -2.0 * tau * msign * 1 / (std::sqrt(1 - std::pow(Crz, 2.0)));
        coeffb = -2.0 * tau * msign * Crz / (std::pow((1 - std::pow(Crz, 2)), 3.0 / 2.0));

        scr_C_dt = relM_dt.pos.z() + relM_dt.rot.e3() * coeffa;
        scr_C_dtdt = relM_dtdt.pos.z() + relM_dt.rot.e3() * coeffb + relM_dtdt.rot.e3() * coeffa;
        scr_Ct = Ct_temp.pos.z() + coeffa * Ct_temp.rot.e3();
        scr_Qc = Q_c_temp(2) + coeffa * Q_c_temp(6) - relM_dt.rot.e3() * coeffb;
        scr_Cq1.setZero();
        scr_Cq2.setZero();
        scr_Cq1.block(0, 3, 1, 4) = coeffa * Cq1_temp.block(6, 3, 1, 4);
        scr_Cq2.block(0, 3, 1, 4) = coeffa * Cq2_temp.block(6, 3, 1, 4);
    }

    Cq1.block(2, 0, 1, 7) = Cq1_temp.block(2, 0, 1, 7) + scr_Cq1;
    Cq2.block(2, 0, 1, 7) = Cq2_temp.block(2, 0, 1, 7) + scr_Cq2;

    Q_c(2) = scr_Qc;
    C(2) = scr_C;
    C_dt(2) = scr_C_dt;
    C_dtdt(2) = scr_C_dtdt;
    Ct(2) = scr_Ct;
}

void ChLinkLockScrew::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChLinkLockScrew>();

    // serialize parent class
    ChLinkLock::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(tau);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkLockScrew::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChLinkLockScrew>();

    // deserialize parent class
    ChLinkLock::ArchiveIn(archive_in);

    // deserialize all member data:
    archive_in >> CHNVP(tau);
}

}  // end namespace chrono
