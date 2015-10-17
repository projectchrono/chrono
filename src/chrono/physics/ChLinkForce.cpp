//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2011 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   ChLinkForce.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "physics/ChLinkForce.h"

namespace chrono {

ChLinkForce::ChLinkForce() {
    active = 0;  // default: inactive limit

    iforce = 0;
    modul_iforce = new ChFunction_Const(1);  // default: const.modulation of iforce

    K = 0;
    modul_K = new ChFunction_Const(1);  // default: const.modulation of K

    R = 0;
    modul_R = new ChFunction_Const(1);  // default: const.modulation of R
}

ChLinkForce::~ChLinkForce() {
    if (modul_iforce)
        delete modul_iforce;
    if (modul_K)
        delete modul_K;
    if (modul_R)
        delete modul_R;
}

void ChLinkForce::Copy(ChLinkForce* source) {
    active = source->active;

    iforce = source->iforce;
    K = source->K;
    R = source->R;

    // replace functions:
    if (modul_iforce)
        delete modul_iforce;
    if (modul_K)
        delete modul_K;
    if (modul_R)
        delete modul_R;

    modul_iforce = source->modul_iforce->new_Duplicate();
    modul_K = source->modul_K->new_Duplicate();
    modul_R = source->modul_R->new_Duplicate();
}

ChLinkForce* ChLinkForce::new_Duplicate() {
    ChLinkForce* m_lim;
    m_lim = new ChLinkForce;
    m_lim->Copy(this);
    return (m_lim);
}

void ChLinkForce::Set_modul_iforce(ChFunction* m_funct) {
    if (modul_iforce)
        delete modul_iforce;
    modul_iforce = m_funct;
}
void ChLinkForce::Set_modul_K(ChFunction* m_funct) {
    if (modul_K)
        delete modul_K;
    modul_K = m_funct;
}
void ChLinkForce::Set_modul_R(ChFunction* m_funct) {
    if (modul_R)
        delete modul_R;
    modul_R = m_funct;
}


void ChLinkForce::ArchiveOUT(ChArchiveOut& marchive) {

    // class version number
    marchive.VersionWrite(1);
    // serialize parent class too

    // stream out all member data
    marchive << CHNVP(active);
    marchive << CHNVP(iforce);
    marchive << CHNVP(modul_iforce);
    marchive << CHNVP(K);
    marchive << CHNVP(modul_K);
    marchive << CHNVP(R);
    marchive << CHNVP(modul_R);
}

void ChLinkForce::ArchiveIN(ChArchiveIn& marchive) {

    // class version number
    int version = marchive.VersionRead();
    // deserialize parent class too

    // stream in all member data
    marchive >> CHNVP(active);
    marchive >> CHNVP(iforce);
    marchive >> CHNVP(modul_iforce);
    marchive >> CHNVP(K);
    marchive >> CHNVP(modul_K);
    marchive >> CHNVP(R);
    marchive >> CHNVP(modul_R);
}



double ChLinkForce::Get_Kcurrent(double x, double x_dt, double t) {
    double mK = 0;
    if (active) {
        double modulator;

        if (modul_K) {
            modulator = modul_K->Get_y(x);
        } else {
            modulator = 1;
        }
        mK = K * modulator;
    }
    return mK;
}

double ChLinkForce::Get_Rcurrent(double x, double x_dt, double t) {
    double mR = 0;
    if (active) {
        double modulator;

        if (modul_R) {
            modulator = modul_R->Get_y(x);
        } else {
            modulator = 1;
        }
        mR = R * modulator;
    }
    return mR;
}

double ChLinkForce::Get_iFcurrent(double x, double x_dt, double t) {
    double mforce = 0;
    if (active) {
        double modulator;

        // the internal force contribute = iforce
        if (modul_iforce) {
            modulator = modul_iforce->Get_y(t);
        } else {
            modulator = 1;
        }
        mforce = iforce * modulator;
    }
    return mforce;
}

///////////////////

double ChLinkForce::Get_Force(double x, double x_dt, double t) {
    double mforce = 0;
    if (active) {
        double modulator;

        // the internal force contribute = iforce
        if (modul_iforce) {
            modulator = modul_iforce->Get_y(t);
        } else {
            modulator = 1;
        }
        mforce = iforce * modulator;

        // the stiffness contribute =  - K x
        if (modul_K) {
            modulator = modul_K->Get_y(x);
        } else {
            modulator = 1;
        }
        mforce -= (K * modulator) * x;

        // the damping contribute =  - R x_dt
        if (modul_R) {
            modulator = modul_R->Get_y(x);
        } else {
            modulator = 1;
        }
        mforce -= (R * modulator) * x_dt;
    }

    return mforce;
}

}  // END_OF_NAMESPACE____

///////////////////
