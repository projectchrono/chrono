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

#include "chrono/motion_functions/ChFunction_Mocap.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunction_Mocap)

ChFunction_Mocap::ChFunction_Mocap() {
    Set_samples(2);  // this creates arrays
    Set_samp_freq(50);
}

ChFunction_Mocap::ChFunction_Mocap(int m_samples, double freq) {
    Set_samples(m_samples);  // this creates arrays
    Set_samp_freq(freq);
}

ChFunction_Mocap::ChFunction_Mocap(const ChFunction_Mocap& other) {
    Set_samples(other.samples);  // this creates arrays
    Set_samp_freq(other.samp_freq);

    array_y = other.array_y;
    array_y_dt = other.array_y_dt;
    array_y_dtdt = other.array_y_dtdt;
}

void ChFunction_Mocap::Estimate_x_range(double& xmin, double& xmax) const {
    xmin = 0.0;
    xmax = Get_timetot();
}

void ChFunction_Mocap::Set_samp_freq(double m_fr) {
    samp_freq = m_fr;
    timetot = ((double)samples / samp_freq);
}

void ChFunction_Mocap::Set_samples(int m_samples) {
    samples = m_samples;

    if (samples < 2)
        samples = 2;

    timetot = ((double)samples / samp_freq);

    array_y.resize(samples);
    array_y_dt.resize(samples);
    array_y_dtdt.resize(samples);
}

// Compute all the y_dt basing on y, using the trapezoidal rule for numerical differentiation
void ChFunction_Mocap::Compute_array_dt(const ChArray<>& array_A, ChArray<>& array_A_dt) const {
    int i, ia, ib;
    double y_dt;

    for (i = 0; i < samples; i++) {
        ia = i - 1;  // boundaries cases
        if (ia <= 0) {
            ia = 0;
        }
        ib = i + 1;
        if (ib >= samples) {
            ib = i;
        }
        // trapezoidal differentiation
        y_dt = ((array_A(ib)) - (array_A(ia))) / Get_timeslice();

        array_A_dt(i) = y_dt;
    }
}

// Interpolation of the in-between values, given the discrete sample array (uniformly spaced points)
double ChFunction_Mocap::LinInterp(const ChArray<>& array, double x, double x_max) const {
    double position;
    double weightA, weightB;
    int ia, ib;

    position = ((double)samples * (x / x_max));

    ia = (int)floor(position);
    if (ia < 0) {
        ia = 0;
    };
    if (ia >= samples) {
        ia = samples - 1;
    }

    ib = ia + 1;
    if (ib < 0) {
        ib = 0;
    };
    if (ib >= samples) {
        ib = samples - 1;
    };

    weightB = position - (int)position;
    weightA = 1 - weightB;

    return (weightA * array(ia) + weightB * array(ib));
}

// Setup of arrays, provided as external vectors of
// samples. These functions automatically compute the
// derivatives (the arrays .._y_dt and y_dtdt)
void ChFunction_Mocap::Set_array_y(const ChArray<>& m_array_y) {
    array_y = m_array_y;

    Compute_array_dt(array_y, array_y_dt);
    Compute_array_dt(array_y_dt, array_y_dtdt);
}

void ChFunction_Mocap::Set_array_y_dt(const ChArray<>& m_array_y_dt) {
    // *** TO DO  ***
}

void ChFunction_Mocap::Set_array_y_dtdt(const ChArray<>& m_array_y_dtdt) {
    // *** TO DO  ***
}

// Parsing of external files, to create mocap streams
// from the output of mocap devices
bool ChFunction_Mocap::Parse_array_AOA() {
    // *** TO DO **** //
    return true;
}

bool ChFunction_Mocap::Parse_array_Elite() {
    // *** TO DO **** //
    return true;
}

// Return the value of the evaluated function, using
// linear interpolation to guess the in-between points,
// having the array samples as references.
double ChFunction_Mocap::Get_y(double x) const {
    return LinInterp(array_y, x, timetot);
}

double ChFunction_Mocap::Get_y_dx(double x) const {
    return LinInterp(array_y_dt, x, timetot);
}

double ChFunction_Mocap::Get_y_dxdx(double x) const {
    return LinInterp(array_y_dtdt, x, timetot);
}

void ChFunction_Mocap::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChFunction_Mocap>();
    // serialize parent class
    ChFunction::ArchiveOut(marchive);
    // serialize all member data:
    ////marchive << CHNVP(array_y);
    ////marchive << CHNVP(array_y_dt);
    ////marchive << CHNVP(array_y_dtdt);
    marchive << CHNVP(samp_freq);
    marchive << CHNVP(samples);
    marchive << CHNVP(timetot);
}

void ChFunction_Mocap::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChFunction_Mocap>();
    // deserialize parent class
    ChFunction::ArchiveIn(marchive);
    // stream in all member data:
    ////marchive >> CHNVP(array_y);
    ////marchive >> CHNVP(array_y_dt);
    ////marchive >> CHNVP(array_y_dtdt);
    marchive >> CHNVP(samp_freq);
    marchive >> CHNVP(samples);
    marchive >> CHNVP(timetot);
}

}  // end namespace chrono
