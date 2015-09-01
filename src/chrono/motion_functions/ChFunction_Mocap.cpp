//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2011 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   ChFunction_Mocap.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "ChFunction_Mocap.h"

namespace chrono {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChFunction_Mocap> a_registration_mocap;

ChFunction_Mocap::ChFunction_Mocap() {
    array_y = NULL;
    array_y_dt = NULL;
    array_y_dtdt = NULL;

    Set_samples(2);  // this creates arrays
    Set_samp_freq(50);
}

ChFunction_Mocap::ChFunction_Mocap(int m_samples, double freq) {
    array_y = NULL;
    array_y_dt = NULL;
    array_y_dtdt = NULL;

    Set_samples(m_samples);  // this creates arrays
    Set_samp_freq(freq);
}

ChFunction_Mocap::~ChFunction_Mocap() {
    if (array_y != NULL)
        delete array_y;
    if (array_y_dt != NULL)
        delete array_y_dt;
    if (array_y_dtdt != NULL)
        delete array_y_dtdt;
}

void ChFunction_Mocap::Copy(ChFunction_Mocap* source) {
    Set_samples(source->samples);  // this creates arrays
    Set_samp_freq(source->samp_freq);

    if (source->Get_array_y())
        array_y->CopyFromMatrix(*source->Get_array_y());
    if (source->Get_array_y_dt())
        array_y_dt->CopyFromMatrix(*source->Get_array_y_dt());
    if (source->Get_array_y_dtdt())
        array_y_dtdt->CopyFromMatrix(*source->Get_array_y_dtdt());
}

ChFunction* ChFunction_Mocap::new_Duplicate() {
    ChFunction_Mocap* m_func;
    m_func = new ChFunction_Mocap;
    m_func->Copy(this);
    return (m_func);
}

void ChFunction_Mocap::Estimate_x_range(double& xmin, double& xmax) {
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

    if (array_y != NULL)
        delete array_y;
    if (array_y_dt != NULL)
        delete array_y_dt;
    if (array_y_dtdt != NULL)
        delete array_y_dtdt;

    array_y = new ChMatrixDynamic<>(1, samples);
    array_y_dt = new ChMatrixDynamic<>(1, samples);
    array_y_dtdt = new ChMatrixDynamic<>(1, samples);
}

// compute all the y_dt basing on y, using the
// trapezioidal rule for numerical differentiation

void ChFunction_Mocap::Compute_array_dt(ChMatrix<>* array_A, ChMatrix<>* array_A_dt) {
    int i, ia, ib;
    double y_dt;

    for (i = 0; i < samples; i++) {
        ia = i - 1;  // boundaries cases
        if (ia <= 0) {
            ia = 0;
        };
        ib = i + 1;
        if (ib >= samples) {
            ib = i;
        };
        // trapezioidal differentiation
        y_dt = ((array_A->GetElement(0, ib)) - (array_A->GetElement(0, ia))) / Get_timeslice();

        array_A_dt->SetElement(0, i, y_dt);
    }
}

// Interpolation of the in-between values, given the discrete
// sample array (uniformly spaced points)

double ChFunction_Mocap::LinInterp(ChMatrix<>* m_array, double x, double x_max) {
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

    return (weightA * (m_array->GetElement(0, ia)) + weightB * (m_array->GetElement(0, ib)));
}

// Setup of arrays, provided as external vectors of
// samples. These functions automatically compute the
// derivatives (the arrays .._y_dt and y_dtdt)

void ChFunction_Mocap::Set_array_y(ChMatrix<>* m_array_y) {
    array_y->CopyFromMatrix(*m_array_y);

    Compute_array_dt(array_y, array_y_dt);
    Compute_array_dt(array_y_dt, array_y_dtdt);
}

void ChFunction_Mocap::Set_array_y_dt(ChMatrix<>* m_array_y_dt) {
    // *** TO DO  ***
}

void ChFunction_Mocap::Set_array_y_dtdt(ChMatrix<>* m_array_y_dtdt) {
    // *** TO DO  ***
}

// Parsing of external files, to create mocap streams
// from the output of mocap devices

int ChFunction_Mocap::Parse_array_AOA() {
    // *** TO DO **** //
    return TRUE;
}

int ChFunction_Mocap::Parse_array_Elite() {
    // *** TO DO **** //
    return TRUE;
}

// Return the value of the evaluated function, using
// linear interpolation to guess the in-between points,
// having the array samples as references.

double ChFunction_Mocap::Get_y(double x) {
    return LinInterp(array_y, x, timetot);
}

double ChFunction_Mocap::Get_y_dx(double x) {
    return LinInterp(array_y_dt, x, timetot);
}

double ChFunction_Mocap::Get_y_dxdx(double x) {
    return LinInterp(array_y_dtdt, x, timetot);
}

// File parsing and dumping

void ChFunction_Mocap::StreamOUT(ChStreamOutBinary& mstream) {
    // class version number
    mstream.VersionWrite(1);
    // serialize parent class too
    ChFunction::StreamOUT(mstream);

    // stream out all member data
    mstream << this->Get_samples();
    mstream << this->Get_samp_freq();
    mstream << *this->array_y;
}

void ChFunction_Mocap::StreamIN(ChStreamInBinary& mstream) {
    // class version number
    int version = mstream.VersionRead();
    // deserialize parent class too
    ChFunction::StreamIN(mstream);

    // stream in all member data
    double dfoo;
    int ifoo;
    mstream >> ifoo;
    Set_samples(ifoo);
    mstream >> dfoo;
    Set_samp_freq(dfoo);
    mstream >> *this->array_y;
}

void ChFunction_Mocap::StreamOUT(ChStreamOutAscii& mstream) {
    mstream << "FUNCT_MOCAP  \n";

    //***TO DO***
}

}  // END_OF_NAMESPACE____

// eof
