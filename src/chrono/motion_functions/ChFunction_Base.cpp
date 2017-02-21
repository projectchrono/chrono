// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include <cfloat>
#include <cmath>
#include <memory.h>
#include <cstdlib>
#include <cstring>
#include <iostream>

#include "chrono/motion_functions/ChFunction_Base.h"
#include "chrono/physics/ChGlobal.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
//CH_FACTORY_REGISTER(ChFunction) // NO! this is an abstract class, rather use for children concrete classes.

double ChFunction::Get_y_dN(double x, int derivate) const {
    switch (derivate) {
        case 0:
            return Get_y(x);
        case 1:
            return Get_y_dx(x);
        case 2:
            return Get_y_dxdx(x);
        default:
            return Get_y(x);
    }
}

void ChFunction::Estimate_y_range(double xmin, double xmax, double& ymin, double& ymax, int derivate) const {
    ymin = 10000;
    ymax = -10000;
    for (double mx = xmin; mx < xmax; mx += (xmax - xmin) / 100.0) {
        if (Get_y_dN(mx, derivate) < ymin)
            ymin = Get_y_dN(mx, derivate);
        if (Get_y_dN(mx, derivate) > ymax)
            ymax = Get_y_dN(mx, derivate);
    }
    if (fabs(ymax - ymin) < 10e-12) {
        ymin = -0.5;
        ymax = +1.0;
    }
    ymax += 0.12 * (ymax - ymin);
    ymin -= 0.12 * (ymax - ymin);
}

// some analysis functions
double ChFunction::Compute_max(double xmin, double xmax, double sampling_step, int derivate) const {
    double mret = -1E30;
    for (double mx = xmin; mx <= xmax; mx += sampling_step) {
        if (this->Get_y_dN(mx, derivate) > mret)
            mret = this->Get_y_dN(mx, derivate);
    }
    return mret;
}

double ChFunction::Compute_min(double xmin, double xmax, double sampling_step, int derivate) const {
    double mret = +1E30;
    for (double mx = xmin; mx <= xmax; mx += sampling_step) {
        if (this->Get_y_dN(mx, derivate) < mret)
            mret = this->Get_y_dN(mx, derivate);
    }
    return mret;
}

double ChFunction::Compute_mean(double xmin, double xmax, double sampling_step, int derivate) const {
    double mret = 0;
    int numpts = 0;
    for (double mx = xmin; mx <= xmax; mx = mx + sampling_step) {
        numpts++;
        mret += this->Get_y_dN(mx, derivate);
    }
    return mret / ((double)numpts);
}

double ChFunction::Compute_sqrmean(double xmin, double xmax, double sampling_step, int derivate) const {
    double mret = 0;
    int numpts = 0;
    for (double mx = xmin; mx <= xmax; mx = mx + sampling_step) {
        numpts++;
        mret += pow(this->Get_y_dN(mx, derivate), 2.);
    }
    return sqrt(mret / ((double)numpts));
}

double ChFunction::Compute_int(double xmin, double xmax, double sampling_step, int derivate) const {
    double mret = 0;
    double ya = this->Get_y_dN(xmin, derivate);
    double yb = 0;
    for (double mx = xmin + sampling_step; mx <= xmax; mx += sampling_step) {
        yb = this->Get_y_dN(mx, derivate);
        mret += sampling_step * (ya + yb) * 0.5;  // trapezoidal quadrature
        ya = yb;
    }
    return mret;
}

void ChFunction::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChFunction>();
}

/// Method to allow de serialization of transient data from archives.
void ChFunction::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChFunction>();
}

int ChFunction::FilePostscriptPlot(ChFile_ps* m_file, int plotY, int plotDY, int plotDDY) {
    int mresol = 800;
    ChMatrixDynamic<> yf(mresol, 1);
    ChMatrixDynamic<> xf(mresol, 1);
    double mx, xmin, xmax;
    ChVector2<> mp;

    // xmin
    mp = m_file->Get_G_p();
    mp = m_file->To_graph_from_page(mp);
    xmin = mp.x();
    // xmax
    mp = m_file->Get_G_p();
    mp.x() = mp.x() + m_file->Get_Gs_p().x();
    mp = m_file->To_graph_from_page(mp);
    xmax = mp.x();

    if (plotY) {
        mx = xmin;
        for (int j = 0; j < mresol; j++) {
            mp.x() = mx;
            mp.y() = this->Get_y(mx);
            xf.SetElement(j, 0, mp.x());
            yf.SetElement(j, 0, mp.y());
            mx += ((xmax - xmin) / ((double)mresol - 1.0));
        }
        m_file->DrawGraphXY(&yf, &xf);
    }
    if (plotDY) {
        mx = xmin;
        for (int j = 0; j < mresol; j++) {
            mp.x() = mx;
            mp.y() = this->Get_y_dx(mx);
            xf.SetElement(j, 0, mp.x());
            yf.SetElement(j, 0, mp.y());
            mx += ((xmax - xmin) / ((double)mresol - 1.0));
        }
        m_file->DrawGraphXY(&yf, &xf);
    }
    if (plotDDY) {
        mx = xmin;
        for (int j = 0; j < mresol; j++) {
            mp.x() = mx;
            mp.y() = this->Get_y_dxdx(mx);
            xf.SetElement(j, 0, mp.x());
            yf.SetElement(j, 0, mp.y());
            mx += ((xmax - xmin) / ((double)mresol - 1.0));
        }
        m_file->DrawGraphXY(&yf, &xf);
    }
    return 1;
}

int ChFunction::FileAsciiPairsSave(ChStreamOutAscii& m_file, double mxmin, double mxmax, int msamples) {
    if (msamples <= 1)
        throw(ChException("Warning! too short range or too long sampling period: no points can be saved"));
    if (msamples >= 100000)
        throw(ChException("Warning! Too many points should be saved"));
    if (mxmax <= mxmin)
        throw(ChException("Warning! Cannot save ChFunction if Xmax < Xmin"));

    m_file.SetNumFormat("%0.8f");

    double period = (mxmax - mxmin) / ((double)msamples - 1);

    double mX = mxmin;
    for (int cnt = 1; cnt <= msamples; cnt++) {
        m_file << mX;
        m_file << "    ";
        m_file << this->Get_y(mX);
        m_file.CR();
        mX += period;
    }
    return 1;
}

}  // end namespace chrono