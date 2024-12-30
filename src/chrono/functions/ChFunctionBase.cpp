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

#include <memory>
#include <cfloat>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <iomanip>
#include <limits>

#include "chrono/functions/ChFunctionBase.h"

namespace chrono {

static const double FD_STEP = 1e-7;  // forward differentiation stepsize

// Register into the object factory, to enable run-time dynamic creation and persistence
// CH_FACTORY_REGISTER(ChFunction) // NO! this is an abstract class, rather use for children concrete classes.

class ChFunction_Type_enum_mapper : public ChFunction {
  public:
    CH_ENUM_MAPPER_BEGIN(Type);
    CH_ENUM_VAL(Type::BSPLINE);
    CH_ENUM_VAL(Type::CONSTANT);
    CH_ENUM_VAL(Type::CONSTACC);
    CH_ENUM_VAL(Type::CONSTJERK);
    CH_ENUM_VAL(Type::CUSTOM);
    CH_ENUM_VAL(Type::CYCLOIDAL);
    CH_ENUM_VAL(Type::DERIVATIVE);
    CH_ENUM_VAL(Type::FILLET3);
    CH_ENUM_VAL(Type::INTEGRAL);
    CH_ENUM_VAL(Type::INTERP);
    CH_ENUM_VAL(Type::LAMBDA);
    CH_ENUM_VAL(Type::MIRROR);
    CH_ENUM_VAL(Type::OPERATOR);
    CH_ENUM_VAL(Type::POLY);
    CH_ENUM_VAL(Type::POLY23);
    CH_ENUM_VAL(Type::POLY345);
    CH_ENUM_VAL(Type::RAMP);
    CH_ENUM_VAL(Type::REPEAT);
    CH_ENUM_VAL(Type::SEQUENCE);
    CH_ENUM_VAL(Type::SINE);
    CH_ENUM_VAL(Type::SINE_STEP);
    CH_ENUM_MAPPER_END(Type);
};

double ChFunction::GetDer(double x) const {
    return (GetVal(x + FD_STEP) - GetVal(x)) / FD_STEP;
}

double ChFunction::GetDer2(double x) const {
    return (GetDer(x + FD_STEP) - GetDer(x)) / FD_STEP;
}

double ChFunction::GetDer3(double x) const {
    return ((GetDer2(x + FD_STEP) - GetDer2(x)) / FD_STEP);
}

double ChFunction::GetDerN(double x, int der_order) const {
    switch (der_order) {
        case 0:
            return GetVal(x);
        case 1:
            return GetDer(x);
        case 2:
            return GetDer2(x);
        case 3:
            return GetDer3(x);
        default:
            return GetVal(x);
    }
}

// some analysis functions
double ChFunction::GetMax(double xmin, double xmax, double sampling_step, int derivative) const {
    double mret = std::numeric_limits<double>::min();
    for (double mx = xmin; mx <= xmax; mx += sampling_step) {
        if (this->GetDerN(mx, derivative) > mret)
            mret = this->GetDerN(mx, derivative);
    }
    return mret;
}

double ChFunction::GetMin(double xmin, double xmax, double sampling_step, int derivative) const {
    double mret = std::numeric_limits<double>::max();
    for (double mx = xmin; mx <= xmax; mx += sampling_step) {
        if (this->GetDerN(mx, derivative) < mret)
            mret = this->GetDerN(mx, derivative);
    }
    return mret;
}

double ChFunction::GetMean(double xmin, double xmax, double sampling_step, int derivative) const {
    double mret = 0;
    int numpts = 0;
    for (double mx = xmin; mx <= xmax; mx = mx + sampling_step) {
        numpts++;
        mret += this->GetDerN(mx, derivative);
    }
    return mret / ((double)numpts);
}

double ChFunction::GetSquaredMean(double xmin, double xmax, double sampling_step, int derivative) const {
    double mret = 0;
    int numpts = 0;
    for (double mx = xmin; mx <= xmax; mx = mx + sampling_step) {
        numpts++;
        mret += std::pow(this->GetDerN(mx, derivative), 2.);
    }
    return sqrt(mret / ((double)numpts));
}

double ChFunction::GetIntegral(double xmin, double xmax, double sampling_step, int derivative) const {
    double mret = 0;
    double ya = this->GetDerN(xmin, derivative);
    double yb = 0;
    for (double mx = xmin + sampling_step; mx <= xmax; mx += sampling_step) {
        yb = this->GetDerN(mx, derivative);
        mret += sampling_step * (ya + yb) * 0.5;  // trapezoidal quadrature
        ya = yb;
    }
    return mret;
}

void ChFunction::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChFunction>();
}

/// Method to allow de serialization of transient data from archives.
void ChFunction::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChFunction>();
}

void ChFunction::OutputToASCIIFile(std::ostream& outfile, double xmin, double xmax, int samples, char delimiter) {
    if (samples <= 1)
        throw std::invalid_argument("Not enough samples requested");
    if (xmax <= xmin)
        throw std::invalid_argument("Cannot save ChFunction if Xmax < Xmin");

    outfile << std::setprecision(8) << std::defaultfloat;

    double dx = (xmax - xmin) / (samples - 1);

    double x = xmin;
    for (int i = 1; i <= samples; i++) {
        outfile << x;
        outfile << delimiter;
        outfile << this->GetVal(x);
        outfile << std::endl;
        x += dx;
    }
}

ChMatrixDynamic<> ChFunction::SampleUpToDerN(double xmin, double xmax, double step, int derN) {
    ChMatrixDynamic<> data;
    int num_samples = (xmax - xmin) / step;
    data.resize(num_samples, derN + 2);  // data = [x, y(x), y_dx(x), ...]
    double x = xmin;
    for (int i = 0; i < num_samples; ++i) {
        data(i, 0) = x;
        for (int j = 0; j < derN + 1; ++j) {
            data(i, j + 1) = GetDerN(x, j);
        }
        x += step;
    }
    return data;
}

}  // end namespace chrono