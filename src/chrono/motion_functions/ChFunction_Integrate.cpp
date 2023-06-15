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

#include "chrono/motion_functions/ChFunction_Integrate.h"
#include "chrono/motion_functions/ChFunction_Const.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunction_Integrate)

ChFunction_Integrate::ChFunction_Integrate() : order(1), C_start(0), x_start(0), x_end(1), num_samples(2000) {
    fa = chrono_types::make_shared<ChFunction_Const>();  // default
    array_x.resize(num_samples);
}

ChFunction_Integrate::ChFunction_Integrate(const ChFunction_Integrate& other) {
    fa = std::shared_ptr<ChFunction>(other.fa->Clone());
    order = other.order;
    C_start = other.C_start;
    x_start = other.x_start;
    x_end = other.x_end;
    num_samples = other.num_samples;
    array_x = other.array_x;
}

void ChFunction_Integrate::ComputeIntegral() {
    double mstep = (x_end - x_start) / ((double)(num_samples - 1));
    double x_a, x_b, y_a, y_b, F_b;

    double F_sum = this->Get_C_start();

    array_x(0) = this->Get_C_start();

    for (int i = 1; i < this->num_samples; i++) {
        x_b = x_start + ((double)i) * (mstep);
        x_a = x_b - mstep;
        y_a = this->fa->Get_y(x_a);
        y_b = this->fa->Get_y(x_b);
        // trapezoidal rule..
        F_b = F_sum + mstep * (y_a + y_b) * 0.5;
        array_x(i) = F_b;
        F_sum = F_b;
    }
}

double ChFunction_Integrate::Get_y(double x) const {
    if ((x < x_start) || (x > x_end))
        return 0.0;
    int i_a, i_b;
    double position = (double)(num_samples - 1) * ((x - x_start) / (x_end - x_start));
    i_a = (int)(floor(position));
    i_b = i_a + 1;

    if (i_b > num_samples - 1)
        return array_x(num_samples - 1);

    if (i_a < 0)
        return array_x(0);

    double weightB = position - (double)i_a;
    double weightA = 1 - weightB;

    return (weightA * (array_x(i_a)) + weightB * (array_x(i_b)));
}

void ChFunction_Integrate::Estimate_x_range(double& xmin, double& xmax) const {
    xmin = x_start;
    xmax = x_end;
}

void ChFunction_Integrate::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChFunction_Integrate>();
    // serialize parent class
    ChFunction::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(fa);
    marchive << CHNVP(order);
    marchive << CHNVP(C_start);
    marchive << CHNVP(x_start);
    marchive << CHNVP(x_end);
    marchive << CHNVP(num_samples);
}

void ChFunction_Integrate::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChFunction_Integrate>();
    // deserialize parent class
    ChFunction::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(fa);
    marchive >> CHNVP(order);
    marchive >> CHNVP(C_start);
    marchive >> CHNVP(x_start);
    marchive >> CHNVP(x_end);
    marchive >> CHNVP(num_samples);
    array_x.setZero(num_samples);
    ComputeIntegral();
}

}  // end namespace chrono
