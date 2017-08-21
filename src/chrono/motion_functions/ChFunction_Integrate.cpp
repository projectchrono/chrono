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

#include "chrono/motion_functions/ChFunction_Const.h"
#include "chrono/motion_functions/ChFunction_Integrate.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunction_Integrate)

ChFunction_Integrate::ChFunction_Integrate() : order(1), C_start(0), x_start(0), x_end(1), num_samples(2000) {
    fa = std::make_shared<ChFunction_Const>();  // default
    array_x = new ChMatrixDynamic<>(num_samples, 1);
}

ChFunction_Integrate::ChFunction_Integrate(const ChFunction_Integrate& other) {
    fa = std::shared_ptr<ChFunction>(other.fa->Clone());
    order = other.order;
    C_start = other.C_start;
    x_start = other.x_start;
    x_end = other.x_end;
    num_samples = other.num_samples;
    array_x->CopyFromMatrix(*other.array_x);
}

ChFunction_Integrate::~ChFunction_Integrate() {
    if (array_x)
        delete array_x;
}

void ChFunction_Integrate::ComputeIntegral() {
    double mstep = (x_end - x_start) / ((double)(num_samples - 1));
    double x_a, x_b, y_a, y_b, F_b;

    double F_sum = this->Get_C_start();

    this->array_x->SetElement(0, 0, this->Get_C_start());

    for (int i = 1; i < this->num_samples; i++) {
        x_b = x_start + ((double)i) * (mstep);
        x_a = x_b - mstep;
        y_a = this->fa->Get_y(x_a);
        y_b = this->fa->Get_y(x_b);
        // trapezoidal rule..
        F_b = F_sum + mstep * (y_a + y_b) * 0.5;
        this->array_x->SetElement(i, 0, F_b);
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

    if (i_a == num_samples - 1)
        return array_x->GetElement(num_samples - 1, 0);

    if ((i_a < 0) || (i_b >= num_samples))
        return 0.0;

    double weightB = position - (double)i_a;
    double weightA = 1 - weightB;

    return (weightA * (array_x->GetElement(i_a, 0)) + weightB * (array_x->GetElement(i_b, 0)));
}

void ChFunction_Integrate::Estimate_x_range(double& xmin, double& xmax) const {
    xmin = x_start;
    xmax = x_end;
}

}  // end namespace chrono
