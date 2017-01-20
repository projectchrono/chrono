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

#include "chrono/motion_functions/ChFunction_Oscilloscope.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunction_Oscilloscope)

ChFunction_Oscilloscope::ChFunction_Oscilloscope(const ChFunction_Oscilloscope& other) {
    values = other.values;
    dx = other.dx;
    end_x = other.end_x;
    amount = other.amount;
    max_amount = other.max_amount;
}

void ChFunction_Oscilloscope::Estimate_x_range(double& xmin, double& xmax) const {
    xmin = this->end_x - this->dx * (this->amount - 1);
    xmax = this->end_x;
    if (xmin >= xmax)
        xmax = xmin + 0.5;
}

void ChFunction_Oscilloscope::AddLastPoint(double mx, double my) {
    if (mx < end_x)
        this->Reset();
    this->end_x = mx;
    this->values.push_back(my);
    if (this->amount < this->max_amount)
        this->amount++;
    else
        this->values.pop_front();

    assert(this->values.size() == this->amount);
}

double ChFunction_Oscilloscope::Get_y(double x) const {
    double y = 0;

    double start_x = this->end_x - this->dx * (this->amount - 1);
    if (x > end_x)
        return 0;
    if (x < start_x)
        return 0;

    int i1 = (int)floor((x - start_x) / this->dx);
    int i2 = i1 + 1;
    double p1x = start_x + dx * (double)i1;
    double p2x = start_x + dx * (double)i2;
    double p2y, p1y = 0;
    int count = 0;
    std::list<double>::const_iterator iter = values.begin();
    while (iter != values.end()) {
        if (count == i1) {
            p2y = *iter;
            iter++;
            p1y = *iter;
            break;
        }
        count++;
        iter++;
    }

    y = ((x - p1x) * p2y + (p2x - x) * p1y) / (p2x - p1x);

    return y;
}

}  // end namespace chrono
