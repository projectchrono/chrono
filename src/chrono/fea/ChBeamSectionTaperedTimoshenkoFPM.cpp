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
// Authors: Alessandro Tasora
// =============================================================================

#include "chrono/fea/ChBeamSectionTaperedTimoshenkoFPM.h"

namespace chrono {
namespace fea {

void ChBeamSectionTaperedTimoshenkoAdvancedGenericFPM::ComputeAverageFPM() {
    // the elements off the diagonal are averaged by arithmetic mean
    this->average_fpm = (this->section_fpmA->GetFullyPopulatedMaterialStiffnessMatrix() +
                         this->section_fpmB->GetFullyPopulatedMaterialStiffnessMatrix()) *
                        0.5;

    // The diagonal terms are averaged by geometrical mean, to be consistent with previous algorithm
    this->average_fpm(0, 0) = this->avg_sec_par->EA;
    this->average_fpm(1, 1) = this->avg_sec_par->GAyy;
    this->average_fpm(2, 2) = this->avg_sec_par->GAzz;
    this->average_fpm(3, 3) = this->avg_sec_par->GJ;
    this->average_fpm(4, 4) = this->avg_sec_par->EIyy;
    this->average_fpm(5, 5) = this->avg_sec_par->EIzz;
}

void ChBeamSectionTaperedTimoshenkoAdvancedGenericFPM::ComputeInertiaMatrix(ChMatrixDynamic<>& M) {
    // Inherit from base class
    ChBeamSectionTaperedTimoshenkoAdvancedGeneric::ComputeInertiaMatrix(M);

    // FPM doesnot influence the Inertia(/Mass) matrix of beam element,
    // so it can be evaluated after calculating the inertia matrix
    ComputeAverageFPM();
}

}  // end namespace fea
}  // end namespace chrono
