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

#ifndef CHCONTINUUMELECTROSTATICS_H
#define CHCONTINUUMELECTROSTATICS_H

#include "chrono_fea/ChContinuumPoisson3D.h"

namespace chrono {
namespace fea {

/// Class for material for FEA electrostatic problems.

class ChContinuumElectrostatics : public ChContinuumPoisson3D {
  private:
    double e_permittivity;

  public:
    ChContinuumElectrostatics() : e_permittivity(1) {}
    ChContinuumElectrostatics(const ChContinuumElectrostatics& other) : ChContinuumPoisson3D(other) {
        e_permittivity = other.e_permittivity;
    }
    virtual ~ChContinuumElectrostatics() {}

    /// Sets the e absolute permittivity constant of the material,
    /// expressed in Faraday per meter  [ F/m ].
    /// Sets the conductivity matrix as isotropic (diagonal k)
    void SetPermittivity(double me) {
        e_permittivity = me;
        ConstitutiveMatrix.Reset();
        ConstitutiveMatrix.FillDiag(e_permittivity);
    }

    /// Sets the e absolute permittivity by using the relative
    /// permittivity er, since e = er*e0 with e0 the constant permittivity
    /// of vacuum, so this is easier than using SetPermittivity(),
    /// Examples: air er=1.000589, rubber er=7, water er=80, glass 4-10 (approx.)
    void SetRelativePermettivity(double re) { SetPermittivity(re * 8.854187817e-12); }

    /// Gets the e absolute permittivity constant of the material,
    /// expressed in Faraday per meter  [ F/m ].
    double GetPermittivity() const { return e_permittivity; }

    /// Get the e permittivity matrix
    ChMatrixDynamic<> Get_PermittivityEmatrix() { return ConstitutiveMatrix; }
};

}  // end namespace fea
}  // end namespace chrono

#endif
