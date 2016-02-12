//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//
// File author: Alessandro Tasora

#ifndef CHSHELLSECTION_H
#define CHSHELLSECTION_H

#include "chrono_fea/ChApiFEA.h"

namespace chrono {
namespace fea {

/// Base class for properties of shell sections.
/// This material can be shared between multiple beams.

class ChShellSection {
  public:
    ChShellSection() {}

    virtual ~ChShellSection() {}
};

/// Basic geometry for a shell section in 3D, along with basic material
/// properties (moments of inertia, area, Young modulus, etc.)
/// This material can be shared between multiple beams.

class ChShellSectionBasic : public ChShellSection {
  public:
    double thickness;
    double J;
    double G;
    double E;
    double density;
    double rdamping;

    ChShellSectionBasic() {
        E = 0.01e9;                 // default E stiffness: (almost rubber)
        SetGwithPoissonRatio(0.3);  // default G (low poisson ratio)

        thickness = 0.01;

        density = 1000;   // default density: water
        rdamping = 0.01;  // default raleygh damping.
    }

    virtual ~ChShellSectionBasic() {}

    /// Set the thickness of the shell (m)
    void SetThickness(const double mt) { this->thickness = mt; }
    double GetThickness() const { return this->thickness; }

    /// Set the density of the beam (kg/m^3)
    void SetDensity(double md) { this->density = md; }
    double GetDensity() const { return this->density; }

    /// Set E, the Young elastic modulus (N/m^2)
    void SetYoungModulus(double mE) { this->E = mE; }
    double GetYoungModulus() const { return this->E; }

    /// Set G, the shear modulus
    void SetGshearModulus(double mG) { this->G = mG; }
    double GetGshearModulus() const { return this->G; }

    /// Set G, the shear modulus, given current E and the specified Poisson ratio
    void SetGwithPoissonRatio(double mpoisson) { this->G = this->E / (2.0 * (1.0 + mpoisson)); }

    /// Set the Raleygh damping ratio r (as in: R = r * K ), to do: also mass-proportional term
    void SetBeamRaleyghDamping(double mr) { this->rdamping = mr; }
    double GetBeamRaleyghDamping() { return this->rdamping; }
};

}  // END_OF_NAMESPACE____
}  // END_OF_NAMESPACE____

#endif
