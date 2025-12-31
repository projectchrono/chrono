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

#ifndef CHBEAMSECTIONCABLE_H
#define CHBEAMSECTIONCABLE_H

#include <cmath>

#include "chrono/fea/ChBeamSection.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_utils
/// @{

/// Simplified geometry for a 'cable' beam section in 3D, that is a beam
/// without torsional stiffness and with circular section (i.e.same Ixx and Iyy properties).
/// This material can be shared between multiple beams.
class ChApi ChBeamSectionCable : public ChBeamSection {
  public:
    double Area;
    double I;
    double E;
    double density;
    double rdamping;

    ChBeamSectionCable()
        : E(0.01e9),      // default E stiffness: (almost rubber)
          density(1000),  // default density: water
          rdamping(0.01)  // default Rayleigh damping.
    {
        SetDiameter(0.01);  // defaults Area, I
    }

    virtual ~ChBeamSectionCable() {}

    /// Set the cross sectional area A of the beam (m^2)
    void SetArea(const double ma) { this->Area = ma; }
    double GetArea() const { return this->Area; }

    /// Set the I moment of inertia of the beam (for flexion about y axis or z axis)
    /// Note: since this simple section assumes circular section, Iyy=Izz=I
    void SetInertia(double ma) { this->I = ma; }
    double GetInertia() const { return this->I; }

    /// Shortcut: set Area and I inertia at once,
    /// given the diameter of the beam assumed
    /// with circular shape.
    void SetDiameter(double diameter) {
        this->Area = CH_PI * std::pow((0.5 * diameter), 2);
        this->I = (CH_PI / 4.0) * std::pow((0.5 * diameter), 4);

        this->SetDrawCircularRadius(diameter / 2);
    }

    /// Set the density of the beam (kg/m^3)
    void SetDensity(double md) { this->density = md; }
    double GetDensity() const { return this->density; }

    /// Set E, the Young elastic modulus (N/m^2)
    void SetYoungModulus(double mE) { this->E = mE; }
    double GetYoungModulus() const { return this->E; }

    /// Set the Rayleigh damping ratio r (as in: R = r * K ), to do: also mass-proportional term
    void SetRayleighDamping(double mr) { this->rdamping = mr; }
    double GetRayleighDamping() { return this->rdamping; }
};

/// @} fea_utils

}  // end namespace fea
}  // end namespace chrono

#endif
