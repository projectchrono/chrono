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

#ifndef CH_BEAM_SECTION_CABLE_H
#define CH_BEAM_SECTION_CABLE_H

#include <cmath>

#include "chrono/fea/ChBeamSection.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_utils
/// @{

/// Simplified geometry for a cable beam section in 3D.
/// This is for a beam without torsional stiffness and with assumed circular section.
/// These properties can be shared between multiple beams.
class ChApi ChBeamSectionCableANCF : public ChBeamSection {
  public:
    /// Create a cable section with default properties.
    ChBeamSectionCableANCF();

    virtual ~ChBeamSectionCableANCF() {}

    /// Set the cross sectional area of the beam.
    void SetArea(const double ma) { Area = ma; }
    double GetArea() const { return Area; }

    /// Set the moment of inertia of the beam (for flexion about y axis or z axis).
    /// This function assumes a circular section shape.
    void SetInertia(double ma) { I = ma; }
    double GetInertia() const { return I; }

    /// Specify the cable diameter.
    /// This function sets the area and inertia, assuming a circular section shape.
    void SetDiameter(double diameter);

    /// Set the density of the beam.
    void SetDensity(double md) { density = md; }
    double GetDensity() const { return density; }

    /// Set the Young elastic modulus.
    void SetYoungModulus(double mE) { E = mE; }
    double GetYoungModulus() const { return E; }

    //// TODO also mass-proportional term

    /// Set the Rayleigh damping ratio r (as in: R = r * K ).
    void SetRayleighDamping(double mr) { rdamping = mr; }
    double GetRayleighDamping() const { return rdamping; }

    // Data

    double Area;
    double I;
    double E;
    double density;
    double rdamping;
};

/// @} fea_utils

}  // end namespace fea
}  // end namespace chrono

#endif
