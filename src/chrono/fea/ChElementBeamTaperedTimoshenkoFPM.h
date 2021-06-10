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

#ifndef CHELEMENTBEAMTAPEREDTIMOSHENKOFPM_H
#define CHELEMENTBEAMTAPEREDTIMOSHENKOFPM_H

#include "chrono/fea/ChBeamSectionTaperedTimoshenkoFPM.h"
#include "chrono/fea/ChElementBeamTaperedTimoshenko.h"
#include "chrono/fea/ChElementBeam.h"
#include "chrono/fea/ChElementCorotational.h"
#include "chrono/fea/ChNodeFEAxyzrot.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_elements
/// @{

/// Simple beam element with two nodes and Euler-Bernoulli formulation.
/// For this 'basic' implementation, constant section and constant
/// material are assumed.
///
/// Further information in the
/// [white paper PDF](http://www.projectchrono.org/assets/white_papers/euler_beams.pdf)
///
/// Note that there are also ChElementCableANCF if no torsional effects
/// are needed, as in cables.

class ChApi ChElementBeamTaperedTimoshenkoFPM : public ChElementBeamTaperedTimoshenko {
  public:

    //ChElementBeamTaperedTimoshenkoFPM();

    ~ChElementBeamTaperedTimoshenkoFPM() {}

    //
    // FEM functions
    // 

    /// Set the section & material of beam element .
    /// It is a shared property, so it can be shared between other beams.
    void SetTaperedSection(std::shared_ptr<ChBeamSectionTaperedTimoshenkoAdvancedGenericFPM> my_material) {
        this->taperedSectionFPM = my_material;
        this->taperedSection = std::dynamic_pointer_cast<ChBeamSectionTaperedTimoshenkoAdvancedGeneric>(my_material);
    }
    /// Get the section & material of the element
    std::shared_ptr<ChBeamSectionTaperedTimoshenkoAdvancedGenericFPM> GetTaperedSection() {
        return this->taperedSectionFPM;
    }

    /// Computes the local (material) stiffness matrix of the element:
    /// K = integral( [B]' * [D] * [B] ),
    /// Note: in this 'basic' implementation, constant section and
    /// constant material are assumed, so the explicit result of quadrature is used.
    /// Also, this local material stiffness matrix is constant, computed only at the beginning
    /// for performance reasons; if you later change some material property, call this or InitialSetup().
    void ComputeStiffnessMatrix();


    void ComputeDampingMatrix();

    /// Gets the force (traction x, shear y, shear z) and the
    /// torque (torsion on x, bending on y, on bending on z) at a section along
    /// the beam line, at abscissa 'eta'.
    /// Note, eta=-1 at node1, eta=+1 at node2.
    /// Results are not corotated, and are expressed in the reference system of beam.
    virtual void EvaluateSectionForceTorque(const double eta, ChVector<>& Fforce, ChVector<>& Mtorque) override;


    /* To be completed: Created to be consistent with base class implementation*/
    virtual void EvaluateSectionStrain(const double eta, ChVector<>& StrainV) override {}


  private:
    /// Initial setup. Precompute mass and matrices that do not change during the simulation, such as the local tangent
    /// stiffness Kl of each element, if needed, etc.
    virtual void SetupInitial(ChSystem* system) override;

    std::shared_ptr<ChBeamSectionTaperedTimoshenkoAdvancedGenericFPM> taperedSectionFPM;

};

/// @} fea_elements

}  // end namespace fea
}  // end namespace chrono

#endif
