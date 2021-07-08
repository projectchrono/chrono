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

    // To store the shape function matrix 'Nx' and strain-displacement relation matrix 'Bx'
    using ShapeFunctionGroupFPM = std::tuple<ChMatrixNM<double, 6, 12>,ChMatrixNM<double, 6, 12>>;

    ChElementBeamTaperedTimoshenkoFPM();  // use the construction function of base class directly

    ~ChElementBeamTaperedTimoshenkoFPM() {}
  
    //
    // FEM functions
    // 

    /// Set the section & material of beam element .
    /// It is a shared property, so it can be shared between other beams.
    void SetTaperedSection(std::shared_ptr<ChBeamSectionTaperedTimoshenkoAdvancedGenericFPM> my_material) {
        this->tapered_section_fpm = my_material;
        this->tapered_section = std::dynamic_pointer_cast<ChBeamSectionTaperedTimoshenkoAdvancedGeneric>(my_material);
    }
    /// Get the section & material of the element
    std::shared_ptr<ChBeamSectionTaperedTimoshenkoAdvancedGenericFPM> GetTaperedSection() {
        return this->tapered_section_fpm;
    }

    // Shape functions and strain-displacement matrix at position eta for FPM Timoshenko beam
    void ShapeFunctionsTimoshenkoFPM(ShapeFunctionGroupFPM& NB, double eta);

    void SetIntegrationPoints(int mv) { this->guass_order = mv; };
    int GetIntegrationPoints() { return this->guass_order; };

    /// Computes the local (material) stiffness matrix of the element:
    /// K = integral( [B]' * [D] * [B] ),
    /// Note: in this 'basic' implementation, constant section and
    /// constant material are assumed, so the explicit result of quadrature is used.
    /// Also, this local material stiffness matrix is constant, computed only at the beginning
    /// for performance reasons; if you later change some material property, call this or InitialSetup().
    void ComputeStiffnessMatrix0();  // discarded, it 's wrong

    void ComputeDampingMatrix0();  // discarded, it 's wrong
    
    // compute the local element stiffness matrix via Guass Quadrature
    void ComputeStiffnessMatrix();

    // Although the shape functions of this beam element have updated, 
    // strictly speaking, we need to update the algorithm of geometric stiffness matrix correspondingly,
    // but this geometric stiffness matrix is already a higher-order correction term,
    // we can directly inherit the previous method for the sake of simplicity.
    // The influence of different shape functions should be very limited.
    // void ComputeGeometricStiffnessMatrix();

    // compute the local element damping matrix via Guass Quadrature
    void ComputeDampingMatrix();

    // compute the local element consistent mass matrix via Guass Quadrature
    void ComputeConsistentMassMatrix();

    // finally, choose the mass matrix of local element, 
    // depending on 'use_lumped_mass_matrix' in its sectional settings
    void ComputeMassMatrix();


    /// Gets the xyz displacement of a point on the beam line,
    /// and the rotation RxRyRz of section plane, at abscyssa 'eta'.
    /// Note, eta=-1 at node1, eta=+1 at node2.
    /// Results are not corotated.
    virtual void EvaluateSectionDisplacement(const double eta, ChVector<>& u_displ, ChVector<>& u_rotaz) override;


    /// Gets the force (traction x, shear y, shear z) and the
    /// torque (torsion on x, bending on y, on bending on z) at a section along
    /// the beam line, at abscissa 'eta'.
    /// Note, eta=-1 at node1, eta=+1 at node2.
    /// Results are not corotated, and are expressed in the reference system of beam.
    virtual void EvaluateSectionForceTorque(const double eta, ChVector<>& Fforce, ChVector<>& Mtorque) override;


    /* To be completed: Created to be consistent with base class implementation*/
    // strain_v = Bx * displ
    virtual void EvaluateSectionStrain(const double eta, ChVector<>& StrainV) override {}


    /// Evaluate N'*F , where N is some type of shape function
    /// evaluated at U coordinates of the line, each ranging in -1..+1
    /// F is a load, N'*F is the resulting generalized load
    /// Returns also det[J] with J=[dx/du,..], that might be useful in gauss quadrature.
    virtual void ComputeNF(const double U,              ///< parametric coordinate in line
                           ChVectorDynamic<>& Qi,       ///< Return result of Q = N'*F  here
                           double& detJ,                ///< Return det[J] here
                           const ChVectorDynamic<>& F,  ///< Input F vector, size is =n. field coords.
                           ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate Q
                           ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate Q
                           ) override;

    /// Evaluate N'*F , where N is some type of shape function
    /// evaluated at U,V,W coordinates of the volume, each ranging in -1..+1
    /// F is a load, N'*F is the resulting generalized load
    /// Returns also det[J] with J=[dx/du,..], that might be useful in gauss quadrature.
    virtual void ComputeNF(const double U,              ///< parametric coordinate in volume
                           const double V,              ///< parametric coordinate in volume
                           const double W,              ///< parametric coordinate in volume
                           ChVectorDynamic<>& Qi,       ///< Return result of N'*F  here, maybe with offset block_offset
                           double& detJ,                ///< Return det[J] here
                           const ChVectorDynamic<>& F,  ///< Input F vector, size is = n.field coords.
                           ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate Q
                           ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate Q
                           ) override;

  private:

    /// Initial setup. Precompute mass and matrices that do not change during the simulation, such as the local tangent
    /// stiffness Kl of each element, if needed, etc.
    virtual void SetupInitial(ChSystem* system) override;

    std::shared_ptr<ChBeamSectionTaperedTimoshenkoAdvancedGenericFPM> tapered_section_fpm;
    
    // the order of Gauss-Legendre quadrature for local element stiffness/damping/mass matrices 
    int guass_order = 4;  // be careful of shear locking effect
  
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// @} fea_elements

}  // end namespace fea
}  // end namespace chrono

#endif
