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

/// For composite beams such as wind turbine blades and helicopter rotor blades, the cross-sectional
/// stiffness properties in axial, shear, bending and torsion directions are coupled with each other,
/// hence the fully-populated matrix(FPM) of cross-sectional stiffness properties is used to describe
/// this complex coupling. The shape functions of classical Timoshenko beam are not applicable for
/// this new Timoshenko beam. In this tapered Timoshenko beam, the shape functions are numerically
/// derived according to the cross-sectional fully-populated stiffness matrix(FPM), and the local mass,
/// stiffness and damping matrices(M K R) are evaluated via Gauss quadrature. When the fully-populated
/// stiffness matrix(FPM) includes only diagonal elements(no coupling in the cross-sectional stiffness
/// matrix, such as a circular section made from steel), the shape functions are reduced to the occasion
/// in classical Timoshenko beam as in ChElementBeamTaperedTimoshenko, then ChElementBeamTaperedTimoshenkoFPM
/// is equal to ChElementBeamTaperedTimoshenko.
/// Note that there are also ChElementCableANCF if no torsional effects
/// are needed, as in cables.

class ChApi ChElementBeamTaperedTimoshenkoFPM : public ChElementBeamTaperedTimoshenko {
  public:
    // To store the shape function matrix 'Nx' and strain-displacement relation matrix 'Bx'
    using ShapeFunctionGroupFPM = std::tuple<ChMatrixNM<double, 6, 12>, ChMatrixNM<double, 6, 12>>;

    ChElementBeamTaperedTimoshenkoFPM();

    ~ChElementBeamTaperedTimoshenkoFPM() {}

    //
    // FEM functions
    //

    /// Set the tapered section & material of beam element.
    void SetTaperedSection(std::shared_ptr<ChBeamSectionTaperedTimoshenkoAdvancedGenericFPM> my_material) {
        this->tapered_section_fpm = my_material;
        this->tapered_section = std::dynamic_pointer_cast<ChBeamSectionTaperedTimoshenkoAdvancedGeneric>(my_material);
    }
    /// Get the tapered section & material of the element.
    std::shared_ptr<ChBeamSectionTaperedTimoshenkoAdvancedGenericFPM> GetTaperedSection() {
        return this->tapered_section_fpm;
    }

    /// Computes the shape function matrix 'Nx' and strain-displacement relation matrix 'Bx' at dimensionless abscissa 'eta'.
    /// Note, eta=-1 at node1, eta=+1 at node2.
    void ShapeFunctionsTimoshenkoFPM(ShapeFunctionGroupFPM& NB,  ///< shape function matrix 'Nx' and strain-displacement relation matrix 'Bx' are stored here.
                                                    double eta   ///< abscissa 'eta'. eta=-1 at node1, eta=+1 at node2.
    );

    /// Set the order of Gauss quadrature, as default it is four.
    void SetIntegrationPoints(int mv) { this->guass_order = mv; };
    /// Get the order of Gauss quadrature.
    int GetIntegrationPoints() { return this->guass_order; };

    /// Computes the local (material) stiffness matrix of the element:
    /// K = integral( [B]' * [D] * [B] ),
    /// Note: the sectional properties at Gauss integration point are linearly interpolated from two
    /// ends of tapered beam. Also, this local material stiffness matrix is constant, computed only
    /// at the beginning for performance reasons; if you later change some material property, call
    /// this or InitialSetup().
    void ComputeStiffnessMatrix();

    // Since the shape functions of this beam element have updated, strictly speaking, we need to
    // update the algorithm of geometric stiffness matrix correspondingly, but this geometric
    // stiffness matrix is already a higher-order correction term, we can directly inherit the
    // previous method for the sake of simplicity. The influence of different shape functions
    // should be very limited.
    // void ComputeGeometricStiffnessMatrix();

    /// Computes the local element damping matrix via Guass Quadrature:
    /// R = beta * integral( [B]' * [D] * [B] ),
    /// Note: the sectional properties at Gauss integration point are linearly interpolated from two
    /// ends of tapered beam. Only the stiffness term(beta) is used for this implemented Rayleigh
    /// damping model. Also, this local material damping matrix is constant, computed only at the beginning
    /// for performance reasons; if you later change some material property, call this or InitialSetup().
    void ComputeDampingMatrix();

    /// Computes the local element consistent mass matrix via Guass Quadrature:
    /// M = integral( [N]' * [D] * [N] ),
    /// Note: the sectional properties at Gauss integration point are linearly interpolated from two
    /// ends of tapered beam. Also, this local element consistent mass matrix is constant, computed only
    /// at the beginning for performance reasons; if you later change some material property, call this
    /// or InitialSetup().
    void ComputeConsistentMassMatrix();

    /// Finally, compute the local mass matrix of element. It could be in lumped or consistent format,
    /// depending on the parameter 'use_lumped_mass_matrix' in its sectional settings.
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
    
    /// Gets the strains(traction along x, shear along y, along shear z, torsion about x, bending about y, on bending
    /// about z) at a section along the beam line, at abscissa 'eta'. It's evaluated at the elastic center. Note, eta=-1
    /// at node1, eta=+1 at node2. Results are not corotated, and are expressed in the reference system of beam.
    virtual void EvaluateSectionStrain(const double eta, ChVector<>& StrainV_trans, ChVector<>& StrainV_rot) override;

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

    /// Tapered section & material of beam element, in fully-populated matrix(FPM) format.
    std::shared_ptr<ChBeamSectionTaperedTimoshenkoAdvancedGenericFPM> tapered_section_fpm;

    /// The order of Gauss-Legendre quadrature for local element stiffness/damping/mass matrices,
    /// as default, it is four.
    int guass_order = 4;  // be careful of shear locking effect

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// @} fea_elements

}  // end namespace fea
}  // end namespace chrono

#endif
