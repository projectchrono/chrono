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

#ifndef CHMATERIAL3DSTRESSKELVINVOIGT_H
#define CHMATERIAL3DSTRESSKELVINVOIGT_H

#include "chrono/fea/ChMaterial3DStress.h"
#include "chrono/core/ChTensors.h"


namespace chrono {
namespace fea {

// Forward:
class ChFieldData;
class ChElementData;


/// @addtogroup chrono_fea
/// @{


/// Class for Kelvin-Voigt damping. This provides damping forces proportional to the spatial velocity
/// gradient. It is assumed to be used in parallel with some elastic stress model like ChMaterial3DStressStVenant.
/// If used alone, it provides a fluid-like viscous damping.

class ChMaterial3DStressKelvinVoigt : public ChMaterial3DStress {
public:

    ChMaterial3DStressKelvinVoigt() : volumetric_damping(0), deviatoric_damping(0) {}

    virtual ~ChMaterial3DStressKelvinVoigt() {}

    /// Set the damping for the deviatoric effect. This corresponds to the viscous
    /// damping μ in Newtonian fluids.
    virtual void SetDeviatoricDamping(double mdamp) { deviatoric_damping = mdamp; }
    virtual double GetDeviatoricDamping() const { return deviatoric_damping; }
    
    /// Set the damping for the volumetric effect. If the material is incompressible or
    /// nearly incompressible, this term has no effect.
    virtual void SetVolumetricDamping(double mdamp) { volumetric_damping = mdamp; }
    virtual double GetVolumetricDamping() const { return volumetric_damping; }

    /// Compute elastic stress from spatial velocity gradient "l"
    
    virtual void ComputeStress(ChStressTensor<>& S_stress,          ///< output stress, PK2
                                const ChMatrix33d& F_def,           ///< current deformation gradient tensor
                                const ChMatrix33d* l,               ///< current spatial velocity gradient (might be nullptr if IsSpatialVelocityGradientNeeded() is false)
                                ChFieldData* data_per_point,        ///< pointer to auxiliary data (ex states), if any, per quadrature point
                                ChElementData* data_per_element     ///< pointer to auxiliary data (ex states), if any, per element 
    ) {
        // compute d, rate of deformation tensor:
        ChMatrix33d d = 0.5 * (*l + l->transpose());  
        
        // Cauchy σ = ζtr(d)I + 2μd
        ChMatrix33d sigma = this->volumetric_damping * d.trace() * Eigen::Matrix3d::Identity()  
                            + 2.0 * d;

        // Cauchy σ -> PK2 stress S
        // S = J F⁻¹ (ζ tr(d) I + 2μ d) F⁻ᵀ
        double J = F_def.determinant();
        ChMatrix33d F_inv = F_def.inverse();
        ChMatrix33d F_inv_T = F_inv.transpose(); // transpose of F_inv (F^{-T})
        ChMatrix33d S = J * F_inv * sigma * F_inv_T;
        S_stress.ConvertFromMatrix(S);
    }

    /// Computes the tangent modulus 

    virtual void ComputeTangentModulus(ChMatrixNM<double, 6, 6>& C, ///< output C tangent modulus, as dS=C*dE
                                const ChMatrix33d& F_def,       ///< current deformation gradient tensor
                                const ChMatrix33d* l,           ///< current spatial velocity gradient (might be nullptr if IsSpatialVelocityGradientNeeded() is false)
                                ChFieldData* data_per_point,    ///< pointer to auxiliary data (ex states), if any, per quadrature point
                                ChElementData* data_per_element ///< pointer to auxiliary data (ex states), if any, per element 
    ) {
        C.setZero();
        //***TODO***
    }

    /// Update your own auxiliary data, if any, at the end of time step (ex for plasticity).
    /// This is called at the end of every time step (or nl static step)
    virtual void ComputeUpdateEndStep(ChFieldData* data_per_point,          ///< pointer to auxiliary data (ex states), if any, per quadrature point
                                      ChElementData* data_per_element,      ///< pointer to auxiliary data (ex states), if any, per element 
                                      const double time
    ) {
         // default: do nothing. 
    }

    /// This material need info on the spatial velocity gradient  l=\nabla_x v 
    virtual bool IsSpatialVelocityGradientNeeded() const {
        return true;
    }

    //virtual void ArchiveOut(ChArchiveOut& archive_out) override;  TODO
    //virtual void ArchiveIn(ChArchiveIn& archive_in) override; 

private:
    double volumetric_damping;
    double deviatoric_damping;
};




/// @} chrono_fea

}  // end namespace fea

}  // end namespace chrono

#endif
