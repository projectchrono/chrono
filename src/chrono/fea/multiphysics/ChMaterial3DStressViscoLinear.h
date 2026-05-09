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

#ifndef CHMATERIAL3DSTRESSVISCOLINEAR_H
#define CHMATERIAL3DSTRESSVISCOLINEAR_H

#include "chrono/fea/multiphysics/ChMaterial3DStress.h"
#include "chrono/core/ChTensors.h"


namespace chrono {
namespace fea {

// Forward:
class ChFieldData;
class ChElementData;


/// @addtogroup chrono_fea
/// @{


/// Class for linear damping (Newton damping in lagrangian solid, as S=D:E_dot where E_dot is the 
/// time derivative of Green Lagrange strain and D is a constant tensor build from viscosity coefficients, 
/// or more in detail:  S = lambda tr(E_dot) I + 2μ E_dot  ). 
/// This is a bit different from the ChMaterial3DStressViscoNewton, which provides damping forces proportional 
/// to the spatial velocity, as in CFD fluids. But this is faster to compute.
/// It is assumed to be used in parallel with some elastic stress model like ChMaterial3DStressStVenant.
/// If used alone, it provides a fluid-like viscous damping.

class ChMaterial3DStressViscoLinear : public ChMaterial3DStress {
public:

    ChMaterial3DStressViscoLinear() : volumetric_damping(0), deviatoric_damping(0) {}

    virtual ~ChMaterial3DStressViscoLinear() {}

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
                                const ChMatrix33d& F,               ///< current deformation gradient tensor
                                const ChMatrix33d* l,               ///< current spatial velocity gradient (might be nullptr if IsSpatialVelocityGradientNeeded() is false)
                                ChFieldData* data_per_point,        ///< pointer to auxiliary data (ex states), if any, per quadrature point
                                ChElementData* data_per_element     ///< pointer to auxiliary data (ex states), if any, per element 
    ) override {
        // compute E_dot, time derivative of Green Lagrange strain:
        ChMatrix33d F_dot = (*l) * F;                                               // F_dot = l F
        ChMatrix33d E_dot = 0.5 * (F.transpose() * F_dot + F_dot.transpose() * F);  // E_dot = 0.5 (F^T F_dot + F_dot^T F)

        // Lamé - type viscous law,   S = lambda tr(E_dot) I + 2μ E_dot
        ChMatrix33d S = this->volumetric_damping * E_dot.trace() * ChMatrix33d::Identity()                            
                             + 2.0 * this->deviatoric_damping * E_dot;  

        S_stress.ConvertFromMatrix(S);
    }

    /// Computes the tangent modulus 

    virtual void ComputeTangentModulus(ChMatrixNM<double, 6, 6>& C, ///< output C tangent modulus, as dS=C*dE
                                       ChMatrixNM<double, 6, 6>* D, ///< output D tangent modulus, as dS=C*d(E_dot) (maybe nullptr if IsSpatialVelocityGradientNeeded() is false)
                                const ChMatrix33d& F_def,       ///< current deformation gradient tensor
                                const ChMatrix33d* l,           ///< current spatial velocity gradient (might be nullptr if IsSpatialVelocityGradientNeeded() is false)
                                ChFieldData* data_per_point,    ///< pointer to auxiliary data (ex states), if any, per quadrature point
                                ChElementData* data_per_element ///< pointer to auxiliary data (ex states), if any, per element 
    ) override {
        C.setZero();
        assert(D);
        D->setZero();

        const double lambda_v = this->volumetric_damping;
        const double mu_v = this->deviatoric_damping;

        // Normal components
        (*D)(0, 0) = lambda_v + 2.0 * mu_v;
        (*D)(1, 1) = lambda_v + 2.0 * mu_v;
        (*D)(2, 2) = lambda_v + 2.0 * mu_v;

        (*D)(0, 1) = lambda_v;
        (*D)(0, 2) = lambda_v;
        (*D)(1, 0) = lambda_v;
        (*D)(1, 2) = lambda_v;
        (*D)(2, 0) = lambda_v;
        (*D)(2, 1) = lambda_v;

        // Shear components
        (*D)(3, 3) = mu_v;
        (*D)(4, 4) = mu_v;
        (*D)(5, 5) = mu_v;
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
