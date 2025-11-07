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

#ifndef CHMATERIAL3DTHERMAL_H
#define CHMATERIAL3DTHERMAL_H

#include "chrono/fea/ChMaterialPoisson.h"


namespace chrono {
namespace fea {

/// @addtogroup chrono_fea
/// @{

// Forward:
class ChFieldData;
class ChElementData;


/// Base class for materials about thermal problems, for FEA problems involving temperature, heat flow, etc.
/// It contains properties and constitutive laws for thermal problems PDEs of the type 
///    Z * dT/dt + div(q_flux) = q_source
/// Children classes must implement the interface  q_flux = f(grad_T,T), to [Z] = f(grad_T,T) 
/// and to the tangent modulus [C] in the linearization of q_flux.
/// The q_source is assumed provided by optional ChLoaderHeatVolumetricSource, and same for boundary conditions. 

class ChApi ChMaterial3DThermal : public ChMaterialPoisson {
public:

    ChMaterial3DThermal() {}

    virtual ~ChMaterial3DThermal() {}


    /// Compute heat flux vector q_flux from a temperature gradient grad_T
    ///  q_flux = f(grad_T)
    /// This MUST be implemented by children classes.

    virtual void ComputeHeatFlux(ChVector3d& q_flux, ///< output heat flux
        const ChVector3d  grad_T,           ///< current temperature gradient 
        const double  T,                    ///< current temperature 
        ChFieldData* data_per_point,        ///< pointer to auxiliary data (ex states), if any, per quadrature point
        ChElementData* data_per_element     ///< pointer to auxiliary data (ex states), if any, per element 
    ) = 0;

    /// Computes the tangent modulus [C] in the linearization of q_flux = f(grad_T)  about temperature:
    ///   δq_flux = [C] δgrad_T
    /// This MUST be implemented by children classes. It is also diffusivity matrix in a diffusion PDE.

    virtual void ComputeTangentModulus(ChMatrix33d& tangent_matrix, ///< output [C] tangent as δq_flux = [C] δgrad_T
        const ChVector3d  grad_T,       ///< current temperature gradient
        const double  T,                ///< current temperature 
        ChFieldData* data_per_point,    ///< pointer to auxiliary data (ex states), if any, per quadrature point
        ChElementData* data_per_element ///< pointer to auxiliary data (ex states), if any, per element 
    ) = 0;

    /// Computes the  Z  multiplier in  Z*dT/dt + div(q_flux) = q_source:
    /// This MUST be implemented by children classes.
    
    virtual void ComputeDtMultiplier(double& Dt_multiplier,   ///< the Z  term  in Z*dT/dt + div(q_flux) = q_source
        const double  T,                ///< current temperature (not used here)
        ChFieldData* data_per_point,    ///< pointer to auxiliary data (ex states), if any, per quadrature point
        ChElementData* data_per_element ///< pointer to auxiliary data (ex states), if any, per element 
    ) = 0;

};




/// @} chrono_fea

}  // end namespace fea

}  // end namespace chrono

#endif
