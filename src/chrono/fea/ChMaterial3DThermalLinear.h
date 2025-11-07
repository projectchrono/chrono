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

#ifndef CHMATERIAL3DTHERMALLINEAR_H
#define CHMATERIAL3DTHERMALLINEAR_H

#include "chrono/fea/ChMaterial3DThermal.h"


namespace chrono {
namespace fea {

/// @addtogroup chrono_fea
/// @{


/// Class for properties of a linear thermal material in finite element problems. This is the simplest case:
/// it contains constant properties (c specific heat, k thermal conductivity, rho density) 
/// for the linear thermal problems PDEs of the type 
///    (c*rho) dT/dt + div (-[k]*grad T) = q_source

class ChApi ChMaterial3DThermalLinear : public ChMaterial3DThermal {
public:

    ChMaterial3DThermalLinear() : k_thermal_conductivity(1), c_mass_specific_heat_capacity(1000) {}

    virtual ~ChMaterial3DThermalLinear() {}

    /// Sets the k conductivity constant of the material,
    /// expressed in watts per meter kelvin [ W/(m K) ].
    /// Ex. (approx.): water = 0.6, aluminium = 200, steel = 50, plastics=0.9-0.2
    /// Sets the conductivity matrix as isotropic (diagonal k)
    void SetThermalConductivity(double mk) {
        k_thermal_conductivity = mk;
        constitutiveMatrix.setZero();
        constitutiveMatrix.fillDiagonal(k_thermal_conductivity);
    }

    /// Gets the k conductivity constant of the material,
    /// expressed in watts per meter kelvin (W/(m K)).
    double GetThermalConductivity() const { return k_thermal_conductivity; }

    /// Sets the c mass-specific heat capacity of the material,
    /// expressed as Joule per kg Kelvin [ J / (kg K) ]
    void SetSpecificHeatCapacity(double mc) { c_mass_specific_heat_capacity = mc; }

    /// Sets the c mass-specific heat capacity of the material,
    /// expressed as Joule per kg Kelvin [ J / (kg K) ]
    double GetSpecificHeatCapacity() const { return c_mass_specific_heat_capacity; }

    /// Get the k conductivity matrix. It is also the constitutive matrix of the Poisson problem.
    /// You can modify its values, for setting material property.
    ChMatrixDynamic<>& GetConductivityMatrix() { return constitutiveMatrix; }

    /// override base: (the dT/dt term has multiplier rho*c with rho=density, c=heat capacity)
    virtual double Get_DtMultiplier() override { return m_density * c_mass_specific_heat_capacity; }

    //
    // INTERFACE TO ChMaterialThermal
    //

    /// Compute heat flux vector q_flux from a temperature gradient grad_T
    ///  q_flux = f(grad_T)
    /// Because of linear model, this is q_flux = -[k]*grad_T  with [k] conductivity matrix.
    
    virtual void ComputeHeatFlux(ChVector3d& q_flux, ///< output stress, PK2
        const ChVector3d  grad_T,           ///< current temperature gradient 
        const double  T,                    ///< current temperature (not used here)
        ChFieldData* data_per_point,        ///< pointer to auxiliary data (ex states), if any, per quadrature point  (not used here)
        ChElementData* data_per_element     ///< pointer to auxiliary data (ex states), if any, per element   (not used here)
    ) override {
        q_flux = - this->constitutiveMatrix * grad_T.eigen();
    };

    /// Computes the tangent conductivity [C] in the linearization of q_flux = f(grad_T)  about temperature:
    ///   δq_flux = [C] δgrad_T
    /// Because of linear model, this is  [C] = [k] conductivity matrix.

    virtual void ComputeTangentModulus(ChMatrix33d& tangent_matrix, ///< output [C] tangent as δq_flux = [C] δgrad_T
        const ChVector3d  grad_T,       ///< current temperature gradient (not used here)
        const double  T,                ///< current temperature  (not used here)
        ChFieldData* data_per_point,    ///< pointer to auxiliary data (ex states), if any, per quadrature point (not used here)
        ChElementData* data_per_element ///< pointer to auxiliary data (ex states), if any, per element (not used here)
    ) override {
        tangent_matrix = this->constitutiveMatrix;
    }

    /// Computes the  Z  multiplier in  Z*dT/dt + div q_flux = q_source:
    /// Because of linear model, this is Z=(m_density * c_mass_specific_heat_capacity)
     
    virtual void ComputeDtMultiplier(double& Dt_multiplier,   ///< the Z  term  in Z*dT/dt + div(q_flux) = q_source
        const double  T,                ///< current temperature (not used here)
        ChFieldData* data_per_point,    ///< pointer to auxiliary data (ex states), if any, per quadrature point (not used here)
        ChElementData* data_per_element ///< pointer to auxiliary data (ex states), if any, per element  (not used here)
    ) override {
        Dt_multiplier = (m_density * c_mass_specific_heat_capacity);
    };

private:
    double k_thermal_conductivity;
    double c_mass_specific_heat_capacity;
};



/// @} chrono_fea

}  // end namespace fea

}  // end namespace chrono

#endif
