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

#ifndef CHMATERIAL3DTHERMALNONLINEAR_H
#define CHMATERIAL3DTHERMALNONLINEAR_H

#include "chrono/fea/ChMaterial3DThermal.h"
#include "chrono/functions/ChFunction.h"

namespace chrono {
namespace fea {

/// @addtogroup chrono_fea
/// @{


/// Class for properties of a non-linear thermal material in finite element problems. This a quite
/// pupular type of non-linearity, where the thermal properties (c specific heat, k thermal conductivity) 
/// are scalar functions of temperature T, i.e. 
///    c=c(T), 
///    k=k(T),   
/// assuming isotropic thermal conductivity matrix  [k](T)=[I]*k(T),  as in:
///    (c(T)*rho) dT/dt - div ([k](T) * grad T) = q_source

class ChApi ChMaterial3DThermalNonlinear : public ChMaterial3DThermal {
public:

    ChMaterial3DThermalNonlinear() {
        // defaults to constant functions
        k_thermal_conductivity = chrono_types::make_shared<ChFunctionConst>(1);
        c_mass_specific_heat_capacity = chrono_types::make_shared<ChFunctionConst>(1000);
    }

    virtual ~ChMaterial3DThermalNonlinear() {}

    /// Sets the k conductivity constant of the material as a function of temperature T,  k=k(T)
    /// where k has the units of watts per meter kelvin [ W/(m K) ].
    /// Ex. (approx.): water = 0.6, aluminium = 200, steel = 50, plastics=0.9-0.2
    /// Note: conductivity matrix [k] is assumed isotropic (diagonal matrix ie. [k]=[I]*k(T) )
    void SetThermalConductivity(std::shared_ptr<ChFunction> mk) { k_thermal_conductivity = mk; }

    /// Shortcut: sets the k conductivity constant of the material as a constant ChFunction.
    /// where k has the units of watts per meter kelvin [ W/(m K) ].
    void SetThermalConductivity(double mk) { 
        k_thermal_conductivity = chrono_types::make_shared<ChFunctionConst>(mk);
    }

    /// Gets the k conductivity of the material as a function of temperature T,  k=k(T)
    /// expressed in watts per meter kelvin (W/(m K)).
    std::shared_ptr<ChFunction> GetThermalConductivity() const { return k_thermal_conductivity; }

    /// Sets the c mass-specific heat capacity of the material, as a function of temperature T,  c=c(T)
    /// where c has the units of Joule per kg Kelvin [ J / (kg K) ]
    void SetSpecificHeatCapacity(std::shared_ptr<ChFunction> mc) { c_mass_specific_heat_capacity = mc; }

    /// Shortcut: sets the c mass-specific heat capacity of the material, as a function of temperature T,  c=c(T)
    /// where c has the units of Joule per kg Kelvin [ J / (kg K) ]
    void SetSpecificHeatCapacity(double mc) {
        c_mass_specific_heat_capacity = chrono_types::make_shared<ChFunctionConst>(mc);
    }

    /// Gets the c mass-specific heat capacity of the material, as a function of temperature T,  c=c(T)
    /// where c has the units of Joule per kg Kelvin [ J / (kg K) ]
    std::shared_ptr<ChFunction> GetSpecificHeatCapacity() const { return c_mass_specific_heat_capacity; }


    /// Get the k conductivity matrix for a given temperature T. It is also the constitutive matrix of the Poisson problem.
    ChMatrixDynamic<>& GetConductivityMatrix(double T) {
        constitutiveMatrix.setZero();
        constitutiveMatrix.fillDiagonal(k_thermal_conductivity->GetVal(T));
        return constitutiveMatrix; 
    }

    /// override base: (the dT/dt term has multiplier rho*c with rho=density, c=heat capacity)
    virtual double Get_DtMultiplier() override { 
        // should never hit into this... redesign ChMaterialPoisson to remove it? or add double T as parameter?
        assert(false); 
        throw(std::exception("Cannot use Get_DtMultiplier because non-linear material"));
        double T = 0; // placeholder...
        return m_density * c_mass_specific_heat_capacity->GetVal(T); 
    }

    //
    // INTERFACE TO ChMaterialThermal
    //

    /// Compute heat flux vector q_flux from a temperature gradient grad_T
    ///  q_flux = f(grad_T)
    /// Because of nonlinear model with temperature-dependant conductivity k=k(T)
    ///  this is q_flux = -[k](T) * grad_T  with conductivity matrix  [k](T) = [I] * k(T) .
    
    virtual void ComputeHeatFlux(ChVector3d& q_flux, ///< output stress, PK2
        const ChVector3d  grad_T,           ///< current temperature gradient 
        const double  T,                    ///< current temperature (not used here)
        ChFieldData* data_per_point,        ///< pointer to auxiliary data (ex states), if any, per quadrature point  (not used here)
        ChElementData* data_per_element     ///< pointer to auxiliary data (ex states), if any, per element   (not used here)
    ) override {
        q_flux = - this->GetConductivityMatrix(T) * grad_T.eigen();
    };

    /// Computes the tangent conductivity [C] in the linearization of q_flux = f(grad_T)  about temperature:
    ///   δq_flux = [C] δgrad_T
    /// Because of nonlinear model, this is  [C] = [k](T) = [I] * k(T)  

    virtual void ComputeTangentModulus(ChMatrix33d& tangent_matrix, ///< output [C] tangent as δq_flux = [C] δgrad_T
        const ChVector3d  grad_T,       ///< current temperature gradient (not used here)
        const double  T,                ///< current temperature  (not used here)
        ChFieldData* data_per_point,    ///< pointer to auxiliary data (ex states), if any, per quadrature point (not used here)
        ChElementData* data_per_element ///< pointer to auxiliary data (ex states), if any, per element (not used here)
    ) override {
        tangent_matrix.setZero();
        tangent_matrix.fillDiagonal(k_thermal_conductivity->GetVal(T));
    }

    /// Computes the  Z  multiplier in  Z*dT/dt + div q_flux = q_source:
    /// Because of linear model, this is Z=(m_density * c_mass_specific_heat_capacity)
     
    virtual void ComputeDtMultiplier(double& Dt_multiplier,   ///< the Z  term  in Z*dT/dt + div(q_flux) = q_source
        const double  T,                ///< current temperature (not used here)
        ChFieldData* data_per_point,    ///< pointer to auxiliary data (ex states), if any, per quadrature point (not used here)
        ChElementData* data_per_element ///< pointer to auxiliary data (ex states), if any, per element  (not used here)
    ) override {
        Dt_multiplier = (m_density * c_mass_specific_heat_capacity->GetVal(T));
    };

private:
    std::shared_ptr<ChFunction>  k_thermal_conductivity;
    std::shared_ptr<ChFunction>  c_mass_specific_heat_capacity;
};



/// @} chrono_fea

}  // end namespace fea

}  // end namespace chrono

#endif
