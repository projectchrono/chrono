﻿// =============================================================================
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



/// Class for thermal fields, for FEA problems involving temperature, heat, etc.
/// It contains properties for thermal problems PDEs of the type 
///    c*density dT/dt + div [k] grad T = q_source
class ChApi ChMaterial3DThermal : public ChMaterialPoisson {
public:

    ChMaterial3DThermal() : k_thermal_conductivity(1), c_mass_specific_heat_capacity(1000) {}

    virtual ~ChMaterial3DThermal() {}

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

private:
    double k_thermal_conductivity;
    double c_mass_specific_heat_capacity;
};



/// @} chrono_fea

}  // end namespace fea

}  // end namespace chrono

#endif
