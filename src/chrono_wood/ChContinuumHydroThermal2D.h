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

#ifndef CHCONTINUUMHYDROTHERMAL2D_H
#define CHCONTINUUMHYDROTHERMAL2D_H

#include "ChContinuumPoisson2D.h"

namespace chrono {
namespace fea {

/// Class for thermal fields, for FEA problems involving temperature, heat, etc.

class ChContinuumHydroThermal2D : public ChContinuumPoisson2D {
  private:
    double k_thermal_conductivity;
    double h_permeability;
    double third_field_conductivity;
    double c_mass_specific_heat_capacity;

  public:
    ChContinuumHydroThermal2D() : k_thermal_conductivity(1), h_permeability(1), third_field_conductivity(1), c_mass_specific_heat_capacity(1000) {}
    ChContinuumHydroThermal2D(const ChContinuumHydroThermal2D& other) : ChContinuumPoisson2D(other) {
        k_thermal_conductivity = other.k_thermal_conductivity;
        h_permeability = other.h_permeability;
        third_field_conductivity = other.third_field_conductivity;
        c_mass_specific_heat_capacity = other.c_mass_specific_heat_capacity;
    }
    virtual ~ChContinuumHydroThermal2D() {}

    /// Sets the k conductivity and h permeability constants of the material,
    /// expressed respectively in watts per meter kelvin [W/(m K)] and [kg/m/s].
    void SetDiffusivityConstants(double mk1, double mk2, double mk3) {
        k_thermal_conductivity = mk1;
        h_permeability = mk2;
        third_field_conductivity = mk3;
        ConstitutiveMatrix.setZero();
        //ConstitutiveMatrix.fillDiagonal(k_thermal_conductivity);
        ConstitutiveMatrix(0, 0) = k_thermal_conductivity;
        ConstitutiveMatrix(1, 1) = h_permeability;
        ConstitutiveMatrix(2, 2) = third_field_conductivity;
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

    /// Get the k conductivity matrix
    ChMatrixDynamic<> GetConductivityMatrix() { return ConstitutiveMatrix; }

    /// override base: (the dT/dt term has multiplier rho*c with rho=density, c=heat capacity)
    virtual double Get_DtMultiplier() override { return m_density * c_mass_specific_heat_capacity; }
};

}  // end namespace fea
}  // end namespace chrono

#endif
