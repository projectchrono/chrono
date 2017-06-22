// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHCONTINUUMTHERMAL_H
#define CHCONTINUUMTHERMAL_H

#include "chrono_fea/ChContinuumPoisson3D.h"

namespace chrono {
namespace fea {

/// Class for thermal fields, for FEA problems involving temperature, heat, etc.

class ChContinuumThermal : public ChContinuumPoisson3D {
  private:
    double k_thermal_conductivity;
    double c_mass_specific_heat_capacity;

  public:
    ChContinuumThermal() : k_thermal_conductivity(1), c_mass_specific_heat_capacity(1000) {}
    ChContinuumThermal(const ChContinuumThermal& other) : ChContinuumPoisson3D(other) {
        k_thermal_conductivity = other.k_thermal_conductivity;
        c_mass_specific_heat_capacity = other.c_mass_specific_heat_capacity;
    }
    virtual ~ChContinuumThermal() {}

    /// Sets the k conductivity constant of the material,
    /// expressed in watts per meter kelvin [ W/(m K) ].
    /// Ex. (approx.): water = 0.6, aluminium = 200, steel = 50, plastics=0.9-0.2
    /// Sets the conductivity matrix as isotropic (diagonal k)
    void SetThermalConductivityK(double mk) {
        k_thermal_conductivity = mk;
        ConstitutiveMatrix.Reset();
        ConstitutiveMatrix.FillDiag(k_thermal_conductivity);
    }

    /// Gets the k conductivity constant of the material,
    /// expressed in watts per meter kelvin (W/(m K)).
    double GetThermalConductivityK() const { return k_thermal_conductivity; }

    /// Sets the c mass-specific heat capacity of the material,
    /// expressed as Joule per kg Kelvin [ J / (kg K) ]
    void SetMassSpecificHeatCapacity(double mc) { c_mass_specific_heat_capacity = mc; }
    /// Sets the c mass-specific heat capacity of the material,
    /// expressed as Joule per kg Kelvin [ J / (kg K) ]
    double GetMassSpecificHeatCapacity() const { return c_mass_specific_heat_capacity; }

    /// Get the k conductivity matrix
    ChMatrixDynamic<> Get_ThermalKmatrix() { return ConstitutiveMatrix; }

    /// override base: (the dT/dt term has multiplier rho*c with rho=density, c=heat capacity)
    virtual double Get_DtMultiplier() override { return density * c_mass_specific_heat_capacity; }
};

}  // end namespace fea
}  // end namespace chrono

#endif
