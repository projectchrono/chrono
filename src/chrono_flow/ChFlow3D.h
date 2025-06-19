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

#ifndef CHFLOW3D_H
#define CHFLOW3D_H

#include "chrono_flow/ChContinuumPoissonFlow3D.h"
#include "chrono_flow/ChElementSpringPPP.h"
#include "chrono_flow/ChFlowApi.h"

using namespace chrono::fea;

namespace chrono {
namespace flow {

/// Class for thermal fields, for FEA problems involving temperature, heat, etc.

class ChFlow3D : public ChContinuumPoissonFlow3D {
  private:
    ChMatrixDynamic<> ConstitutiveMatrixM;  // mass matrix
    ChVectorDynamic<> ConstitutiveVectorS;  // source term vector

    double k_thermal_conductivity;
    double h_permeability;
    double third_field_conductivity;
    double c_mass_specific_heat_capacity;

  public:
    ChFlow3D() : k_thermal_conductivity(1), h_permeability(1), third_field_conductivity(1), c_mass_specific_heat_capacity(1000) { ConstitutiveMatrixM.setIdentity(3, 3); ConstitutiveVectorS.setZero(3);}
    ChFlow3D(const ChFlow3D& other) : ChContinuumPoissonFlow3D(other) {
        k_thermal_conductivity = other.k_thermal_conductivity;
        h_permeability = other.h_permeability;
        third_field_conductivity = other.third_field_conductivity;
        c_mass_specific_heat_capacity = other.c_mass_specific_heat_capacity;
        ConstitutiveMatrixM = other.ConstitutiveMatrixM;
    }
    virtual ~ChFlow3D() {}

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

    /// Compute the k conductivity matrix
    ChMatrixDynamic<> ComputeUpdatedConstitutiveMatrixK(ChVectorDynamic<>& NValP, ChVectorDynamic<>& NValDtP) override {
      // Retrieve element interpolated nodal values 
      double tp = NValP(0);
      double hp = NValP(1);
      double thirdVarp = NValP(2);
      
      // Retrieve rate of element interpolated nodal values 
      double tDtp = NValDtP(0);
      double hDtp = NValDtP(1);
      double thirdVarDtp = NValDtP(2);

      // define f(h, T, ...) (currently equal to 1.0)
      ConstitutiveMatrix(0,0) *= 1.0;
      ConstitutiveMatrix(0,1) = 0.0;
      ConstitutiveMatrix(0,2) = 0.0; 
      ConstitutiveMatrix(1,0) = 0.0;    
      ConstitutiveMatrix(1,1) *= 1.0;
      ConstitutiveMatrix(1,2) = 0.0;
      ConstitutiveMatrix(2,0) = 0.0;
      ConstitutiveMatrix(2,1) = 0.0;      
      ConstitutiveMatrix(2,2) *= 1.0;
      
      return ConstitutiveMatrix; 
    }

    /// Compute the mass matrix
    ChMatrixDynamic<> ComputeUpdatedConstitutiveMatrixM(ChVectorDynamic<>& NValP, ChVectorDynamic<>& NValDtP) override {
      // Retrieve element interpolated nodal values 
      double tp = NValP(0);
      double hp = NValP(1);
      double thirdVarp = NValP(2);
          
      // Retrieve rate of element interpolated nodal values 
      double tDtp = NValDtP(0);
      double hDtp = NValDtP(1);
      double thirdVarDtp = NValDtP(2);
    
      // define M(h, T, ...) 
      ConstitutiveMatrixM(0,0) = 1.0 * m_density * c_mass_specific_heat_capacity;
      ConstitutiveMatrixM(0,1) *= 1.0;
      ConstitutiveMatrixM(0,2) *= 1.0;
      ConstitutiveMatrixM(1,0) *= 1.0;
      ConstitutiveMatrixM(1,1) = 1.0 * m_density * c_mass_specific_heat_capacity;
      ConstitutiveMatrixM(1,2) *= 1.0;
      ConstitutiveMatrixM(2,0) *= 1.0;
      ConstitutiveMatrixM(2,1) *= 1.0;
      ConstitutiveMatrixM(2,2) = 1.0 * m_density * c_mass_specific_heat_capacity;
          
      return ConstitutiveMatrixM; 
    }

    /// Compute source term vector
    ChMatrixDynamic<> ComputeUpdatedConstitutiveVectorS(ChVectorDynamic<>& NValP, ChVectorDynamic<>& NValDtP) override {
      // Retrieve element interpolated nodal values 
      double tp = NValP(0);
      double hp = NValP(1);
      double thirdVarp = NValP(2);
          
      // Retrieve rate of element interpolated nodal values 
      double tDtp = NValDtP(0);
      double hDtp = NValDtP(1);
      double thirdVarDtp = NValDtP(2);
    
      // define S(h, T, ...) (currently equal to 0.0)
      ConstitutiveVectorS(0) = 0.0;
      ConstitutiveVectorS(1) = 0.0;
      ConstitutiveVectorS(2) = 0.0;
          
      return ConstitutiveVectorS; 
    }    
};

}  // end namespace flow
}  // end namespace chrono

#endif
