//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//
// File authors: Alessandro Tasora

#ifndef CHCONTINUUMTHERMAL_H
#define CHCONTINUUMTHERMAL_H


#include "unit_FEM/ChContinuumPoisson3D.h"


namespace chrono
{
namespace fem
{


/// Class for thermal fields, for FEM problems involving 
/// temperature, heat, etc.

class ChApiFem ChContinuumThermal : public ChContinuumPoisson3D
{
private:
	double k_thermal_conductivity;
	double c_mass_specific_heat_capacity;

public:

			
	ChContinuumThermal()
		{
			 // default, almost a plastic
			k_thermal_conductivity = 1.0;	
			c_mass_specific_heat_capacity = 1000.0; 
		}

	virtual ~ChContinuumThermal() {};
	
			/// Sets the k conductivity constant of the material, 
			/// expressed in watts per meter kelvin [ W/(m K) ].
			/// Ex. (approx.): water = 0.6, alluminium = 200, steel = 50, plastics=0.9-0.2
			/// Sets the conductivity matrix as isotropic (diagonal k) 
	void SetThermalConductivityK(double mk)
		{
			k_thermal_conductivity = mk;
			this->ConstitutiveMatrix.Reset();
			this->ConstitutiveMatrix.FillDiag(k_thermal_conductivity);
		}

			/// Gets the k conductivity constant of the material, 
			/// expressed in watts per meter kelvin (W/(m K)).
	double GetThermalConductivityK() {return k_thermal_conductivity;}

			/// Get the k conductivity matrix 
	ChMatrixDynamic<> Get_ThermalKmatrix () {return ConstitutiveMatrix;}


			/// Sets the c mass-specific heat capacity of the material,
			/// expressed as Joule per kg Kelvin [ J / (kg K) ]
	void SetMassSpecificHeatCapacity(double mc) {this->c_mass_specific_heat_capacity = mc;}
			/// Sets the c mass-specific heat capacity of the material,
			/// expressed as Joule per kg Kelvin [ J / (kg K) ]
	double GetMassSpecificHeatCapacity() { return c_mass_specific_heat_capacity;}

			/// override base: (the dT/dt term has multiplier rho*c with rho=density, c=heat capacity)
	virtual double Get_DtMultiplier () {return this->density*this->c_mass_specific_heat_capacity;}

};




}//___end of namespace fem___
}//___end of namespace chrono___

#endif
