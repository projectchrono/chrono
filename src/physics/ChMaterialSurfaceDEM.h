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

#ifndef CHMATERIALSURFACEDEM_H
#define CHMATERIALSURFACEDEM_H

///////////////////////////////////////////////////
//
//   ChMaterialSurfaceDEM.h
//
//   Class for material surface data for DEM contact
//
//   HEADER file for CHRONO,
//   Multibody dynamics engine
//
// ------------------------------------------------
//             www.projectchrono.org
// ------------------------------------------------
///////////////////////////////////////////////////

#include "core/ChShared.h"
#include "core/ChVector.h"
#include "physics/ChContactDEM.h"


namespace chrono
{

// Forward references
class ChBodyDEM;


struct ChCompositeMaterialDEM
{
	float E_eff;                ///< Effective elasticity modulus
	float G_eff;                ///< Effective shear modulus
	float mu_eff;               ///< Effective coefficient of friction
	float cr_eff;               ///< Effective coefficient of restitution
	float alpha_eff;            ///< Effective dissipation factor (Hunt-Crossley)
	float cohesion_eff;         ///< Effective cohesion force
};


class ChApi ChMaterialSurfaceDEM : public ChShared
{
public:
			//
			// DATA
			//

	float young_modulus;         ///< Young's modulus (elastic modulus)
	float poisson_ratio;         ///< Poisson ratio

	float static_friction;       ///< Static coefficient of friction
	float sliding_friction;      ///< Kinetic coefficient of friction

	float restitution;           ///< Coefficient of restitution
	float dissipation_factor;    ///< Dissipation factor (Hunt-Crossley model)

	float cohesion;              ///< Constant cohesion force

			//
			// CONSTRUCTORS
			//

	ChMaterialSurfaceDEM()
	:	young_modulus(2e5),
		poisson_ratio(0.3f),
		static_friction(0.6f),
		sliding_friction(0.6f),
		restitution(0.5f),
		dissipation_factor(0.1f),
		cohesion(0)
	{}

	// Copy constructor
	ChMaterialSurfaceDEM(const ChMaterialSurfaceDEM& other) 
	{
		young_modulus = other.young_modulus;
		poisson_ratio = other.poisson_ratio;
		static_friction = other.static_friction;
		sliding_friction = other.sliding_friction;
		restitution = other.restitution;
		dissipation_factor = other.dissipation_factor;
		cohesion = other.cohesion;
	}

	~ChMaterialSurfaceDEM() {}

			//
			// FUNCTIONS
			//

	/// Young's modulus and Poisson ratio.
	float GetYoungModulus() const    {return young_modulus;}
	void  SetYoungModulus(float val) {young_modulus = val;}

	float GetPoissonRatio() const    {return poisson_ratio;}
	void  SetPoissonRatio(float val) {poisson_ratio = val;}

	/// Static and kinetic friction coefficients.
	/// Usually in 0..1 range, rarely above. Default 0.6
	float GetSfriction() const       {return static_friction;}
	void  SetSfriction(float val)    {static_friction = val;}

	float GetKfriction() const       {return sliding_friction;}
	void  SetKfriction(float val)    {sliding_friction = val;}

	/// Set both static friction and kinetic friction at once, with same value.
	void  SetFriction(float val)     {SetSfriction(val); SetKfriction(val);}

	/// Normal restitution coefficient 
	float GetRestitution() const     {return restitution;}
	void  SetRestitution(float val)  {restitution = val;}

	/// Dissipation factor (Hunt-Crossley model)
	float GetDissipationFactor() const     {return dissipation_factor;}
	void  SetDissipationFactor(float val)  {dissipation_factor = val;}
	
	// Constant cohesion force
	float GetCohesion() const        {return cohesion;}
	void  SetCohesion(float val)     {cohesion = val;}

	/// Calculate composite material properties
	static ChCompositeMaterialDEM
	CompositeMaterial(const ChSharedPtr<ChMaterialSurfaceDEM>& mat1,
	                  const ChSharedPtr<ChMaterialSurfaceDEM>& mat2);

			//
			// STREAMING
			//

	/// Method to allow serializing transient data into in ascii
	/// as a readable item, for example   "chrono::GetLog() << myobject;"
	virtual void StreamOUT(ChStreamOutAscii& mstream)
	{
		mstream << "Material DEM \n";
	}

	/// Method to allow serializing transient data into a persistent
	/// binary archive (ex: a file).
	virtual void StreamOUT(ChStreamOutBinary& mstream)
	{
		// class version number
		mstream.VersionWrite(1);

		// deserialize parent class too
		//ChShared::StreamOUT(mstream); // nothing 

		// stream out all member data
		mstream << young_modulus;
		mstream << poisson_ratio;
		mstream << static_friction;
		mstream << sliding_friction;
		mstream << restitution;
		mstream << dissipation_factor;
		mstream << cohesion;
	}

	/// Operator to allow deserializing a persistent binary archive (ex: a file)
	/// into transient data.
	virtual void StreamIN(ChStreamInBinary& mstream)
	{
		// class version number
		int version = mstream.VersionRead();

		// deserialize parent class too
		//ChShared::StreamIN(mstream); // nothing 

		// stream in all member data
		mstream >> young_modulus;
		mstream >> poisson_ratio;
		mstream >> static_friction;
		mstream >> sliding_friction;
		mstream >> restitution;
		mstream >> dissipation_factor;
		mstream >> cohesion;
	}

};



} // END_OF_NAMESPACE____

#endif
