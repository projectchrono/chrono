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
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.projectchrono.org
// ------------------------------------------------
///////////////////////////////////////////////////

#include "core/ChShared.h"


namespace chrono
{

class ChApi ChMaterialSurfaceDEM : public ChShared
{
public:

			//
			// DATA
			//

	float static_friction;
	float sliding_friction;
	float normal_stiffness;
	float normal_damping;
	float tangential_stiffness;


			//
			// CONSTRUCTORS
			//

	ChMaterialSurfaceDEM() 
	:	static_friction(0.6f),
		sliding_friction(0.6f),
		normal_stiffness(2e5),
		normal_damping(7.5e2),
		tangential_stiffness(2e5)
	{}

	~ChMaterialSurfaceDEM() {}

	// Copy constructor 
	ChMaterialSurfaceDEM(const ChMaterialSurfaceDEM& other) 
	{
		static_friction = other.static_friction;
		sliding_friction = other.sliding_friction;
		normal_stiffness = other.normal_stiffness;
		normal_damping = other.normal_damping;
		tangential_stiffness = other.tangential_stiffness;
	}

			//
			// FUNCTIONS
			//

	// Normal stiffness
	float GetNormalStiffness() const {return normal_stiffness;}
	void  SetNormalStiffness(float val) {normal_stiffness = val;}

	// Normal damping coefficient
	float GetNormalDamping() const {return normal_damping;}
	void  SetNormalDamping(float val) {normal_damping = val;}

	// Tangential stiffness
	float GetTangentialStiffness() const {return tangential_stiffness;}
	void  SetTangentialStiffness(float val) {tangential_stiffness = val;}

	/// The static friction coefficient. 
	/// Usually in 0..1 range, rarely above.
	/// Default 0.6
	float GetSfriction() const {return static_friction;}
	void  SetSfriction(float val) {static_friction = val;}

	/// The sliding ('kinetic')friction coefficient. Default 0.6
	/// Usually in 0..1 range, rarely above. 
	float GetKfriction() const {return sliding_friction;}
	void  SetKfriction(float val) {sliding_friction = val;}

	/// Set both static friction and kinetic friction at once, with same value.
	void  SetFriction(float val) {SetSfriction(val); SetKfriction(val);}

	static void compositeMaterial(const ChSharedPtr<ChMaterialSurfaceDEM>& matA,
	                              const ChSharedPtr<ChMaterialSurfaceDEM>& matB,
	                              double& kn,
	                              double& gn,
	                              double& kt,
	                              double& mu)
	{
		kn = (matA->GetNormalStiffness() + matB->GetNormalStiffness()) / 2;
		gn = (matA->GetNormalDamping() + matB->GetNormalDamping()) / 2;
		kt = (matA->GetTangentialStiffness() + matB->GetTangentialStiffness()) / 2;
		mu = (matA->GetSfriction() + matB->GetSfriction()) / 2;
	}

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
		mstream <<   static_friction;
		mstream <<   sliding_friction;
		mstream <<   normal_stiffness;
		mstream <<   normal_damping;
		mstream <<   tangential_stiffness;
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
		mstream >>   static_friction;
		mstream >>   sliding_friction;
		mstream >>   normal_stiffness;
		mstream >>   normal_damping;
		mstream >>   tangential_stiffness;
	}

};



} // END_OF_NAMESPACE____

#endif
