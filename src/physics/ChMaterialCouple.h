//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHMATERIALCOUPLE_H
#define CHMATERIALCOUPLE_H

///////////////////////////////////////////////////
//
//   ChMaterialCouple.h
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



namespace chrono
{

/// Class to pass data about material properties of a contact pair.
/// Used by ChAddContactCallback for generating cutom combinations
/// of material properties (by default, friction & c. are average
/// of values of the two parts in contact)

class ChApi ChMaterialCouple
{
public:
	float  static_friction;	
	float  sliding_friction;	
	float  rolling_friction;	
	float  spinning_friction;			
	float  restitution;	
	float  cohesion;
	float  dampingf;
	float  compliance;
	float  complianceT;
	float  complianceRoll;
	float  complianceSpin;

	ChMaterialCouple() :static_friction(0),
						sliding_friction(0),
						rolling_friction(0),
						spinning_friction(0),
						restitution(0),
						cohesion(0),
						dampingf(0),
						compliance(0),
						complianceT(0),
						complianceRoll(0),
						complianceSpin(0)
						{};
};



} // END_OF_NAMESPACE____

#endif
