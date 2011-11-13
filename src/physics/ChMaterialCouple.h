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
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
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

	ChMaterialCouple() :static_friction(0),
						sliding_friction(0),
						rolling_friction(0),
						spinning_friction(0),
						restitution(0),
						cohesion(0),
						dampingf(0),
						compliance(0),
						complianceT(0)
						{};
};



} // END_OF_NAMESPACE____

#endif
