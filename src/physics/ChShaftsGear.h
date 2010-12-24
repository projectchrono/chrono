#ifndef CHSHAFTSGEAR_H
#define CHSHAFTSGEAR_H

//////////////////////////////////////////////////
//
//   ChShaftsGear.h
//
//   Class for defining a transmission ratio between
//   two one-degree-of-freedom parts, that is,
//   shafts that can be used to build 1D models
//   of power trains. This is more efficient than 
//   simulating power trains modeled full 3D ChBody
//   objects. 
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "physics/ChShaftsCouple.h"
#include "lcp/ChLcpConstraintTwoGeneric.h"



namespace chrono
{

///  Class for defining a 'transmission ratio' (a 1D gear) 
///  between two one-degree-of-freedom parts, that is,
///  shafts that can be used to build 1D models
///  of power trains. This is more efficient than 
///  simulating power trains modeled with full 3D ChBody
///  objects. 

class ChApi ChShaftsGear : public ChShaftsCouple {

						// Chrono simulation of RTTI, needed for serialization
	CH_RTTI(ChShaftsGear,ChShaftsCouple);

private:
			//
	  		// DATA
			//

	double ratio;		// transmission ratio t, as in w2=t*w1, or t=w2/w1

	double torque_react;					
	
						// used as an interface to the LCP solver.
	ChLcpConstraintTwoGeneric constraint;

	float cache_li_speed;	// used to cache the last computed value of multiplier (solver warm starting)
	float cache_li_pos;		// used to cache the last computed value of multiplier (solver warm starting)	

public:

			//
	  		// CONSTRUCTORS
			//

				/// Build a shaft.
	ChShaftsGear ();
				/// Destructor
	~ChShaftsGear ();

				/// Copy from another ChShaftsGear. 
	void Copy(ChShaftsGear* source);


			//
	  		// FLAGS
			//

			//
	  		// FUNCTIONS
			//

				/// Number of scalar constraints 
	virtual int GetDOC_c  () {return 1;}


			// Override/implement LCP system functions of ChShaftsCouple
			// (to assembly/manage data for LCP system solver

	virtual void InjectConstraints(ChLcpSystemDescriptor& mdescriptor);
	virtual void ConstraintsBiReset();
	virtual void ConstraintsBiLoad_C(double factor=1., double recovery_clamp=0.1, bool do_clamp=false);
	virtual void ConstraintsBiLoad_Ct(double factor=1.);
	//virtual void ConstraintsFbLoadForces(double factor=1.);
	virtual void ConstraintsLoadJacobians();
	virtual void ConstraintsLiLoadSuggestedSpeedSolution();
	virtual void ConstraintsLiLoadSuggestedPositionSolution();
	virtual void ConstraintsLiFetchSuggestedSpeedSolution();
	virtual void ConstraintsLiFetchSuggestedPositionSolution();
	virtual void ConstraintsFetch_react(double factor=1.);


			   // Other functions

				/// Use this function after gear creation, to initialize it, given  
				/// two shafts to join. 
				/// Each shaft must belong to the same ChSystem. 
	virtual int Initialize(ChSharedShaftPtr& mshaft1, ///< first  shaft to join
						   ChSharedShaftPtr& mshaft2  ///< second shaft to join 
						   );


				/// Set the transmission ratio t, as in w2=t*w1, or t=w2/w1 , or  t*w1 - w2 = 0.
				/// For example, t=1 for a rigid joint; t=-0.5 for representing
				/// a couple of spur gears with teeths z1=20 & z2=40; t=0.1 for
				/// a gear with inner teeths (or epicycloidal reducer), etc.
	void   SetTransmissionRatio(double mt) { this->ratio = mt;}
				/// Get the transmission ratio t, as in w2=t*w1, or t=w2/w1
	double GetTransmissionRatio() {return this->ratio;}

				/// Get the reaction torque exchanged between the two shafts,
				/// considered as applied to the 1st axis.
	double GetTorqueReactionOn1() {return (this->ratio*this->torque_react);}

				/// Get the reaction torque exchanged between the two shafts,
				/// considered as applied to the 2nd axis.
	double GetTorqueReactionOn2() {return -(this->torque_react);}




			//
			// UPDATE FUNCTIONS
			//

				/// Update all auxiliary data of the gear transmission at given time
	virtual void Update (double mytime);
	


			//
			// STREAMING
			//


				/// Method to allow deserializing a persistent binary archive (ex: a file)
				/// into transient data.
	void StreamIN(ChStreamInBinary& mstream);

				/// Method to allow serializing transient data into a persistent
				/// binary archive (ex: a file).
	void StreamOUT(ChStreamOutBinary& mstream);

};



typedef ChSharedPtr<ChShaftsGear> ChSharedShaftsGearPtr;



} // END_OF_NAMESPACE____


#endif
