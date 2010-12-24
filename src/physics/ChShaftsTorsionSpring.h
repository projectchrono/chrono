#ifndef CHSHAFTSTORSIONSPRING_H
#define CHSHAFTSTORSIONSPRING_H

//////////////////////////////////////////////////
//
//   ChShaftsTorsionSpring.h
//
//   Class for defining a torsional spring-damper between
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


///  Class for defining a torsional spring-damper between
///  between two one-degree-of-freedom parts, that is,
///  shafts that can be used to build 1D models
///  of power trains. This is more efficient than 
///  simulating power trains modeled with full 3D ChBody
///  objects. 

class ChApi ChShaftsTorsionSpring : public ChShaftsCouple {

						// Chrono simulation of RTTI, needed for serialization
	CH_RTTI(ChShaftsTorsionSpring,ChShaftsCouple);

private:
			//
	  		// DATA
			//

	double stiffness;	
	double damping;

	double torque_kr;					


public:

			//
	  		// CONSTRUCTORS
			//

				/// Constructor.
	ChShaftsTorsionSpring ();
				/// Destructor
	~ChShaftsTorsionSpring ();

				/// Copy from another ChShaftsTorsionSpring. 
	void Copy(ChShaftsTorsionSpring* source);


			//
	  		// FLAGS
			//

			//
	  		// FUNCTIONS
			//

				/// Number of scalar constraints 
	virtual int GetDOC_c  () {return 0;}


			// Override/implement LCP system functions of ChShaftsCouple
			// (to assembly/manage data for LCP system solver

				// Adds the torsional torques in the 'fb' part: qf+=torques*factor 
				// of both shafts 
	void VariablesFbLoadForces(double factor=1.);


				/// Set the torsional stiffness between the two shafts 
	void   SetTorsionalStiffness(double mt) { this->stiffness = mt;}
				/// Get the torsional stiffness between the two shafts 
	double GetTorsionalStiffness() { return this->stiffness;}

				/// Set the torsional damping between the two shafts 
	void   SetTorsionalDamping(double mt) { this->damping = mt;}
				/// Get the torsional damping between the two shafts 
	double GetTorsionalDamping() { return this->damping;}

				/// Get the reaction torque exchanged between the two shafts,
				/// considered as applied to the 1st axis.
	virtual double GetTorqueReactionOn1() {return  (this->torque_kr); }

				/// Get the reaction torque exchanged between the two shafts,
				/// considered as applied to the 2nd axis.
	virtual double GetTorqueReactionOn2() {return -(this->torque_kr); }


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



typedef ChSharedPtr<ChShaftsTorsionSpring> ChSharedShaftsTorsionSpringPtr;



} // END_OF_NAMESPACE____


#endif
