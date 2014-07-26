//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHSHAFTSTORQUEBASE_H
#define CHSHAFTSTORQUEBASE_H

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
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "physics/ChShaftsCouple.h"
#include "lcp/ChLcpConstraintTwoGeneric.h"



namespace chrono
{


///  'Easy' base class for all stuff defining a torque
///  between two one-degree-of-freedom parts, for
///  example torsional dampers, torsional springs, 
///  electric engines, etc. 
///  This helps to keep things simple: children classes
///  just have to implement ComputeTorque(). 

class ChApi ChShaftsTorqueBase : public ChShaftsCouple {

						// Chrono simulation of RTTI, needed for serialization
	CH_RTTI(ChShaftsTorqueBase,ChShaftsCouple);

protected:
			//
	  		// DATA
			//

	double torque;	// store actual value of torque 			

public:

			//
	  		// CONSTRUCTORS
			//

				/// Constructor.
	ChShaftsTorqueBase ();
				/// Destructor
	~ChShaftsTorqueBase ();

				/// Copy from another ChShaftsTorqueBase. 
	void Copy(ChShaftsTorqueBase* source);


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



				/// Get the reaction torque exchanged between the two shafts,
				/// considered as applied to the 1st axis.
	virtual double GetTorqueReactionOn1() {return  (this->torque); }

				/// Get the reaction torque exchanged between the two shafts,
				/// considered as applied to the 2nd axis.
	virtual double GetTorqueReactionOn2() {return -(this->torque); }


			//
			// UPDATE FUNCTIONS
			//

				/// NOTE: children classes MUST implement this. 
				/// In most cases, this is the ONLY function you need to implement
				/// in children classes. It will be called at each Update().
	virtual double ComputeTorque() = 0;

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




} // END_OF_NAMESPACE____


#endif
