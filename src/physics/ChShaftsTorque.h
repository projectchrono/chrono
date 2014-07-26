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

#ifndef CHSHAFTSTORQUE_H
#define CHSHAFTSTORQUE_H


#include "physics/ChShaftsTorqueBase.h"



namespace chrono
{


///  Class for defining a user-defined torque
///  between two one-degree-of-freedom parts, that is,
///  shafts that can be used to build 1D models
///  of power trains. This is more efficient than 
///  simulating power trains modeled with full 3D ChBody
///  objects. 
///  Two shaftsa are needed, because one gets the torque,
///  and the other is the 'reference' that gets the negative
///  torque as a reaction. For instance, a therma engine
///  applies a torque T to a crankshaft, but also applies
///  -T to the engine block!
///  Note that another way of imposing a torque is to do just
///  my_shaftA->SetAppliedTorque(6);
///  but in such case is an 'absolute' torque does not create 
///  a reaction.

class ChApi ChShaftsTorque : public ChShaftsTorqueBase {

						// Chrono simulation of RTTI, needed for serialization
	CH_RTTI(ChShaftsTorque,ChShaftsTorqueBase);

private:
			//
	  		// DATA
			//

	double stiffness;	
	double damping;				


public:

			//
	  		// CONSTRUCTORS
			//

				/// Constructor.
	ChShaftsTorque ();
				/// Destructor
	~ChShaftsTorque ();

				/// Copy from another ChShaftsTorsionSpring. 
	void Copy(ChShaftsTorque* source);


			//
	  		// FUNCTIONS
			//


				/// Set the imposed torque between the two shafts 
	void   SetTorque(double mt) { this->torque = mt;}
				/// Get the imposed torque between the two shafts 
	double GetTorque() { return this->torque;}


			//
			// UPDATE FUNCTIONS
			//
				/// (does nothing, just eaves the last user defined this->torque)
	virtual double ComputeTorque();


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
