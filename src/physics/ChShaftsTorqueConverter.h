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

#ifndef CHSHAFTSTORQUECONVERTER_H
#define CHSHAFTSTORQUECONVERTER_H

#include "physics/ChShaftsCouple.h"
//#include "lcp/ChLcpConstraintTwoGeneric.h"
#include "physics/ChFunction.h"



namespace chrono
{

///  Class for defining a torque converter  
///  between two one-degree-of-freedom parts, that is,
///  shafts that can be used to build 1D models
///  of power trains. 
///  The torque converter multiplies the input torque
///  if there is slippage between input and output, then
///  the multiplicative effect becomes closer to unity
///  when the slippage is almost null; so it is similar
///  to a variable-transmission-ratio gearbox, and just 
///  like any gearbox it requires a truss (the 'stator')
///  that gets some torque.
///  Note: it can work only in a given direction.

class ChApi ChShaftsTorqueConverter : public ChPhysicsItem  {

						// Chrono simulation of RTTI, needed for serialization
	CH_RTTI(ChShaftsTorqueConverter,ChPhysicsItem);

private:
			//
	  		// DATA
			//

	ChShaft* shaft1;
	ChShaft* shaft2;
	ChShaft* shaft_stator;

	double torque_in;
	double torque_out;

	ChSharedPtr<ChFunction> K;
	ChSharedPtr<ChFunction> T;

	bool state_warning_reverseflow;
	bool state_warning_wrongimpellerdirection;

public:

			//
	  		// CONSTRUCTORS
			//

				/// Constructor.
	ChShaftsTorqueConverter ();
				/// Destructor
	~ChShaftsTorqueConverter ();

				/// Copy from another ChShaftsTorqueConverter. 
	void Copy(ChShaftsTorqueConverter* source);


			//
	  		// FUNCTIONS
			//

				/// Number of scalar constraints 
	virtual int GetDOC_c  () {return 0;}


				// Override/implement LCP system functions of ChPhysicsItem
				// (to assembly/manage data for LCP system solver
	
	virtual void VariablesFbLoadForces(double factor);


			   // Other functions

				/// Use this function after torque converter creation, to initialize it, given  
				/// input and output shafts to join (plus the stator shaft, that should be fixed). 
				/// Each shaft must belong to the same ChSystem. 
	virtual int Initialize(ChSharedPtr<ChShaft> mshaft1, ///< input shaft 
						   ChSharedPtr<ChShaft> mshaft2, ///< output shaft 
						   ChSharedPtr<ChShaft> mshaft_stator  ///< stator shaft (often fixed)
						   );

				/// Get the input shaft
	ChShaft* GetShaftInput() {return shaft1;}
				/// Get the output shaft
	ChShaft* GetShaftOutput() {return shaft2;}
				/// Get the stator shaft (the truss)
	ChShaft* GetShaftStator() {return shaft_stator;}

				/// Set the capacity factor curve, function of speed ratio R.
				/// It is K(R)= input speed / square root of the input torque.
				/// Units: (rad/s) / sqrt(Nm)
	void SetCurveCapacityFactor( ChSharedPtr<ChFunction> mf) { K = mf;}
				/// Get the capacity factor curve.
	ChSharedPtr<ChFunction> GetCurveCapacityFactor() {return K;}

				/// Set the torque ratio curve, function of speed ratio R.
				/// It is T(R) = (output torque) / (input torque) 
	void SetCurveTorqueRatio( ChSharedPtr<ChFunction> mf) { T = mf;}
				/// Get the torque ratio curve.
	ChSharedPtr<ChFunction> GetCurveTorqueRatio() {return T;}

				
				/// Get the torque applied to the input shaft 
	double GetTorqueReactionOnInput()  {return  torque_in;}

				/// Get the torque applied to the output shaft
	double GetTorqueReactionOnOutput() {return  torque_out;}

				/// Get the torque applied to the stator shaft (the truss)
	double GetTorqueReactionOnStator() {return  -torque_out-torque_in;}


					/// Get the actual peed ratio, as output speed / input speed. 
					/// Assumes output has same direction as input, and slower than input
					/// otherwise exchanges input and output.
					/// For speed ratio = 0, complete slippage, for ratio=1 perfect locking.
	double GetSpeedRatio(); 

					/// Get the actual slippage, for slippage = 1 complete slippage, 
					/// for slippage = 0 perfect locking.
	double GetSlippage() {return 1.0- GetSpeedRatio();}
			
					/// State warning, at last update. Tell if the torque converter is working 
					/// in reverse power flow, i.e. the output turbine is running faster than
					/// input impeller shaft. 
	bool StateWarningReverseFlow() {return state_warning_reverseflow;}

					/// State warning, at last update. Tell if the torque converter is working
					/// with the input shaft in the reverse direction (negative speed).
					/// This is considered an abnormal behavior, and torques are forced to zero.
	bool StateWarningWrongImpellerDirection() {return state_warning_wrongimpellerdirection;}


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





} // END_OF_NAMESPACE____


#endif
