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

#ifndef CHSHAFTSGEARBOX_H
#define CHSHAFTSGEARBOX_H



#include "physics/ChPhysicsItem.h"
#include "physics/ChBodyFrame.h"
#include "physics/ChShaft.h"
#include "lcp/ChLcpConstraintThreeGeneric.h"



namespace chrono
{

// Forward references (for parent hierarchy pointer)

class ChShaft;
class ChBodyFrame;

///  Class for defining a gearbox. It defines a 
///  transmission ratio between two 1D entities of ChShaft type, and
///  transmits the reaction of the gearbox to a 3D body that acts as the 
///  support truss.
///  Note that the more basic ChShaftsGear can do the same, except
///  that it does not provide a way to transmit reaction
///  to a truss body.
///  Also note that this can also be seen as a ChShaftPlanetary
///  where one has joined the carrier shaft to a fixed body
///  via a ChShaftsBody constraint. 

class ChApi ChShaftsGearbox : public ChPhysicsItem {

						// Chrono simulation of RTTI, needed for serialization
	CH_RTTI(ChShaftsGearbox,ChPhysicsItem);

private:
			//
	  		// DATA
			//

	double r1;		// transmission ratios  as in   r1*w1 + r2*w2 + r3*w3 = 0
	double r2;
	double r3;

	double torque_react;					
	
						// used as an interface to the LCP solver.
	ChLcpConstraintThreeGeneric constraint;

	float cache_li_speed;	// used to cache the last computed value of multiplier (solver warm starting)
	float cache_li_pos;		// used to cache the last computed value of multiplier (solver warm starting)	

	ChShaft* shaft1;
	ChShaft* shaft2;
	ChBodyFrame*  body;

	ChVector<> shaft_dir;

public:

			//
	  		// CONSTRUCTORS
			//

				/// Constructor.
	ChShaftsGearbox ();
				/// Destructor
	~ChShaftsGearbox ();

				/// Copy from another ChShaftsGearbox. 
	void Copy(ChShaftsGearbox* source);


			//
	  		// FLAGS
			//

			//
	  		// FUNCTIONS
			//

				/// Number of scalar costraints 
	virtual int GetDOC_c  () {return 1;}


			// Override/implement LCP system functions of ChPhysicsItem
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
				/// two shafts to join, and the 3D body that acts as a truss and
				/// receives the reaction torque of the gearbox.
				/// Each shaft and body must belong to the same ChSystem.
				/// Direction is expressed in the local coordinates of the body.
	virtual int Initialize(ChSharedPtr<ChShaft> mshaft1, ///< first (input) shaft to join
						   ChSharedPtr<ChShaft> mshaft2, ///< second  (output) shaft to join
						   ChSharedPtr<ChBodyFrame>  mbody,  ///< 3D body to use as truss (also carrier, if rotates as in planetary gearboxes) 
						   ChVector<>& mdir			 ///< the direction of the shaft on 3D body (applied on COG: pure torque)
						   );

				/// Get the first shaft (carrier wheel)
	ChShaft* GetShaft1() {return shaft1;}
				/// Get the second shaft 
	ChShaft* GetShaft2() {return shaft2;}
				/// Get the third shaft 
	ChBodyFrame* GetBodyTruss() {return body;}


				/// Set the transmission ratio t, as in w2=t*w1, or t=w2/w1 , or  t*w1 - w2 = 0.
				/// For example, t=1 for a rigid joint; t=-0.5 for representing
				/// a couple of spur gears with teeths z1=20 & z2=40; t=0.1 for
				/// a gear with inner teeths (or epicycloidal reducer), etc.
				/// Also the t0 ordinary equivalent ratio of the inverted planetary,
				/// if the 3D body is rotating as in planetary gears.
	void   SetTransmissionRatio(double t0) { r3=(1.-t0); r1=t0; r2=-1.0;}
		
				/// Get the transmission ratio t, as in w2=t*w1, or t=w2/w1.
				/// Also the t0 ordinary equivalent ratio of the inverted planetary,
				/// if the 3D body is rotating as in planetary gears.
	double GetTransmissionRatio() { return -r1/r2;}


				/// Set the direction of the shaft respect to 3D body, as a 
				/// normalized vector expressed in the coordinates of the body.
				/// The shaft applies only torque, about this axis.
	void  SetShaftDirection(ChVector<> md) {shaft_dir = Vnorm(md);}

				/// Get the direction of the shaft respect to 3D body, as a 
				/// normalized vector expressed in the coordinates of the body. 
	ChVector<>  GetShaftDirection() {return shaft_dir;}



				/// Get the reaction torque considered as applied to the 1st axis.
	double GetTorqueReactionOn1() {return (this->r1*torque_react);}

				/// Get the reaction torque considered as applied to the 2nd axis.
	double GetTorqueReactionOn2() {return (this->r2*torque_react);}

				/// Get the reaction torque considered as applied to the body 
				/// (the truss of the gearbox), expressed in the coordinates of the body.
	ChVector<> GetTorqueReactionOnBody() {return (shaft_dir* (this->r3*torque_react));}

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
