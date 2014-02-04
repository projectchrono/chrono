//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2012 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHLINKPOINTFRAME_H
#define CHLINKPOINTFRAME_H

//////////////////////////////////////////////////
//
//   ChLinkPointFrame.h
//
//   Class for creating a constraint between a node point
//   and a ChBody object (that is, it fixes a 3-DOF point
//   to a 6-DOF body). 
//   Nodes are 3-DOF points that are used in point-based 
//   primitives, such as ChMatterSPH or finite elements.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "physics/ChBodyFrame.h"
#include "physics/ChLinkBase.h"
#include "unit_FEM/ChNodeFEMxyz.h"
#include "lcp/ChLcpConstraintTwoGeneric.h"



namespace chrono
{

class ChIndexedNodes; // forward ref

namespace fem
{



/// Class for creating a constraint between a xyz FEM node (point)
/// and a ChBodyFrame (frame) object (that is, it fixes a 3-DOF point
/// to a 6-DOF frame). 
/// Nodes are 3-DOF points that are used in point-based 
/// primitives, such as ChMatterSPH or finite elements.

class ChApiFem ChLinkPointFrame : public ChLinkBase {

						// Chrono simulation of RTTI, needed for serialization
	CH_RTTI(ChLinkPointFrame,ChLinkBase);

private:
			//
	  		// DATA
			//

	ChVector<> react;					
	
						// used as an interface to the LCP solver.
	ChLcpConstraintTwoGeneric constraint1;
	ChLcpConstraintTwoGeneric constraint2;
	ChLcpConstraintTwoGeneric constraint3;

	ChVector<> cache_li_speed;	// used to cache the last computed value of multiplier (solver warm starting)
	ChVector<> cache_li_pos;	// used to cache the last computed value of multiplier (solver warm starting)	

	ChSharedPtr<fem::ChNodeFEMxyz> mnode;
	ChSharedPtr<ChBodyFrame>  body;

	ChVector<> attach_position; 

public:

			//
	  		// CONSTRUCTORS
			//

				/// Build a shaft.
	ChLinkPointFrame ();
				/// Destructor
	~ChLinkPointFrame ();

				/// Copy from another ChLinkPointFrame. 
	void Copy(ChLinkPointFrame* source);


			//
	  		// FLAGS
			//

			//
	  		// FUNCTIONS
			//

				/// Number of scalar costraints 
	virtual int GetDOC_c  () {return 3;}

				/// Get the number of scalar variables affected by constraints in this link 
	virtual int GetNumCoords() {return 3 + 7;}

	virtual ChFrame<> GetAssetsFrame(unsigned int nclone =0);

			// Override/implement LCP system functions of ChPhysicsItem
			// (to assembly/manage data for LCP system solver

	virtual void InjectConstraints(ChLcpSystemDescriptor& mdescriptor);
	virtual void ConstraintsBiReset();
	virtual void ConstraintsBiLoad_C(double factor=1., double recovery_clamp=0.1, bool do_clamp=false);
	virtual void ConstraintsBiLoad_Ct(double factor=1.);
	virtual void ConstraintsLoadJacobians();
	virtual void ConstraintsLiLoadSuggestedSpeedSolution();
	virtual void ConstraintsLiLoadSuggestedPositionSolution();
	virtual void ConstraintsLiFetchSuggestedSpeedSolution();
	virtual void ConstraintsLiFetchSuggestedPositionSolution();
	virtual void ConstraintsFetch_react(double factor=1.);


			   // Other functions

				/// Use this function after object creation, to initialize it, given  
				/// the node and body to join. 
				/// The attachment position is the actual position of the node (unless
				/// otherwise defines, using the optional 'mattach' parameter).
				/// Note, mnodes and mbody must belong to the same ChSystem. 
	virtual int Initialize(ChSharedPtr<ChIndexedNodes> mnodes,  ///< nodes container
						   unsigned int mnode_index,			///< index of the xyz node (point) to join
						   ChSharedPtr<ChBodyFrame>   mbody,    ///< body (frame) to join 
						   ChVector<>* mattach=0				///< optional: if not null, sets the attachment position in absolute coordinates 
						   );
				/// Use this function after object creation, to initialize it, given  
				/// the node and body frame to join. 
				/// The attachment position is the actual position of the node (unless
				/// otherwise defines, using the optional 'mattach' parameter).
				/// Note, mnodes and mbody must belong to the same ChSystem. 
	virtual int Initialize(ChSharedPtr<ChNodeFEMxyz> anode,  ///< xyz node (point) to join
						   ChSharedPtr<ChBodyFrame>  mbody,  ///< body (frame) to join 
						   ChVector<>* mattach=0			 ///< optional: if not null, sets the attachment position in absolute coordinates 
						   );

				/// Get the connected xyz node (point)
	virtual ChSharedPtr<fem::ChNodeFEMxyz> GetConstrainedNode() { return this->mnode;}
				
				/// Get the connected body (frame)
	virtual ChSharedPtr<ChBodyFrame> GetConstrainedBodyFrame() { return this->body;}

					/// Get the attachment position, in the reference coordinates of the body.
	ChVector<> GetAttachPosition() {return attach_position;}
					/// Set the attachment position, in the reference coordinates of the body
	void SetAttachPositionInBodyCoords(ChVector<> mattach) {attach_position = mattach;}
					/// Set the attachment position, in the absolute coordinates
	void SetAttachPositionInAbsoluteCoords(ChVector<> mattach) {attach_position = body->TrasformPointParentToLocal(mattach);}


				/// Get the reaction torque considered as applied to ChShaft.
	ChVector<> GetReactionOnNode() {return -(react);}

				/// Get the reaction torque considered as applied to ChBody.
	ChVector<> GetReactionOnBody() {return react;}

	
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
} // END_OF_NAMESPACE____


#endif
