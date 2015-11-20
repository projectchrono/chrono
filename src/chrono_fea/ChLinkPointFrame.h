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
#include "chrono_fea/ChNodeFEAxyz.h"
#include "lcp/ChLcpConstraintTwoGeneric.h"



namespace chrono
{

class ChIndexedNodes; // forward ref

namespace fea
{



/// Class for creating a constraint between a xyz FEA node (point)
/// and a ChBodyFrame (frame) object (that is, it fixes a 3-DOF point
/// to a 6-DOF frame). 
/// Nodes are 3-DOF points that are used in point-based 
/// primitives, such as ChMatterSPH or finite elements.

class ChApiFea ChLinkPointFrame : public ChLinkBase {

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

	ChSharedPtr<fea::ChNodeFEAxyz> mnode;
	ChSharedPtr<ChBodyFrame>  body;

	ChCoordsys<> attach_reference; 

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

				/// Get the number of scalar variables affected by constraints in this link 
	virtual int GetNumCoords() {return 3 + 7;}

				/// Number of scalar costraints 
	virtual int GetDOC_c  () {return 3;}

				/// To get reaction force, expressed in link coordinate system:
	virtual ChVector<> Get_react_force() {return GetReactionOnBody();}

    // Get constraint violations
    ChMatrix<> GetC();

	 		//
			// STATE FUNCTIONS
			//

				// (override/implement interfaces for global state vectors, see ChPhysicsItem for comments.)
	virtual void IntStateGatherReactions(const unsigned int off_L,	ChVectorDynamic<>& L);	
	virtual void IntStateScatterReactions(const unsigned int off_L,	const ChVectorDynamic<>& L);
	virtual void IntLoadResidual_CqL(const unsigned int off_L, ChVectorDynamic<>& R, const ChVectorDynamic<>& L, const double c);
	virtual void IntLoadConstraint_C(const unsigned int off, ChVectorDynamic<>& Qc,	const double c, bool do_clamp,	double recovery_clamp);
	virtual void IntToLCP(const unsigned int off_v,	const ChStateDelta& v, const ChVectorDynamic<>& R, const unsigned int off_L, const ChVectorDynamic<>& L, const ChVectorDynamic<>& Qc);
	virtual void IntFromLCP(const unsigned int off_v, ChStateDelta& v, const unsigned int off_L, ChVectorDynamic<>& L);


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
	
	virtual ChCoordsys<> GetLinkAbsoluteCoords();

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
	virtual int Initialize(ChSharedPtr<ChNodeFEAxyz> anode,  ///< xyz node (point) to join
						   ChSharedPtr<ChBodyFrame>  mbody,  ///< body (frame) to join 
						   ChVector<>* mattach=0			 ///< optional: if not null, sets the attachment position in absolute coordinates 
						   );

				/// Get the connected xyz node (point)
	virtual ChSharedPtr<fea::ChNodeFEAxyz> GetConstrainedNode() { return this->mnode;}
				
				/// Get the connected body (frame)
	virtual ChSharedPtr<ChBodyFrame> GetConstrainedBodyFrame() { return this->body;}

					/// Get the attachment position, in the coordinates of the body.
	ChVector<> GetAttachPosition() {return attach_reference.pos;}
					/// Set the attachment position, in the coordinates of the body
	void SetAttachPositionInBodyCoords(ChVector<> mattach) {attach_reference.pos = mattach;}
					/// Set the attachment position, in the absolute coordinates
	void SetAttachPositionInAbsoluteCoords(ChVector<> mattach) {attach_reference.pos = body->TransformPointParentToLocal(mattach);}

					/// Get the attachment reference, in the coordinates of the body.
	ChCoordsys<> GetAttachReference() {return attach_reference;}
					/// Set the attachment reference, in the coordinates of the body
	void SetAttachReferenceInBodyCoords(ChCoordsys<> mattach) {attach_reference = mattach;}
					/// Set the attachment position, in the absolute coordinates
	void SetAttachReferenceInAbsoluteCoords(ChCoordsys<> mattach) {attach_reference = body->coord.TransformParentToLocal(mattach);}


				/// Get the reaction force considered as applied to ChShaft.
	ChVector<> GetReactionOnNode() {return -(react);}

				/// Get the reaction force considered as applied to ChBody.
	ChVector<> GetReactionOnBody() {return react;}

	
			//
			// UPDATE FUNCTIONS
			//

				/// Update all auxiliary data of the gear transmission at given time
  virtual void Update(double mytime, bool update_assets = true);


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
