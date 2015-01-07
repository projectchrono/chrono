//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010, 2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHSHAFTSCOUPLE_H
#define CHSHAFTSCOUPLE_H

//////////////////////////////////////////////////
//
//   ChShaftsCouple.h
//
//   Base class for defining constraints between a couple
//   of two one-degree-of-freedom parts, that is,
//   shafts that can be used to build 1D models
//   of power trains. 
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "physics/ChShaft.h"

namespace chrono
{

/// Base class for defining constraints between a couple
/// of two one-degree-of-freedom parts, that is,
/// shafts that can be used to build 1D models
/// of power trains. 

class ChApi ChShaftsCouple : public ChPhysicsItem {

						// Chrono simulation of RTTI, needed for serialization
	CH_RTTI(ChShaftsCouple,ChPhysicsItem);

protected:
			//
	  		// DATA
			//

	ChShaft* shaft1;
	ChShaft* shaft2;

public:

			//
	  		// CONSTRUCTORS
			//

				/// Build a shaft.
	ChShaftsCouple ()
						{
							this->shaft1 = 0;
							this->shaft2 = 0;
						}

				/// Destructor
	~ChShaftsCouple () {};

				/// Copy from another ChShaftsClutch. 
	void Copy(ChShaftsCouple* source)
						{
							this->shaft1 = 0;
							this->shaft2 = 0;
						}


			//
	  		// FUNCTIONS
			//

				/// Use this function after gear creation, to initialize it, given  
				/// two shafts to join. 
				/// Each shaft must belong to the same ChSystem. 
				/// Children classes might overload this (here, basically it only sets the two
				/// pointers)
	virtual bool Initialize(ChSharedPtr<ChShaft> mshaft1, ///< first  shaft to join
	                        ChSharedPtr<ChShaft> mshaft2  ///< second shaft to join
	                       )
	          {
	            ChShaft* mm1 = mshaft1.get_ptr();
	            ChShaft* mm2 = mshaft2.get_ptr();
	            assert(mm1 && mm2);
	            assert(mm1 != mm2);
	            assert(mm1->GetSystem() == mm2->GetSystem());
	            this->shaft1 = mm1;
	            this->shaft2 = mm2;
	            this->SetSystem(this->shaft1->GetSystem());
	            return true;
	          }

				/// Get the first (input) shaft
	ChShaft* GetShaft1() {return shaft1;}
				/// Get the second (output) shaft
	ChShaft* GetShaft2() {return shaft2;}

				/// Get the reaction torque exchanged between the two shafts,
				/// considered as applied to the 1st axis.
				/// Children classes might overload this.
	virtual double GetTorqueReactionOn1() const {return 0; }

				/// Get the reaction torque exchanged between the two shafts,
				/// considered as applied to the 2nd axis.
				/// Children classes might overload this.
	virtual double GetTorqueReactionOn2() const {return 0; }

				/// Get the actual reative angle in terms of phase of shaft 1 respect to 2.
	double GetRelativeRotation() const {return (this->shaft1->GetPos() - this->shaft2->GetPos());}
				/// Get the actual relative speed in terms of speed of shaft 1 respect to 2.
	double GetRelativeRotation_dt() const {return (this->shaft1->GetPos_dt() - this->shaft2->GetPos_dt());}
				/// Get the actual relative acceleration in terms of speed of shaft 1 respect to 2.
	double GetRelativeRotation_dtdt() const {return (this->shaft1->GetPos_dtdt() - this->shaft2->GetPos_dtdt());}



			//
			// STREAMING
			//


				/// Method to allow deserializing a persistent binary archive (ex: a file)
				/// into transient data.
	void StreamIN(ChStreamInBinary& mstream)
					{
							// class version number
						int version = mstream.VersionRead();

							// deserialize parent class too
						ChPhysicsItem::StreamIN(mstream);

							// deserialize class
						// nothing - pointers must be rebound
					}

				/// Method to allow serializing transient data into a persistent
				/// binary archive (ex: a file).
	void StreamOUT(ChStreamOutBinary& mstream)
					{
								// class version number
						mstream.VersionWrite(1);

							// serialize parent class too
						ChPhysicsItem::StreamOUT(mstream);

							// stream out all member data
						// nothing - pointers must be rebound
					}
					
};



typedef ChSharedPtr<ChShaftsCouple> ChSharedShaftsCouplePtr;



} // END_OF_NAMESPACE____


#endif
