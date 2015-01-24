//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//
// File author: A.Tasora

#ifndef CHNODEXYZ_H
#define CHNODEXYZ_H


#include "physics/ChNodeBase.h"
#include "lcp/ChLcpVariablesBodyOwnMass.h"


namespace chrono
{


/// Class for a single 'point' node, that has 3 DOF degrees of
/// freedom and a mass.

class ChApi ChNodeXYZ : public ChNodeBase 
{
public:
	ChNodeXYZ ();
	virtual ~ChNodeXYZ ();

	ChNodeXYZ (const ChNodeXYZ& other); // Copy constructor
	ChNodeXYZ& operator= (const ChNodeXYZ& other); //Assignment operator

					//
					// FUNCTIONS
					//

			// Position of the node - in absolute csys.
	ChVector<> GetPos() {return pos;}
			// Position of the node - in absolute csys.
	void SetPos(const ChVector<>& mpos) {pos = mpos;}

			// Velocity of the node - in absolute csys.
	ChVector<> GetPos_dt() {return pos_dt;}
			// Velocity of the node - in absolute csys.
	void SetPos_dt(const ChVector<>& mposdt) {pos_dt = mposdt;}

			// Acceleration of the node - in absolute csys.
	ChVector<> GetPos_dtdt() {return pos_dtdt;}
			// Acceleration of the node - in absolute csys.
	void SetPos_dtdt(const ChVector<>& mposdtdt) {pos_dtdt = mposdtdt;}

			// Get mass of the node. To be implemented in children classes
	virtual double GetMass() const = 0;
			// Set mass of the node. To be implemented in children classes
	virtual void SetMass(double mm) = 0;


			/// Get the number of degrees of freedom
	virtual int Get_ndof_x() { return 3; }


					//
					// DATA
					// 
	ChVector<> pos;		
	ChVector<> pos_dt;
	ChVector<> pos_dtdt;
};




} // END_OF_NAMESPACE____


#endif
