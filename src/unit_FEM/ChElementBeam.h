//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//
// File author: Alessandro Tasora

#ifndef CHELEMENTBEAM_H
#define CHELEMENTBEAM_H


#include "ChElementGeneric.h"
#include "ChElementCorotational.h"
#include "ChNodeFEMxyzrot.h"


namespace chrono
{
namespace fem
{



/// Base class for most structral elements of 'beam' type.

class ChApiFem ChElementBeam : public ChElementGeneric,
							   public ChElementCorotational
{
protected:
	double Area;
	double Iyy;
	double Izz;
	double G;
	double E;
	double density;
	double rdamping;
	double mass;
	double length;

	double y_drawsize;
	double z_drawsize;

public:

	ChElementBeam()
				{
					SetAsRectangularSection(0.01, 0.01); // defaults Area, Ixx, Iyy
				
					E = 0.01e9;		  // default stiffness: rubber
					G = 0.3 * E;

					density = 1000;   // default density: water
					rdamping = 0.01;  // default raleygh damping.

					length = 0;		// will be computed by Setup(), later
					mass = 0;		// will be computed by Setup(), later
				}

	virtual ~ChElementBeam() {}


			//
			// beam-specific functions
			//

				/// Gets the xyz displacement of a point on the beam line, 
				/// and the rotation RxRyRz of section plane, at abscyssa 'eta'.
				/// Note, eta=-1 at node1, eta=+1 at node2.
				/// Note, 'displ' is the displ.state of 2 nodes, ex. get it as GetField()
				/// Results are not corotated.
	virtual void EvaluateSectionDisplacement(const double eta, const ChMatrix<>& displ, ChVector<>& u_displ, ChVector<>& u_rotaz) = 0;
	
				/// Gets the absolute xyz position of a point on the beam line, 
				/// and the absolute rotation of section plane, at abscyssa 'eta'.
				/// Note, eta=-1 at node1, eta=+1 at node2.
				/// Note, 'displ' is the displ.state of 2 nodes, ex. get it as GetField()
				/// Results are corotated.
	virtual void EvaluateSectionFrame(const double eta, const ChMatrix<>& displ, ChVector<>& point, ChQuaternion<>& rot) = 0;
				
				/// Gets the torque at a section along the beam line, at abscyssa 'eta'.
				/// Note, eta=-1 at node1, eta=+1 at node2.
				/// Note, 'displ' is the displ.state of 2 nodes, ex. get it as GetField()
				/// Results are not corotated.
	virtual void EvaluateSectionTorque(const double eta, const ChMatrix<>& displ, ChVector<>& Mtorque) = 0;

				/// Gets the force (traction x, shear y, shear z) at a section along the beam line, at abscyssa 'eta'.
				/// Note, eta=-1 at node1, eta=+1 at node2.
				/// Note, 'displ' is the displ.state of 2 nodes, ex. get it as GetField()
				/// Results are not corotated.
	virtual void EvaluateSectionForce(const double eta, const ChMatrix<>& displ, ChVector<>& Fforce) = 0;

				/// Set the cross sectional area A of the beam (m^2) 
	void   SetArea(double ma) { this->Area = ma;  }
	double GetArea() {return this->Area;}

				/// Set the Iyy moment of inertia of the beam 
	void   SetIyy(double ma) { this->Iyy = ma;  }
	double GetIyy() {return this->Iyy;}

				/// Set the Iyy moment of inertia of the beam 
	void   SetIzz(double ma) { this->Izz = ma;  }
	double GetIzz() {return this->Izz;}

				/// Shortcut: set area, Ixx and Iyy moment of inertia  
				/// at once, given the y and z widths of the beam assumed
				/// with rectangular shape.
				/// Also, sets the y and z width of drawn shape.
	void   SetAsRectangularSection(double width_y, double width_z) 
				{ 
					this->Area = width_y * width_z; 
					this->Izz = (1.0/12.0)*width_z*pow(width_y,3);
					this->Iyy = (1.0/12.0)*width_y*pow(width_z,3);
					
					this->y_drawsize = width_y;
					this->z_drawsize = width_z;
				}

				/// Sets the rectangular thickness of the beam on y and z directions,
				/// only for drawing/rendering purposes (these thickenss values do NOT
				/// have any meaning at a physical level, use SetAsRectangularSection() instead
				/// if you want to affect also the inertias of the beam section).
	void SetDrawThickness(double thickness_y, double thickness_z)
				{
					this->y_drawsize = thickness_y;
					this->z_drawsize = thickness_z;
				}
	double GetDrawThicknessY() {return this->y_drawsize;}
	double GetDrawThicknessZ() {return this->z_drawsize;}


				/// Set the density of the beam (kg/m^3)
	void   SetDensity(double md) { this->density = md;  }
	double GetDensity() {return this->density;}

				/// Set E, the Young elastic modulus (N/m^2) 
	void   SetYoungModulus(double mE) { this->E = mE; }
	double GetYoungModulus() {return this->E;}
				/// Set G, the shear modulus 
	void   SetGshearModulus(double mG) { this->G = mG; }
	double GetGshearModulus() {return this->G;}

				/// Set the Raleygh damping ratio r (as in: R = r * K ), to do: also mass-proportional term
	void   SetBeamRaleyghDamping(double mr) { this->rdamping = mr; }
	double GetBeamRaleyghDamping() {return this->rdamping;}

				/// The full mass of the beam, (with const. section, density, etc.)
	double  GetMass() {return this->mass;}

				/// The rest length of the bar
	double  GetRestLength() {return this->length;}


};





} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif






