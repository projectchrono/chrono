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

#ifndef CHBEAMSECTION_H
#define CHBEAMSECTION_H


#include "core/ChShared.h"
#include "unit_FEM/ChApiFEM.h"


namespace chrono
{
namespace fem
{


/// Basic geometry for a beam section in 3D, along with basic material 
/// properties.
/// This material can be shared between multiple beams.

class ChApiFem ChBeamSection : public ChShared
{
public:

	double Area;
	double Iyy;
	double Izz;
	double G;
	double E;
	double density;
	double rdamping;

	ChBeamSection()
				{
					
					SetAsRectangularSection(0.01, 0.01); // defaults Area, Ixx, Iyy
				
					E = 0.01e9;		  // default stiffness: rubber
					G = 0.3 * E;

					density = 1000;   // default density: water
					rdamping = 0.01;  // default raleygh damping.
				}

	virtual ~ChBeamSection() {}

				/// Set the cross sectional area A of the beam (m^2) 
	void   SetArea(const double ma) { this->Area = ma;  }
	double GetArea() const {return this->Area;} 

				/// Set the Iyy moment of inertia of the beam (for flexion about y axis)
				/// Note: some textbook calls this Iyy as Iz
	void   SetIyy(double ma) { this->Iyy = ma;  }
	double GetIyy() const {return this->Iyy;}

				/// Set the Izz moment of inertia of the beam (for flexion about z axis)
				/// Note: some textbook calls this Izz as Iy
	void   SetIzz(double ma) { this->Izz = ma;  }
	double GetIzz() const {return this->Izz;}

				/// Shortcut: set area, Ixx and Iyy moment of inertia  
				/// at once, given the y and z widths of the beam assumed
				/// with rectangular shape.
	void   SetAsRectangularSection(double width_y, double width_z) 
				{ 
					this->Area = width_y * width_z; 
					this->Izz = (1.0/12.0)*width_z*pow(width_y,3);
					this->Iyy = (1.0/12.0)*width_y*pow(width_z,3);
				}

				/// Set the density of the beam (kg/m^3)
	void   SetDensity(double md) { this->density = md;  }
	double GetDensity() const {return this->density;}

				/// Set E, the Young elastic modulus (N/m^2) 
	void   SetYoungModulus(double mE) { this->E = mE; }
	double GetYoungModulus() const {return this->E;}

				/// Set G, the shear modulus 
	void   SetGshearModulus(double mG) { this->G = mG; }
	double GetGshearModulus() const {return this->G;}

				/// Set the Raleygh damping ratio r (as in: R = r * K ), to do: also mass-proportional term
	void   SetBeamRaleyghDamping(double mr) { this->rdamping = mr; }
	double GetBeamRaleyghDamping() {return this->rdamping;}

};




/// Geometry for a beam section in 3D, along with basic material 
/// properties. It also supports the advanced case of 
/// Iyy and Izz axes rotated respect reference, centroid with offset
/// from reference, and shear center with offset from reference.
/// This material can be shared between multiple beams.

class ChApiFem ChBeamSectionAdvanced : public ChBeamSection
{
public:

	double alpha;	// Rotation of Izz Iyy respect to reference line x
	double Cy;		// Centroid (elastic center, tension center)
	double Cz;	
	double Sy;		// Shear center
	double Sz;

	ChBeamSectionAdvanced()
				{
					alpha = 0;
					Cy = 0;
					Cz = 0;
					Sy = 0;
					Sz = 0;
				}

	virtual ~ChBeamSectionAdvanced() {}


				/// Set the rotation of the section for which the Iyy Izz are
				/// defined.
	void   SetSectionRotation(double ma) { this->alpha = ma;  }
	double GetSectionRotation() {return this->alpha;}

				/// Set the displacement of the centroid C (i.e. the elastic center,
				/// or tension center) respect to the reference beam line.
	void   SetCentroid(double my, double mz) { this->Cy = my; this->Cz = mz;}
	double GetCentroidY() {return this->Cy;}
	double GetCentroidZ() {return this->Cz;}

				/// Set the displacement of the shear center S 
				/// respect to the reference beam line. For shapes like rectangles,
				/// rotated rectangles, etc., it corresponds to the centroid C, but
				/// for "L" shaped or "U" shaped beams this is not always true, and
				/// the shear center accounts for torsion effects when a shear force is applied.
	void   SetShearCenter(double my, double mz) { this->Cy = my; this->Cz = mz;}
	double GetShearCenterY() {return this->Cy;}
	double GetShearCenterZ() {return this->Cz;}
};






} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif






