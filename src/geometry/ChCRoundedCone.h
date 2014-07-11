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

#ifndef CHC_ROUNDEDCONE_H
#define CHC_ROUNDEDCONE_H

//////////////////////////////////////////////////
//  
//   ChCRoundedCone.h
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "ChCGeometry.h"

namespace chrono {
namespace geometry {

#define EPS_SHPEREDEGENERATE 1e-20

#define CH_GEOCLASS_ROUNDEDCONE   17

///
/// A sphere.
/// Geometric object for collisions and such.
///

class ChApi ChRoundedCone: public ChGeometry {
		// Chrono simulation of RTTI, needed for serialization
	CH_RTTI(ChRoundedCone,ChGeometry)
		;

	public:

		//
		// CONSTRUCTORS
		//

		ChRoundedCone() {
			center = VNULL;
			rad = 0;
		};

		ChRoundedCone(Vector& mc, Vector mrad)
		{
			center = mc;
			rad = mrad;
		}

		ChRoundedCone(const ChRoundedCone & source)
		{
			Copy(&source);
		}

		void Copy (const ChRoundedCone* source)
		{
			center = source->center;
			rad = source->rad;
		};

		ChGeometry* Duplicate ()
		{
			ChGeometry* mgeo = new ChRoundedCone();
			mgeo->Copy(this); return mgeo;
		};

		//
		// OVERRIDE BASE CLASS FUNCTIONS
		//

					virtual int GetClassType () {return CH_GEOCLASS_ROUNDEDCONE;};

					virtual void GetBoundingBox(double& xmin, double& xmax,
							double& ymin, double& ymax,
							double& zmin, double& zmax,
							ChMatrix33<>* Rot = NULL)
					{

					}

					virtual Vector Baricenter() {return center;};

					virtual void CovarianceMatrix(ChMatrix33<>& C)
					{

					};

					/// This is a solid
					virtual int GetManifoldDimension() {return 3;}

					//
					// DATA
					//

					Vector center;

					Vector rad;
									/// Radius of sweeping sphere
					double radsphere;

					//
					// STREAMING
					//

					void StreamOUT(ChStreamOutBinary& mstream);

					void StreamIN(ChStreamInBinary& mstream);

				};

				}     // END_OF_NAMESPACE____
				}     // END_OF_NAMESPACE____

#endif
