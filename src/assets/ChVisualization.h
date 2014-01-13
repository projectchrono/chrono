//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHVISUALIZATION_H
#define CHVISUALIZATION_H

///////////////////////////////////////////////////
//
//   ChVisualization.h
//
//   Base class for assets that define something about
//   visualization (rendering, post processing, etc.)
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "assets/ChAsset.h"
#include "assets/ChColor.h"
#include "core/ChMath.h"
using namespace chrono;

namespace chrono
{
/// Base class for assets that define something about
/// visualization (rendering, post processing, etc.)
/// It contains basic information about color and visibility.

class ChApi ChVisualization : public ChAsset {

protected:
				//
	  			// DATA
				//
	bool	visible;
	ChColor color;	
	float	fading;

public:
				//
	  			// CONSTRUCTORS
				//

	ChVisualization () : visible(true), color(1,1,1,0), fading(0) {};

	virtual ~ChVisualization () {};

				//
	  			// FUNCTIONS
				//

	bool IsVisible() const {return visible;}
	void SetVisible(bool mv) {visible = mv;}

		// Get the color of the surface. This information could be used by visualization postprocessing. 
	ChColor GetColor() const {return color;}
		// Set the color of the surface. This information could be used by visualization postprocessing.
	void SetColor(const ChColor& mc) {color = mc;}

		// Get the fading amount, 0..1. 
		// If =0, no transparency of surface, it =1 surface is completely transparent.
	float GetFading() const {return fading;}
		// Set the fading amount, 0..1. 
		// If =0, no transparency of surface, it =1 surface is completely transparent.
	void SetFading(const float mc) {fading = mc;}

				//
		  		// DATA
				//
	/// Rotation of Asset
	ChMatrix33<> Rot;
	/// Position of Asset
	ChVector<> Pos;
};




//////////////////////////////////////////////////////
//////////////////////////////////////////////////////


} // END_OF_NAMESPACE____

#endif
