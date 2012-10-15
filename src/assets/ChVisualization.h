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
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "assets/ChAsset.h"
#include "assets/ChColor.h"

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

};




//////////////////////////////////////////////////////
//////////////////////////////////////////////////////


} // END_OF_NAMESPACE____

#endif
