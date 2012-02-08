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


namespace chrono
{

/// Class for setting a color (used by ChVisualization)

class ChApi ChColor 
{
public:
		// Red, green, blue in 0..1 range.
	double R;
	double G;
	double B;
		// Alpha channell
	double A;	

		// Constructors
	ChColor() : R(1), G(1), B(1), A(0) {};
	ChColor(double mR, double mG, double mB, double mA=0) : R(mR), G(mG), B(mB), A(mA) {};
		/// Copy constructor
	ChColor(const ChColor& other) :R(other.R), G(other.G), B(other.B), A(other.A) {};
		/// Assignment: copy from another color
	ChColor& operator=(const ChColor& other) {if (&other == this) return *this;  R = other.R; G = other.G; B = other.B; A = other.A; return *this; }

		// Streaming:
	void StreamOUT(ChStreamOutAscii& mstream)
						{
							mstream << "\nRGB=" << R << "\n" << G << "\n" << B << " A=" << A << "\n";
						}
	void StreamOUT(ChStreamOutBinary& mstream)
						{
							mstream << R;
							mstream << G;
							mstream << B;
							mstream << A;
						}
	void StreamIN(ChStreamInBinary& mstream)
						{
							mstream >> R;
							mstream >> G;
							mstream >> B;
							mstream >> A;
						}
};



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

public:
				//
	  			// CONSTRUCTORS
				//

	ChVisualization () : visible(true), color(1,1,1,0) {};

	virtual ~ChVisualization () {};

				//
	  			// FUNCTIONS
				//

	bool IsVisible() const {return visible;}
	void SetVisible(bool mv) {visible = mv;}

	ChColor GetColor() const {return color;}
	void SetColor(const ChColor& mc) {color = mc;}

};




//////////////////////////////////////////////////////
//////////////////////////////////////////////////////


} // END_OF_NAMESPACE____

#endif
