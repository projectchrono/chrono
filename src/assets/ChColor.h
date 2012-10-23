#ifndef CHCOLOR_H
#define CHCOLOR_H

///////////////////////////////////////////////////
//
//   ChColor.h
//
//   Class for storing a color as an object asset
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
///////////////////////////////////////////////////

#include "assets/ChAsset.h"

namespace chrono {
/// Class for setting a color (used by ChVisualization)

class ChApi ChColor {
public:
	// Red, green, blue in 0..1 range.
	float R;
	float G;
	float B;
	// Alpha channell
	float A;

	// Constructors
	ChColor() :
			R(1), G(1), B(1), A(0) {
	}
	;
	ChColor(float mR, float mG, float mB, float mA = 0) :
			R(mR), G(mG), B(mB), A(mA) {
	}
	;
	/// Copy constructor
	ChColor(const ChColor& other) :
			R(other.R), G(other.G), B(other.B), A(other.A) {
	}
	;
	/// Assignment: copy from another color
	ChColor& operator=(const ChColor& other) {
		if (&other == this)
			return *this;
		R = other.R;
		G = other.G;
		B = other.B;
		A = other.A;
		return *this;
	}

	// Streaming:
	void StreamOUT(ChStreamOutAscii& mstream) {
		mstream << "\nRGB=" << R << "\n" << G << "\n" << B << " A=" << A << "\n";
	}
	void StreamOUT(ChStreamOutBinary& mstream) {
		mstream << R;
		mstream << G;
		mstream << B;
		mstream << A;
	}
	void StreamIN(ChStreamInBinary& mstream) {
		mstream >> R;
		mstream >> G;
		mstream >> B;
		mstream >> A;
	}
};

}
#endif
