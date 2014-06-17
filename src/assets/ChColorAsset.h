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

#ifndef CHCOLORASSET_H
#define CHCOLORASSET_H


#include "assets/ChAsset.h"
#include "assets/ChColor.h"
#include <string>
#include <vector>


namespace chrono {




/// Base class for assets that carry basic informations about
/// the surface color for visualization assets.
/// Here we assume that a ChColorAsset 'paints'
/// all the assets of the list (or in same ChAssetLevel).
/// It can be used when exporting to postprocessing or
/// in OpenGL views. The postprocessor must be able to
/// recognize it and implement the proper translation.

class ChApi ChColorAsset : public ChAsset {

public:

	ChColorAsset() 
	{
		fading = 0;
	};

	~ChColorAsset() {};

		/// Get the color of the surface. This information could be used by visualization postprocessing.
	ChColor GetColor() const {
		return color;
	}
		/// Set the color of the surface. This information could be used by visualization postprocessing.
	void SetColor(const ChColor& mc) {
		color = mc;
	}

		/// Get the fading amount, 0..1.
		/// If =0, no transparency of surface, it =1 surface is completely transparent.
	float GetFading() const {
		return fading;
	}
		/// Set the fading amount, 0..1.
		/// If =0, no transparency of surface, it =1 surface is completely transparent.
	void SetFading(const float mc) {
		fading = mc;
	}

	ChColor color; //color of material
	float fading; //transparency of material

};

}
#endif
