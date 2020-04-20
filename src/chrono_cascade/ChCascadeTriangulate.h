// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================

#ifndef CHCASCADETRIANGULATE_H
#define CHCASCADETRIANGULATE_H

#include "chrono/core/ChTypes.h"

namespace chrono {
namespace cascade {

/// @addtogroup cascade_module
/// @{

// This header includes a class for making easier to pass parameters on tesselation of Cascade shapes

/// Base class for storing settings on openCASCADE tesselation of shapes.
class ChCascadeTriangulate{
public:
	virtual ~ChCascadeTriangulate() {}

	// Support cloning
	virtual std::unique_ptr<ChCascadeTriangulate> clone() const = 0;
};

/// Class for storing settings on openCASCADE tesselation of shapes. 
class ChCascadeTriangulateTolerances : public ChCascadeTriangulate {
  public:
	  ChCascadeTriangulateTolerances(
		  double mdeflection = 0.05,	///< maximum allowed chordal deflection
		  bool mrel = false,			///< chordal deflection is assumed relative to triangle chord (default false)
		  double mang = 0.5				///< angular deflection
	  ) : deflection(mdeflection), deflection_is_relative(mrel), angular_deflection(mang) {}

	  double deflection; 
	  bool   deflection_is_relative;
	  double angular_deflection;

	// Support cloning
	virtual std::unique_ptr<ChCascadeTriangulate> clone() const override {
        return chrono_types::make_unique<ChCascadeTriangulateTolerances>(*this);
    }

};

/// Class for storing settings on openCASCADE tesselation of shapes. 
class ChCascadeTriangulateNone : public ChCascadeTriangulate {

	// Support cloning
	virtual std::unique_ptr<ChCascadeTriangulate> clone() const override {
        return chrono_types::make_unique<ChCascadeTriangulateNone>(*this);
    }
};


/// @} cascade_module

}  // end namespace cascade
}  // end namespace chrono

#endif
