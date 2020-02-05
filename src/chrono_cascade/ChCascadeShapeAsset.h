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
// Authors: Alessandro Tasora
// =============================================================================

#ifndef CHCASCADESHAPEASSET_H
#define CHCASCADESHAPEASSET_H

#include "chrono_cascade/ChApiCASCADE.h"
#include "chrono/assets/ChAsset.h"

#include <TopoDS_Shape.hxx>



namespace chrono {

namespace cascade {

/// @addtogroup cascade_module
/// @{

/// Class for an asset that contains an OpenCASCADE shape.
/// In this way one can attach a 3D cad shape to a physics item.

class ChApiCASCADE ChCascadeShapeAsset : public chrono::ChAsset {

  protected:
    //
    // DATA
    //
    TopoDS_Shape mshape;

  public:
    //
    // CONSTRUCTORS
    //

    ChCascadeShapeAsset();

    ChCascadeShapeAsset(const TopoDS_Shape& ms);

    virtual ~ChCascadeShapeAsset();

    //
    // FUNCTIONS
    //

    // Access the OpenCASCADE shape
    TopoDS_Shape& Shape() { return mshape; }


    //
    // SERIALIZATION
    //

	virtual void ArchiveOUT(ChArchiveOut& marchive);
    
	virtual void ArchiveIN(ChArchiveIn& marchive);
    
};


/// @} cascade_module

}  // END_OF_NAMESPACE____
}  // END_OF_NAMESPACE____


#endif
