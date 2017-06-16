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
// Author: A.Tasora

#ifndef CHLINESHAPE_H
#define CHLINESHAPE_H

#include "chrono/assets/ChAsset.h"

#include <TopoDS_Shape.hxx>



namespace chrono {

namespace cascade {

/// @addtogroup cascade_module
/// @{

/// Class for an asset that contains an OpenCASCADE shape.
/// In this way one can attach a 3D cad shape to a physics item.

class ChApiCASCADE ChCascadeShapeAsset : public chrono::ChAsset {
    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChCascadeShapeAsset)

  protected:
    //
    // DATA
    //
    TopoDS_Shape mshape;

  public:
    //
    // CONSTRUCTORS
    //

    ChCascadeShapeAsset() {
        
    };
    ChCascadeShapeAsset(const TopoDS_Shape& ms) : mshape(ms){};

    virtual ~ChCascadeShapeAsset(){};

    //
    // FUNCTIONS
    //

    // Access the OpenCASCADE shape
    TopoDS_Shape& Shape() { return mshape; }


    //
    // SERIALIZATION
    //

    virtual void ArchiveOUT(ChArchiveOut& marchive)
    {
        // version number
        marchive.VersionWrite<ChCascadeShapeAsset>();
        // serialize parent class
        ChAsset::ArchiveOUT(marchive);
        // serialize all member data:
        //marchive << ...; //***TODO*** serialize shape chunk using Cascade xml or STEP formats
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) 
    {
        // version number
        int version = marchive.VersionRead<ChCascadeShapeAsset>();
        // deserialize parent class
        ChAsset::ArchiveIN(marchive);
        // stream in all member data:
        //marchive >> ...; //***TODO*** deserialize shape chunk using Cascade xml or STEP formats
    }
};


/// @} cascade_module

}  // END_OF_NAMESPACE____
}  // END_OF_NAMESPACE____


#endif
