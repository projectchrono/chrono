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

#ifndef CHNODEFEAMULTIXYZ_H
#define CHNODEFEAMULTIXYZ_H

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChFrame.h"
#include "chrono/fea/ChNodeFEAbase.h"


namespace chrono {
namespace fea {

/// @addtogroup chrono_fea
/// @{

 
/// Class for a node of a generic 3D finite element node with x,y,z position.
/// This is the typical node that can be used for tetrahedrons, etc.
/// Note: this type of multiphysics node do not carry any ChVariable with it,
/// because attaching ChVariables is up to some ChFeaField. See also ChFeaDomain.
class ChNodeFEAmultiXYZ : public ChNodeFEAbase, public ChVector3d {
public:
    ChNodeFEAmultiXYZ(ChVector3d reference_pos = VNULL) : ChNodeFEAbase(), ChVector3d(reference_pos) {};
    ChNodeFEAmultiXYZ(const ChNodeFEAmultiXYZ& other) {};
    virtual ~ChNodeFEAmultiXYZ() {}

    //ChNodeFEAmultiXYZ& operator=(const ChNodeFEAmultiXYZ& other);

    void SetReferencePos(const ChVector3d ref_pos) { this->Set(ref_pos); }
    ChVector3d GetReferencePos() { return *this; }

    // INTERFACE to ChNodeFEAbase  
    // ***NOTE*** none of these are useful, neither supported, maybe in future
    // inherit from the empty ChFeaNode. Now do this way to reuse ChMesh stuff.

    virtual void Relax() override {}
    virtual void ForceToRest() override {}
    virtual void SetFixed(bool fixed) override {}
    virtual bool IsFixed() const override { return false; }
    virtual unsigned int GetNumCoordsPosLevel() const override { return 0; }

    // SERIALIZATION

    /// Method to allow serialization of transient data to archives.
    //virtual void ArchiveOut(ChArchiveOut& archive) override;

    /// Method to allow de-serialization of transient data from archives.
    //virtual void ArchiveIn(ChArchiveIn& archive) override;

protected:
};



/// @} chrono_fea

}  // end namespace fea


//CH_CLASS_VERSION(fea::ChNodeFEAmultiXYZ, 0)


}  // end namespace chrono

#endif
