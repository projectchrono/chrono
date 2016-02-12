//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2011 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   ChBodyAuxRef.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "physics/ChBodyAuxRef.h"

namespace chrono {

using namespace collision;
using namespace geometry;

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChBodyAuxRef> a_registration_ChBodyAuxRef;

// Hierarchy-handling shortcuts

#define MARKpointer (*imarker)
#define HIER_MARKER_INIT std::vector<std::shared_ptr<ChMarker> >::iterator imarker = marklist.begin();
#define HIER_MARKER_NOSTOP (imarker != marklist.end())
#define HIER_MARKER_NEXT imarker++;

#define FORCEpointer (*iforce)
#define HIER_FORCE_INIT std::vector<std::shared_ptr<ChForce> >::iterator iforce = forcelist.begin();
#define HIER_FORCE_NOSTOP (iforce != forcelist.end())
#define HIER_FORCE_NEXT iforce++;

//////////////////////////////////////
//////////////////////////////////////

/// CLASS FOR SOLID BODIES WITH AUXILIARY REFERENCE

void ChBodyAuxRef::Copy(ChBodyAuxRef* source) {
    // copy the parent class data...
    ChBody::Copy(source);

    // copy own data
    this->auxref_to_cog = source->auxref_to_cog;
    this->auxref_to_abs = source->auxref_to_abs;
}

void ChBodyAuxRef::SetFrame_COG_to_REF(const ChFrame<>& mloc) {
    ChFrameMoving<> old_cog_to_abs = *this;

    ChFrameMoving<> tmpref_to_abs;
    this->TransformLocalToParent(this->auxref_to_cog, tmpref_to_abs);
    tmpref_to_abs.TransformLocalToParent(ChFrameMoving<>(mloc), *this);
    // or, also, using overloaded operators for frames:
    //   tmpref_to_abs = auxref_to_cog >> *this;
    //   *this         = ChFrameMoving<>(mloc) >> tmpref_to_abs;

    ChFrameMoving<> new_cog_to_abs = *this;

    auxref_to_cog = mloc.GetInverse();
    this->TransformLocalToParent(this->auxref_to_cog, this->auxref_to_abs);
    // or, also, using overloaded operators for frames:
    //   *this->auxref_to_abs = this->auxref_to_cog.GetInverse() >> this;

    // Restore marker/forces positions, keeping unchanged respect to aux ref.
    ChFrameMoving<> cog_oldnew = old_cog_to_abs >> new_cog_to_abs.GetInverse();

    HIER_MARKER_INIT
    while (HIER_MARKER_NOSTOP) {
        MARKpointer->ConcatenatePreTransformation(cog_oldnew);
        MARKpointer->Update(this->ChTime);

        HIER_MARKER_NEXT
    }

    // Forces: ?? to do...
    /*
    HIER_FORCE_INIT
    while (HIER_FORCE_NOSTOP)
    {
        FORCEpointer->
        FORCEpointer->Update (mytime);

        HIER_FORCE_NEXT
    }
    */
}

void ChBodyAuxRef::SetFrame_REF_to_abs(const ChFrame<>& mfra) {
    mfra.TransformLocalToParent(this->auxref_to_cog.GetInverse(), *this);
    // or, also, using overloaded operators for frames:
    //   *this = this->auxref_to_cog.GetInverse() >> mfra;

    auxref_to_abs = mfra;
}

void ChBodyAuxRef::Update(bool update_assets) {
    // update parent class
    ChBody::Update(update_assets);

    // update own data
    this->TransformLocalToParent(this->auxref_to_cog, this->auxref_to_abs);
}

//////// FILE I/O

void ChBodyAuxRef::ArchiveOUT(ChArchiveOut& marchive)
{
    // version number
    marchive.VersionWrite(1);

    // serialize parent class
    ChBody::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(auxref_to_cog);
    marchive << CHNVP(auxref_to_abs);
}

/// Method to allow de serialization of transient data from archives.
void ChBodyAuxRef::ArchiveIN(ChArchiveIn& marchive) 
{
    // version number
    int version = marchive.VersionRead();

    // deserialize parent class
    ChBody::ArchiveIN(marchive);

    // stream in all member data:
    marchive >> CHNVP(auxref_to_cog);
    marchive >> CHNVP(auxref_to_abs);
}


}  // END_OF_NAMESPACE____

/////////////////////
