//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2011-2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHMATERIALSURFACEBASE_H
#define CHMATERIALSURFACEBASE_H

#include "core/ChShared.h"
#include "serialization/ChArchive.h"

namespace chrono {
///
/// Base class for specifying material properties for contact force
/// generation.
///
class ChApi ChMaterialSurfaceBase : public ChShared {

    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChMaterialSurfaceBase, ChShared);

  public:
    enum ContactMethod {
      DVI,  ///< constraint-based (a.k.a. rigid-body) contact
      DEM   ///< penalty-based (a.k.a. soft-body) contact
    };

    virtual ContactMethod GetContactMethod() = 0;

    // SERIALIZATION

    virtual void ArchiveOUT(ChArchiveOut& marchive) {};
    virtual void ArchiveIN(ChArchiveIn& marchive) {};
    //***OBSOLETE***
    virtual void StreamOUT(ChStreamOutAscii& mstream) = 0;
    virtual void StreamOUT(ChStreamOutBinary& mstream) = 0;
    virtual void StreamIN(ChStreamInBinary& mstream) = 0;
};

}  // END_OF_NAMESPACE____

#endif
