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

#include "chrono/core/ChRunTimeType.h"
#include "chrono/serialization/ChArchive.h"

namespace chrono {
///
/// Base class for specifying material properties for contact force
/// generation.
///
class ChApi ChMaterialSurfaceBase {

    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI_ROOT(ChMaterialSurfaceBase);

  public:
    enum ContactMethod {
      DVI,  ///< constraint-based (a.k.a. rigid-body) contact
      DEM   ///< penalty-based (a.k.a. soft-body) contact
    };

    virtual ContactMethod GetContactMethod() = 0;

    // SERIALIZATION

    virtual void ArchiveOUT(ChArchiveOut& marchive) {
        // version number:
        marchive.VersionWrite(1);
        // serialize parent class:
        // serialize all member data:
    };
    virtual void ArchiveIN(ChArchiveIn& marchive) {
        // version number:
        int version = marchive.VersionRead();
        // deserialize parent class:
        // stream in all member data:
    };

};

}  // END_OF_NAMESPACE____

#endif
