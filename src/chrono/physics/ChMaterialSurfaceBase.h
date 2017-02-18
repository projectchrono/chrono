// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHMATERIALSURFACEBASE_H
#define CHMATERIALSURFACEBASE_H

#include "chrono/core/ChClassFactory.h"
#include "chrono/serialization/ChArchive.h"

namespace chrono {

/// Base class for specifying material properties for contact force generation.

class ChApi ChMaterialSurfaceBase {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChMaterialSurfaceBase)

  public:
    enum ContactMethod {
        DVI,  ///< constraint-based (a.k.a. rigid-body) contact
        DEM   ///< penalty-based (a.k.a. soft-body) contact
    };

    virtual ~ChMaterialSurfaceBase() {}

    /// "Virtual" copy constructor.
    virtual ChMaterialSurfaceBase* Clone() const = 0;

    virtual ContactMethod GetContactMethod()const  = 0;

    // SERIALIZATION

    virtual void ArchiveOUT(ChArchiveOut& marchive) {
        // version number:
        marchive.VersionWrite<ChMaterialSurfaceBase>();
    }

    virtual void ArchiveIN(ChArchiveIn& marchive) {
        // version number:
        int version = marchive.VersionRead<ChMaterialSurfaceBase>();
    }
};

CH_CLASS_VERSION(ChMaterialSurfaceBase,0)


}  // end namespace chrono

#endif
