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

#ifndef CH_MATERIAL_SURFACE_H
#define CH_MATERIAL_SURFACE_H

#include "chrono/core/ChClassFactory.h"
#include "chrono/serialization/ChArchive.h"

namespace chrono {

/// Base class for specifying material properties for contact force generation.
class ChApi ChMaterialSurface {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChMaterialSurface)

  public:
    enum ContactMethod {
        NSC,  ///< non-smooth, constraint-based (a.k.a. rigid-body) contact
        SMC   ///< smooth, penalty-based (a.k.a. soft-body) contact
    };

    virtual ~ChMaterialSurface() {}

    /// "Virtual" copy constructor.
    virtual ChMaterialSurface* Clone() const = 0;

    virtual ContactMethod GetContactMethod()const  = 0;

    // SERIALIZATION

    virtual void ArchiveOUT(ChArchiveOut& marchive) {
        // version number:
        marchive.VersionWrite<ChMaterialSurface>();
    }

    virtual void ArchiveIN(ChArchiveIn& marchive) {
        // version number:
        int version = marchive.VersionRead<ChMaterialSurface>();
    }
};

CH_CLASS_VERSION(ChMaterialSurface,0)

/// Base class for composite material for a contact pair.
class ChApi ChMaterialComposite {
  public:
    virtual ~ChMaterialComposite() {}
};

}  // end namespace chrono

#endif
