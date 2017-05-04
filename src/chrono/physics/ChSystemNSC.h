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
//
// Physical system in which contact is modeled using a non-smooth
// (complementarity-based) method.
//
// =============================================================================

#ifndef CH_SYSTEM_NSC_H
#define CH_SYSTEM_NSC_H

#include "chrono/physics/ChSystem.h"

namespace chrono {

/// Class for a physical system in which contact is modeled using a non-smooth
/// (complementarity-based) method.
class ChApi ChSystemNSC : public ChSystem {
    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChSystemNSC)

  public:
    /// Create a physical system.
    /// Note, in case you will use collision detection, the values of
    /// 'max_objects' and 'scene_size' can be used to initialize the broadphase
    /// collision algorithm in an optimal way. Scene size should be approximately
    /// the radius of the expected area where colliding objects will move.
    /// The default collision broadphase does not make use of max_objects and scene_size.
    /// If init_sys is false it does not initialize the collision system or solver
    /// assumes that the user will do so.
    ChSystemNSC(unsigned int max_objects = 16000, double scene_size = 500, bool init_sys = true);

    /// Copy constructor
    ChSystemNSC(const ChSystemNSC& other);

    /// Destructor
    virtual ~ChSystemNSC() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChSystemNSC* Clone() const override { return new ChSystemNSC(*this); }

    /// Return the contact method supported by this system.
    /// Bodies added to this system must be compatible.
    virtual ChMaterialSurface::ContactMethod GetContactMethod() const override {
        return ChMaterialSurface::NSC;
    }

    /// Create a new body, consistent with the contact method and collision model used by this system.
    /// The returned body is not added to the system.
    virtual ChBody* NewBody() override { return new ChBody(ChMaterialSurface::NSC); }

    /// Create a new body with non-centroidal reference frame, consistent with the contact method and
    /// collision model used by this system.  The returned body is not added to the system.
    virtual ChBodyAuxRef* NewBodyAuxRef() override { return new ChBodyAuxRef(ChMaterialSurface::NSC); }

    /// Replace the contact continer.
    virtual void SetContactContainer(std::shared_ptr<ChContactContainer> container) override;

    // SERIALIZATION

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};

CH_CLASS_VERSION(ChSystemNSC, 0)

}  // end namespace chrono

#endif
