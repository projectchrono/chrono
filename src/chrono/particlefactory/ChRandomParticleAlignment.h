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

#ifndef CHRANDOMPARTICLEALIGNMENT_H
#define CHRANDOMPARTICLEALIGNMENT_H

#include "chrono/core/ChMathematics.h"
#include "chrono/core/ChVector.h"
#include "chrono/core/ChMatrix.h"
#include "chrono/core/ChDistribution.h"

namespace chrono {
namespace particlefactory {

/// BASE class for generators of random particle alignment.
/// By default it simply always returns quaternion {1,0,0,0} (no rotation) so it
/// it is up to sub-classes to implement more sophisticated randomizations.
class ChRandomParticleAlignment {
  public:
    ChRandomParticleAlignment() {}

    /// Function that creates a random alignment (as a rotation quaternion) each
    /// time it is called.
    /// Note: it must be implemented by children classes!
    virtual ChQuaternion<> RandomAlignment() { return QUNIT; }
};

/// Class for generator of random particle alignment.
/// The S3 space is not uniformly sampled - this is a quick approximation anyway.
class ChRandomParticleAlignmentUniform : public ChRandomParticleAlignment {
  public:
    ChRandomParticleAlignmentUniform() {}

    /// Function that creates a random alignment (as a rotation quaternion) each
    /// time it is called. The alignment is a random polar rotation.
    virtual ChQuaternion<> RandomAlignment() {
        ChQuaternion<> mq(1. - ChRandom() * 2., 1. - ChRandom() * 2., 1. - ChRandom() * 2., 1. - ChRandom() * 2.);
        return mq.GetNormalized();
    }
};

}  // end of namespace particlefactory
}  // end of namespace chrono

#endif
