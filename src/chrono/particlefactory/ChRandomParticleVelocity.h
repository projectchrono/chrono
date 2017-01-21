//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//
// File author: A.Tasora

#ifndef CHRANDOMPARTICLEVELOCITY_H
#define CHRANDOMPARTICLEVELOCITY_H

#include <memory>

#include "chrono/core/ChMathematics.h"
#include "chrono/core/ChVector.h"
#include "chrono/core/ChMatrix.h"
#include "chrono/core/ChDistribution.h"

namespace chrono {
namespace particlefactory {

/// BASE class for generators of random particle velocities.
/// By default it simply always returns Vxyz={0,0,0}, so it
/// it is up to sub-classes to implement more sophisticated randomizations.
class ChRandomParticleVelocity {
  public:
    ChRandomParticleVelocity() {}
    virtual ~ChRandomParticleVelocity() {}

    /// Function that creates a random velocity each
    /// time it is called.
    /// This base behavior simply uses zero velocity by default.
    /// Children classes implmeent more advanced velocity randomizations.
    virtual ChVector<> RandomVelocity() { return VNULL; }
};

/// Generator of random particle velocities with constant direction.
/// Modulus is constant by default, too, but it can be set as
/// randomized via a statistical distribution.
class ChRandomParticleVelocityConstantDirection : public ChRandomParticleVelocity {
  public:
    ChRandomParticleVelocityConstantDirection() {
        direction = VECT_X;
        modulus = std::make_shared<ChConstantDistribution>(0.1);
    }

    /// Function that creates a random velocity each
    /// time it is called.
    virtual ChVector<> RandomVelocity() override { return direction * modulus->GetRandom(); }

    /// Set the direction for all the randomized velocities
    void SetDirection(ChVector<> mdir) { direction = mdir.GetNormalized(); }

    /// Set the statistical distribution for the modulus of velocities
    void SetModulusDistribution(std::shared_ptr<ChDistribution> mdistr) { modulus = mdistr; }
    /// Set the modulus of velocities as a constant value
    void SetModulusDistribution(double mval) {
        modulus = std::make_shared<ChConstantDistribution>(mval);
    }

  private:
    ChVector<> direction;
    std::shared_ptr<ChDistribution> modulus;
};

/// Generator of random particle velocities with any direction.
/// Modulus is constant by default, too, but it can be set as
/// randomized via a statistical distribution.
class ChRandomParticleVelocityAnyDirection : public ChRandomParticleVelocity {
  public:
    ChRandomParticleVelocityAnyDirection() {
        modulus = std::make_shared<ChConstantDistribution>(0.1);
    }

    /// Function that creates a random velocity each
    /// time it is called.
    virtual ChVector<> RandomVelocity() override {
        ChVector<> random_direction(ChRandom() - 0.5, ChRandom() - 0.5, ChRandom() - 0.5);
        random_direction.Normalize();

        return random_direction * modulus->GetRandom();
    }

    /// Set the statistical distribution for the modulus of velocities
    void SetModulusDistribution(std::shared_ptr<ChDistribution> mdistr) { modulus = mdistr; }
    /// Set the modulus of velocities as a constant value
    void SetModulusDistribution(double mval) {
        modulus = std::make_shared<ChConstantDistribution>(mval);
    }

  private:
    std::shared_ptr<ChDistribution> modulus;
};

}  // end of namespace particlefactory
}  // end of namespace chrono

#endif
