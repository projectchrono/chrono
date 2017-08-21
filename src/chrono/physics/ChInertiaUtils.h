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

#ifndef CHINERTIAUTILS_H
#define CHINERTIAUTILS_H

#include <cstdlib>

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChMath.h"
#include "chrono/core/ChLinearAlgebra.h"

namespace chrono {

/// Class with some static functions that can be used to make
/// computations with inertia tensors

class ChInertiaUtils {
  public:
    /// Given a cluster of bodies, each with local inertia tensor, and position and rotation respect
    /// to absolute coordinates, compute the total mass, the barycenter and
    /// the inertia tensor of the cluster.
    /// Note: the resulting total inertia tensor is expressed in absolute coordinates
    /// Note: we assume that body masses are not overlapping
    /// Note: we assume that all local inertias are expressed in body local coords
    /// Note: we assume that all local inertias are expressed relative to body barycenter
    static void InertiaFromCluster(const std::vector<ChVector<> >& positions,
                                   const std::vector<ChMatrix33<> >& rotations,
                                   const std::vector<ChMatrix33<> >& Jlocal,
                                   const std::vector<double>& masses,
                                   ChMatrix33<>& totJ,
                                   double& totmass,
                                   ChVector<>& baricenter) {
        assert(positions.size() == Jlocal.size());
        assert(positions.size() == masses.size());

        totmass = 0;
        totJ.Reset();
        baricenter = VNULL;

        // compute tot mass and baricenter position
        for (unsigned int i = 0; i < positions.size(); ++i) {
            baricenter = (baricenter * totmass + positions[i] * masses[i]) / (totmass + masses[i]);
            totmass += masses[i];
        }

        // compute inertia
        for (unsigned int i = 0; i < positions.size(); ++i) {
            // rotate ith tensor in absolute space
            ChMatrix33<> R = rotations[i];
            ChMatrix33<> Rt = rotations[i];
            Rt.MatrTranspose();
            ChMatrix33<> absJ = R * Jlocal[i] * Rt;

            // Huygens-Steiner parallel axis theorem:

            ChVector<> dist = positions[i] - baricenter;

            ChMatrix33<> absJtranslated;
            absJtranslated = absJ;
            absJtranslated(0, 0) += masses[i] * (dist.Length2() - dist.x * dist.x);
            absJtranslated(1, 1) += masses[i] * (dist.Length2() - dist.y * dist.y);
            absJtranslated(2, 2) += masses[i] * (dist.Length2() - dist.z * dist.z);
            absJtranslated(0, 1) += masses[i] * (-dist.x * dist.y);
            absJtranslated(0, 2) += masses[i] * (-dist.x * dist.z);
            absJtranslated(1, 2) += masses[i] * (-dist.y * dist.z);
            // symmetric part
            absJtranslated(1, 0) = absJtranslated(0, 1);
            absJtranslated(2, 0) = absJtranslated(0, 2);
            absJtranslated(2, 1) = absJtranslated(1, 2);

            totJ += absJtranslated;
        }
    }

    /// Rotate an inertia tensor ,given a rotation matrix R
    static void RotateInertia(const ChMatrix33<> inertiaIn, const ChMatrix33<> R, ChMatrix33<>& inertiaOut) {
        ChMatrix33<> Rt = R;
        Rt.MatrTranspose();
        inertiaOut = R * inertiaIn * Rt;
    }

    /// Translate an inertia tensor to a non-barycentric reference,
    /// given a displacement 'dist', using the Huygens-Steiner
    /// parallel axis theorem.
    static void TranslateInertia(const ChMatrix33<> inertiaIn,
                                 const ChVector<> dist,
                                 const double mass,
                                 ChMatrix33<>& inertiaOut) {
        // Huygens-Steiner parallel axis theorem:
        inertiaOut = inertiaIn;
        inertiaOut(0, 0) += mass * (dist.Length2() - dist.x * dist.x);
        inertiaOut(1, 1) += mass * (dist.Length2() - dist.y * dist.y);
        inertiaOut(2, 2) += mass * (dist.Length2() - dist.z * dist.z);
        inertiaOut(0, 1) += mass * (-dist.x * dist.y);
        inertiaOut(0, 2) += mass * (-dist.x * dist.z);
        inertiaOut(1, 2) += mass * (-dist.y * dist.z);
        // symmetric part
        inertiaOut(1, 0) = inertiaOut(0, 1);
        inertiaOut(2, 0) = inertiaOut(0, 2);
        inertiaOut(2, 1) = inertiaOut(1, 2);
    }
};

}  // end namespace chrono

#endif
