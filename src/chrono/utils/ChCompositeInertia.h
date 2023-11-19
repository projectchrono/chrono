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
// Authors: Radu Serban
// =============================================================================

#ifndef CH_COMPOSITE_INERTIA_H
#define CH_COMPOSITE_INERTIA_H

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChVector.h"
#include "chrono/core/ChMatrix33.h"
#include "chrono/core/ChFrame.h"

namespace chrono {
namespace utils {

/// @addtogroup chrono_utils
/// @{

/// Utility class for calculating inertia properties of a composite body.
/// A composite body is a collection of sub-components, each specified through
/// its mass, a centroidal frame, and its inertia tensor (w.r.t. the centroidal frame).
/// New sub-components can be included in the composite body and optionally
/// marked as "voids" (holes), by specifying:
/// - the mass of the sub-component
/// - a sub-component centroidal frame, relative to the composite reference frame;
///   the location of this frame represents the sub-component COM location
/// - the sub-component inertia tensor w.r.t. its centroidal frame

class ChApi CompositeInertia {
  public:
    CompositeInertia();

    /// Get the composite mass.
    double GetMass() const { return m_mass; }

    /// Get the location of the COM.
    const ChVector<>& GetCOM() const { return m_com; }

    /// Get the inertia tensor w.r.t. a centroidal frame.
    /// The return 3x3 symmetric matrix represents the inertia tensor with respect to a
    /// centroidal frame of the composite, aligned with the axes of the reference frame.
    ChMatrix33<> GetInertia() const;

    /// Get the inertia tensor w.r.t. the reference frame.
    const ChMatrix33<>& GetInertiaReference() const { return m_inertia; }

    /// Include sub-component inertia properties.
    /// Update the inertia properties of the composite object with the specified
    /// mass and inertia of a sub-component. A sub-component is
    void AddComponent(const ChFrame<>& frame,       ///< centroidal frame of sub-component (relative to reference frame)
                      double mass,                  ///< mass of sub-component
                      const ChMatrix33<>& inertia,  ///< sub-component inertia tensor w.r.t. its centroidal frame
                      bool is_void = false          ///< indicate if sub-component represents a material void
                      );

    /// Utility function for calculating an inertia shift matrix from a given vector.
    /// This matrix is used when applying the parallel axis theorem.
    static ChMatrix33<> InertiaShiftMatrix(const ChVector<>& v);

  private:
    ChMatrix33<> m_inertia;  ///< inertia tensor w.r.t reference frame
    ChVector<> m_com;        ///< location of COM (relative to reference frame)
    double m_mass;           ///< mass of composite body
};

/// @} chrono_utils

}  // end namespace utils
}  // end namespace chrono

#endif
