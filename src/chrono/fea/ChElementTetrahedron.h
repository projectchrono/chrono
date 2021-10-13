// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
// Base class for a FEA element with tetrahedral shape.
// =============================================================================

#ifndef CH_TERAHEDRON_H
#define CH_TERAHEDRON_H

#include "chrono/fea/ChNodeFEAxyz.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_elements
/// @{

/// Base class for a FEA element with tetrahedral shape.
class ChApi ChElementTetrahedron {
  public:
    ChElementTetrahedron() {}
    virtual ~ChElementTetrahedron() {}

    /// Return the specified tetrahedron node (0 <= n <= 3).
    virtual std::shared_ptr<ChNodeFEAxyz> GetTetrahedronNode(int n) = 0;
};

/// @} fea_elements

}  // namespace fea
}  // namespace chrono

#endif
