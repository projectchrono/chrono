// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

#ifndef CH_MATERIAL_FEA_H
#define CH_MATERIAL_FEA_H

#include "chrono/core/ChApiCE.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_elements
/// @{

/// Base class for element materials.
class ChApi ChMaterialFEA {
  public:
    /// Set the material density.
    void SetDensity(double density) { m_density = density; }

    /// Return the material density.
    double GetDensity() const { return m_density; }

  protected:
    ChMaterialFEA() {}
    ChMaterialFEA(double density) : m_density(density) {}

    double m_density;  ///< density
};

/// @} fea_elements

}  // end of namespace fea
}  // end of namespace chrono

#endif
