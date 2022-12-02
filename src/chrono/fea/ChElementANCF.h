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

#ifndef CH_ELEMENT_ANCF_H
#define CH_ELEMENT_ANCF_H

#include "chrono/core/ChQuadrature.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_elements
/// @{

/// Base class for ANCF elements.
class ChApi ChElementANCF {
  public:
    ChElementANCF() : m_full_dof(true), m_element_dof(0) {}
    virtual ~ChElementANCF() {}

  protected:
    int m_element_dof;           ///< actual number of degrees of freedom for the element
    bool m_full_dof;             ///< true if all node variables are active (not fixed)
    ChArray<int> m_mapping_dof;  ///< indices of active DOFs (set only is some are fixed)
};

/// @} fea_elements

}  // end namespace fea
}  // end namespace chrono

#endif
