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

#ifndef CHMATERIAL_H
#define CHMATERIAL_H

#include "chrono/core/ChApiCE.h"
#include "chrono/fea/ChFieldData.h"

namespace chrono {
namespace fea {

/// @addtogroup chrono_fea
/// @{



/// Base class for all material properties of continua. 
/// Materials inherited from this class are organized in a tree of subclasses that
/// introduce interfaces for 3D elasticity, 3D thermal problems, etc., each specialized
/// by additional subclasses. These are mostly used by ChDomain  objects for FEA problems.

class ChMaterial {
public:
    ChMaterial() {}
    virtual ~ChMaterial() {}

    /// Implement this if the material needs custom data per material point,
    /// returning a std::make_unique<ChFieldDataCustom>()  where ChFieldDataCustom is 
    /// your custom class with aditional states/properties per-point.
    virtual std::unique_ptr<ChFieldData> CreateMaterialPointData() const {
        return nullptr;
    }
};



/// @} chrono_fea

}  // end namespace fea

}  // end namespace chrono

#endif
