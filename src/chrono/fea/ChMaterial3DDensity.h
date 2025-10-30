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

#ifndef CHMATERIAL3DDENSITY_H
#define CHMATERIAL3DDENSITY_H

#include "chrono/fea/ChMaterial.h"


namespace chrono {
namespace fea {

/// @addtogroup chrono_fea
/// @{



/// Base class for density in a continuum.
class ChMaterial3DDensity : public ChMaterial {
public:
    ChMaterial3DDensity(double density = 1000) : m_density(density) {}

    virtual ~ChMaterial3DDensity() {}

    /// Set the density of the material, in kg/m^2.
    void SetDensity(double density) { m_density = density; }

    /// Get the density of the material, in kg/m^2.
    double GetDensity() const { return m_density; }

    //virtual void ArchiveOut(ChArchiveOut& archive_out);
    //virtual void ArchiveIn(ChArchiveIn& archive_in);

protected:
    double m_density;
};



/// @} chrono_fea

}  // end namespace fea

}  // end namespace chrono

#endif
