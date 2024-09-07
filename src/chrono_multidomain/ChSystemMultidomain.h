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

#ifndef CHSYSTEMMULTIDOMAIN_H
#define CHSYSTEMMULTIDOMAIN_H

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono_multidomain/ChApiMultiDomain.h"
#include "chrono_multidomain/ChDomainManager.h"


namespace chrono {
namespace multidomain {

/// Custom ChSystem class that does SKIP adding loads to the shared nodes when not
/// master, otherwise fully parallel solvers would add twice the same load when 
/// exchanging deltas in updating the state.

class ChApiMultiDomain ChSystemNSCmultidomain : public ChSystemNSC {
  public:
    /// Create a physical system.
    ChSystemNSCmultidomain() {};

    /// Destructor
    virtual ~ChSystemNSCmultidomain() {};


    /// Increment a vector R with the term c*F:
    ///    R += c*F    
    /// EXCEPT FOR SHARED NODES OF "SLAVE" TYPE, WHERE LOAD IS SET TO ZERO 
    virtual void LoadResidual_F(ChVectorDynamic<>& R,  ///< result: the R residual, R += c*F
                                const double c         ///< a scaling factor
                                ) override;

    /// Increment a vector R with a term that has M multiplied a given vector w:
    ///    R += c*M*w
    /// EXCEPT FOR SHARED NODES OF "SLAVE" TYPE, WHERE LOAD IS SET TO ZERO 
    virtual void LoadResidual_Mv(ChVectorDynamic<>& R,        ///< result: the R residual, R += c*M*v
                                 const ChVectorDynamic<>& w,  ///< the w vector
                                 const double c               ///< a scaling factor
                                 ) override;

    std::shared_ptr<ChDomain> mdomain;

};


}  // end namespace multidomain
}  // end namespace chrono

#endif
