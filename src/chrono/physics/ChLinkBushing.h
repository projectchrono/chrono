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
// Authors: Antonio Recuero, Radu Serban
// =============================================================================
// Class that inherits from ChLinkLock as a free joint. Compliances are added
// to the relative motion between two rigid bodies. Out of the 6 possible dofs
// available to apply compliance, only those corresponding to the bushing type
// selected by the user are introduced.
// =============================================================================

#ifndef CHLINKBUSHING_H
#define CHLINKBUSHING_H

#include "chrono/physics/ChLinkLock.h"

namespace chrono {

/// Link with up to 6-degree-of-freedom linear compliance between rigid bodies.
/// In case a spherical or revolute joint is selected upon construction of the joint,
/// the compliance will only be applied to the 3 or 5 degrees of freedom, respectively,
/// which are constrained for the case of an ideal constraint.
class ChApi ChLinkBushing : public ChLinkLock {
  public:
    enum class Type {
        Mount,      ///< Mount bushing: 6 compliant degrees of freedom
        Spherical,  ///< Spherical bushing: 3 compliant degrees of freedom
    };

    /// Construct a bushing link (default: 6 compliant degrees of freedom).
    ChLinkBushing(Type bushing_type = Type::Mount);

    virtual ~ChLinkBushing();

    virtual ChLinkBushing* Clone() const override { return new ChLinkBushing(*this); }

    void Initialize(std::shared_ptr<ChBody> body1,
                    std::shared_ptr<ChBody> body2,
                    const ChCoordsys<>& pos,
                    const ChMatrixNM<double, 6, 6>& K,
                    const ChMatrixNM<double, 6, 6>& R);

    /// Return the current bushing force vector.
    ChVector<> GetForce() const;

    /// Return the current bushing torque vector (if rotational compliance).
    ChVector<> GetTorque() const;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

  private:
    using ChLinkMarkers::Initialize;

    Type m_type;  ///< bushing link type

    ChMatrixNM<double, 6, 6> m_constants_K;  ///< 6x6 matrices for linear stiffness- TODO, coupling terms
    ChMatrixNM<double, 6, 6> m_constants_R;  ///< 6x6 matrices for linear damping- TODO, coupling terms

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

CH_CLASS_VERSION(ChLinkBushing, 0)

}  // end namespace chrono

#endif
