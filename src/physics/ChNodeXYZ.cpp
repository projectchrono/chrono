//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#include "physics/ChNodeXYZ.h"

namespace chrono {

ChNodeXYZ::ChNodeXYZ() {
    this->pos = VNULL;
    this->pos_dt = VNULL;
    this->pos_dtdt = VNULL;
}

ChNodeXYZ::~ChNodeXYZ() {
}

ChNodeXYZ::ChNodeXYZ(const ChNodeXYZ& other) : ChNodeBase(other) {
    this->pos = other.pos;
    this->pos_dt = other.pos_dt;
    this->pos_dtdt = other.pos_dtdt;
}

ChNodeXYZ& ChNodeXYZ::operator=(const ChNodeXYZ& other) {
    if (&other == this)
        return *this;

    ChNodeBase::operator=(other);

    this->pos = other.pos;
    this->pos_dt = other.pos_dt;
    this->pos_dtdt = other.pos_dtdt;

    return *this;
}

void ChNodeXYZ::ContactForceLoadResidual_F(const ChVector<>& F, const ChVector<>& abs_point, 
                            const unsigned int off, ChVectorDynamic<>& R, const double c) {
    R.PasteSumVector(F * c, off + 0, 0);
}

void ChNodeXYZ::ComputeJacobianForContactPart(const ChVector<>& abs_point, ChMatrix33<>& contact_plane, 
            type_constraint_tuple& jacobian_tuple_N, 
            type_constraint_tuple& jacobian_tuple_U, 
            type_constraint_tuple& jacobian_tuple_V, 
            bool second) {
    ChMatrix33<> Jx1;

    Jx1.CopyFromMatrixT(contact_plane);
    if (!second)
        Jx1.MatrNeg();

    jacobian_tuple_N.Get_Cq()->PasteClippedMatrix(&Jx1, 0, 0, 1, 3, 0, 0);
    jacobian_tuple_U.Get_Cq()->PasteClippedMatrix(&Jx1, 1, 0, 1, 3, 0, 0);
    jacobian_tuple_V.Get_Cq()->PasteClippedMatrix(&Jx1, 2, 0, 1, 3, 0, 0);
}


}  // END_OF_NAMESPACE____

/////////////////////
