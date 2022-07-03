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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include <cstdio>

#include "chrono/core/ChTransform.h"
#include "chrono/geometry/ChRoundedBox.h"

namespace chrono {
namespace geometry {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChRoundedBox)

ChRoundedBox::ChRoundedBox(const ChRoundedBox& source) {
    Size = source.Size;
}

/*
ChRoundedBox::ChRoundedBox(const ChVector<>& mC0, const ChVector<>& mC1, const ChVector<>& mC2, const ChVector<>& mC3) {
    ChVector<> D1 = Vsub(mC1, mC0);
    ChVector<> D2 = Vsub(mC2, mC0);
    ChVector<> D3 = Vsub(mC3, mC0);
    ChVector<> C0 = mC0;

    ChVector<> zax;
    zax.Cross(D1, D2);
    if (Vdot(D3, zax) < 0) {
        C0 += D3;
        D3 = -D3;
    }

    this->Size.x() = 0.5 * Vlength(D1);
    this->Size.y() = 0.5 * Vlength(D2);
    this->Size.z() = 0.5 * Vlength(D3);
    this->Pos = Vadd(Vadd(Vadd(C0, Vmul(D1, 0.5)), Vmul(D2, 0.5)), Vmul(D3, 0.5));
    this->Rot.Set_A_axis(Vnorm(D1), Vnorm(D2), Vnorm(D3));
}
*/

void ChRoundedBox::Evaluate(ChVector<>& pos, const double parU, const double parV, const double parW) const {
    pos.x() = Size.x() * (parU - 0.5);
    pos.y() = Size.y() * (parV - 0.5);
    pos.z() = Size.z() * (parW - 0.5);
}

ChVector<> ChRoundedBox::GetP1() const {
    return ChVector<>(+Size.x(), +Size.y(), +Size.z());
}
ChVector<> ChRoundedBox::GetP2() const {
    return ChVector<>(-Size.x(), +Size.y(), +Size.z());
}
ChVector<> ChRoundedBox::GetP3() const {
    return ChVector<>(-Size.x(), -Size.y(), +Size.z());
}
ChVector<> ChRoundedBox::GetP4() const {
    return ChVector<>(+Size.x(), -Size.y(), +Size.z());
}
ChVector<> ChRoundedBox::GetP5() const {
    return ChVector<>(+Size.x(), +Size.y(), -Size.z());
}
ChVector<> ChRoundedBox::GetP6() const {
    return ChVector<>(-Size.x(), +Size.y(), -Size.z());
}
ChVector<> ChRoundedBox::GetP7() const {
    return ChVector<>(-Size.x(), -Size.y(), -Size.z());
}
ChVector<> ChRoundedBox::GetP8() const {
    return ChVector<>(+Size.x(), -Size.y(), -Size.z());
}

ChVector<> ChRoundedBox::GetPn(int ipoint) const {
    switch (ipoint) {
        case 1:
            return GetP1();
        case 2:
            return GetP2();
        case 3:
            return GetP3();
        case 4:
            return GetP4();
        case 5:
            return GetP5();
        case 6:
            return GetP6();
        case 7:
            return GetP7();
        case 8:
            return GetP8();
        default:
            return GetP1();
    }
}
void ChRoundedBox::GetBoundingBox(ChVector<>& cmin, ChVector<>& cmax, const ChMatrix33<>& rot) const {
    std::vector<ChVector<>> vertices{GetP1(), GetP2(), GetP3(), GetP4(), GetP5(), GetP6(), GetP7(), GetP8()};

    cmin = ChVector<>(+std::numeric_limits<double>::max());
    cmax = ChVector<>(-std::numeric_limits<double>::max());

    for (const auto& v : vertices) {
        auto p = rot.transpose() * v;

        cmin.x() = ChMin(cmin.x(), p.x());
        cmin.y() = ChMin(cmin.y(), p.y());
        cmin.z() = ChMin(cmin.z(), p.z());

        cmax.x() = ChMax(cmax.x(), p.x());
        cmax.y() = ChMax(cmax.y(), p.y());
        cmax.z() = ChMax(cmax.z(), p.z());
    }
}

void ChRoundedBox::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChRoundedBox>();
    // serialize parent class
    ChVolume::ArchiveOUT(marchive);
    // serialize all member data:
    ChVector<> Lengths = GetLengths();
    marchive << CHNVP(Lengths);  // avoid storing 'Size', i.e. half lengths, because less intuitive
    marchive << CHNVP(radsphere);
}

void ChRoundedBox::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChRoundedBox>();
    // deserialize parent class
    ChVolume::ArchiveIN(marchive);
    // stream in all member data:
    ChVector<> Lengths;
    marchive >> CHNVP(Lengths);
    SetLengths(Lengths);
    marchive >> CHNVP(radsphere);
}

}  // end namespace geometry
}  // end namespace chrono
