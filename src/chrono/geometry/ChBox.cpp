// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
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
#include "chrono/geometry/ChBox.h"

namespace chrono {
namespace geometry {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChBox)

ChBox::ChBox(const ChVector<>& mpos, const ChMatrix33<>& mrot, const ChVector<>& mlengths)
    : Pos(mpos), Rot(mrot), Size(0.5 * mlengths) {}

ChBox::ChBox(ChVector<>& mC0, ChVector<>& mC1, ChVector<>& mC2, ChVector<>& mC3) {
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

ChBox::ChBox(const ChBox& source) {
    Pos = source.Pos;
    Size = source.Size;
    Rot.CopyFromMatrix(source.Rot);
}

void ChBox::Evaluate(ChVector<>& pos, const double parU, const double parV, const double parW) const {
    ChVector<> Pr;
    Pr.x() = Size.x() * (parU - 0.5);
    Pr.y() = Size.y() * (parV - 0.5);
    Pr.z() = Size.z() * (parW - 0.5);
    pos = ChTransform<>::TransformLocalToParent(Pr, Pos, Rot);
}

ChVector<> ChBox::GetP1() const {
    ChVector<> P1r;
    P1r.x() = +Size.x();
    P1r.y() = +Size.y();
    P1r.z() = +Size.z();
    return ChTransform<>::TransformLocalToParent(P1r, Pos, Rot);
}
ChVector<> ChBox::GetP2() const {
    ChVector<> P2r;
    P2r.x() = -Size.x();
    P2r.y() = +Size.y();
    P2r.z() = +Size.z();
    return ChTransform<>::TransformLocalToParent(P2r, Pos, Rot);
}
ChVector<> ChBox::GetP3() const {
    ChVector<> P3r;
    P3r.x() = -Size.x();
    P3r.y() = -Size.y();
    P3r.z() = +Size.z();
    return ChTransform<>::TransformLocalToParent(P3r, Pos, Rot);
}
ChVector<> ChBox::GetP4() const {
    ChVector<> P4r;
    P4r.x() = +Size.x();
    P4r.y() = -Size.y();
    P4r.z() = +Size.z();
    return ChTransform<>::TransformLocalToParent(P4r, Pos, Rot);
}
ChVector<> ChBox::GetP5() const {
    ChVector<> P5r;
    P5r.x() = +Size.x();
    P5r.y() = +Size.y();
    P5r.z() = -Size.z();
    return ChTransform<>::TransformLocalToParent(P5r, Pos, Rot);
}
ChVector<> ChBox::GetP6() const {
    ChVector<> P6r;
    P6r.x() = -Size.x();
    P6r.y() = +Size.y();
    P6r.z() = -Size.z();
    return ChTransform<>::TransformLocalToParent(P6r, Pos, Rot);
}
ChVector<> ChBox::GetP7() const {
    ChVector<> P7r;
    P7r.x() = -Size.x();
    P7r.y() = -Size.y();
    P7r.z() = -Size.z();
    return ChTransform<>::TransformLocalToParent(P7r, Pos, Rot);
}
ChVector<> ChBox::GetP8() const {
    ChVector<> P8r;
    P8r.x() = +Size.x();
    P8r.y() = -Size.y();
    P8r.z() = -Size.z();
    return ChTransform<>::TransformLocalToParent(P8r, Pos, Rot);
}

ChVector<> ChBox::GetPn(int ipoint) const {
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

void ChBox::GetBoundingBox(double& xmin,
                           double& xmax,
                           double& ymin,
                           double& ymax,
                           double& zmin,
                           double& zmax,
                           ChMatrix33<>* bbRot) const {
    // TO OPTIMIZE for speed

    ChVector<> p1, p2, p3, p4, p5, p6, p7, p8;

    xmax = ymax = zmax = -10e20;
    xmin = ymin = zmin = +10e20;

    if (bbRot == NULL) {
        p1 = GetP1();
        p2 = GetP2();
        p3 = GetP3();
        p4 = GetP4();
        p5 = GetP5();
        p6 = GetP6();
        p7 = GetP7();
        p8 = GetP8();
    } else {
        p1 = bbRot->MatrT_x_Vect(GetP1());
        p2 = bbRot->MatrT_x_Vect(GetP2());
        p3 = bbRot->MatrT_x_Vect(GetP3());
        p4 = bbRot->MatrT_x_Vect(GetP4());
        p5 = bbRot->MatrT_x_Vect(GetP5());
        p6 = bbRot->MatrT_x_Vect(GetP6());
        p7 = bbRot->MatrT_x_Vect(GetP7());
        p8 = bbRot->MatrT_x_Vect(GetP8());
    }

    if (p1.x() > xmax)
        xmax = p1.x();
    if (p1.y() > ymax)
        ymax = p1.y();
    if (p1.z() > zmax)
        zmax = p1.z();
    if (p2.x() > xmax)
        xmax = p2.x();
    if (p2.y() > ymax)
        ymax = p2.y();
    if (p2.z() > zmax)
        zmax = p2.z();
    if (p3.x() > xmax)
        xmax = p3.x();
    if (p3.y() > ymax)
        ymax = p3.y();
    if (p3.z() > zmax)
        zmax = p3.z();
    if (p4.x() > xmax)
        xmax = p4.x();
    if (p4.y() > ymax)
        ymax = p4.y();
    if (p4.z() > zmax)
        zmax = p4.z();
    if (p5.x() > xmax)
        xmax = p5.x();
    if (p5.y() > ymax)
        ymax = p5.y();
    if (p5.z() > zmax)
        zmax = p5.z();
    if (p6.x() > xmax)
        xmax = p6.x();
    if (p6.y() > ymax)
        ymax = p6.y();
    if (p6.z() > zmax)
        zmax = p6.z();
    if (p7.x() > xmax)
        xmax = p7.x();
    if (p7.y() > ymax)
        ymax = p7.y();
    if (p7.z() > zmax)
        zmax = p7.z();
    if (p8.x() > xmax)
        xmax = p8.x();
    if (p8.y() > ymax)
        ymax = p8.y();
    if (p8.z() > zmax)
        zmax = p8.z();

    if (p1.x() < xmin)
        xmin = p1.x();
    if (p1.y() < ymin)
        ymin = p1.y();
    if (p1.z() < zmin)
        zmin = p1.z();
    if (p2.x() < xmin)
        xmin = p2.x();
    if (p2.y() < ymin)
        ymin = p2.y();
    if (p2.z() < zmin)
        zmin = p2.z();
    if (p3.x() < xmin)
        xmin = p3.x();
    if (p3.y() < ymin)
        ymin = p3.y();
    if (p3.z() < zmin)
        zmin = p3.z();
    if (p4.x() < xmin)
        xmin = p4.x();
    if (p4.y() < ymin)
        ymin = p4.y();
    if (p4.z() < zmin)
        zmin = p4.z();
    if (p5.x() < xmin)
        xmin = p5.x();
    if (p5.y() < ymin)
        ymin = p5.y();
    if (p5.z() < zmin)
        zmin = p5.z();
    if (p6.x() < xmin)
        xmin = p6.x();
    if (p6.y() < ymin)
        ymin = p6.y();
    if (p6.z() < zmin)
        zmin = p6.z();
    if (p7.x() < xmin)
        xmin = p7.x();
    if (p7.y() < ymin)
        ymin = p7.y();
    if (p7.z() < zmin)
        zmin = p7.z();
    if (p8.x() < xmin)
        xmin = p8.x();
    if (p8.y() < ymin)
        ymin = p8.y();
    if (p8.z() < zmin)
        zmin = p8.z();
}

void ChBox::CovarianceMatrix(ChMatrix33<>& C) const {
    ChVector<> p1, p2, p3, p4, p5, p6, p7, p8;

    p1 = GetP1();
    p2 = GetP2();
    p3 = GetP3();
    p4 = GetP4();
    p5 = GetP5();
    p6 = GetP6();
    p7 = GetP7();
    p8 = GetP8();

    C(0, 0) =
        p1.x() * p1.x() + p2.x() * p2.x() + p3.x() * p3.x() + p4.x() * p4.x() + p5.x() * p5.x() + p6.x() * p6.x() + p7.x() * p7.x() + p8.x() * p8.x();
    C(1, 1) =
        p1.y() * p1.y() + p2.y() * p2.y() + p3.y() * p3.y() + p4.y() * p4.y() + p5.y() * p5.y() + p6.y() * p6.y() + p7.y() * p7.y() + p8.y() * p8.y();
    C(2, 2) =
        p1.z() * p1.z() + p2.z() * p2.z() + p3.z() * p3.z() + p4.z() * p4.z() + p5.z() * p5.z() + p6.z() * p6.z() + p7.z() * p7.z() + p8.z() * p8.z();
    C(0, 1) =
        p1.x() * p1.y() + p2.x() * p2.y() + p3.x() * p3.y() + p4.x() * p4.y() + p5.x() * p5.y() + p6.x() * p6.y() + p7.x() * p7.y() + p8.x() * p8.y();
    C(0, 2) =
        p1.x() * p1.z() + p2.x() * p2.z() + p3.x() * p3.z() + p4.x() * p4.z() + p5.x() * p5.z() + p6.x() * p6.z() + p7.x() * p7.z() + p8.x() * p8.z();
    C(1, 2) =
        p1.y() * p1.z() + p2.y() * p2.z() + p3.y() * p3.z() + p4.y() * p4.z() + p5.y() * p5.z() + p6.y() * p6.z() + p7.y() * p7.z() + p8.y() * p8.z();
}

}  // end namespace geometry
}  // end namespace chrono
