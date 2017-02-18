//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#include <cstdio>
#include <cstring>

#include "chrono/collision/edgetempest/ChCMatVec.h"
#include "chrono/collision/edgetempest/ChCGetTime.h"
#include "chrono/collision/edgetempest/ChCOBBTree.h"
#include "chrono/core/ChTransform.h"

namespace chrono {
namespace collision {

CHOBBTree::CHOBBTree() {
    current_box = 0;
}

CHOBBTree::~CHOBBTree() {
    ResetModel();
}

int CHOBBTree::ResetModel() {
    // INHERIT PARENT CLASS RESET FUNCTION
    ChCollisionTree::ResetModel();

    b.clear();

    current_box = 0;

    return ChC_OK;
}

int CHOBBTree::BuildModel(double envelope) {
    // INHERIT PARENT CLASS RESET FUNCTION
    ChCollisionTree::BuildModel(envelope);

    if (build_state == ChC_BUILD_STATE_PROCESSED) {
        return ChC_OK;
    }

    // create an array of BVs for the model

    b.clear();

    if (num_geometries == 0)
        return ChC_OK;

    b.resize(2 * num_geometries - 1);

    current_box = 0;

    // we should build the model now.

    build_model(envelope);  // %%%%%%%%%%%% BUILD HIERARCHY HERE %%%%%%%%%%%

    build_state = ChC_BUILD_STATE_PROCESSED;

    return ChC_OK;
}

void recurse_scan_OBBs(ChMatrix33<>& PrevRot,
                       Vector& PrevPos,
                       CHOBBTree* mmodel,
                       int nb,
                       void* userdata,
                       int current_level,
                       int& counter,
                       void callback(ChMatrix33<>& Rot, Vector& Pos, Vector& d, int level, void* userdata)) {
    counter++;

    // compute root-relative coords for each box
    ChMatrix33<> Rot;
    Vector Pos;
    ChMatrix33<> tempRot;
    Vector tempPos;
    tempRot.MatrMultiply(PrevRot, mmodel->child(nb)->Rot);
    tempPos = ChTransform<>::TransformLocalToParent(mmodel->child(nb)->To, PrevPos, PrevRot);
    Rot.CopyFromMatrix(tempRot);
    Pos = tempPos;

    // execute callback
    callback(Rot, Pos, mmodel->child(nb)->d, current_level, userdata);

    // break recursion on leaf
    if (mmodel->child(nb)->IsLeaf())
        return;

    int c1 = mmodel->child(nb)->first_child;
    int c2 = c1 + 1;

    int new_currlevel = current_level + 1;

    recurse_scan_OBBs(Rot, Pos, mmodel, c1, userdata, new_currlevel, counter, callback);

    recurse_scan_OBBs(Rot, Pos, mmodel, c2, userdata, new_currlevel, counter, callback);
}

int CHOBBTree::TraverseBoundingBoxes(
    void callback(ChMatrix33<>& Rot, Vector& Pos, Vector& d, int level, void* userdata),
    void* userdata) {
    int nboxes = 0;

    static ChMatrix33<> mrot;
    mrot.Set33Identity();
    static Vector mpos;
    mpos = VNULL;

    recurse_scan_OBBs(mrot, mpos, this, 0, userdata, 0, nboxes, callback);

    return nboxes;
}

///////////////////////////////////////////////////////////////////

//
// Building of the model
//

void get_centroid_geometries(PQP_REAL c[3], std::vector<geometry::ChGeometry*>& mgeos, int firstgeo, int ngeos) {
    c[0] = c[1] = c[2] = 0.0;

    // get center of mass
    geometry::ChGeometry* nit;
    for (int count = 0; count < ngeos; ++count) {
        nit = mgeos[firstgeo + count];

        Vector baricenter = nit->Baricenter();

        c[0] += baricenter.x();
        c[1] += baricenter.y();
        c[2] += baricenter.z();
    }

    c[0] /= ngeos;
    c[1] /= ngeos;
    c[2] /= ngeos;
}

void get_covariance_geometries(PQP_REAL M[3][3], std::vector<geometry::ChGeometry*>& mgeos, int firstgeo, int ngeos) {
    static Vector S1;
    static Vector S1_geo;
    static ChMatrix33<> S2;
    static ChMatrix33<> S2_geo;
    S1 = VNULL;
    S1_geo = VNULL;
    S2.Reset();
    S2_geo.Reset();

    // get center of mass
    geometry::ChGeometry* nit;
    for (int count = 0; count < ngeos; ++count) {
        nit = mgeos[firstgeo + count];

        S1_geo = nit->Baricenter();

        S1 = Vadd(S1, S1_geo);

        nit->CovarianceMatrix(S2_geo);

        S2.MatrInc(S2_geo);
    }

    // now get covariances

    M[0][0] = S2(0, 0) - S1.x() * S1.x() / ngeos;
    M[1][1] = S2(1, 1) - S1.y() * S1.y() / ngeos;
    M[2][2] = S2(2, 2) - S1.z() * S1.z() / ngeos;
    M[0][1] = S2(0, 1) - S1.x() * S1.y() / ngeos;
    M[1][2] = S2(1, 2) - S1.y() * S1.z() / ngeos;
    M[0][2] = S2(0, 2) - S1.x() * S1.z() / ngeos;
    M[1][0] = M[0][1];
    M[2][0] = M[0][2];
    M[2][1] = M[1][2];
}

int split_geometries(std::vector<geometry::ChGeometry*>& mgeos, int firstgeo, int ngeos, PQP_REAL a[3], PQP_REAL c) {
    int c1 = 0;
    PQP_REAL p[3];
    PQP_REAL x;
    geometry::ChGeometry* temp;

    for (int count = 0; count < ngeos; count++) {
        int i = count + firstgeo;

        // loop invariant: up to (but not including) index c1 in group 1,
        // then up to (but not including) index i in group 2
        //
        //  [1] [1] [1] [1] [2] [2] [2] [x] [x] ... [x]
        //                   c1          i
        //
        Vector vg = mgeos[i]->Baricenter();
        p[0] = vg.x();
        p[1] = vg.y();
        p[2] = vg.z();

        x = VdotV(p, a);

        if (x <= c) {
            // group 1
            temp = mgeos[i];
            mgeos[i] = mgeos[firstgeo + c1];
            mgeos[firstgeo + c1] = temp;
            c1++;
        } else {
            // group 2 -- do nothing
        }
    }

    // split arbitrarily if one group empty

    if ((c1 == 0) || (c1 == ngeos))
        c1 = ngeos / 2;

    return c1;
}

// Fits m->child(bn) to the num_tris triangles starting at first_tri
// Then, if num_tris is greater than one, partitions the tris into two
// sets, and recursively builds two children of m->child(bn)

int build_recurse(CHOBBTree* m, int bn, int first_geo, int num_geos, double envelope) {
    CHOBB* b = m->child(bn);

    // compute a rotation matrix

    PQP_REAL C[3][3], E[3][3], R[3][3], s[3], axis[3], mean[3], coord;

    get_covariance_geometries(C, m->geometries, first_geo, num_geos);

    Meigen(E, s, C);

    // place axes of E in order of increasing s

    int min, mid, max;
    if (s[0] > s[1]) {
        max = 0;
        min = 1;
    } else {
        min = 0;
        max = 1;
    }
    if (s[2] < s[min]) {
        mid = min;
        min = 2;
    } else if (s[2] > s[max]) {
        mid = max;
        max = 2;
    } else {
        mid = 2;
    }
    McolcMcol(R, 0, E, max);
    McolcMcol(R, 1, E, mid);
    R[0][2] = E[1][max] * E[2][mid] - E[1][mid] * E[2][max];
    R[1][2] = E[0][mid] * E[2][max] - E[0][max] * E[2][mid];
    R[2][2] = E[0][max] * E[1][mid] - E[0][mid] * E[1][max];

    static ChMatrix33<> Rch;
    Rch.Set33Element(0, 0, R[0][0]);
    Rch.Set33Element(0, 1, R[0][1]);
    Rch.Set33Element(0, 2, R[0][2]);
    Rch.Set33Element(1, 0, R[1][0]);
    Rch.Set33Element(1, 1, R[1][1]);
    Rch.Set33Element(1, 2, R[1][2]);
    Rch.Set33Element(2, 0, R[2][0]);
    Rch.Set33Element(2, 1, R[2][1]);
    Rch.Set33Element(2, 2, R[2][2]);

    // fit the BV

    b->FitToGeometries(Rch, m->geometries, first_geo, num_geos, envelope);

    if (num_geos == 1) {
        // BV is a leaf BV - first_child will index a triangle

        b->first_child = -(first_geo + 1);
    } else if (num_geos > 1) {
        // BV not a leaf - first_child will index a BV

        b->first_child = m->current_box;
        m->current_box += 2;

        // choose splitting axis and splitting coord

        McolcV(axis, R, 0);

        get_centroid_geometries(mean, m->geometries, first_geo, num_geos);

        coord = VdotV(axis, mean);

        // now split

        int num_first_half = split_geometries(m->geometries, first_geo, num_geos, axis, coord);

        // recursively build the children

        build_recurse(m, m->child(bn)->first_child, first_geo, num_first_half, envelope);
        build_recurse(m, m->child(bn)->first_child + 1, first_geo + num_first_half, num_geos - num_first_half,
                      envelope);
    }
    return ChC_OK;
}

// This descends the hierarchy, converting world-relative
// transforms to parent-relative transforms

void make_parent_relative(CHOBBTree* m, int bn, ChMatrix33<>& parentR, Vector& parentTo) {
    static ChMatrix33<> Rpc;
    static Vector Tpc;

    if (!m->child(bn)->IsLeaf()) {
        // make children parent-relative

        make_parent_relative(m, m->child(bn)->first_child, m->child(bn)->Rot, m->child(bn)->To);
        make_parent_relative(m, m->child(bn)->first_child + 1, m->child(bn)->Rot, m->child(bn)->To);
    }

    // make self parent relative

    Rpc.MatrTMultiply(parentR, m->child(bn)->Rot);  // MTxM(Rpc,parentR,m->child(bn)->R);
    m->child(bn)->Rot.CopyFromMatrix(Rpc);          // McM(m->child(bn)->R,Rpc);
    Tpc = Vsub(m->child(bn)->To, parentTo);         // VmV(Tpc,m->child(bn)->To,parentTo);
    m->child(bn)->To = parentR.MatrT_x_Vect(Tpc);   // Tpc MTxV(m->child(bn)->To,parentR,Tpc);
}

int CHOBBTree::build_model(double envelope) {
    // set num_bvs to 1, the first index for a child bv

    current_box = 1;

    // build recursively

    build_recurse(this, 0, 0, num_geometries, envelope);

    // change BV orientations from world-relative to parent-relative

    static ChMatrix33<> R;
    static Vector T;

    R.Set33Identity();
    T = VNULL;

    make_parent_relative(this, 0, R, T);

    return ChC_OK;
}

}  // end namespace collision
}  // end namespace chrono
