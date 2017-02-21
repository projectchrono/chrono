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
#include "chrono/collision/edgetempest/ChCAABBTree.h"

namespace chrono {
namespace collision {

CHAABBTree::CHAABBTree() {
    current_box = 0;
}

CHAABBTree::~CHAABBTree() {
    ResetModel();
}

int CHAABBTree::ResetModel() {
    // INHERIT PARENT CLASS RESET FUNCTION
    ChCollisionTree::ResetModel();

    b.clear();

    current_box = 0;

    return ChC_OK;
}

int CHAABBTree::BuildModel(double envelope) {
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

void recurse_scan_AABBs(CHAABBTree* mmodel,
                        int nb,
                        void* userdata,
                        int current_level,
                        int& counter,
                        void callback(ChMatrix33<>& Rot, Vector& Pos, Vector& d, int level, void* userdata)) {
    counter++;

    // compute root-relative coords for each box
    ChMatrix33<> Rot;
    Rot.Set33Identity();

    // execute callback
    callback(Rot, mmodel->child(nb)->To, mmodel->child(nb)->d, current_level, userdata);

    // break recursion on leaf
    if (mmodel->child(nb)->IsLeaf())
        return;

    int c1 = mmodel->child(nb)->first_child;
    int c2 = c1 + 1;

    int new_currlevel = current_level + 1;

    recurse_scan_AABBs(mmodel, c1, userdata, new_currlevel, counter, callback);

    recurse_scan_AABBs(mmodel, c2, userdata, new_currlevel, counter, callback);
}

int CHAABBTree::TraverseBoundingBoxes(
    void callback(ChMatrix33<>& Rot, Vector& Pos, Vector& d, int level, void* userdata),
    void* userdata) {
    int nboxes = 0;

    recurse_scan_AABBs(this, 0, userdata, 0, nboxes, callback);

    return nboxes;
}

///////////////////////////////////////////////////////////////////

//
// Building of the model
//

static void get_centroid_geometries(Vector& mean, std::vector<geometry::ChGeometry*>& mgeos, int firstgeo, int ngeos) {
    mean = VNULL;

    // get center of mass
    geometry::ChGeometry* nit = mgeos[firstgeo];
    for (int count = 0; count < ngeos; ++count) {
        nit = mgeos[firstgeo + count];
        Vector baricenter = nit->Baricenter();

        mean.x() += baricenter.x();
        mean.y() += baricenter.y();
        mean.z() += baricenter.z();
    }

    mean.x() /= ngeos;
    mean.y() /= ngeos;
    mean.z() /= ngeos;
}

static int split_geometries(std::vector<geometry::ChGeometry*>& mgeos,
                            int firstgeo,
                            int ngeos,
                            Vector& direction,
                            double c) {
    int c1 = 0;
    double x;
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

        x = Vdot(vg, direction);

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

int build_recurse(CHAABBTree* m, int bn, int first_geo, int num_geos, double envelope) {
    CHAABB* b = m->child(bn);

    double coord;
    Vector axis;
    Vector mean;

    // fit the BV

    b->FitToGeometries(m->geometries, first_geo, num_geos, envelope);

    if (num_geos == 1) {
        // BV is a leaf BV - first_child will index a triangle

        b->first_child = -(first_geo + 1);
    } else if (num_geos > 1) {
        // BV not a leaf - first_child will index a BV

        b->first_child = m->current_box;
        m->current_box += 2;

        // choose splitting axis

        axis = VECT_X;
        if (b->d.y() > b->d.x())
            axis = VECT_Y;
        if (b->d.z() > b->d.y())
            axis = VECT_Z;
        if (b->d.z() > b->d.x())
            axis = VECT_Z;

        // choose splitting coord

        get_centroid_geometries(mean, m->geometries, first_geo, num_geos);

        coord = Vdot(axis, mean);

        //*** TO DO***??? other splitting criterions??

        // now split

        int num_first_half = split_geometries(m->geometries, first_geo, num_geos, axis, coord);

        // recursively build the children

        build_recurse(m, m->child(bn)->first_child, first_geo, num_first_half, envelope);
        build_recurse(m, m->child(bn)->first_child + 1, first_geo + num_first_half, num_geos - num_first_half,
                      envelope);
    }

    return ChC_OK;
}

int CHAABBTree::build_model(double envelope) {
    // set num_bvs to 1, the first index for a child bv

    current_box = 1;

    // build recursively

    build_recurse(this, 0, 0, num_geometries, envelope);

    return ChC_OK;
}

}  // end namespace collision
}  // end namespace chrono
