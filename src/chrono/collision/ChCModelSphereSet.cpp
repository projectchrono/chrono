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

#include "ChCModelSphereSet.h"
#include "physics/ChPhysicsItem.h"
#include "physics/ChSystem.h"
#include "collision/ChCCollisionSystemSpheres.h"
#include "collision/ChCConvexDecomposition.h"

namespace chrono {

namespace collision {

ChModelSphereSet::ChModelSphereSet() {
    nSpheres = 0;
    colFam = 0;
    noCollWith = -1;
    this->model_envelope = 0.0f;
}

ChModelSphereSet::~ChModelSphereSet() {
    // ClearModel(); not possible, would call GetPhysicsItem() that is pure virtual, enough to use instead..
    nSpheres = 0;
    sphPosLocal.clear();
    sphRad.clear();
}

int ChModelSphereSet::ClearModel() {
    if (GetPhysicsItem()->GetSystem()) {
        if (GetPhysicsItem()->GetCollide()) {
            GetPhysicsItem()->GetSystem()->GetCollisionSystem()->Remove(this);
        }
    }
    nSpheres = 0;
    sphPosLocal.clear();
    sphRad.clear();

    return 1;
}

int ChModelSphereSet::BuildModel() {
    // assert (GetPhysicsItem());

    // insert again (we assume it was removed by ClearModel!!!)
    if (GetPhysicsItem()->GetSystem())
        if (GetPhysicsItem()->GetCollide())
            GetPhysicsItem()->GetSystem()->GetCollisionSystem()->Add(this);

    return 1;
}

bool ChModelSphereSet::AddCompoundBody(int numSpheres,
                                       std::vector<ChVector<float> >* posLocal,
                                       std::vector<float>* rads) {
    if (numSpheres < 0)
        return false;

    nSpheres = numSpheres;

    float maxR = 0.0f;
    for (uint i = 0; i < nSpheres; i++) {
        sphPosLocal.push_back((*posLocal)[i]);
        sphRad.push_back((*rads)[i]);
        if (posLocal->at(i).Length() + rads->at(i) > maxR)
            maxR = posLocal->at(i).Length() + rads->at(i);
    }

    myBBminLocal.x = -maxR;
    myBBminLocal.y = -maxR;
    myBBminLocal.z = -maxR;
    myBBmaxLocal.x = maxR;
    myBBmaxLocal.y = maxR;
    myBBmaxLocal.z = maxR;

    return true;
}

bool ChModelSphereSet::AddSphere(double radius, ChVector<>* pos) {
    nSpheres = 1;
    if (pos == 0)
        sphPosLocal.push_back(ChVector<>(0.0, 0.0, 0.0));
    else
        sphPosLocal.push_back(*pos);

    sphRad.push_back(float(radius));

    myBBminLocal.x = -radius;
    myBBminLocal.y = -radius;
    myBBminLocal.z = -radius;
    myBBmaxLocal.x = radius;
    myBBmaxLocal.y = radius;
    myBBmaxLocal.z = radius;

    model_type = SPHERE;
    return true;
}

bool ChModelSphereSet::AddEllipsoid(double rx, double ry, double rz, ChVector<>* pos, ChMatrix33<>* rot) {
    model_type = ELLIPSOID;
    return false;
}

bool ChModelSphereSet::AddBox(double hx, double hy, double hz, ChVector<>* pos, ChMatrix33<>* rot) {
    model_type = BOX;
    return false;
}

/// Add a cylinder to this model (default axis on Y direction), for collision purposes
bool ChModelSphereSet::AddCylinder(double rx, double rz, double hy, ChVector<>* pos, ChMatrix33<>* rot) {
    model_type = CYLINDER;
    return false;
}

bool ChModelSphereSet::AddBarrel(double Y_low,
                                 double Y_high,
                                 double R_vert,
                                 double R_hor,
                                 double R_offset,
                                 ChVector<>* pos,
                                 ChMatrix33<>* rot) {
    model_type = BARREL;
    return false;
}

bool ChModelSphereSet::AddConvexHull(std::vector<ChVector<double> >& pointlist, ChVector<>* pos, ChMatrix33<>* rot) {
    model_type = CONVEXHULL;
    return false;
}

/// Add a triangle mesh to this model
bool ChModelSphereSet::AddTriangleMesh(const geometry::ChTriangleMesh& trimesh,
                                       bool is_static,
                                       bool is_convex,
                                       ChVector<>* pos,
                                       ChMatrix33<>* rot,
                                       double sphereswept_thickness) {
    model_type = TRIANGLEMESH;
    return false;
}

bool ChModelSphereSet::AddTriangleMeshConcave(const geometry::ChTriangleMesh& trimesh,  ///< the concave triangle mesh
                                              ChVector<>* pos,
                                              ChMatrix33<>* rot  ///< displacement respect to COG (optional)
                                              ) {
    return false;
}

bool ChModelSphereSet::AddTriangleMeshConcaveDecomposed(ChConvexDecomposition& mydecomposition,
                                                        ChVector<>* pos,
                                                        ChMatrix33<>* rot  ///< displacement respect to COG (optional)
                                                        ) {
    return false;
}

bool ChModelSphereSet::AddCopyOfAnotherModel(ChCollisionModel* another) {
    return false;
}

void ChModelSphereSet::SetFamily(int mfamily) {
    colFam = mfamily;
}

int ChModelSphereSet::GetFamily() {
    return colFam;
}

void ChModelSphereSet::SetFamilyMaskNoCollisionWithFamily(int mfamily) {
    noCollWith = mfamily;
}

void ChModelSphereSet::SetFamilyMaskDoCollisionWithFamily(int mfamily) {
    if (noCollWith == mfamily)
        noCollWith = -1;
}

bool ChModelSphereSet::GetFamilyMaskDoesCollisionWithFamily(int mfamily) {
    return (noCollWith != mfamily);
}

void ChModelSphereSet::GetAABB(ChVector<>& bbmin, ChVector<>& bbmax) const {
    bbmin.Set(myBBminGlobal.x, myBBminGlobal.y, myBBminGlobal.z);
    bbmax.Set(myBBmaxGlobal.x, myBBmaxGlobal.y, myBBmaxGlobal.z);
}

void ChModelSphereSet::GetGlobalSpherePos(thrust::host_vector<ChVector<float> >& globalPos) {
    globalPos = sphPosGlobal;
}

void ChModelSphereSet::GetSphereRad(thrust::host_vector<float>& rad) {
    // thrust::host_vector<float> pRad=sphRad;
    // rad.swap(pRad);
    rad = sphRad;
}

void ChModelSphereSet::StreamIN(ChStreamInBinary& mstream) {
    // class version number
    int version = mstream.VersionRead();

    // parent class deserialize
    ChCollisionModel::StreamIN(mstream);

    // deserialize custom data:
    mstream >> nSpheres;
    ChVector<float> tmpPos;
    float tmpRad;
    for (uint i = 0; i < nSpheres; i++) {
        mstream >> tmpPos;
        sphPosLocal.push_back(tmpPos);
        mstream >> tmpRad;
        sphRad.push_back(tmpRad);
    }
    mstream >> myBBminLocal;
    mstream >> myBBmaxLocal;
    mstream >> colFam;
    mstream >> noCollWith;
}

void ChModelSphereSet::StreamOUT(ChStreamOutBinary& mstream) {
    // class version number
    mstream.VersionWrite(1);

    // parent class serialize
    ChCollisionModel::StreamOUT(mstream);

    // serialize custom data:
    mstream << nSpheres;
    for (uint i = 0; i < nSpheres; i++) {
        mstream << sphPosLocal[i];
        mstream << sphRad[i];
    }
    mstream << myBBminLocal;
    mstream << myBBmaxLocal;
    mstream << colFam;
    mstream << noCollWith;
}

}  // END_OF_NAMESPACE____
}  // END_OF_NAMESPACE____
