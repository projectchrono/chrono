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

#include "chrono/collision/ChCollisionModelBullet.h"
#include "chrono/core/ChMath.h"
#include "chrono/physics/ChSystem.h"

#include "chrono/fea/ChContactSurfaceMesh.h"
#include "chrono/fea/ChElementShellANCF.h"
#include "chrono/fea/ChElementShellANCF_8.h"
#include "chrono/fea/ChElementShellReissner4.h"
#include "chrono/fea/ChElementTetra_4.h"
#include "chrono/fea/ChElementBrick_9.h"
#include "chrono/fea/ChElementCableANCF.h"
#include "chrono/fea/ChElementBeamEuler.h"
#include "chrono/fea/ChFaceTetra_4.h"
#include "chrono/fea/ChFaceBrick_9.h"
#include "chrono/fea/ChFaceHexa_8.h"
#include "chrono/fea/ChMesh.h"

#include <unordered_map>
#include <map>
#include <set>
#include <array>
#include <algorithm>

using namespace std;

namespace chrono {
namespace fea {

//////////////////////////////////////////////////////////////////////////////
////  ChContactTriangleXYZ

ChContactTriangleXYZ::ChContactTriangleXYZ() {
    this->collision_model = new collision::ChCollisionModelBullet;
    this->collision_model->SetContactable(this);
}

ChContactTriangleXYZ::ChContactTriangleXYZ(std::shared_ptr<ChNodeFEAxyz> n1,
                                           std::shared_ptr<ChNodeFEAxyz> n2,
                                           std::shared_ptr<ChNodeFEAxyz> n3,
                                           ChContactSurface* acontainer) {
    mnode1 = n1;
    mnode1 = n2;
    mnode1 = n3;
    container = acontainer;

    this->collision_model = new collision::ChCollisionModelBullet;
    this->collision_model->SetContactable(this);
}

ChPhysicsItem* ChContactTriangleXYZ::GetPhysicsItem() {
    return (ChPhysicsItem*)container->GetMesh();
}

// interface to ChLoadableUV

// Gets all the DOFs packed in a single vector (position part).
void ChContactTriangleXYZ::LoadableGetStateBlock_x(int block_offset, ChState& mD) {
    mD.segment(block_offset + 0, 3) = mnode1->GetPos().eigen();
    mD.segment(block_offset + 3, 3) = mnode2->GetPos().eigen();
    mD.segment(block_offset + 6, 3) = mnode3->GetPos().eigen();
}

// Gets all the DOFs packed in a single vector (velocity part).
void ChContactTriangleXYZ::LoadableGetStateBlock_w(int block_offset, ChStateDelta& mD) {
    mD.segment(block_offset + 0, 3) = mnode1->GetPos_dt().eigen();
    mD.segment(block_offset + 3, 3) = mnode2->GetPos_dt().eigen();
    mD.segment(block_offset + 6, 3) = mnode3->GetPos_dt().eigen();
}

/// Increment all DOFs using a delta.
void ChContactTriangleXYZ::LoadableStateIncrement(const unsigned int off_x, ChState& x_new, const ChState& x, const unsigned int off_v, const ChStateDelta& Dv)  {
    mnode1->NodeIntStateIncrement(off_x   , x_new, x, off_v   , Dv);
    mnode2->NodeIntStateIncrement(off_x+3 , x_new, x, off_v+3 , Dv);
    mnode3->NodeIntStateIncrement(off_x+6 , x_new, x, off_v+6 , Dv);
}
// Get the pointers to the contained ChVariables, appending to the mvars vector.
void ChContactTriangleXYZ::LoadableGetVariables(std::vector<ChVariables*>& mvars) {
    mvars.push_back(&mnode1->Variables());
    mvars.push_back(&mnode2->Variables());
    mvars.push_back(&mnode3->Variables());
}


// Evaluate N'*F , where N is the shape function evaluated at (U,V) coordinates of the surface.
void ChContactTriangleXYZ::ComputeNF(
    const double U,              // parametric coordinate in surface
    const double V,              // parametric coordinate in surface
    ChVectorDynamic<>& Qi,       // Return result of Q = N'*F  here
    double& detJ,                // Return det[J] here
    const ChVectorDynamic<>& F,  // Input F vector, size is =n. field coords.
    ChVectorDynamic<>* state_x,  // if != 0, update state (pos. part) to this, then evaluate Q
    ChVectorDynamic<>* state_w   // if != 0, update state (speed part) to this, then evaluate Q
    ) {
    ChMatrixNM<double, 1, 3> N;
    // shape functions (U and V in 0..1 as triangle integration)
    N(0) = 1 - U - V;
    N(1) = U;
    N(2) = V;

    // determinant of jacobian is also =2*areaoftriangle, also length of cross product of sides
    ChVector<> p0 = GetNode1()->GetPos();
    ChVector<> p1 = GetNode2()->GetPos();
    ChVector<> p2 = GetNode3()->GetPos();
    detJ = (Vcross(p2 - p0, p1 - p0)).Length();

    Qi.segment(0, 3) = N(0) * F.segment(0, 3);
    Qi.segment(3, 3) = N(1) * F.segment(0, 3);
    Qi.segment(6, 3) = N(2) * F.segment(0, 3);
}

ChVector<> ChContactTriangleXYZ::ComputeNormal(const double U, const double V) {
    ChVector<> p0 = GetNode1()->GetPos();
    ChVector<> p1 = GetNode2()->GetPos();
    ChVector<> p2 = GetNode3()->GetPos();
    return Vcross(p1 - p0, p2 - p0).GetNormalized();
}

//////////////////////////////////////////////////////////////////////////////
////  ChContactTriangleXYZROT

ChContactTriangleXYZROT::ChContactTriangleXYZROT() {
    this->collision_model = new collision::ChCollisionModelBullet;
    this->collision_model->SetContactable(this);
}

ChContactTriangleXYZROT::ChContactTriangleXYZROT(std::shared_ptr<ChNodeFEAxyzrot> n1,
                                                 std::shared_ptr<ChNodeFEAxyzrot> n2,
                                                 std::shared_ptr<ChNodeFEAxyzrot> n3,
                                                 ChContactSurface* acontainer) {
    mnode1 = n1;
    mnode1 = n2;
    mnode1 = n3;
    container = acontainer;

    this->collision_model = new collision::ChCollisionModelBullet;
    this->collision_model->SetContactable(this);
}

ChPhysicsItem* ChContactTriangleXYZROT::GetPhysicsItem() {
    return (ChPhysicsItem*)container->GetMesh();
}

// interface to ChLoadableUV

// Gets all the DOFs packed in a single vector (position part).
void ChContactTriangleXYZROT::LoadableGetStateBlock_x(int block_offset, ChState& mD) {
    mD.segment(block_offset + 0, 3) = mnode1->GetPos().eigen();
    mD.segment(block_offset + 3, 4) = mnode1->GetRot().eigen();

    mD.segment(block_offset + 7, 3) = mnode2->GetPos().eigen();
    mD.segment(block_offset + 10, 4) = mnode2->GetRot().eigen();

    mD.segment(block_offset + 14, 3) = mnode3->GetPos().eigen();
    mD.segment(block_offset + 17, 4) = mnode3->GetRot().eigen();
}

// Gets all the DOFs packed in a single vector (velocity part).
void ChContactTriangleXYZROT::LoadableGetStateBlock_w(int block_offset, ChStateDelta& mD) {
    mD.segment(block_offset + 0, 3) = mnode1->GetPos_dt().eigen();
    mD.segment(block_offset + 3, 3) = mnode1->GetWvel_loc().eigen();

    mD.segment(block_offset + 6, 3) = mnode2->GetPos_dt().eigen();
    mD.segment(block_offset + 9, 3) = mnode2->GetWvel_loc().eigen();

    mD.segment(block_offset + 12, 3) = mnode3->GetPos_dt().eigen();
    mD.segment(block_offset + 15, 3) = mnode3->GetWvel_loc().eigen();
}

// Increment all DOFs using a delta.
void ChContactTriangleXYZROT::LoadableStateIncrement(const unsigned int off_x,
                                                     ChState& x_new,
                                                     const ChState& x,
                                                     const unsigned int off_v,
                                                     const ChStateDelta& Dv) {
    mnode1->NodeIntStateIncrement(off_x   , x_new, x, off_v    , Dv);
    mnode2->NodeIntStateIncrement(off_x+7 , x_new, x, off_v+6  , Dv);
    mnode3->NodeIntStateIncrement(off_x+14, x_new, x, off_v+12 , Dv);
}
// Get the pointers to the contained ChVariables, appending to the mvars vector.
void ChContactTriangleXYZROT::LoadableGetVariables(std::vector<ChVariables*>& mvars) {
    mvars.push_back(&mnode1->Variables());
    mvars.push_back(&mnode2->Variables());
    mvars.push_back(&mnode3->Variables());
}

// Evaluate N'*F , where N is the shape function evaluated at (U,V) coordinates of the surface.
void ChContactTriangleXYZROT::ComputeNF(
    const double U,              // parametric coordinate in surface
    const double V,              // parametric coordinate in surface
    ChVectorDynamic<>& Qi,       // Return result of Q = N'*F  here
    double& detJ,                // Return det[J] here
    const ChVectorDynamic<>& F,  // Input F vector, size is =n. field coords.
    ChVectorDynamic<>* state_x,  // if != 0, update state (pos. part) to this, then evaluate Q
    ChVectorDynamic<>* state_w   // if != 0, update state (speed part) to this, then evaluate Q
    ) {
    ChMatrixNM<double, 1, 3> N;
    // shape functions (U and V in 0..1 as triangle integration)
    N(0) = 1 - U - V;
    N(1) = U;
    N(2) = V;

    // determinant of jacobian is also =2*areaoftriangle, also length of cross product of sides
    ChVector<> p0 = GetNode1()->GetPos();
    ChVector<> p1 = GetNode2()->GetPos();
    ChVector<> p2 = GetNode3()->GetPos();
    detJ = (Vcross(p2 - p0, p1 - p0)).Length();

    Qi.segment(0, 3) = N(0) * F.segment(0, 3);
    Qi.segment(3, 3) = N(0) * F.segment(3, 3);

    Qi.segment(6, 3) = N(1) * F.segment(0, 3);
    Qi.segment(9, 3) = N(1) * F.segment(3, 3);
    
    Qi.segment(12, 3) = N(2) * F.segment(0, 3);
    Qi.segment(15, 3) = N(2) * F.segment(3, 3);
}

ChVector<> ChContactTriangleXYZROT::ComputeNormal(const double U, const double V) {
    ChVector<> p0 = GetNode1()->GetPos();
    ChVector<> p1 = GetNode2()->GetPos();
    ChVector<> p2 = GetNode3()->GetPos();
    return Vcross(p1 - p0, p2 - p0).GetNormalized();
}

// -----------------------------------------------------------------------------
//  ChContactSurfaceMesh

ChContactSurfaceMesh::ChContactSurfaceMesh(std::shared_ptr<ChMaterialSurface> material, ChMesh* mesh)
    : ChContactSurface(material, mesh) {}

void ChContactSurfaceMesh::AddFacesFromBoundary(double sphere_swept, bool ccw) {
    std::vector<std::array<ChNodeFEAxyz*, 3>> triangles;
    std::vector<std::array<std::shared_ptr<ChNodeFEAxyz>, 3>> triangles_ptrs;

    std::vector<std::array<ChNodeFEAxyzrot*, 3>> triangles_rot;
    std::vector<std::array<std::shared_ptr<ChNodeFEAxyzrot>, 3>> triangles_rot_ptrs;

    //
    // Case1. Outer skin boundary of meshes of TETRAHEDRONS:
    //
    std::multimap<std::array<ChNodeFEAxyz*, 3>, ChFaceTetra_4> face_map;

    for (unsigned int ie = 0; ie < this->m_mesh->GetNelements(); ++ie) {
        if (auto mtetra = std::dynamic_pointer_cast<ChElementTetra_4>(m_mesh->GetElement(ie))) {
            for (int nface = 0; nface < 4; ++nface) {
                ChFaceTetra_4 mface(mtetra, nface);
                std::array<ChNodeFEAxyz*, 3> mface_key = {mface.GetNodeN(0).get(), mface.GetNodeN(1).get(),
                                                          mface.GetNodeN(2).get()};
                std::sort(mface_key.begin(), mface_key.end());
                face_map.insert({mface_key, mface});
            }
        }
    }
    for (unsigned int ie = 0; ie < this->m_mesh->GetNelements(); ++ie) {
        if (auto mtetra = std::dynamic_pointer_cast<ChElementTetra_4>(m_mesh->GetElement(ie))) {
            for (int nface = 0; nface < 4; ++nface) {
                ChFaceTetra_4 mface(mtetra, nface);
                std::array<ChNodeFEAxyz*, 3> mface_key = {mface.GetNodeN(0).get(), mface.GetNodeN(1).get(),
                                                          mface.GetNodeN(2).get()};
                std::sort(mface_key.begin(), mface_key.end());
                if (face_map.count(mface_key) == 1) {
                    // Found a face that is not shared.. so it is a boundary face.
                    triangles.push_back({{mface.GetNodeN(0).get(), mface.GetNodeN(1).get(), mface.GetNodeN(2).get()}});
                    triangles_ptrs.push_back({{mface.GetNodeN(0), mface.GetNodeN(1), mface.GetNodeN(2)}});
                }
            }
        }
    }

    //
    // Case2. skin of ANCF SHELLS:
    //
    for (unsigned int ie = 0; ie < this->m_mesh->GetNelements(); ++ie) {
        if (auto mshell = std::dynamic_pointer_cast<ChElementShellANCF>(m_mesh->GetElement(ie))) {
            std::shared_ptr<ChNodeFEAxyz> nA = mshell->GetNodeA();
            std::shared_ptr<ChNodeFEAxyz> nB = mshell->GetNodeB();
            std::shared_ptr<ChNodeFEAxyz> nC = mshell->GetNodeC();
            std::shared_ptr<ChNodeFEAxyz> nD = mshell->GetNodeD();
            if (ccw) {
                triangles.push_back({{nA.get(), nD.get(), nB.get()}});
                triangles.push_back({{nB.get(), nD.get(), nC.get()}});
                triangles_ptrs.push_back({{nA, nD, nB}});
                triangles_ptrs.push_back({{nB, nD, nC}});
            } else {
                triangles.push_back({{nA.get(), nB.get(), nD.get()}});
                triangles.push_back({{nB.get(), nC.get(), nD.get()}});
                triangles_ptrs.push_back({{nA, nB, nD}});
                triangles_ptrs.push_back({{nB, nC, nD}});
            }
        }
    }

    for (unsigned int ie = 0; ie < this->m_mesh->GetNelements(); ++ie) {
        if (auto mshell = std::dynamic_pointer_cast<ChElementShellANCF_8>(m_mesh->GetElement(ie))) {
            auto nA = mshell->GetNodeA();
            auto nB = mshell->GetNodeB();
            auto nC = mshell->GetNodeC();
            auto nD = mshell->GetNodeD();
            auto nE = mshell->GetNodeE();
            auto nF = mshell->GetNodeF();
            auto nG = mshell->GetNodeG();
            auto nH = mshell->GetNodeH();
            if (ccw) {
                triangles.push_back({{nA.get(), nH.get(), nE.get()}});
                triangles.push_back({{nB.get(), nE.get(), nF.get()}});
                triangles.push_back({{nC.get(), nF.get(), nG.get()}});
                triangles.push_back({{nD.get(), nG.get(), nH.get()}});
                triangles.push_back({{nH.get(), nG.get(), nE.get()}});
                triangles.push_back({{nF.get(), nE.get(), nG.get()}});
                triangles_ptrs.push_back({{nA, nH, nE}});
                triangles_ptrs.push_back({{nB, nE, nF}});
                triangles_ptrs.push_back({{nC, nF, nG}});
                triangles_ptrs.push_back({{nD, nG, nH}});
                triangles_ptrs.push_back({{nH, nG, nE}});
                triangles_ptrs.push_back({{nF, nE, nG}});
            } else {
                triangles.push_back({{nA.get(), nE.get(), nH.get()}});
                triangles.push_back({{nB.get(), nF.get(), nE.get()}});
                triangles.push_back({{nC.get(), nG.get(), nF.get()}});
                triangles.push_back({{nD.get(), nH.get(), nG.get()}});
                triangles.push_back({{nH.get(), nE.get(), nG.get()}});
                triangles.push_back({{nF.get(), nG.get(), nE.get()}});
                triangles_ptrs.push_back({{nA, nE, nH}});
                triangles_ptrs.push_back({{nB, nF, nE}});
                triangles_ptrs.push_back({{nC, nG, nF}});
                triangles_ptrs.push_back({{nD, nH, nG}});
                triangles_ptrs.push_back({{nH, nE, nG}});
                triangles_ptrs.push_back({{nF, nG, nE}});
            }
        }
    }

    //
    // Case3. EULER BEAMS (handles as a skinny triangle, with sphere swept radii, i.e. a capsule):
    //
    for (unsigned int ie = 0; ie < this->m_mesh->GetNelements(); ++ie) {
        if (auto mbeam = std::dynamic_pointer_cast<ChElementBeamEuler>(m_mesh->GetElement(ie))) {
            std::shared_ptr<ChNodeFEAxyzrot> nA = mbeam->GetNodeA();
            std::shared_ptr<ChNodeFEAxyzrot> nB = mbeam->GetNodeB();

            auto contact_triangle = chrono_types::make_shared<ChContactTriangleXYZROT>();
            contact_triangle->SetNode1(nA);
            contact_triangle->SetNode2(nB);
            contact_triangle->SetNode3(nB);
            this->vfaces_rot.push_back(contact_triangle);
            contact_triangle->SetContactSurface(this);

            contact_triangle->GetCollisionModel()->ClearModel();
            ((collision::ChCollisionModelBullet*)contact_triangle->GetCollisionModel())
                ->AddTriangleProxy(m_material,                                      // contact material
                                   &nA->coord.pos, &nB->coord.pos, &nB->coord.pos,  // vertices
                                   0, 0, 0,                                         // no wing vertexes
                                   false, false, false,  // are vertexes owned by this triangle?
                                   true, false, true,    // are edges owned by this triangle?
                                   mbeam->GetSection()->GetDrawCircularRadius());
            contact_triangle->GetCollisionModel()->BuildModel();
        }
    }

    //
    // Case4. ANCF BEAMS (handles as a skinny triangle, with sphere swept radii, i.e. a capsule):
    //
    for (unsigned int ie = 0; ie < this->m_mesh->GetNelements(); ++ie) {
        if (auto mbeam = std::dynamic_pointer_cast<ChElementCableANCF>(m_mesh->GetElement(ie))) {
            std::shared_ptr<ChNodeFEAxyzD> nA = mbeam->GetNodeA();
            std::shared_ptr<ChNodeFEAxyzD> nB = mbeam->GetNodeB();

            auto contact_triangle = chrono_types::make_shared<ChContactTriangleXYZ>();
            contact_triangle->SetNode1(nA);
            contact_triangle->SetNode2(nB);
            contact_triangle->SetNode3(nB);
            this->vfaces.push_back(contact_triangle);
            contact_triangle->SetContactSurface(this);

            contact_triangle->GetCollisionModel()->ClearModel();
            ((collision::ChCollisionModelBullet*)contact_triangle->GetCollisionModel())
                ->AddTriangleProxy(m_material,                    // contact materials
                                   &nA->pos, &nB->pos, &nB->pos,  // vertices
                                   0, 0, 0,                       // no wing vertexes
                                   false, false, false,           // are vertexes owned by this triangle?
                                   true, false, true,             // are edges owned by this triangle?
                                   mbeam->GetSection()->GetDrawCircularRadius());
            contact_triangle->GetCollisionModel()->BuildModel();
        }
    }

    //
    // Case5. Outer surface boundaries of 9-node brick meshes:
    //

    std::multimap<std::array<ChNodeFEAxyz*, 4>, ChFaceBrick_9> face_map_brick;

    for (unsigned int ie = 0; ie < this->m_mesh->GetNelements(); ++ie) {
        if (auto mbrick = std::dynamic_pointer_cast<ChElementBrick_9>(m_mesh->GetElement(ie))) {
            for (int nface = 0; nface < 6; ++nface) {
                ChFaceBrick_9 mface(mbrick, nface);
                std::array<ChNodeFEAxyz*, 4> mface_key = {mface.GetNodeN(0).get(), mface.GetNodeN(1).get(),
                                                          mface.GetNodeN(2).get(), mface.GetNodeN(3).get()};
                std::sort(mface_key.begin(), mface_key.end());
                face_map_brick.insert({mface_key, mface});
            }
        }
    }
    for (unsigned int ie = 0; ie < this->m_mesh->GetNelements(); ++ie) {
        if (auto mbrick = std::dynamic_pointer_cast<ChElementBrick_9>(m_mesh->GetElement(ie))) {
            for (int nface = 0; nface < 6; ++nface) {  // Each of the 6 faces of a brick
                ChFaceBrick_9 mface(mbrick, nface);    // Create a face of the element
                std::array<ChNodeFEAxyz*, 4> mface_key = {mface.GetNodeN(0).get(), mface.GetNodeN(1).get(),
                                                          mface.GetNodeN(2).get(), mface.GetNodeN(3).get()};
                std::sort(mface_key.begin(), mface_key.end());
                if (face_map_brick.count(mface_key) == 1) {
                    // Found a face that is not shared.. so it is a boundary face: Make two triangles out of that face
                    triangles.push_back({{mface.GetNodeN(0).get(), mface.GetNodeN(1).get(), mface.GetNodeN(2).get()}});
                    triangles.push_back({{mface.GetNodeN(0).get(), mface.GetNodeN(2).get(), mface.GetNodeN(3).get()}});
                    triangles_ptrs.push_back({{mface.GetNodeN(0), mface.GetNodeN(1), mface.GetNodeN(2)}});
                    triangles_ptrs.push_back({{mface.GetNodeN(0), mface.GetNodeN(2), mface.GetNodeN(3)}});
                }
            }
        }
    }

    //
    // Case6. skin of REISSNER SHELLS:
    //
    for (unsigned int ie = 0; ie < this->m_mesh->GetNelements(); ++ie) {
        if (auto mshell = std::dynamic_pointer_cast<ChElementShellReissner4>(m_mesh->GetElement(ie))) {
            std::shared_ptr<ChNodeFEAxyzrot> nA = mshell->GetNodeA();
            std::shared_ptr<ChNodeFEAxyzrot> nB = mshell->GetNodeB();
            std::shared_ptr<ChNodeFEAxyzrot> nC = mshell->GetNodeC();
            std::shared_ptr<ChNodeFEAxyzrot> nD = mshell->GetNodeD();
            if (ccw) {
                triangles_rot.push_back({{nA.get(), nD.get(), nB.get()}});
                triangles_rot.push_back({{nB.get(), nD.get(), nC.get()}});
                triangles_rot_ptrs.push_back({{nA, nD, nB}});
                triangles_rot_ptrs.push_back({{nB, nD, nC}});
            } else {
                triangles_rot.push_back({{nA.get(), nB.get(), nD.get()}});
                triangles_rot.push_back({{nB.get(), nC.get(), nD.get()}});
                triangles_rot_ptrs.push_back({{nA, nB, nD}});
                triangles_rot_ptrs.push_back({{nB, nC, nD}});
            }
        }
    }

    //
    // Case7. Outer surface boundaries of 8-node hexahedron brick meshes:
    //
    std::multimap<std::array<ChNodeFEAxyz*, 4>, ChFaceHexa_8> face_map_hexa;

    for (unsigned int ie = 0; ie < this->m_mesh->GetNelements(); ++ie) {
        if (auto mbrick = std::dynamic_pointer_cast<ChElementHexa_8>(m_mesh->GetElement(ie))) {
            for (int nface = 0; nface < 6; ++nface) {
                ChFaceHexa_8 mface(mbrick, nface);
                std::array<ChNodeFEAxyz*, 4> mface_key = {mface.GetNodeN(0).get(), mface.GetNodeN(1).get(),
                                                          mface.GetNodeN(2).get(), mface.GetNodeN(3).get()};
                std::sort(mface_key.begin(), mface_key.end());
                face_map_hexa.insert({mface_key, mface});
            }
        }
    }
    for (unsigned int ie = 0; ie < this->m_mesh->GetNelements(); ++ie) {
        if (auto mbrick = std::dynamic_pointer_cast<ChElementHexa_8>(m_mesh->GetElement(ie))) {
            for (int nface = 0; nface < 6; ++nface) {  // Each of the 6 faces of a brick
                ChFaceHexa_8 mface(mbrick, nface);     // Create a face of the element
                std::array<ChNodeFEAxyz*, 4> mface_key = {mface.GetNodeN(0).get(), mface.GetNodeN(1).get(),
                                                          mface.GetNodeN(2).get(), mface.GetNodeN(3).get()};
                std::sort(mface_key.begin(), mface_key.end());
                if (face_map_hexa.count(mface_key) == 1) {
                    // Found a face that is not shared.. so it is a boundary face: Make two triangles out of that face
                    triangles.push_back({{mface.GetNodeN(0).get(), mface.GetNodeN(1).get(), mface.GetNodeN(2).get()}});
                    triangles.push_back({{mface.GetNodeN(0).get(), mface.GetNodeN(2).get(), mface.GetNodeN(3).get()}});
                    triangles_ptrs.push_back({{mface.GetNodeN(0), mface.GetNodeN(1), mface.GetNodeN(2)}});
                    triangles_ptrs.push_back({{mface.GetNodeN(0), mface.GetNodeN(2), mface.GetNodeN(3)}});
                }
            }
        }
    }

    //
    // Compute triangles connectivity
    //
    std::multimap<std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*>, int> edge_map;

    for (int it = 0; it < triangles.size(); ++it) {
        // edges = pairs of vertexes indexes
        std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*> medgeA(triangles[it][0], triangles[it][1]);
        std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*> medgeB(triangles[it][1], triangles[it][2]);
        std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*> medgeC(triangles[it][2], triangles[it][0]);
        // vertex indexes in edges: always in increasing order to avoid ambiguous duplicated edges
        if (medgeA.first > medgeA.second)
            medgeA = std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*>(medgeA.second, medgeA.first);
        if (medgeB.first > medgeB.second)
            medgeB = std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*>(medgeB.second, medgeB.first);
        if (medgeC.first > medgeC.second)
            medgeC = std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*>(medgeC.second, medgeC.first);
        edge_map.insert({medgeA, it});
        edge_map.insert({medgeB, it});
        edge_map.insert({medgeC, it});
    }

    // Create a map of neighboring triangles, vector of:
    // [Ti TieA TieB TieC]
    std::vector<std::array<int, 4>> tri_map;
    tri_map.resize(triangles.size());

    for (int it = 0; it < triangles.size(); ++it) {
        tri_map[it][0] = it;
        tri_map[it][1] = -1;  // default no neighbor
        tri_map[it][2] = -1;  // default no neighbor
        tri_map[it][3] = -1;  // default no neighbor
        // edges = pairs of vertexes indexes
        std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*> medgeA(triangles[it][0], triangles[it][1]);
        std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*> medgeB(triangles[it][1], triangles[it][2]);
        std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*> medgeC(triangles[it][2], triangles[it][0]);
        // vertex indexes in edges: always in increasing order to avoid ambiguous duplicated edges
        if (medgeA.first > medgeA.second)
            medgeA = std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*>(medgeA.second, medgeA.first);
        if (medgeB.first > medgeB.second)
            medgeB = std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*>(medgeB.second, medgeB.first);
        if (medgeC.first > medgeC.second)
            medgeC = std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*>(medgeC.second, medgeC.first);
        auto retA = edge_map.equal_range(medgeA);
        for (auto fedge = retA.first; fedge != retA.second; ++fedge) {
            if (fedge->second != it) {
                tri_map[it][1] = fedge->second;
                break;
            }
        }
        auto retB = edge_map.equal_range(medgeB);
        for (auto fedge = retB.first; fedge != retB.second; ++fedge) {
            if (fedge->second != it) {
                tri_map[it][2] = fedge->second;
                break;
            }
        }
        auto retC = edge_map.equal_range(medgeC);
        for (auto fedge = retC.first; fedge != retC.second; ++fedge) {
            if (fedge->second != it) {
                tri_map[it][3] = fedge->second;
                break;
            }
        }
    }

    std::map<std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*>, std::pair<int, int>> winged_edges;
    bool allow_single_wing = true;

    for (auto aedge = edge_map.begin(); aedge != edge_map.end(); ++aedge) {
        auto ret = edge_map.equal_range(aedge->first);
        int nt = 0;
        std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*> wingedge;
        std::pair<int, int> wingtri;
        wingtri.first = -1;
        wingtri.second = -1;
        for (auto fedge = ret.first; fedge != ret.second; ++fedge) {
            if (fedge->second == -1)
                break;
            wingedge.first = fedge->first.first;
            wingedge.second = fedge->first.second;
            if (nt == 0)
                wingtri.first = fedge->second;
            if (nt == 1)
                wingtri.second = fedge->second;
            ++nt;
            if (nt == 2)
                break;
        }
        if ((nt == 2) || ((nt == 1) && allow_single_wing)) {
            winged_edges.insert(std::pair<std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*>, std::pair<int, int>>(
                wingedge, wingtri));  // ok found winged edge!
            aedge->second = -1;       // deactivate this way otherwise found again by sister
        }
    }

    // ....repeat: compute connectivity also for triangles with rotational dofs:

    std::multimap<std::pair<ChNodeFEAxyzrot*, ChNodeFEAxyzrot*>, int> edge_map_rot;

    for (int it = 0; it < triangles_rot.size(); ++it) {
        // edges = pairs of vertexes indexes
        std::pair<ChNodeFEAxyzrot*, ChNodeFEAxyzrot*> medgeA(triangles_rot[it][0], triangles_rot[it][1]);
        std::pair<ChNodeFEAxyzrot*, ChNodeFEAxyzrot*> medgeB(triangles_rot[it][1], triangles_rot[it][2]);
        std::pair<ChNodeFEAxyzrot*, ChNodeFEAxyzrot*> medgeC(triangles_rot[it][2], triangles_rot[it][0]);
        // vertex indexes in edges: always in increasing order to avoid ambiguous duplicated edges
        if (medgeA.first > medgeA.second)
            medgeA = std::pair<ChNodeFEAxyzrot*, ChNodeFEAxyzrot*>(medgeA.second, medgeA.first);
        if (medgeB.first > medgeB.second)
            medgeB = std::pair<ChNodeFEAxyzrot*, ChNodeFEAxyzrot*>(medgeB.second, medgeB.first);
        if (medgeC.first > medgeC.second)
            medgeC = std::pair<ChNodeFEAxyzrot*, ChNodeFEAxyzrot*>(medgeC.second, medgeC.first);
        edge_map_rot.insert({medgeA, it});
        edge_map_rot.insert({medgeB, it});
        edge_map_rot.insert({medgeC, it});
    }

    // Create a map of neighboring triangles, vector of:
    // [Ti TieA TieB TieC]
    std::vector<std::array<int, 4>> tri_map_rot;
    tri_map_rot.resize(triangles_rot.size());

    for (int it = 0; it < triangles_rot.size(); ++it) {
        tri_map_rot[it][0] = it;
        tri_map_rot[it][1] = -1;  // default no neighbor
        tri_map_rot[it][2] = -1;  // default no neighbor
        tri_map_rot[it][3] = -1;  // default no neighbor
        // edges = pairs of vertexes indexes
        std::pair<ChNodeFEAxyzrot*, ChNodeFEAxyzrot*> medgeA(triangles_rot[it][0], triangles_rot[it][1]);
        std::pair<ChNodeFEAxyzrot*, ChNodeFEAxyzrot*> medgeB(triangles_rot[it][1], triangles_rot[it][2]);
        std::pair<ChNodeFEAxyzrot*, ChNodeFEAxyzrot*> medgeC(triangles_rot[it][2], triangles_rot[it][0]);
        // vertex indexes in edges: always in increasing order to avoid ambiguous duplicated edges
        if (medgeA.first > medgeA.second)
            medgeA = std::pair<ChNodeFEAxyzrot*, ChNodeFEAxyzrot*>(medgeA.second, medgeA.first);
        if (medgeB.first > medgeB.second)
            medgeB = std::pair<ChNodeFEAxyzrot*, ChNodeFEAxyzrot*>(medgeB.second, medgeB.first);
        if (medgeC.first > medgeC.second)
            medgeC = std::pair<ChNodeFEAxyzrot*, ChNodeFEAxyzrot*>(medgeC.second, medgeC.first);
        auto retA = edge_map_rot.equal_range(medgeA);
        for (auto fedge = retA.first; fedge != retA.second; ++fedge) {
            if (fedge->second != it) {
                tri_map_rot[it][1] = fedge->second;
                break;
            }
        }
        auto retB = edge_map_rot.equal_range(medgeB);
        for (auto fedge = retB.first; fedge != retB.second; ++fedge) {
            if (fedge->second != it) {
                tri_map_rot[it][2] = fedge->second;
                break;
            }
        }
        auto retC = edge_map_rot.equal_range(medgeC);
        for (auto fedge = retC.first; fedge != retC.second; ++fedge) {
            if (fedge->second != it) {
                tri_map_rot[it][3] = fedge->second;
                break;
            }
        }
    }

    std::map<std::pair<ChNodeFEAxyzrot*, ChNodeFEAxyzrot*>, std::pair<int, int>> winged_edges_rot;
    bool allow_single_wing_rot = true;

    for (auto aedge = edge_map_rot.begin(); aedge != edge_map_rot.end(); ++aedge) {
        auto ret = edge_map_rot.equal_range(aedge->first);
        int nt = 0;
        std::pair<ChNodeFEAxyzrot*, ChNodeFEAxyzrot*> wingedge;
        std::pair<int, int> wingtri;
        wingtri.first = -1;
        wingtri.second = -1;
        for (auto fedge = ret.first; fedge != ret.second; ++fedge) {
            if (fedge->second == -1)
                break;
            wingedge.first = fedge->first.first;
            wingedge.second = fedge->first.second;
            if (nt == 0)
                wingtri.first = fedge->second;
            if (nt == 1)
                wingtri.second = fedge->second;
            ++nt;
            if (nt == 2)
                break;
        }
        if ((nt == 2) || ((nt == 1) && allow_single_wing_rot)) {
            winged_edges_rot.insert(std::pair<std::pair<ChNodeFEAxyzrot*, ChNodeFEAxyzrot*>, std::pair<int, int>>(
                wingedge, wingtri));  // ok found winged edge!
            aedge->second = -1;       // deactivate this way otherwise found again by sister
        }
    }

    //
    // Now create triangles with collision models:
    //
    std::set<ChNodeFEAxyz*> added_vertexes;

    // iterate on triangles
    for (int it = 0; it < triangles.size(); ++it) {
        // edges = pairs of vertexes indexes
        std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*> medgeA(triangles[it][0], triangles[it][1]);
        std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*> medgeB(triangles[it][1], triangles[it][2]);
        std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*> medgeC(triangles[it][2], triangles[it][0]);
        // vertex indexes in edges: always in increasing order to avoid ambiguous duplicated edges
        if (medgeA.first > medgeA.second)
            medgeA = std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*>(medgeA.second, medgeA.first);
        if (medgeB.first > medgeB.second)
            medgeB = std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*>(medgeB.second, medgeB.first);
        if (medgeC.first > medgeC.second)
            medgeC = std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*>(medgeC.second, medgeC.first);
        auto wingedgeA = winged_edges.find(medgeA);
        auto wingedgeB = winged_edges.find(medgeB);
        auto wingedgeC = winged_edges.find(medgeC);

        ChNodeFEAxyz* i_wingvertex_A = 0;
        ChNodeFEAxyz* i_wingvertex_B = 0;
        ChNodeFEAxyz* i_wingvertex_C = 0;

        if (tri_map[it][1] != -1) {
            i_wingvertex_A = triangles[tri_map[it][1]][0];
            if (triangles[tri_map[it][1]][1] != wingedgeA->first.first &&
                triangles[tri_map[it][1]][1] != wingedgeA->first.second)
                i_wingvertex_A = triangles[tri_map[it][1]][1];
            if (triangles[tri_map[it][1]][2] != wingedgeA->first.first &&
                triangles[tri_map[it][1]][2] != wingedgeA->first.second)
                i_wingvertex_A = triangles[tri_map[it][1]][2];
        }

        if (tri_map[it][2] != -1) {
            i_wingvertex_B = triangles[tri_map[it][2]][0];
            if (triangles[tri_map[it][2]][1] != wingedgeB->first.first &&
                triangles[tri_map[it][2]][1] != wingedgeB->first.second)
                i_wingvertex_B = triangles[tri_map[it][2]][1];
            if (triangles[tri_map[it][2]][2] != wingedgeB->first.first &&
                triangles[tri_map[it][2]][2] != wingedgeB->first.second)
                i_wingvertex_B = triangles[tri_map[it][2]][2];
        }

        if (tri_map[it][3] != -1) {
            i_wingvertex_C = triangles[tri_map[it][3]][0];
            if (triangles[tri_map[it][3]][1] != wingedgeC->first.first &&
                triangles[tri_map[it][3]][1] != wingedgeC->first.second)
                i_wingvertex_C = triangles[tri_map[it][3]][1];
            if (triangles[tri_map[it][3]][2] != wingedgeC->first.first &&
                triangles[tri_map[it][3]][2] != wingedgeC->first.second)
                i_wingvertex_C = triangles[tri_map[it][3]][2];
        }

        auto contact_triangle = chrono_types::make_shared<ChContactTriangleXYZ>();
        contact_triangle->SetNode1(triangles_ptrs[it][0]);
        contact_triangle->SetNode2(triangles_ptrs[it][1]);
        contact_triangle->SetNode3(triangles_ptrs[it][2]);
        this->vfaces.push_back(contact_triangle);
        contact_triangle->SetContactSurface(this);

        contact_triangle->GetCollisionModel()->ClearModel();
        ((collision::ChCollisionModelBullet*)contact_triangle->GetCollisionModel())
            ->AddTriangleProxy(m_material,  // contact material
                               &triangles[it][0]->pos, &triangles[it][1]->pos, &triangles[it][2]->pos,
                               // if no wing vertex (ie. 'free' edge), point to opposite vertex, ie vertex in triangle
                               // not belonging to edge
                               wingedgeA->second.second != -1 ? &i_wingvertex_A->pos : &triangles[it][2]->pos,
                               wingedgeB->second.second != -1 ? &i_wingvertex_B->pos : &triangles[it][0]->pos,
                               wingedgeC->second.second != -1 ? &i_wingvertex_C->pos : &triangles[it][1]->pos,
                               (added_vertexes.find(triangles[it][0]) == added_vertexes.end()),
                               (added_vertexes.find(triangles[it][1]) == added_vertexes.end()),
                               (added_vertexes.find(triangles[it][2]) == added_vertexes.end()),
                               // are edges owned by this triangle? (if not, they belong to a neighboring triangle)
                               wingedgeA->second.first != -1, wingedgeB->second.first != -1,
                               wingedgeC->second.first != -1, sphere_swept);
        contact_triangle->GetCollisionModel()->BuildModel();

        // Mark added vertexes
        added_vertexes.insert(triangles[it][0]);
        added_vertexes.insert(triangles[it][1]);
        added_vertexes.insert(triangles[it][2]);
        // Mark added edges, setting to -1 the 'ti' id of 1st triangle in winged edge {{vi,vj}{ti,tj}}
        wingedgeA->second.first = -1;
        wingedgeB->second.first = -1;
        wingedgeC->second.first = -1;
    }

    // ....repeat: create triangles with collision models for nodes with rotationaldofs too:

    std::set<ChNodeFEAxyzrot*> added_vertexes_rot;

    // iterate on triangles
    for (int it = 0; it < triangles_rot.size(); ++it) {
        // edges = pairs of vertexes indexes
        std::pair<ChNodeFEAxyzrot*, ChNodeFEAxyzrot*> medgeA(triangles_rot[it][0], triangles_rot[it][1]);
        std::pair<ChNodeFEAxyzrot*, ChNodeFEAxyzrot*> medgeB(triangles_rot[it][1], triangles_rot[it][2]);
        std::pair<ChNodeFEAxyzrot*, ChNodeFEAxyzrot*> medgeC(triangles_rot[it][2], triangles_rot[it][0]);
        // vertex indexes in edges: always in increasing order to avoid ambiguous duplicated edges
        if (medgeA.first > medgeA.second)
            medgeA = std::pair<ChNodeFEAxyzrot*, ChNodeFEAxyzrot*>(medgeA.second, medgeA.first);
        if (medgeB.first > medgeB.second)
            medgeB = std::pair<ChNodeFEAxyzrot*, ChNodeFEAxyzrot*>(medgeB.second, medgeB.first);
        if (medgeC.first > medgeC.second)
            medgeC = std::pair<ChNodeFEAxyzrot*, ChNodeFEAxyzrot*>(medgeC.second, medgeC.first);
        auto wingedgeA = winged_edges_rot.find(medgeA);
        auto wingedgeB = winged_edges_rot.find(medgeB);
        auto wingedgeC = winged_edges_rot.find(medgeC);

        ChNodeFEAxyzrot* i_wingvertex_A = 0;
        ChNodeFEAxyzrot* i_wingvertex_B = 0;
        ChNodeFEAxyzrot* i_wingvertex_C = 0;

        if (tri_map_rot[it][1] != -1) {
            i_wingvertex_A = triangles_rot[tri_map_rot[it][1]][0];
            if (triangles_rot[tri_map_rot[it][1]][1] != wingedgeA->first.first &&
                triangles_rot[tri_map_rot[it][1]][1] != wingedgeA->first.second)
                i_wingvertex_A = triangles_rot[tri_map_rot[it][1]][1];
            if (triangles_rot[tri_map_rot[it][1]][2] != wingedgeA->first.first &&
                triangles_rot[tri_map_rot[it][1]][2] != wingedgeA->first.second)
                i_wingvertex_A = triangles_rot[tri_map_rot[it][1]][2];
        }

        if (tri_map_rot[it][2] != -1) {
            i_wingvertex_B = triangles_rot[tri_map_rot[it][2]][0];
            if (triangles_rot[tri_map_rot[it][2]][1] != wingedgeB->first.first &&
                triangles_rot[tri_map_rot[it][2]][1] != wingedgeB->first.second)
                i_wingvertex_B = triangles_rot[tri_map_rot[it][2]][1];
            if (triangles_rot[tri_map_rot[it][2]][2] != wingedgeB->first.first &&
                triangles_rot[tri_map_rot[it][2]][2] != wingedgeB->first.second)
                i_wingvertex_B = triangles_rot[tri_map_rot[it][2]][2];
        }

        if (tri_map_rot[it][3] != -1) {
            i_wingvertex_C = triangles_rot[tri_map_rot[it][3]][0];
            if (triangles_rot[tri_map_rot[it][3]][1] != wingedgeC->first.first &&
                triangles_rot[tri_map_rot[it][3]][1] != wingedgeC->first.second)
                i_wingvertex_C = triangles_rot[tri_map_rot[it][3]][1];
            if (triangles_rot[tri_map_rot[it][3]][2] != wingedgeC->first.first &&
                triangles_rot[tri_map_rot[it][3]][2] != wingedgeC->first.second)
                i_wingvertex_C = triangles_rot[tri_map_rot[it][3]][2];
        }

        auto contact_triangle_rot = chrono_types::make_shared<ChContactTriangleXYZROT>();
        contact_triangle_rot->SetNode1(triangles_rot_ptrs[it][0]);
        contact_triangle_rot->SetNode2(triangles_rot_ptrs[it][1]);
        contact_triangle_rot->SetNode3(triangles_rot_ptrs[it][2]);
        this->vfaces_rot.push_back(contact_triangle_rot);
        contact_triangle_rot->SetContactSurface(this);

        contact_triangle_rot->GetCollisionModel()->ClearModel();
        ((collision::ChCollisionModelBullet*)contact_triangle_rot->GetCollisionModel())
            ->AddTriangleProxy(
                m_material,  // contact material
                &triangles_rot[it][0]->coord.pos, &triangles_rot[it][1]->coord.pos, &triangles_rot[it][2]->coord.pos,
                // if no wing vertex (ie. 'free' edge), point to opposite vertex, ie vertex in triangle not belonging to
                // edge
                wingedgeA->second.second != -1 ? &i_wingvertex_A->coord.pos : &triangles_rot[it][2]->coord.pos,
                wingedgeB->second.second != -1 ? &i_wingvertex_B->coord.pos : &triangles_rot[it][0]->coord.pos,
                wingedgeC->second.second != -1 ? &i_wingvertex_C->coord.pos : &triangles_rot[it][1]->coord.pos,
                (added_vertexes_rot.find(triangles_rot[it][0]) == added_vertexes_rot.end()),
                (added_vertexes_rot.find(triangles_rot[it][1]) == added_vertexes_rot.end()),
                (added_vertexes_rot.find(triangles_rot[it][2]) == added_vertexes_rot.end()),
                // are edges owned by this triangle? (if not, they belong to a neighboring triangle)
                wingedgeA->second.first != -1, wingedgeB->second.first != -1, wingedgeC->second.first != -1,
                sphere_swept);
        contact_triangle_rot->GetCollisionModel()->BuildModel();

        // Mark added vertexes
        added_vertexes_rot.insert(triangles_rot[it][0]);
        added_vertexes_rot.insert(triangles_rot[it][1]);
        added_vertexes_rot.insert(triangles_rot[it][2]);
        // Mark added edges, setting to -1 the 'ti' id of 1st triangle in winged edge {{vi,vj}{ti,tj}}
        wingedgeA->second.first = -1;
        wingedgeB->second.first = -1;
        wingedgeC->second.first = -1;
    }
}

unsigned int ChContactSurfaceMesh::GetNumVertices() const {
    std::map<ChNodeFEAxyz*, size_t> ptr_ind_map;
    size_t count = 0;
    for (size_t i = 0; i < vfaces.size(); ++i) {
        if (!ptr_ind_map.count(vfaces[i]->GetNode1().get())) {
            ptr_ind_map.insert({vfaces[i]->GetNode1().get(), count});
            count++;
        }
        if (!ptr_ind_map.count(vfaces[i]->GetNode2().get())) {
            ptr_ind_map.insert({vfaces[i]->GetNode2().get(), count});
            count++;
        }
        if (!ptr_ind_map.count(vfaces[i]->GetNode3().get())) {
            ptr_ind_map.insert({vfaces[i]->GetNode3().get(), count});
            count++;
        }
    }

    std::map<ChNodeFEAxyzrot*, size_t> ptr_ind_map_rot;
    size_t count_rot = 0;
    for (size_t i = 0; i < vfaces_rot.size(); ++i) {
        if (!ptr_ind_map_rot.count(vfaces_rot[i]->GetNode1().get())) {
            ptr_ind_map_rot.insert({vfaces_rot[i]->GetNode1().get(), count_rot});
            count_rot++;
        }
        if (!ptr_ind_map_rot.count(vfaces_rot[i]->GetNode2().get())) {
            ptr_ind_map_rot.insert({vfaces_rot[i]->GetNode2().get(), count_rot});
            count_rot++;
        }
        if (!ptr_ind_map_rot.count(vfaces_rot[i]->GetNode3().get())) {
            ptr_ind_map_rot.insert({vfaces_rot[i]->GetNode3().get(), count_rot});
            count_rot++;
        }
    }

    return (unsigned int)(count + count_rot);
}

void ChContactSurfaceMesh::SurfaceSyncCollisionModels() {
    for (unsigned int j = 0; j < vfaces.size(); j++) {
        this->vfaces[j]->GetCollisionModel()->SyncPosition();
    }
    for (unsigned int j = 0; j < vfaces_rot.size(); j++) {
        this->vfaces_rot[j]->GetCollisionModel()->SyncPosition();
    }
}

void ChContactSurfaceMesh::SurfaceAddCollisionModelsToSystem(ChSystem* msys) {
    assert(msys);
    SurfaceSyncCollisionModels();
    for (unsigned int j = 0; j < vfaces.size(); j++) {
        msys->GetCollisionSystem()->Add(this->vfaces[j]->GetCollisionModel());
    }
    for (unsigned int j = 0; j < vfaces_rot.size(); j++) {
        msys->GetCollisionSystem()->Add(this->vfaces_rot[j]->GetCollisionModel());
    }
}

void ChContactSurfaceMesh::SurfaceRemoveCollisionModelsFromSystem(ChSystem* msys) {
    assert(msys);
    for (unsigned int j = 0; j < vfaces.size(); j++) {
        msys->GetCollisionSystem()->Remove(this->vfaces[j]->GetCollisionModel());
    }
    for (unsigned int j = 0; j < vfaces_rot.size(); j++) {
        msys->GetCollisionSystem()->Remove(this->vfaces_rot[j]->GetCollisionModel());
    }
}

}  // end namespace fea
}  // end namespace chrono
