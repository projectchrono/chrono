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

#include "chrono/assets/ChGlyphs.h"
#include "chrono/assets/ChTriangleMeshShape.h"

#include "chrono/fea/ChContactSurfaceMesh.h"
#include "chrono/fea/ChContactSurfaceNodeCloud.h"
#include "chrono/fea/ChElementCableANCF.h"
#include "chrono/fea/ChElementBeamEuler.h"
#include "chrono/fea/ChElementBeamIGA.h"
#include "chrono/fea/ChElementBrick.h"
#include "chrono/fea/ChElementBrick_9.h"
#include "chrono/fea/ChElementHexa_20.h"
#include "chrono/fea/ChElementHexa_8.h"
#include "chrono/fea/ChElementShell.h"
#include "chrono/fea/ChElementShellReissner4.h"
#include "chrono/fea/ChElementTetra_10.h"
#include "chrono/fea/ChElementTetra_4.h"
#include "chrono/fea/ChFaceTetra_4.h"
#include "chrono/fea/ChVisualizationFEAmesh.h"

namespace chrono {
namespace fea {

ChVisualizationFEAmesh::ChVisualizationFEAmesh(ChMesh& mymesh) {
    FEMmesh = &mymesh;
    fem_data_type = E_PLOT_SURFACE;
    fem_glyph = E_GLYPH_NONE;

    colorscale_min = 0;
    colorscale_max = 1;

    shrink_elements = false;
    shrink_factor = 0.9;

    symbols_scale = 1.0;
    symbols_thickness = 0.002;

    wireframe = false;

    zbuffer_hide = true;

    smooth_faces = false;

    beam_resolution = 8;
    beam_resolution_section = 10;
    shell_resolution = 3;

    meshcolor = ChColor(1, 1, 1, 0);
    symbolscolor = ChColor(0, 0.5, 0.5, 0);

    undeformed_reference = false;

    auto new_mesh_asset = std::make_shared<ChTriangleMeshShape>();
    this->AddAsset(new_mesh_asset);

    auto new_glyphs_asset = std::make_shared<ChGlyphs>();
    this->AddAsset(new_glyphs_asset);
}

ChColor ChVisualizationFEAmesh::ComputeFalseColor2(double mv) {
    ChColor c = ChColor::ComputeFalseColor(mv, this->colorscale_min, this->colorscale_max, true);

    if (this->fem_data_type == E_PLOT_SURFACE)
        c = ChColor(meshcolor.R, meshcolor.G, meshcolor.B);

    return c;
}

ChVector<float> ChVisualizationFEAmesh::ComputeFalseColor(double mv) {
    ChColor c = this->ComputeFalseColor2(mv);
    ChVector<float> vc(c.R, c.G, c.B);
    return vc;
}

double ChVisualizationFEAmesh::ComputeScalarOutput(std::shared_ptr<ChNodeFEAxyz> mnode,
                                                   int nodeID,
                                                   std::shared_ptr<ChElementBase> melement) {
    switch (this->fem_data_type) {
        case E_PLOT_SURFACE:
            return 1e30;  // to force 'white' in false color scale. Hack, to be improved.
        case E_PLOT_NODE_DISP_NORM:
            return (mnode->GetPos() - mnode->GetX0()).Length();
        case E_PLOT_NODE_DISP_X:
            return (mnode->GetPos() - mnode->GetX0()).x();
        case E_PLOT_NODE_DISP_Y:
            return (mnode->GetPos() - mnode->GetX0()).y();
        case E_PLOT_NODE_DISP_Z:
            return (mnode->GetPos() - mnode->GetX0()).z();
        case E_PLOT_NODE_SPEED_NORM:
            return mnode->GetPos_dt().Length();
        case E_PLOT_NODE_SPEED_X:
            return mnode->GetPos_dt().x();
        case E_PLOT_NODE_SPEED_Y:
            return mnode->GetPos_dt().y();
        case E_PLOT_NODE_SPEED_Z:
            return mnode->GetPos_dt().z();
        case E_PLOT_NODE_ACCEL_NORM:
            return mnode->GetPos_dtdt().Length();
        case E_PLOT_NODE_ACCEL_X:
            return mnode->GetPos_dtdt().x();
        case E_PLOT_NODE_ACCEL_Y:
            return mnode->GetPos_dtdt().y();
        case E_PLOT_NODE_ACCEL_Z:
            return mnode->GetPos_dtdt().z();
        case E_PLOT_ELEM_STRAIN_VONMISES:
            if (auto mytetra = std::dynamic_pointer_cast<ChElementTetra_4>(melement)) {
                return mytetra->GetStrain().GetEquivalentVonMises();
            }
        case E_PLOT_ELEM_STRESS_VONMISES:
            if (auto mytetra = std::dynamic_pointer_cast<ChElementTetra_4>(melement)) {
                return mytetra->GetStress().GetEquivalentVonMises();
            }
        case E_PLOT_ELEM_STRAIN_HYDROSTATIC:
            if (auto mytetra = std::dynamic_pointer_cast<ChElementTetra_4>(melement)) {
                return mytetra->GetStrain().GetEquivalentMeanHydrostatic();
            }
        case E_PLOT_ELEM_STRESS_HYDROSTATIC:
            if (auto mytetra = std::dynamic_pointer_cast<ChElementTetra_4>(melement)) {
                return mytetra->GetStress().GetEquivalentMeanHydrostatic();
            }
        default:
            return 1e30;
    }
    //***TO DO*** other types of scalar outputs
    return 0;
}

double ChVisualizationFEAmesh::ComputeScalarOutput(std::shared_ptr<ChNodeFEAxyzP> mnode,
                                                   int nodeID,
                                                   std::shared_ptr<ChElementBase> melement) {
    switch (this->fem_data_type) {
        case E_PLOT_SURFACE:
            return 1e30;  // to force 'white' in false color scale. Hack, to be improved.
        case E_PLOT_NODE_P:
            return (mnode->GetP());
        default:
            return 1e30;
    }
    //***TO DO*** other types of scalar outputs
    return 0;
}

ChVector<float>& FetchOrAllocate(std::vector<ChVector<float>>& mvector, unsigned int& id) {
    if (id > mvector.size()) {
        id = 0;
        return mvector[0];  // error
    }
    if (id == mvector.size()) {
        mvector.push_back(ChVector<float>(0, 0, 0));
    }
    ++id;
    return mvector[id - 1];
}

void TriangleNormalsReset(std::vector<ChVector<>>& normals, std::vector<int>& accumul) {
    for (unsigned int nn = 0; nn < normals.size(); ++nn) {
        normals[nn] = ChVector<>(0, 0, 0);
        accumul[nn] = 0;
    }
}
void TriangleNormalsCompute(ChVector<int> norm_indexes,
                            ChVector<int> vert_indexes,
                            std::vector<ChVector<>>& vertexes,
                            std::vector<ChVector<>>& normals,
                            std::vector<int>& accumul) {
    ChVector<> tnorm =
        Vcross(vertexes[vert_indexes.y()] - vertexes[vert_indexes.x()], vertexes[vert_indexes.z()] - vertexes[vert_indexes.x()])
            .GetNormalized();
    normals[norm_indexes.x()] += tnorm;
    normals[norm_indexes.y()] += tnorm;
    normals[norm_indexes.z()] += tnorm;
    accumul[norm_indexes.x()] += 1;
    accumul[norm_indexes.y()] += 1;
    accumul[norm_indexes.z()] += 1;
}
void TriangleNormalsSmooth(std::vector<ChVector<>>& normals, std::vector<int>& accumul) {
    for (unsigned int nn = 0; nn < normals.size(); ++nn) {
        normals[nn] = normals[nn] * (1.0 / (double)accumul[nn]);
    }
}

// Helper function for updating visualization mesh buffers for hex elements.
void ChVisualizationFEAmesh::UpdateBuffers_Hex(std::shared_ptr<ChElementBase> element,
                                               geometry::ChTriangleMeshConnected& trianglemesh,
                                               unsigned int& i_verts,
                                               unsigned int& i_vnorms,
                                               unsigned int& i_vcols,
                                               unsigned int& i_triindex) {
    unsigned int ivert_el = i_verts;
    unsigned int inorm_el = i_vnorms;

    std::shared_ptr<ChNodeFEAxyz> nodes[8];
    ChVector<> pt[8];

    for (int in = 0; in < 8; ++in) {
        nodes[in] = std::static_pointer_cast<ChNodeFEAxyz>(element->GetNodeN(in));
        if (!undeformed_reference)
            pt[in] = nodes[in]->GetPos();
        else
            pt[in] = nodes[in]->GetX0();
    }

    // vertexes

    if (this->shrink_elements) {
        ChVector<> vc(0, 0, 0);
        for (int in = 0; in < 8; ++in)
            vc += pt[in];
        vc = vc * (1.0 / 8.0);  // average, center of element
        for (int in = 0; in < 8; ++in)
            pt[in] = vc + this->shrink_factor * (pt[in] - vc);
    }

    for (int in = 0; in < 8; ++in) {
        trianglemesh.getCoordsVertices()[i_verts] = pt[in];
        ++i_verts;
    }

    // colours and colours indexes
    for (int in = 0; in < 8; ++in) {
        trianglemesh.getCoordsColors()[i_vcols] = ComputeFalseColor(ComputeScalarOutput(nodes[in], in, element));
        ++i_vcols;
    }

    // faces indexes
    ChVector<int> ivert_offset(ivert_el, ivert_el, ivert_el);
    trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int>(0, 2, 1) + ivert_offset;
    ++i_triindex;
    trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int>(0, 3, 2) + ivert_offset;
    ++i_triindex;
    trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int>(4, 5, 6) + ivert_offset;
    ++i_triindex;
    trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int>(4, 6, 7) + ivert_offset;
    ++i_triindex;
    trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int>(0, 7, 3) + ivert_offset;
    ++i_triindex;
    trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int>(0, 4, 7) + ivert_offset;
    ++i_triindex;
    trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int>(0, 5, 4) + ivert_offset;
    ++i_triindex;
    trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int>(0, 1, 5) + ivert_offset;
    ++i_triindex;
    trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int>(3, 7, 6) + ivert_offset;
    ++i_triindex;
    trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int>(3, 6, 2) + ivert_offset;
    ++i_triindex;
    trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int>(2, 5, 1) + ivert_offset;
    ++i_triindex;
    trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int>(2, 6, 5) + ivert_offset;
    ++i_triindex;

    // normals indices (if not defaulting to flat triangles)
    if (this->smooth_faces) {
        ChVector<int> inorm_offset = ChVector<int>(inorm_el, inorm_el, inorm_el);
        trianglemesh.getIndicesNormals()[i_triindex - 12] = ChVector<int>(0, 2, 1) + inorm_offset;
        trianglemesh.getIndicesNormals()[i_triindex - 11] = ChVector<int>(0, 3, 2) + inorm_offset;
        trianglemesh.getIndicesNormals()[i_triindex - 10] = ChVector<int>(4, 5, 6) + inorm_offset;
        trianglemesh.getIndicesNormals()[i_triindex - 9] = ChVector<int>(4, 6, 7) + inorm_offset;
        trianglemesh.getIndicesNormals()[i_triindex - 8] = ChVector<int>(8, 9, 10) + inorm_offset;
        trianglemesh.getIndicesNormals()[i_triindex - 7] = ChVector<int>(8, 11, 9) + inorm_offset;
        trianglemesh.getIndicesNormals()[i_triindex - 6] = ChVector<int>(12, 13, 14) + inorm_offset;
        trianglemesh.getIndicesNormals()[i_triindex - 5] = ChVector<int>(12, 15, 13) + inorm_offset;
        trianglemesh.getIndicesNormals()[i_triindex - 4] = ChVector<int>(16, 18, 17) + inorm_offset;
        trianglemesh.getIndicesNormals()[i_triindex - 3] = ChVector<int>(16, 17, 19) + inorm_offset;
        trianglemesh.getIndicesNormals()[i_triindex - 2] = ChVector<int>(20, 21, 23) + inorm_offset;
        trianglemesh.getIndicesNormals()[i_triindex - 1] = ChVector<int>(20, 22, 21) + inorm_offset;
        i_vnorms += 24;
    }
}

void ChVisualizationFEAmesh::Update(ChPhysicsItem* updater, const ChCoordsys<>& coords) {
    if (!this->FEMmesh)
        return;

    std::shared_ptr<ChTriangleMeshShape> mesh_asset;
    std::shared_ptr<ChGlyphs> glyphs_asset;

    // try to retrieve previously added mesh asset and glyhs asset in sublevel..
    if (this->GetAssets().size() == 2) {
        mesh_asset = std::dynamic_pointer_cast<ChTriangleMeshShape>(GetAssets()[0]);
        glyphs_asset = std::dynamic_pointer_cast<ChGlyphs>(GetAssets()[1]);
    }

    // if not available, create ...
    if (!mesh_asset) {
        this->GetAssets().resize(0);  // this to delete other sub assets that are not in mesh & glyphs, if any

        auto new_mesh_asset = std::make_shared<ChTriangleMeshShape>();
        this->AddAsset(new_mesh_asset);
        mesh_asset = new_mesh_asset;

        auto new_glyphs_asset = std::make_shared<ChGlyphs>();
        this->AddAsset(new_glyphs_asset);
        glyphs_asset = new_glyphs_asset;
    }
    auto trianglemesh = mesh_asset->GetMesh();

    size_t n_verts = 0;
    size_t n_vcols = 0;
    size_t n_vnorms = 0;
    size_t n_triangles = 0;

    //
    // A - Count the needed vertexes and faces
    //

    //   In case of colormap drawing:
    //
    if (this->fem_data_type != E_PLOT_NONE && this->fem_data_type != E_PLOT_LOADSURFACES &&
        this->fem_data_type != E_PLOT_CONTACTSURFACES) {
        for (unsigned int iel = 0; iel < this->FEMmesh->GetNelements(); ++iel) {
            if (std::dynamic_pointer_cast<ChElementTetra_4>(this->FEMmesh->GetElement(iel))) {
                // ELEMENT IS A TETRAHEDRON
                n_verts += 4;
                n_vcols += 4;
                n_vnorms += 4;     // flat faces
                n_triangles += 4;  // n. triangle faces
            } else if (std::dynamic_pointer_cast<ChElementTetra_4_P>(this->FEMmesh->GetElement(iel))) {
                // ELEMENT IS A TETRAHEDRON for scalar field
                n_verts += 4;
                n_vcols += 4;
                n_vnorms += 4;     // flat faces
                n_triangles += 4;  // n. triangle faces
            } else if (std::dynamic_pointer_cast<ChElementHexa_8>(FEMmesh->GetElement(iel)) ||
                       std::dynamic_pointer_cast<ChElementBrick>(FEMmesh->GetElement(iel)) ||
                       std::dynamic_pointer_cast<ChElementBrick_9>(FEMmesh->GetElement(iel))) {
                // ELEMENT IS A HEXAHEDRON
                n_verts += 8;
                n_vcols += 8;
                n_vnorms += 24;
                n_triangles += 12;  // n. triangle faces
            } else if (std::dynamic_pointer_cast<ChElementBeam>(this->FEMmesh->GetElement(iel))) {
                // ELEMENT IS A BEAM
                bool m_circular = false;
                // downcasting
                if (auto mybeameuler = std::dynamic_pointer_cast<ChElementBeamEuler>(this->FEMmesh->GetElement(iel))) {
                    if (mybeameuler->GetSection()->IsCircular())
                        m_circular = true;
                } else if (auto mybeamancf = std::dynamic_pointer_cast<ChElementCableANCF>(this->FEMmesh->GetElement(iel))) {
                    if (mybeamancf->GetSection()->IsCircular())
                        m_circular = true;
                } else if (auto mybeamiga = std::dynamic_pointer_cast<ChElementBeamIGA>(this->FEMmesh->GetElement(iel))) {
                    if (mybeamiga->GetSection()->IsCircular())
                        m_circular = true;
                }
                if (m_circular) {
                    n_verts += beam_resolution_section * beam_resolution;
                    n_vcols += beam_resolution_section * beam_resolution;
                    n_vnorms += beam_resolution_section * beam_resolution;
                    n_triangles += 2 * beam_resolution_section * (beam_resolution - 1);  // n. triangle faces
                } else {                                                                 // rectangular
                    n_verts += 4 * beam_resolution;
                    n_vcols += 4 * beam_resolution;
                    n_vnorms += 8 * beam_resolution;
                    n_triangles += 8 * (beam_resolution - 1);  // n. triangle faces
                }
            } else if (std::dynamic_pointer_cast<ChElementShell>(this->FEMmesh->GetElement(iel))) {
                // ELEMENT IS A SHELL
                n_verts += shell_resolution * shell_resolution;
                n_vcols += shell_resolution * shell_resolution;
                n_vnorms += shell_resolution * shell_resolution;
                n_triangles += 2 * (shell_resolution - 1) * (shell_resolution - 1);  // n. triangle faces
            }

            //***TO DO*** other types of elements...
        }
    }

    //   In case mesh surfaces for pressure loads etc.:
    //
    if (this->fem_data_type == E_PLOT_LOADSURFACES) {
        for (unsigned int isu = 0; isu < this->FEMmesh->GetNmeshSurfaces(); ++isu) {
            std::shared_ptr<ChMeshSurface> msurface = this->FEMmesh->GetMeshSurface(isu);
            for (unsigned int ifa = 0; ifa < msurface->GetFacesList().size(); ++ifa) {
                std::shared_ptr<ChLoadableUV> mface = msurface->GetFacesList()[ifa];
                if (std::dynamic_pointer_cast<ChFaceTetra_4>(mface)) {
                    // FACE ELEMENT IS A TETRAHEDRON FACE
                    n_verts += 3;
                    n_vcols += 3;
                    n_vnorms += 1;     // flat face
                    n_triangles += 1;  // n. triangle faces
                } else if (std::dynamic_pointer_cast<ChElementTetra_4_P>(mface)) {
                    // FACE ELEMENT IS A SHELL
                    n_verts += shell_resolution * shell_resolution;
                    n_vcols += shell_resolution * shell_resolution;
                    n_vnorms += shell_resolution * shell_resolution;
                    n_triangles += 2 * (shell_resolution - 1) * (shell_resolution - 1);  // n. triangle faces
                }
            }
        }
    }

    //   In case of contact surfaces:
    //
    if (this->fem_data_type == E_PLOT_CONTACTSURFACES) {
        for (unsigned int isu = 0; isu < this->FEMmesh->GetNcontactSurfaces(); ++isu) {
            if (auto msurface =
                    std::dynamic_pointer_cast<ChContactSurfaceMesh>(this->FEMmesh->GetContactSurface(isu))) {
                n_verts += 3 * msurface->GetTriangleList().size();
                n_vcols += 3 * msurface->GetTriangleList().size();
                n_vnorms += msurface->GetTriangleList().size();     // flat faces
                n_triangles += msurface->GetTriangleList().size();  // n. triangle faces
            }
        }
    }

    //
    // B - resize mesh buffers if needed
    //

    if (trianglemesh->getCoordsVertices().size() != n_verts)
        trianglemesh->getCoordsVertices().resize(n_verts);
    if (trianglemesh->getCoordsColors().size() != n_vcols)
        trianglemesh->getCoordsColors().resize(n_vcols);
    if (trianglemesh->getIndicesVertexes().size() != n_triangles)
        trianglemesh->getIndicesVertexes().resize(n_triangles);

    if (this->smooth_faces) {
        if (trianglemesh->getCoordsNormals().size() != n_vnorms)
            trianglemesh->getCoordsNormals().resize(n_vnorms);
        if (trianglemesh->getIndicesNormals().size() != n_triangles)
            trianglemesh->getIndicesNormals().resize(n_triangles);
        if (normal_accumulators.size() != n_vnorms)
            normal_accumulators.resize(n_vnorms);

        TriangleNormalsReset(trianglemesh->getCoordsNormals(), normal_accumulators);
    }

    //
    // C - update mesh buffers
    //

    bool need_automatic_smoothing = this->smooth_faces;

    unsigned int i_verts = 0;
    unsigned int i_vcols = 0;
    unsigned int i_vnorms = 0;
    unsigned int i_triindex = 0;
    unsigned int i_normindex = 0;

    //   In case of colormap drawing:
    if (this->fem_data_type != E_PLOT_NONE && this->fem_data_type != E_PLOT_LOADSURFACES &&
        this->fem_data_type != E_PLOT_CONTACTSURFACES) {
        for (unsigned int iel = 0; iel < this->FEMmesh->GetNelements(); ++iel) {
            // ------------ELEMENT IS A TETRAHEDRON 4 NODES?

            if (auto mytetra = std::dynamic_pointer_cast<ChElementTetra_4>(this->FEMmesh->GetElement(iel))) {
                auto node0 = std::dynamic_pointer_cast<ChNodeFEAxyz>(mytetra->GetNodeN(0));
                auto node1 = std::dynamic_pointer_cast<ChNodeFEAxyz>(mytetra->GetNodeN(1));
                auto node2 = std::dynamic_pointer_cast<ChNodeFEAxyz>(mytetra->GetNodeN(2));
                auto node3 = std::dynamic_pointer_cast<ChNodeFEAxyz>(mytetra->GetNodeN(3));

                unsigned int ivert_el = i_verts;
                unsigned int inorm_el = i_vnorms;

                // vertexes
                ChVector<> p0 = node0->GetPos();
                ChVector<> p1 = node1->GetPos();
                ChVector<> p2 = node2->GetPos();
                ChVector<> p3 = node3->GetPos();
                if (undeformed_reference) {
                    p0 = node0->GetX0();
                    p1 = node1->GetX0();
                    p2 = node2->GetX0();
                    p3 = node3->GetX0();
                }

                if (this->shrink_elements) {
                    ChVector<> vc = (p0 + p1 + p2 + p3) * (0.25);
                    p0 = vc + this->shrink_factor * (p0 - vc);
                    p1 = vc + this->shrink_factor * (p1 - vc);
                    p2 = vc + this->shrink_factor * (p2 - vc);
                    p3 = vc + this->shrink_factor * (p3 - vc);
                }
                trianglemesh->getCoordsVertices()[i_verts] = p0;
                ++i_verts;
                trianglemesh->getCoordsVertices()[i_verts] = p1;
                ++i_verts;
                trianglemesh->getCoordsVertices()[i_verts] = p2;
                ++i_verts;
                trianglemesh->getCoordsVertices()[i_verts] = p3;
                ++i_verts;

                // color
                trianglemesh->getCoordsColors()[i_vcols] =
                    ComputeFalseColor(ComputeScalarOutput(node0, 0, this->FEMmesh->GetElement(iel)));
                ++i_vcols;
                trianglemesh->getCoordsColors()[i_vcols] =
                    ComputeFalseColor(ComputeScalarOutput(node1, 1, this->FEMmesh->GetElement(iel)));
                ++i_vcols;
                trianglemesh->getCoordsColors()[i_vcols] =
                    ComputeFalseColor(ComputeScalarOutput(node2, 2, this->FEMmesh->GetElement(iel)));
                ++i_vcols;
                trianglemesh->getCoordsColors()[i_vcols] =
                    ComputeFalseColor(ComputeScalarOutput(node3, 3, this->FEMmesh->GetElement(iel)));
                ++i_vcols;

                // faces indexes
                ChVector<int> ivert_offset(ivert_el, ivert_el, ivert_el);
                trianglemesh->getIndicesVertexes()[i_triindex] = ChVector<int>(0, 1, 2) + ivert_offset;
                ++i_triindex;
                trianglemesh->getIndicesVertexes()[i_triindex] = ChVector<int>(1, 3, 2) + ivert_offset;
                ++i_triindex;
                trianglemesh->getIndicesVertexes()[i_triindex] = ChVector<int>(2, 3, 0) + ivert_offset;
                ++i_triindex;
                trianglemesh->getIndicesVertexes()[i_triindex] = ChVector<int>(3, 1, 0) + ivert_offset;
                ++i_triindex;

                // normals indices (if not defaulting to flat triangles)
                if (this->smooth_faces) {
                    ChVector<int> inorm_offset = ChVector<int>(inorm_el, inorm_el, inorm_el);
                    trianglemesh->getIndicesNormals()[i_triindex - 4] = ChVector<int>(0, 0, 0) + inorm_offset;
                    trianglemesh->getIndicesNormals()[i_triindex - 3] = ChVector<int>(1, 1, 1) + inorm_offset;
                    trianglemesh->getIndicesNormals()[i_triindex - 2] = ChVector<int>(2, 2, 2) + inorm_offset;
                    trianglemesh->getIndicesNormals()[i_triindex - 1] = ChVector<int>(3, 3, 3) + inorm_offset;
                    i_vnorms += 4;
                }
            }

            // ------------ELEMENT IS A TETRAHEDRON 4 NODES -for SCALAR field- ?

            if (auto mytetra = std::dynamic_pointer_cast<ChElementTetra_4_P>(this->FEMmesh->GetElement(iel))) {
                auto node0 = std::dynamic_pointer_cast<ChNodeFEAxyzP>(mytetra->GetNodeN(0));
                auto node1 = std::dynamic_pointer_cast<ChNodeFEAxyzP>(mytetra->GetNodeN(1));
                auto node2 = std::dynamic_pointer_cast<ChNodeFEAxyzP>(mytetra->GetNodeN(2));
                auto node3 = std::dynamic_pointer_cast<ChNodeFEAxyzP>(mytetra->GetNodeN(3));

                unsigned int ivert_el = i_verts;
                unsigned int inorm_el = i_vnorms;

                // vertexes
                ChVector<> p0 = node0->GetPos();
                ChVector<> p1 = node1->GetPos();
                ChVector<> p2 = node2->GetPos();
                ChVector<> p3 = node3->GetPos();

                if (this->shrink_elements) {
                    ChVector<> vc = (p0 + p1 + p2 + p3) * (0.25);
                    p0 = vc + this->shrink_factor * (p0 - vc);
                    p1 = vc + this->shrink_factor * (p1 - vc);
                    p2 = vc + this->shrink_factor * (p2 - vc);
                    p3 = vc + this->shrink_factor * (p3 - vc);
                }
                trianglemesh->getCoordsVertices()[i_verts] = p0;
                ++i_verts;
                trianglemesh->getCoordsVertices()[i_verts] = p1;
                ++i_verts;
                trianglemesh->getCoordsVertices()[i_verts] = p2;
                ++i_verts;
                trianglemesh->getCoordsVertices()[i_verts] = p3;
                ++i_verts;

                // color
                trianglemesh->getCoordsColors()[i_vcols] =
                    ComputeFalseColor(ComputeScalarOutput(node0, 0, this->FEMmesh->GetElement(iel)));
                ++i_vcols;
                trianglemesh->getCoordsColors()[i_vcols] =
                    ComputeFalseColor(ComputeScalarOutput(node1, 1, this->FEMmesh->GetElement(iel)));
                ++i_vcols;
                trianglemesh->getCoordsColors()[i_vcols] =
                    ComputeFalseColor(ComputeScalarOutput(node2, 2, this->FEMmesh->GetElement(iel)));
                ++i_vcols;
                trianglemesh->getCoordsColors()[i_vcols] =
                    ComputeFalseColor(ComputeScalarOutput(node3, 3, this->FEMmesh->GetElement(iel)));
                ++i_vcols;

                // faces indexes
                ChVector<int> ivert_offset(ivert_el, ivert_el, ivert_el);
                trianglemesh->getIndicesVertexes()[i_triindex] = ChVector<int>(0, 1, 2) + ivert_offset;
                ++i_triindex;
                trianglemesh->getIndicesVertexes()[i_triindex] = ChVector<int>(1, 3, 2) + ivert_offset;
                ++i_triindex;
                trianglemesh->getIndicesVertexes()[i_triindex] = ChVector<int>(2, 3, 0) + ivert_offset;
                ++i_triindex;
                trianglemesh->getIndicesVertexes()[i_triindex] = ChVector<int>(3, 1, 0) + ivert_offset;
                ++i_triindex;

                // normals indices (if not defaulting to flat triangles)
                if (this->smooth_faces) {
                    ChVector<int> inorm_offset = ChVector<int>(inorm_el, inorm_el, inorm_el);
                    trianglemesh->getIndicesNormals()[i_triindex - 4] = ChVector<int>(0, 0, 0) + inorm_offset;
                    trianglemesh->getIndicesNormals()[i_triindex - 3] = ChVector<int>(1, 1, 1) + inorm_offset;
                    trianglemesh->getIndicesNormals()[i_triindex - 2] = ChVector<int>(2, 2, 2) + inorm_offset;
                    trianglemesh->getIndicesNormals()[i_triindex - 1] = ChVector<int>(3, 3, 3) + inorm_offset;
                    i_vnorms += 4;
                }
            }

            // ------------ELEMENT IS A HEXAHEDRON 8 NODES?
            if (std::dynamic_pointer_cast<ChElementHexa_8>(FEMmesh->GetElement(iel)) ||
                std::dynamic_pointer_cast<ChElementBrick>(FEMmesh->GetElement(iel)) ||
                std::dynamic_pointer_cast<ChElementBrick_9>(FEMmesh->GetElement(iel))) {
                UpdateBuffers_Hex(FEMmesh->GetElement(iel), *trianglemesh, i_verts, i_vnorms, i_vcols, i_triindex);
            }

            // ------------ELEMENT IS A BEAM?
            if (auto mybeam = std::dynamic_pointer_cast<ChElementBeam>(this->FEMmesh->GetElement(iel))) {
                double y_thick = 0.01;  // line thickness default value
                double z_thick = 0.01;
                bool m_circular = false;
                double m_rad = 0;

                if (auto mybeameuler = std::dynamic_pointer_cast<ChElementBeamEuler>(mybeam)) {
                    // if the beam has a section info, use section specific thickness for drawing
                    y_thick = 0.5 * mybeameuler->GetSection()->GetDrawThicknessY();
                    z_thick = 0.5 * mybeameuler->GetSection()->GetDrawThicknessZ();
                    m_circular = mybeameuler->GetSection()->IsCircular();
                    m_rad = mybeameuler->GetSection()->GetDrawCircularRadius();
                } else if (auto mybeamancf = std::dynamic_pointer_cast<ChElementCableANCF>(mybeam)) {
                    // if the beam has a section info, use section specific thickness for drawing
                    y_thick = 0.5 * mybeamancf->GetSection()->GetDrawThicknessY();
                    z_thick = 0.5 * mybeamancf->GetSection()->GetDrawThicknessZ();
                    m_circular = mybeamancf->GetSection()->IsCircular();
                    m_rad = mybeamancf->GetSection()->GetDrawCircularRadius();
                } else if (auto mybeamiga = std::dynamic_pointer_cast<ChElementBeamIGA>(mybeam)) {
                    // if the beam has a section info, use section specific thickness for drawing
                    y_thick = 0.5 * mybeamiga->GetSection()->GetDrawThicknessY();
                    z_thick = 0.5 * mybeamiga->GetSection()->GetDrawThicknessZ();
                    m_circular = mybeamiga->GetSection()->IsCircular();
                    m_rad = mybeamiga->GetSection()->GetDrawCircularRadius();
                }

                unsigned int ivert_el = i_verts;
                unsigned int inorm_el = i_vnorms;


                for (int in = 0; in < beam_resolution; ++in) {
                    double eta = -1.0 + (2.0 * in / (beam_resolution - 1));

                    ChVector<> P;
                    ChQuaternion<> msectionrot;
                    mybeam->EvaluateSectionFrame(eta, P,
                                                 msectionrot);  // compute abs. pos and rot of section plane

                    ChVector<> vresult;
                    ChVector<> vresultB;
                    double sresult = 0;
                    switch (this->fem_data_type) {
                        case E_PLOT_ELEM_BEAM_MX:
                            mybeam->EvaluateSectionForceTorque(eta, vresult, vresultB);
                            sresult = vresultB.x();
                            break;
                        case E_PLOT_ELEM_BEAM_MY:
                            mybeam->EvaluateSectionForceTorque(eta, vresult, vresultB);
                            sresult = vresultB.y();
                            break;
                        case E_PLOT_ELEM_BEAM_MZ:
                            mybeam->EvaluateSectionForceTorque(eta, vresult, vresultB);
                            sresult = vresultB.z();
                            break;
                        case E_PLOT_ELEM_BEAM_TX:
                            mybeam->EvaluateSectionForceTorque(eta, vresult, vresultB);
                            sresult = vresult.x();
                            break;
                        case E_PLOT_ELEM_BEAM_TY:
                            mybeam->EvaluateSectionForceTorque(eta, vresult, vresultB);
                            sresult = vresult.y();
                            break;
                        case E_PLOT_ELEM_BEAM_TZ:
                            mybeam->EvaluateSectionForceTorque(eta, vresult, vresultB);
                            sresult = vresult.z();
                            break;
                        case E_PLOT_ANCF_BEAM_AX:
                            mybeam->EvaluateSectionStrain(eta, vresult);
                            sresult = vresult.x();
                            break;
                        case E_PLOT_ANCF_BEAM_BD:
                            mybeam->EvaluateSectionStrain(eta, vresult);
                            sresult = vresult.y();
                            break;
                        default:
                            break;
                    }
                    ChVector<float> mcol = ComputeFalseColor(sresult);

                    if (m_circular) {
                        // prepare a circular section
                        std::vector<ChVector<>> msection_pts(beam_resolution_section);
                        for (size_t is = 0; is < msection_pts.size(); ++is) {
                            double sangle = CH_C_2PI * ((double)is / (double)msection_pts.size());
                            msection_pts[is] = ChVector<>(0, cos(sangle) * m_rad, sin(sangle) * m_rad);
                        }

                        for (int is = 0; is < msection_pts.size(); ++is) {
                            ChVector<> Rw = msectionrot.Rotate(msection_pts[is]);
                            trianglemesh->getCoordsVertices()[i_verts] = P + Rw;
                            ++i_verts;
                            trianglemesh->getCoordsColors()[i_vcols] = mcol;
                            ++i_vcols;
							if (this->smooth_faces) {
								trianglemesh->getCoordsNormals()[i_vnorms] = Rw.GetNormalized();
								++i_vnorms;
							}
                        }
                        // no need to compute normals later with TriangleNormalsCompute
                        need_automatic_smoothing = false;

                        if (in > 0) {
                            ChVector<int> ivert_offset(ivert_el, ivert_el, ivert_el);
                            ChVector<int> islice_offset((in - 1) * (int)msection_pts.size(), (in - 1) * (int)msection_pts.size(),
                                                        (in - 1) * (int)msection_pts.size());
                            for (size_t is = 0; is < msection_pts.size(); ++is) {
                                int ipa = (int)is;
                                int ipb = int((is + 1) % msection_pts.size());
                                int ipaa = ipa + (int)msection_pts.size();
                                int ipbb = ipb + (int)msection_pts.size();

                                trianglemesh->getIndicesVertexes()[i_triindex] =
                                    ChVector<int>(ipa, ipbb, ipaa) + islice_offset + ivert_offset;
								if (this->smooth_faces) {
									trianglemesh->getIndicesNormals()[i_triindex] =
										ChVector<int>(ipa, ipbb, ipaa) + islice_offset + ivert_offset;
								}
                                ++i_triindex;

                                trianglemesh->getIndicesVertexes()[i_triindex] =
                                    ChVector<int>(ipa, ipb, ipbb) + islice_offset + ivert_offset;
								if (this->smooth_faces) {
									trianglemesh->getIndicesNormals()[i_triindex] =
										ChVector<int>(ipa, ipb, ipbb) + islice_offset + ivert_offset;
								}
                                ++i_triindex;
                            }
                        }
                    }
                    // if rectangle shape...
                    else {
                        trianglemesh->getCoordsVertices()[i_verts] =
                            P + msectionrot.Rotate(ChVector<>(0, -y_thick, -z_thick));
                        ++i_verts;
                        trianglemesh->getCoordsVertices()[i_verts] =
                            P + msectionrot.Rotate(ChVector<>(0, y_thick, -z_thick));
                        ++i_verts;
                        trianglemesh->getCoordsVertices()[i_verts] =
                            P + msectionrot.Rotate(ChVector<>(0, y_thick, z_thick));
                        ++i_verts;
                        trianglemesh->getCoordsVertices()[i_verts] =
                            P + msectionrot.Rotate(ChVector<>(0, -y_thick, z_thick));
                        ++i_verts;

                        trianglemesh->getCoordsColors()[i_vcols] = mcol;
                        ++i_vcols;
                        trianglemesh->getCoordsColors()[i_vcols] = mcol;
                        ++i_vcols;
                        trianglemesh->getCoordsColors()[i_vcols] = mcol;
                        ++i_vcols;
                        trianglemesh->getCoordsColors()[i_vcols] = mcol;
                        ++i_vcols;

                        if (in > 0) {
                            ChVector<int> ivert_offset(ivert_el, ivert_el, ivert_el);
                            ChVector<int> islice_offset((in - 1) * 4, (in - 1) * 4, (in - 1) * 4);
                            trianglemesh->getIndicesVertexes()[i_triindex] =
                                ChVector<int>(4, 0, 1) + islice_offset + ivert_offset;
                            ++i_triindex;
                            trianglemesh->getIndicesVertexes()[i_triindex] =
                                ChVector<int>(4, 1, 5) + islice_offset + ivert_offset;
                            ++i_triindex;
                            trianglemesh->getIndicesVertexes()[i_triindex] =
                                ChVector<int>(5, 1, 2) + islice_offset + ivert_offset;
                            ++i_triindex;
                            trianglemesh->getIndicesVertexes()[i_triindex] =
                                ChVector<int>(5, 2, 6) + islice_offset + ivert_offset;
                            ++i_triindex;
                            trianglemesh->getIndicesVertexes()[i_triindex] =
                                ChVector<int>(6, 2, 3) + islice_offset + ivert_offset;
                            ++i_triindex;
                            trianglemesh->getIndicesVertexes()[i_triindex] =
                                ChVector<int>(6, 3, 7) + islice_offset + ivert_offset;
                            ++i_triindex;
                            trianglemesh->getIndicesVertexes()[i_triindex] =
                                ChVector<int>(7, 3, 0) + islice_offset + ivert_offset;
                            ++i_triindex;
                            trianglemesh->getIndicesVertexes()[i_triindex] =
                                ChVector<int>(7, 0, 4) + islice_offset + ivert_offset;
                            ++i_triindex;

                            if (this->smooth_faces) {
                                ChVector<int> islice_normoffset((in - 1) * 8, (in - 1) * 8,
                                                                (in - 1) * 8);  //***TO DO*** fix errors in normals
                                ChVector<int> inorm_offset = ChVector<int>(inorm_el, inorm_el, inorm_el);
                                trianglemesh->getIndicesNormals()[i_triindex - 8] =
                                    ChVector<int>(8, 0, 1) + islice_normoffset + inorm_offset;
                                trianglemesh->getIndicesNormals()[i_triindex - 7] =
                                    ChVector<int>(8, 1, 9) + islice_normoffset + inorm_offset;
                                trianglemesh->getIndicesNormals()[i_triindex - 6] =
                                    ChVector<int>(9 + 4, 1 + 4, 2 + 4) + islice_normoffset + inorm_offset;
                                trianglemesh->getIndicesNormals()[i_triindex - 5] =
                                    ChVector<int>(9 + 4, 2 + 4, 10 + 4) + islice_normoffset + inorm_offset;
                                trianglemesh->getIndicesNormals()[i_triindex - 4] =
                                    ChVector<int>(10, 2, 3) + islice_normoffset + inorm_offset;
                                trianglemesh->getIndicesNormals()[i_triindex - 3] =
                                    ChVector<int>(10, 3, 11) + islice_normoffset + inorm_offset;
                                trianglemesh->getIndicesNormals()[i_triindex - 2] =
                                    ChVector<int>(11 + 4, 3 + 4, 0 + 4) + islice_normoffset + inorm_offset;
                                trianglemesh->getIndicesNormals()[i_triindex - 1] =
                                    ChVector<int>(11 + 4, 0 + 4, 8 + 4) + islice_normoffset + inorm_offset;
                                i_vnorms += 8;
                            }
                        }

                    }  // end if rectangle
                }      // end sections loop
            }

            // ------------ELEMENT IS A SHELL?
            if (auto myshell = std::dynamic_pointer_cast<ChElementShell>(this->FEMmesh->GetElement(iel))) {
                unsigned int ivert_el = i_verts;
                unsigned int inorm_el = i_vnorms;

                for (int iu = 0; iu < shell_resolution; ++iu)
                    for (int iv = 0; iv < shell_resolution; ++iv) {
                        double u = -1.0 + (2.0 * iu / (shell_resolution - 1));
                        double v = -1.0 + (2.0 * iv / (shell_resolution - 1));

                        ChVector<> P;
                        myshell->EvaluateSectionPoint(u, v, P);  // compute abs. pos and rot of section plane

                        ChVector<float> mcol(1, 1, 1);
                        /*
                        ChVector<> vresult;
                        ChVector<> vresultB;
                        double sresult = 0;
                        switch(this->fem_data_type)
                        {
                            case E_PLOT_ELEM_SHELL_blabla:
                                myshell->EvaluateSectionForceTorque(eta, vresult, vresultB);
                                sresult = vresultB.x();
                                break;

                        }
                        ChVector<float> mcol = ComputeFalseColor(sresult);
                        */

                        trianglemesh->getCoordsVertices()[i_verts] = P;
                        ++i_verts;

                        trianglemesh->getCoordsColors()[i_vcols] = mcol;
                        ++i_vcols;

                        ++i_vnorms;

                        if (iu > 0 && iv > 0) {
                            ChVector<int> ivert_offset(ivert_el, ivert_el, ivert_el);

                            trianglemesh->getIndicesVertexes()[i_triindex] =
                                ChVector<int>(iu * shell_resolution + iv, (iu - 1) * shell_resolution + iv,
                                              iu * shell_resolution + iv - 1) +
                                ivert_offset;
                            ++i_triindex;
                            trianglemesh->getIndicesVertexes()[i_triindex] =
                                ChVector<int>(iu * shell_resolution + iv - 1, (iu - 1) * shell_resolution + iv,
                                              (iu - 1) * shell_resolution + iv - 1) +
                                ivert_offset;
                            ++i_triindex;

                            if (this->smooth_faces) {
                                ChVector<int> inorm_offset = ChVector<int>(inorm_el, inorm_el, inorm_el);
                                trianglemesh->getIndicesNormals()[i_triindex - 2] =
                                    ChVector<int>(iu * shell_resolution + iv, (iu - 1) * shell_resolution + iv,
                                                  iu * shell_resolution + iv - 1) +
                                    inorm_offset;
                                trianglemesh->getIndicesNormals()[i_triindex - 1] =
                                    ChVector<int>(iu * shell_resolution + iv - 1, (iu - 1) * shell_resolution + iv,
                                                  (iu - 1) * shell_resolution + iv - 1) +
                                    inorm_offset;
                            }
                        }
                    }
            }

            // ------------***TO DO*** other types of elements...

        }  // End of loop on elements
    }      //  End of case of colormap drawing:

    //   In case mesh surfaces for pressure loads etc.:
    //
    if (this->fem_data_type == E_PLOT_LOADSURFACES) {
        for (unsigned int isu = 0; isu < this->FEMmesh->GetNmeshSurfaces(); ++isu) {
            std::shared_ptr<ChMeshSurface> msurface = this->FEMmesh->GetMeshSurface(isu);
            for (unsigned int ifa = 0; ifa < msurface->GetFacesList().size(); ++ifa) {
                std::shared_ptr<ChLoadableUV> mface = msurface->GetFacesList()[ifa];
                // FACE ELEMENT IS A TETRAHEDRON FACE
                if (auto mfacetetra = std::dynamic_pointer_cast<ChFaceTetra_4>(mface)) {
                    auto node0 = std::static_pointer_cast<ChNodeFEAxyz>(mfacetetra->GetNodeN(0));
                    auto node1 = std::static_pointer_cast<ChNodeFEAxyz>(mfacetetra->GetNodeN(1));
                    auto node2 = std::static_pointer_cast<ChNodeFEAxyz>(mfacetetra->GetNodeN(2));

                    unsigned int ivert_el = i_verts;
                    unsigned int inorm_el = i_vnorms;

                    // vertexes
                    ChVector<> p0 = node0->GetPos();
                    ChVector<> p1 = node1->GetPos();
                    ChVector<> p2 = node2->GetPos();

                    // debug: offset 1 m to show it better..
                    //    p0.x() +=1;
                    //    p1.x() +=1;
                    //    p2.x() +=1;

                    trianglemesh->getCoordsVertices()[i_verts] = p0;
                    ++i_verts;
                    trianglemesh->getCoordsVertices()[i_verts] = p1;
                    ++i_verts;
                    trianglemesh->getCoordsVertices()[i_verts] = p2;
                    ++i_verts;

                    // color
                    trianglemesh->getCoordsColors()[i_vcols] = ChVector<float>(meshcolor.R, meshcolor.G, meshcolor.B);
                    ++i_vcols;
                    trianglemesh->getCoordsColors()[i_vcols] = ChVector<float>(meshcolor.R, meshcolor.G, meshcolor.B);
                    ++i_vcols;
                    trianglemesh->getCoordsColors()[i_vcols] = ChVector<float>(meshcolor.R, meshcolor.G, meshcolor.B);
                    ++i_vcols;

                    // faces indexes
                    ChVector<int> ivert_offset(ivert_el, ivert_el, ivert_el);
                    trianglemesh->getIndicesVertexes()[i_triindex] = ChVector<int>(0, 1, 2) + ivert_offset;
                    ++i_triindex;

                    // normals indices (if not defaulting to flat triangles)
                    if (this->smooth_faces) {
                        ChVector<int> inorm_offset = ChVector<int>(inorm_el, inorm_el, inorm_el);
                        trianglemesh->getIndicesNormals()[i_triindex - 4] = ChVector<int>(0, 0, 0) + inorm_offset;
                        i_vnorms += 1;
                    }
                }

                // FACE ELEMENT IS A SHELL
                if (auto mfacetetra = std::dynamic_pointer_cast<ChElementTetra_4_P>(mface)) {
                    //***TODO***
                }
            }
        }  // end loop on load surfaces
    }      // End of case of load surfaces

    //   In case of contact surfaces:
    //
    if (this->fem_data_type == E_PLOT_CONTACTSURFACES) {
        for (unsigned int isu = 0; isu < this->FEMmesh->GetNcontactSurfaces(); ++isu) {
            if (auto msurface =
                    std::dynamic_pointer_cast<ChContactSurfaceMesh>(this->FEMmesh->GetContactSurface(isu))) {
                for (unsigned int ifa = 0; ifa < msurface->GetTriangleList().size(); ++ifa) {
                    std::shared_ptr<ChContactTriangleXYZ> mface = msurface->GetTriangleList()[ifa];

                    unsigned int ivert_el = i_verts;
                    unsigned int inorm_el = i_vnorms;

                    // vertexes
                    ChVector<> p0 = mface->GetNode1()->pos;
                    ChVector<> p1 = mface->GetNode2()->pos;
                    ChVector<> p2 = mface->GetNode3()->pos;

                    trianglemesh->getCoordsVertices()[i_verts] = p0;
                    ++i_verts;
                    trianglemesh->getCoordsVertices()[i_verts] = p1;
                    ++i_verts;
                    trianglemesh->getCoordsVertices()[i_verts] = p2;
                    ++i_verts;

                    // color
                    trianglemesh->getCoordsColors()[i_vcols] = ChVector<float>(meshcolor.R, meshcolor.G, meshcolor.B);
                    ++i_vcols;
                    trianglemesh->getCoordsColors()[i_vcols] = ChVector<float>(meshcolor.R, meshcolor.G, meshcolor.B);
                    ++i_vcols;
                    trianglemesh->getCoordsColors()[i_vcols] = ChVector<float>(meshcolor.R, meshcolor.G, meshcolor.B);
                    ++i_vcols;

                    // faces indexes
                    ChVector<int> ivert_offset(ivert_el, ivert_el, ivert_el);
                    trianglemesh->getIndicesVertexes()[i_triindex] = ChVector<int>(0, 1, 2) + ivert_offset;
                    ++i_triindex;

                    // normals indices (if not defaulting to flat triangles)
                    if (this->smooth_faces) {
                        ChVector<int> inorm_offset = ChVector<int>(inorm_el, inorm_el, inorm_el);
                        trianglemesh->getIndicesNormals()[i_triindex - 4] = ChVector<int>(0, 0, 0) + inorm_offset;
                        i_vnorms += 1;
                    }
                }
            }
        }  // end loop on contact surfaces
    }      // End of case of contact surfaces

    if (need_automatic_smoothing) {
        for (unsigned int itri = 0; itri < trianglemesh->getIndicesVertexes().size(); ++itri)
            TriangleNormalsCompute(trianglemesh->getIndicesNormals()[itri], trianglemesh->getIndicesVertexes()[itri],
                                   trianglemesh->getCoordsVertices(), trianglemesh->getCoordsNormals(),
                                   normal_accumulators);

        TriangleNormalsSmooth(trianglemesh->getCoordsNormals(), normal_accumulators);
    }

    // other flags
    mesh_asset->SetWireframe(this->wireframe);

    //
    // GLYPHS
    //

    //***TEST***
    glyphs_asset->Reserve(0);  // unoptimal, should reuse buffers as much as possible

    glyphs_asset->SetGlyphsSize(this->symbols_thickness);

    glyphs_asset->SetZbufferHide(this->zbuffer_hide);

    if (this->fem_glyph == ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS) {
        glyphs_asset->SetDrawMode(ChGlyphs::GLYPH_POINT);
        for (unsigned int inode = 0; inode < this->FEMmesh->GetNnodes(); ++inode) {
            if (auto mynode = std::dynamic_pointer_cast<ChNodeFEAxyz>(this->FEMmesh->GetNode(inode))) {
                glyphs_asset->SetGlyphPoint(inode, mynode->GetPos(), this->symbolscolor);
            } else if (auto mynode = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(this->FEMmesh->GetNode(inode))) {
                glyphs_asset->SetGlyphPoint(inode, mynode->GetPos(), this->symbolscolor);
            } else if (auto mynode = std::dynamic_pointer_cast<ChNodeFEAxyzD>(this->FEMmesh->GetNode(inode))) {
                glyphs_asset->SetGlyphPoint(inode, mynode->GetPos(), this->symbolscolor);
            }
        }
    }
    if (this->fem_glyph == ChVisualizationFEAmesh::E_GLYPH_NODE_CSYS) {
        glyphs_asset->SetDrawMode(ChGlyphs::GLYPH_COORDSYS);
        for (unsigned int inode = 0; inode < this->FEMmesh->GetNnodes(); ++inode) {
            if (auto mynode = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(this->FEMmesh->GetNode(inode))) {
                glyphs_asset->SetGlyphCoordsys(inode, mynode->Frame().GetCoord());
            }
            // else if (auto mynode = std::dynamic_pointer_cast<ChNodeFEAxyzD>(this->FEMmesh->GetNode(inode))) {
            //	glyphs_asset->SetGlyphVector(inode, mynode->GetPos(), mynode->GetD() * this->symbols_scale,
            //this->symbolscolor );
            //}
        }
    }
    if (this->fem_glyph == ChVisualizationFEAmesh::E_GLYPH_NODE_VECT_SPEED) {
        glyphs_asset->SetDrawMode(ChGlyphs::GLYPH_VECTOR);
        for (unsigned int inode = 0; inode < this->FEMmesh->GetNnodes(); ++inode)
            if (auto mynode = std::dynamic_pointer_cast<ChNodeFEAxyz>(this->FEMmesh->GetNode(inode))) {
                glyphs_asset->SetGlyphVector(inode, mynode->GetPos(), mynode->GetPos_dt() * this->symbols_scale,
                                             this->symbolscolor);
            }
    }
    if (this->fem_glyph == ChVisualizationFEAmesh::E_GLYPH_NODE_VECT_ACCEL) {
        glyphs_asset->SetDrawMode(ChGlyphs::GLYPH_VECTOR);
        for (unsigned int inode = 0; inode < this->FEMmesh->GetNnodes(); ++inode)
            if (auto mynode = std::dynamic_pointer_cast<ChNodeFEAxyz>(this->FEMmesh->GetNode(inode))) {
                glyphs_asset->SetGlyphVector(inode, mynode->GetPos(), mynode->GetPos_dtdt() * this->symbols_scale,
                                             this->symbolscolor);
            }
    }
    if (this->fem_glyph == ChVisualizationFEAmesh::E_GLYPH_ELEM_VECT_DP) {
        glyphs_asset->SetDrawMode(ChGlyphs::GLYPH_VECTOR);
        for (unsigned int iel = 0; iel < this->FEMmesh->GetNelements(); ++iel)
            if (auto myelement = std::dynamic_pointer_cast<ChElementTetra_4_P>(this->FEMmesh->GetElement(iel))) {
                ChMatrixNM<double, 3, 1> mP = myelement->GetPgradient();
                ChVector<> mvP(mP(0), mP(1), mP(2));
                auto n0 = std::static_pointer_cast<ChNodeFEAxyzP>(myelement->GetNodeN(0));
                auto n1 = std::static_pointer_cast<ChNodeFEAxyzP>(myelement->GetNodeN(1));
                auto n2 = std::static_pointer_cast<ChNodeFEAxyzP>(myelement->GetNodeN(2));
                auto n3 = std::static_pointer_cast<ChNodeFEAxyzP>(myelement->GetNodeN(3));
                ChVector<> mPoint = (n0->GetPos() + n1->GetPos() + n2->GetPos() + n3->GetPos()) *
                                    0.25;  // to do: better placement in Gauss point
                glyphs_asset->SetGlyphVector(iel, mPoint, mvP * this->symbols_scale, this->symbolscolor);
            }
    }
    if (this->fem_glyph == ChVisualizationFEAmesh::E_GLYPH_ELEM_TENS_STRAIN) {
        glyphs_asset->SetDrawMode(ChGlyphs::GLYPH_VECTOR);
        int nglyvect = 0;
        for (unsigned int iel = 0; iel < this->FEMmesh->GetNelements(); ++iel)
            if (auto myelement = std::dynamic_pointer_cast<ChElementTetra_4>(this->FEMmesh->GetElement(iel))) {
                ChStrainTensor<> mstrain = myelement->GetStrain();
                // mstrain.Rotate(myelement->Rotation());
                double e1, e2, e3;
                ChVector<> v1, v2, v3;
                mstrain.ComputePrincipalStrains(e1, e2, e3);
                mstrain.ComputePrincipalStrainsDirections(e1, e2, e3, v1, v2, v3);
                v1.Normalize();
                v2.Normalize();
                v3.Normalize();
                auto n0 = std::static_pointer_cast<ChNodeFEAxyz>(myelement->GetNodeN(0));
                auto n1 = std::static_pointer_cast<ChNodeFEAxyz>(myelement->GetNodeN(1));
                auto n2 = std::static_pointer_cast<ChNodeFEAxyz>(myelement->GetNodeN(2));
                auto n3 = std::static_pointer_cast<ChNodeFEAxyz>(myelement->GetNodeN(3));
                //// TODO: better placement in Gauss point
                ChVector<> mPoint = (n0->GetPos() + n1->GetPos() + n2->GetPos() + n3->GetPos()) * 0.25;
                glyphs_asset->SetGlyphVector(nglyvect, mPoint, myelement->Rotation() * v1 * e1 * this->symbols_scale,
                                             ComputeFalseColor2(e1));
                ++nglyvect;
                glyphs_asset->SetGlyphVector(nglyvect, mPoint, myelement->Rotation() * v2 * e2 * this->symbols_scale,
                                             ComputeFalseColor2(e2));
                ++nglyvect;
                glyphs_asset->SetGlyphVector(nglyvect, mPoint, myelement->Rotation() * v3 * e3 * this->symbols_scale,
                                             ComputeFalseColor2(e3));
                ++nglyvect;
            }
    }
    if (this->fem_glyph == ChVisualizationFEAmesh::E_GLYPH_ELEM_TENS_STRESS) {
        glyphs_asset->SetDrawMode(ChGlyphs::GLYPH_VECTOR);
        int nglyvect = 0;
        for (unsigned int iel = 0; iel < this->FEMmesh->GetNelements(); ++iel)
            if (auto myelement = std::dynamic_pointer_cast<ChElementTetra_4>(this->FEMmesh->GetElement(iel))) {
                ChStressTensor<> mstress = myelement->GetStress();
                mstress.Rotate(myelement->Rotation());
                double e1, e2, e3;
                ChVector<> v1, v2, v3;
                mstress.ComputePrincipalStresses(e1, e2, e3);
                mstress.ComputePrincipalStressesDirections(e1, e2, e3, v1, v2, v3);
                v1.Normalize();
                v2.Normalize();
                v3.Normalize();
                auto n0 = std::static_pointer_cast<ChNodeFEAxyz>(myelement->GetNodeN(0));
                auto n1 = std::static_pointer_cast<ChNodeFEAxyz>(myelement->GetNodeN(1));
                auto n2 = std::static_pointer_cast<ChNodeFEAxyz>(myelement->GetNodeN(2));
                auto n3 = std::static_pointer_cast<ChNodeFEAxyz>(myelement->GetNodeN(3));
                //// TODO: better placement in Gauss point
                ChVector<> mPoint = (n0->GetPos() + n1->GetPos() + n2->GetPos() + n3->GetPos()) * 0.25;
                glyphs_asset->SetGlyphVector(nglyvect, mPoint, myelement->Rotation() * v1 * e1 * this->symbols_scale,
                                             ComputeFalseColor2(e1));
                ++nglyvect;
                glyphs_asset->SetGlyphVector(nglyvect, mPoint, myelement->Rotation() * v2 * e2 * this->symbols_scale,
                                             ComputeFalseColor2(e2));
                ++nglyvect;
                glyphs_asset->SetGlyphVector(nglyvect, mPoint, myelement->Rotation() * v3 * e3 * this->symbols_scale,
                                             ComputeFalseColor2(e3));
                ++nglyvect;
            }
    }

    //***TEST***
    if (false)
        for (unsigned int iel = 0; iel < this->FEMmesh->GetNelements(); ++iel) {
            // ------------ELEMENT IS A ChElementShellReissner4?
            if (auto myshell = std::dynamic_pointer_cast<ChElementShellReissner4>(this->FEMmesh->GetElement(iel))) {
                glyphs_asset->SetGlyphsSize(0.4);
                // average rotation
                if (false) {
                    glyphs_asset->SetDrawMode(ChGlyphs::GLYPH_COORDSYS);
                    glyphs_asset->GetNumberOfGlyphs();
                    glyphs_asset->SetGlyphCoordsys(
                        (unsigned int)glyphs_asset->GetNumberOfGlyphs(),
                        ChCoordsys<>((myshell->GetNodeA()->GetPos() + myshell->GetNodeB()->GetPos() +
                                      myshell->GetNodeC()->GetPos() + myshell->GetNodeD()->GetPos()) *
                                         0.25,
                                     myshell->GetAvgRot()));
                }
                // gauss point coordsys
                if (false) {
                    glyphs_asset->SetDrawMode(ChGlyphs::GLYPH_COORDSYS);
                    for (int igp = 0; igp < 4; ++igp) {
                        glyphs_asset->GetNumberOfGlyphs();
                        glyphs_asset->SetGlyphCoordsys(
                            (unsigned int)glyphs_asset->GetNumberOfGlyphs(),
                            ChCoordsys<>(myshell->EvaluateGP(igp), myshell->T_i[igp].Get_A_quaternion()));
                    }
                }
                // gauss point weights
                if (false) {
                    glyphs_asset->SetDrawMode(ChGlyphs::GLYPH_VECTOR);
                    for (int igp = 0; igp < 4; ++igp) {
                        glyphs_asset->GetNumberOfGlyphs();
                        glyphs_asset->SetGlyphVector((unsigned int)glyphs_asset->GetNumberOfGlyphs(), myshell->EvaluateGP(igp),
                                                     ChVector<>(0, myshell->alpha_i[igp] * 4, 0));
                    }
                }
                // gauss curvatures
                if (false) {
                    for (int igp = 0; igp < 4; ++igp) {
                        glyphs_asset->SetDrawMode(ChGlyphs::GLYPH_VECTOR);
                        /*
                        glyphs_asset->GetNumberOfGlyphs();
                        glyphs_asset->SetGlyphVector(glyphs_asset->GetNumberOfGlyphs(),
                            myshell->EvaluateGP(igp),
                            myshell->T_i[igp].Rotate(myshell->kur_u_tilde[igp]*50), ChColor(1,0,0) );
                        glyphs_asset->GetNumberOfGlyphs();
                        glyphs_asset->SetGlyphVector(glyphs_asset->GetNumberOfGlyphs(),
                            myshell->EvaluateGP(igp),
                            myshell->T_i[igp].Rotate(myshell->kur_v_tilde[igp]*50), ChColor(0,1,0) );
                            */
                        glyphs_asset->GetNumberOfGlyphs();
                        glyphs_asset->SetGlyphVector(
                            (unsigned int)glyphs_asset->GetNumberOfGlyphs(), myshell->EvaluateGP(igp),
                            myshell->T_i[igp] * ((myshell->k_tilde_1_i[igp] + myshell->k_tilde_2_i[igp]) * 50),
                            ChColor(0, 0, 0));
                    }
                }
                // gauss strains
                if (true) {
                    for (int igp = 0; igp < 4; ++igp) {
                        glyphs_asset->SetDrawMode(ChGlyphs::GLYPH_VECTOR);
                        double scale = 1;
                        glyphs_asset->GetNumberOfGlyphs();
                        glyphs_asset->SetGlyphVector((unsigned int)glyphs_asset->GetNumberOfGlyphs(), myshell->EvaluateGP(igp),
                                                     myshell->T_i[igp] * (myshell->eps_tilde_1_i[igp] * scale),
                                                     ChColor(1, 0, 0));
                        glyphs_asset->GetNumberOfGlyphs();
                        glyphs_asset->SetGlyphVector((unsigned int)glyphs_asset->GetNumberOfGlyphs(), myshell->EvaluateGP(igp),
                                                     myshell->T_i[igp] * (myshell->eps_tilde_2_i[igp] * scale),
                                                     ChColor(0, 0, 1));
                        glyphs_asset->GetNumberOfGlyphs();
                    }
                }
                // other...
            }
        }

    // Finally, update also the children, in case they implemented Update(),
    // and do this by calling the parent class implementation of ChAssetLevel
    ChAssetLevel::Update(updater, coords);
}

}  // end namespace fea
}  // end namespace chrono
