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

#include "chrono/assets/ChGlyphs.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/assets/ChVisualShapeFEA.h"

#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChNodeFEAxyz.h"
#include "chrono/fea/ChNodeFEAxyzP.h"
#include "chrono/fea/ChNodeFEAxyzrot.h"
#include "chrono/fea/ChNodeFEAxyzDD.h"
#include "chrono/fea/ChElementCableANCF.h"
#include "chrono/fea/ChElementBeamANCF_3243.h"
#include "chrono/fea/ChElementBeamANCF_3333.h"
#include "chrono/fea/ChElementBeamEuler.h"
#include "chrono/fea/ChElementBeamTaperedTimoshenko.h"
#include "chrono/fea/ChElementBeamTaperedTimoshenkoFPM.h"
#include "chrono/fea/ChElementBeamIGA.h"
#include "chrono/fea/ChElementShell.h"
#include "chrono/fea/ChElementShellReissner4.h"
#include "chrono/fea/ChElementShellBST.h"
#include "chrono/fea/ChElementTetraCorot_4.h"
#include "chrono/fea/ChTetrahedronFace.h"
#include "chrono/fea/ChHexahedronFace.h"
#include "chrono/fea/ChContactSurfaceMesh.h"
#include "chrono/fea/ChContactSurfaceNodeCloud.h"

namespace chrono {

using namespace fea;

ChVisualShapeFEA::ChVisualShapeFEA(std::shared_ptr<fea::ChMesh> fea_mesh) {
    FEMmesh = fea_mesh;
    fem_data_type = DataType::SURFACE;
    fem_glyph = GlyphType::NONE;

    colorscale_min = 0;
    colorscale_max = 1;

    shrink_elements = false;
    shrink_factor = 0.9;

    symbols_scale = 1.0;
    symbols_thickness = 0.002;

    wireframe = false;
    backface_cull = false;

    zbuffer_hide = true;

    smooth_faces = false;

    beam_resolution = 8;
    beam_resolution_section = 10;
    shell_resolution = 3;

    meshcolor = ChColor(1, 1, 1);
    symbolscolor = ChColor(0, 0.5, 0.5);

    undeformed_reference = false;

    m_trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
    m_glyphs_shape = chrono_types::make_shared<ChGlyphs>();
}

ChColor ChVisualShapeFEA::ComputeFalseColor(double mv) {
    ChColor c = ChColor::ComputeFalseColor(mv, colorscale_min, colorscale_max, true);

    if (fem_data_type == DataType::SURFACE)
        c = meshcolor;

    return c;
}

double ChVisualShapeFEA::ComputeScalarOutput(std::shared_ptr<ChNodeFEAxyz> mnode,
                                             int nodeID,
                                             std::shared_ptr<ChElementBase> melement) {
    switch (fem_data_type) {
        case DataType::SURFACE:
            return 1e30;  // to force 'white' in false color scale. Hack, to be improved.
        case DataType::NODE_DISP_NORM:
            return (mnode->GetPos() - mnode->GetX0()).Length();
        case DataType::NODE_DISP_X:
            return (mnode->GetPos() - mnode->GetX0()).x();
        case DataType::NODE_DISP_Y:
            return (mnode->GetPos() - mnode->GetX0()).y();
        case DataType::NODE_DISP_Z:
            return (mnode->GetPos() - mnode->GetX0()).z();
        case DataType::NODE_SPEED_NORM:
            return mnode->GetPos_dt().Length();
        case DataType::NODE_SPEED_X:
            return mnode->GetPos_dt().x();
        case DataType::NODE_SPEED_Y:
            return mnode->GetPos_dt().y();
        case DataType::NODE_SPEED_Z:
            return mnode->GetPos_dt().z();
        case DataType::NODE_ACCEL_NORM:
            return mnode->GetPos_dtdt().Length();
        case DataType::NODE_ACCEL_X:
            return mnode->GetPos_dtdt().x();
        case DataType::NODE_ACCEL_Y:
            return mnode->GetPos_dtdt().y();
        case DataType::NODE_ACCEL_Z:
            return mnode->GetPos_dtdt().z();
        case DataType::ELEM_STRAIN_VONMISES:
            if (auto mytetra = std::dynamic_pointer_cast<ChElementTetraCorot_4>(melement)) {
                return mytetra->GetStrain().GetEquivalentVonMises();
            }
        case DataType::ELEM_STRESS_VONMISES:
            if (auto mytetra = std::dynamic_pointer_cast<ChElementTetraCorot_4>(melement)) {
                return mytetra->GetStress().GetEquivalentVonMises();
            }
        case DataType::ELEM_STRAIN_HYDROSTATIC:
            if (auto mytetra = std::dynamic_pointer_cast<ChElementTetraCorot_4>(melement)) {
                return mytetra->GetStrain().GetEquivalentMeanHydrostatic();
            }
        case DataType::ELEM_STRESS_HYDROSTATIC:
            if (auto mytetra = std::dynamic_pointer_cast<ChElementTetraCorot_4>(melement)) {
                return mytetra->GetStress().GetEquivalentMeanHydrostatic();
            }
        default:
            return 1e30;
    }
    //***TO DO*** other types of scalar outputs
    return 0;
}

double ChVisualShapeFEA::ComputeScalarOutput(std::shared_ptr<ChNodeFEAxyzP> mnode,
                                             int nodeID,
                                             std::shared_ptr<ChElementBase> melement) {
    switch (fem_data_type) {
        case DataType::SURFACE:
            return 1e30;  // to force 'white' in false color scale. Hack, to be improved.
        case DataType::NODE_P:
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
    ChVector<> tnorm = Vcross(vertexes[vert_indexes.y()] - vertexes[vert_indexes.x()],
                              vertexes[vert_indexes.z()] - vertexes[vert_indexes.x()])
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
void ChVisualShapeFEA::UpdateBuffers_Hex(std::shared_ptr<ChElementBase> element,
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

    if (shrink_elements) {
        ChVector<> vc(0, 0, 0);
        for (int in = 0; in < 8; ++in)
            vc += pt[in];
        vc = vc * (1.0 / 8.0);  // average, center of element
        for (int in = 0; in < 8; ++in)
            pt[in] = vc + shrink_factor * (pt[in] - vc);
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
    if (smooth_faces) {
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

void ChVisualShapeFEA::Update(ChPhysicsItem* updater, const ChFrame<>& frame) {
    if (!FEMmesh)
        return;

    auto trianglemesh = m_trimesh_shape->GetMesh();

    size_t n_verts = 0;
    size_t n_vcols = 0;
    size_t n_vnorms = 0;
    size_t n_triangles = 0;

    //
    // A - Count the needed vertexes and faces
    //

    //   In case of colormap drawing:
    //
    if (fem_data_type != DataType::NONE && fem_data_type != DataType::LOADSURFACES &&
        fem_data_type != DataType::CONTACTSURFACES) {
        for (unsigned int iel = 0; iel < FEMmesh->GetNelements(); ++iel) {
            if (std::dynamic_pointer_cast<ChElementTetrahedron>(FEMmesh->GetElement(iel)) ||
                std::dynamic_pointer_cast<ChElementTetraCorot_4_P>(FEMmesh->GetElement(iel))) {
                n_verts += 4;
                n_vcols += 4;
                n_vnorms += 4;     // flat faces
                n_triangles += 4;  // n. triangle faces
            } else if (std::dynamic_pointer_cast<ChElementHexahedron>(FEMmesh->GetElement(iel))) {
                n_verts += 8;
                n_vcols += 8;
                n_vnorms += 24;
                n_triangles += 12;  // n. triangle faces
            } else if (auto beam = std::dynamic_pointer_cast<ChElementBeam>(FEMmesh->GetElement(iel))) {
                // ELEMENT HAS A ChBeamSectionShape
                std::shared_ptr<ChBeamSectionShape> sectionshape;
                if (auto beamEuler = std::dynamic_pointer_cast<ChElementBeamEuler>(beam)) {
                    sectionshape = beamEuler->GetSection()->GetDrawShape();
                } else if (auto cableANCF = std::dynamic_pointer_cast<ChElementCableANCF>(beam)) {
                    sectionshape = cableANCF->GetSection()->GetDrawShape();
                } else if (auto beamIGA = std::dynamic_pointer_cast<ChElementBeamIGA>(beam)) {
                    sectionshape = beamIGA->GetSection()->GetDrawShape();
                } else if (auto beamTimoshenko = std::dynamic_pointer_cast<ChElementBeamTaperedTimoshenko>(beam)) {
                    sectionshape = beamTimoshenko->GetTaperedSection()->GetSectionA()->GetDrawShape();
                } else if (auto beamTimoshenkoFPM =
                               std::dynamic_pointer_cast<ChElementBeamTaperedTimoshenkoFPM>(beam)) {
                    sectionshape = beamTimoshenkoFPM->GetTaperedSection()->GetSectionA()->GetDrawShape();
                } else if (auto beam3243 = std::dynamic_pointer_cast<ChElementBeamANCF_3243>(beam)) {
                    sectionshape = chrono_types::make_shared<ChBeamSectionShapeRectangular>(
                        beam3243->GetThicknessY(),
                        beam3243->GetThicknessZ());  // TO DO use ChBeamSection also in ANCF beam
                } else if (auto beam3333 = std::dynamic_pointer_cast<ChElementBeamANCF_3333>(beam)) {
                    sectionshape = chrono_types::make_shared<ChBeamSectionShapeRectangular>(
                        beam3333->GetThicknessY(),
                        beam3333->GetThicknessZ());  // TO DO use ChBeamSection also in ANCF beam
                }
                if (sectionshape) {
                    for (int il = 0; il < sectionshape->GetNofLines(); ++il) {
                        n_verts += sectionshape->GetNofPoints(il) * beam_resolution;
                        n_vcols += sectionshape->GetNofPoints(il) * beam_resolution;
                        n_vnorms += sectionshape->GetNofPoints(il) * beam_resolution;
                        n_triangles += 2 * (sectionshape->GetNofPoints(il) - 1) * (beam_resolution - 1);
                    }
                }

            } else if (auto mshell = std::dynamic_pointer_cast<ChElementShell>(FEMmesh->GetElement(iel))) {
                // ELEMENT IS A SHELL
                if (!mshell->IsTriangleShell()) {
                    n_verts += shell_resolution * shell_resolution;
                    n_vcols += shell_resolution * shell_resolution;
                    n_vnorms += shell_resolution * shell_resolution;
                    n_triangles += 2 * (shell_resolution - 1) * (shell_resolution - 1);  // n. triangle faces
                } else {
                    for (int idp = 1; idp <= shell_resolution; ++idp) {
                        n_verts += idp;
                        n_vcols += idp;
                        n_vnorms += idp;
                    }
                    n_triangles +=
                        2 * (shell_resolution - 1) *
                        (shell_resolution - 1);  // n. triangle faces (double as twin-triangles for back lightning)
                }
            }

            //***TO DO*** other types of elements...
        }
    }

    //   In case mesh surfaces for pressure loads etc.:
    //
    if (fem_data_type == DataType::LOADSURFACES) {
        for (unsigned int isu = 0; isu < FEMmesh->GetNmeshSurfaces(); ++isu) {
            std::shared_ptr<ChMeshSurface> surface = FEMmesh->GetMeshSurface(isu);
            for (unsigned int ifa = 0; ifa < surface->GetFacesList().size(); ++ifa) {
                std::shared_ptr<ChLoadableUV> face = surface->GetFacesList()[ifa];
                if (std::dynamic_pointer_cast<ChTetrahedronFace>(face)) {
                    // FACE ELEMENT IS A TETRAHEDRON FACE
                    n_verts += 3;
                    n_vcols += 3;
                    n_vnorms += 1;                                                      // flat face
                    n_triangles += 1;                                                   // n. triangle faces
                } else if (std::dynamic_pointer_cast<ChElementTetraCorot_4_P>(face)) {  //// RADU ?!?!?
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
    if (fem_data_type == DataType::CONTACTSURFACES) {
        for (unsigned int isu = 0; isu < FEMmesh->GetNcontactSurfaces(); ++isu) {
            if (auto msurface = std::dynamic_pointer_cast<ChContactSurfaceMesh>(FEMmesh->GetContactSurface(isu))) {
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

    if (smooth_faces) {
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

    bool need_automatic_smoothing = smooth_faces;

    unsigned int i_verts = 0;
    unsigned int i_vcols = 0;
    unsigned int i_vnorms = 0;
    unsigned int i_triindex = 0;

    //   In case of colormap drawing:
    if (fem_data_type != DataType::NONE && fem_data_type != DataType::LOADSURFACES &&
        fem_data_type != DataType::CONTACTSURFACES) {
        for (unsigned int iel = 0; iel < FEMmesh->GetNelements(); ++iel) {
            // ------------ELEMENT IS A TETRAHEDRON 4 NODES?

            if (auto mytetra = std::dynamic_pointer_cast<ChElementTetrahedron>(FEMmesh->GetElement(iel))) {
                auto node0 = std::dynamic_pointer_cast<ChNodeFEAxyz>(mytetra->GetTetrahedronNode(0));
                auto node1 = std::dynamic_pointer_cast<ChNodeFEAxyz>(mytetra->GetTetrahedronNode(1));
                auto node2 = std::dynamic_pointer_cast<ChNodeFEAxyz>(mytetra->GetTetrahedronNode(2));
                auto node3 = std::dynamic_pointer_cast<ChNodeFEAxyz>(mytetra->GetTetrahedronNode(3));

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

                if (shrink_elements) {
                    ChVector<> vc = (p0 + p1 + p2 + p3) * (0.25);
                    p0 = vc + shrink_factor * (p0 - vc);
                    p1 = vc + shrink_factor * (p1 - vc);
                    p2 = vc + shrink_factor * (p2 - vc);
                    p3 = vc + shrink_factor * (p3 - vc);
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
                    ComputeFalseColor(ComputeScalarOutput(node0, 0, FEMmesh->GetElement(iel)));
                ++i_vcols;
                trianglemesh->getCoordsColors()[i_vcols] =
                    ComputeFalseColor(ComputeScalarOutput(node1, 1, FEMmesh->GetElement(iel)));
                ++i_vcols;
                trianglemesh->getCoordsColors()[i_vcols] =
                    ComputeFalseColor(ComputeScalarOutput(node2, 2, FEMmesh->GetElement(iel)));
                ++i_vcols;
                trianglemesh->getCoordsColors()[i_vcols] =
                    ComputeFalseColor(ComputeScalarOutput(node3, 3, FEMmesh->GetElement(iel)));
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
                if (smooth_faces) {
                    ChVector<int> inorm_offset = ChVector<int>(inorm_el, inorm_el, inorm_el);
                    trianglemesh->getIndicesNormals()[i_triindex - 4] = ChVector<int>(0, 0, 0) + inorm_offset;
                    trianglemesh->getIndicesNormals()[i_triindex - 3] = ChVector<int>(1, 1, 1) + inorm_offset;
                    trianglemesh->getIndicesNormals()[i_triindex - 2] = ChVector<int>(2, 2, 2) + inorm_offset;
                    trianglemesh->getIndicesNormals()[i_triindex - 1] = ChVector<int>(3, 3, 3) + inorm_offset;
                    i_vnorms += 4;
                }
            }

            // ------------ELEMENT IS A TETRAHEDRON 4 NODES -for SCALAR field- ?

            if (auto mytetra = std::dynamic_pointer_cast<ChElementTetraCorot_4_P>(FEMmesh->GetElement(iel))) {
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

                if (shrink_elements) {
                    ChVector<> vc = (p0 + p1 + p2 + p3) * (0.25);
                    p0 = vc + shrink_factor * (p0 - vc);
                    p1 = vc + shrink_factor * (p1 - vc);
                    p2 = vc + shrink_factor * (p2 - vc);
                    p3 = vc + shrink_factor * (p3 - vc);
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
                    ComputeFalseColor(ComputeScalarOutput(node0, 0, FEMmesh->GetElement(iel)));
                ++i_vcols;
                trianglemesh->getCoordsColors()[i_vcols] =
                    ComputeFalseColor(ComputeScalarOutput(node1, 1, FEMmesh->GetElement(iel)));
                ++i_vcols;
                trianglemesh->getCoordsColors()[i_vcols] =
                    ComputeFalseColor(ComputeScalarOutput(node2, 2, FEMmesh->GetElement(iel)));
                ++i_vcols;
                trianglemesh->getCoordsColors()[i_vcols] =
                    ComputeFalseColor(ComputeScalarOutput(node3, 3, FEMmesh->GetElement(iel)));
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
                if (smooth_faces) {
                    ChVector<int> inorm_offset = ChVector<int>(inorm_el, inorm_el, inorm_el);
                    trianglemesh->getIndicesNormals()[i_triindex - 4] = ChVector<int>(0, 0, 0) + inorm_offset;
                    trianglemesh->getIndicesNormals()[i_triindex - 3] = ChVector<int>(1, 1, 1) + inorm_offset;
                    trianglemesh->getIndicesNormals()[i_triindex - 2] = ChVector<int>(2, 2, 2) + inorm_offset;
                    trianglemesh->getIndicesNormals()[i_triindex - 1] = ChVector<int>(3, 3, 3) + inorm_offset;
                    i_vnorms += 4;
                }
            }

            // ------------ELEMENT IS A HEXAHEDRON 8 NODES?
            if (std::dynamic_pointer_cast<ChElementHexahedron>(FEMmesh->GetElement(iel))) {
                UpdateBuffers_Hex(FEMmesh->GetElement(iel), *trianglemesh, i_verts, i_vnorms, i_vcols, i_triindex);
            }

            // ------------ELEMENT IS A BEAM with the new ChBeamSectionShape?
            if (auto beam = std::dynamic_pointer_cast<ChElementBeam>(FEMmesh->GetElement(iel))) {
                std::shared_ptr<ChBeamSectionShape> sectionshape;
                if (auto beamEuler = std::dynamic_pointer_cast<ChElementBeamEuler>(beam)) {
                    sectionshape = beamEuler->GetSection()->GetDrawShape();
                } else if (auto cableANCF = std::dynamic_pointer_cast<ChElementCableANCF>(beam)) {
                    sectionshape = cableANCF->GetSection()->GetDrawShape();
                } else if (auto beamIGA = std::dynamic_pointer_cast<ChElementBeamIGA>(beam)) {
                    sectionshape = beamIGA->GetSection()->GetDrawShape();
                } else if (auto beamTimoshenko = std::dynamic_pointer_cast<ChElementBeamTaperedTimoshenko>(beam)) {
                    sectionshape = beamTimoshenko->GetTaperedSection()->GetSectionA()->GetDrawShape();
                } else if (auto beamTimoshenkoFPM =
                               std::dynamic_pointer_cast<ChElementBeamTaperedTimoshenkoFPM>(beam)) {
                    sectionshape = beamTimoshenkoFPM->GetTaperedSection()->GetSectionA()->GetDrawShape();
                } else if (auto beam3243 = std::dynamic_pointer_cast<ChElementBeamANCF_3243>(beam)) {
                    sectionshape = chrono_types::make_shared<ChBeamSectionShapeRectangular>(beam3243->GetThicknessY(),
                                                                                            beam3243->GetThicknessZ());
                } else if (auto beam3333 = std::dynamic_pointer_cast<ChElementBeamANCF_3333>(beam)) {
                    sectionshape = chrono_types::make_shared<ChBeamSectionShapeRectangular>(beam3333->GetThicknessY(),
                                                                                            beam3333->GetThicknessZ());
                }

                if (sectionshape) {
                    unsigned int ivert_el = i_verts;
                    int n_section_pts = 0;
                    for (int i = 0; i < sectionshape->GetNofLines(); ++i)
                        n_section_pts += sectionshape->GetNofPoints(i);

                    for (int in = 0; in < beam_resolution; ++in) {
                        double eta = -1.0 + (2.0 * in / (beam_resolution - 1));

                        ChVector<> P;
                        ChQuaternion<> msectionrot;
                        beam->EvaluateSectionFrame(eta, P,
                                                     msectionrot);  // compute abs. pos and rot of section plane

                        ChVector<> vresult;
                        ChVector<> vresultB;
                        double sresult = 0;
                        switch (fem_data_type) {
                            case DataType::ELEM_BEAM_MX:
                                beam->EvaluateSectionForceTorque(eta, vresult, vresultB);
                                sresult = vresultB.x();
                                break;
                            case DataType::ELEM_BEAM_MY:
                                beam->EvaluateSectionForceTorque(eta, vresult, vresultB);
                                sresult = vresultB.y();
                                break;
                            case DataType::ELEM_BEAM_MZ:
                                beam->EvaluateSectionForceTorque(eta, vresult, vresultB);
                                sresult = vresultB.z();
                                break;
                            case DataType::ELEM_BEAM_TX:
                                beam->EvaluateSectionForceTorque(eta, vresult, vresultB);
                                sresult = vresult.x();
                                break;
                            case DataType::ELEM_BEAM_TY:
                                beam->EvaluateSectionForceTorque(eta, vresult, vresultB);
                                sresult = vresult.y();
                                break;
                            case DataType::ELEM_BEAM_TZ:
                                beam->EvaluateSectionForceTorque(eta, vresult, vresultB);
                                sresult = vresult.z();
                                break;
                            case DataType::ANCF_BEAM_AX:
                                beam->EvaluateSectionStrain(eta, vresult);
                                sresult = vresult.x();
                                break;
                            case DataType::ANCF_BEAM_BD:
                                beam->EvaluateSectionStrain(eta, vresult);
                                sresult = vresult.y();
                                break;
                            default:
                                break;
                        }
                        ChColor mcol = ComputeFalseColor(sresult);

                        int subline_stride = 0;

                        for (int il = 0; il < sectionshape->GetNofLines(); ++il) {
                            std::vector<ChVector<>> msubline_pts(
                                sectionshape->GetNofPoints(il));  // suboptimal temp - better store&transform in place
                            std::vector<ChVector<>> msubline_normals(
                                sectionshape->GetNofPoints(il));  // suboptimal temp - better store&transform in place

                            // compute the point yz coords and yz normals in sectional frame
                            sectionshape->GetPoints(il, msubline_pts);
                            sectionshape->GetNormals(il, msubline_normals);

                            // store rotated vertexes, colors, normals
                            for (int is = 0; is < msubline_pts.size(); ++is) {
                                ChVector<> Rw = msectionrot.Rotate(msubline_pts[is]);
                                ChVector<> Rn = msectionrot.Rotate(msubline_normals[is]);
                                trianglemesh->getCoordsVertices()[i_verts] = P + Rw;
                                ++i_verts;
                                trianglemesh->getCoordsColors()[i_vcols] = mcol;
                                ++i_vcols;
                                if (smooth_faces) {
                                    trianglemesh->getCoordsNormals()[i_vnorms] = Rn;
                                    ++i_vnorms;
                                }
                            }
                            // store face connectivity
                            if (in > 0) {
                                ChVector<int> ivert_offset(ivert_el, ivert_el, ivert_el);
                                ChVector<int> islice_offset((in - 1) * n_section_pts, (in - 1) * n_section_pts,
                                                            (in - 1) * n_section_pts);
                                for (size_t is = 0; is < msubline_pts.size() - 1; ++is) {
                                    int ipa = int(is) + subline_stride;
                                    int ipb =
                                        int(is + 1) + subline_stride;  // % n_section_pts; if wrapped - not needed here;
                                    int ipaa = ipa + n_section_pts;
                                    int ipbb = ipb + n_section_pts;

                                    trianglemesh->getIndicesVertexes()[i_triindex] =
                                        ChVector<int>(ipa, ipbb, ipaa) + islice_offset + ivert_offset;
                                    if (smooth_faces) {
                                        trianglemesh->getIndicesNormals()[i_triindex] =
                                            ChVector<int>(ipa, ipbb, ipaa) + islice_offset + ivert_offset;
                                    }
                                    ++i_triindex;

                                    trianglemesh->getIndicesVertexes()[i_triindex] =
                                        ChVector<int>(ipa, ipb, ipbb) + islice_offset + ivert_offset;
                                    if (smooth_faces) {
                                        trianglemesh->getIndicesNormals()[i_triindex] =
                                            ChVector<int>(ipa, ipb, ipbb) + islice_offset + ivert_offset;
                                    }
                                    ++i_triindex;
                                }
                            }  // end if not first section

                            subline_stride += int(msubline_pts.size());

                        }  // end sublines loop

                    }  // end sections loop

                    // normals are already computed in the best way
                    need_automatic_smoothing = false;
                }
            }

            // ------------ELEMENT IS A SHELL?
            if (auto myshell = std::dynamic_pointer_cast<ChElementShell>(FEMmesh->GetElement(iel))) {
                unsigned int ivert_el = i_verts;
                unsigned int inorm_el = i_vnorms;

                if (!myshell->IsTriangleShell()) {
                    for (int iu = 0; iu < shell_resolution; ++iu)
                        for (int iv = 0; iv < shell_resolution; ++iv) {
                            double u = -1.0 + (2.0 * iu / (shell_resolution - 1));
                            double v = -1.0 + (2.0 * iv / (shell_resolution - 1));

                            ChVector<> P;
                            myshell->EvaluateSectionPoint(u, v, P);  // compute abs. pos and rot of section plane

                            ChColor mcol(1, 1, 1);
                            /*
                            ChVector<> vresult;
                            ChVector<> vresultB;
                            double sresult = 0;
                            switch(fem_data_type)
                            {
                                case ELEM_SHELL_blabla:
                                    myshell->EvaluateSectionForceTorque(eta, vresult, vresultB);
                                    sresult = vresultB.x();
                                    break;

                            }
                            ChColor mcol = ComputeFalseColor(sresult);
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

                                if (smooth_faces) {
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

                if (myshell->IsTriangleShell()) {
                    need_automatic_smoothing = false;

                    int triangle_pt = 0;
                    for (int iu = 0; iu < shell_resolution; ++iu) {
                        for (int iv = 0; iv + iu < shell_resolution; ++iv) {
                            double u = ((double)iu / (double)(shell_resolution - 1));
                            double v = ((double)iv / (double)(shell_resolution - 1));
                            ChVector<> P;
                            myshell->EvaluateSectionPoint(u, v, P);  // compute abs. pos and rot of section plane

                            ChColor mcol(1, 1, 1);
                            /*
                            ChVector<> vresult;
                            ChVector<> vresultB;
                            double sresult = 0;
                            switch(fem_data_type)
                            {
                                case ELEM_SHELL_blabla:
                                    myshell->EvaluateSectionForceTorque(eta, vresult, vresultB);
                                    sresult = vresultB.x();
                                    break;

                            }
                            ChColor mcol = ComputeFalseColor(sresult);
                            */

                            trianglemesh->getCoordsVertices()[i_verts] = P;
                            ++i_verts;

                            trianglemesh->getCoordsColors()[i_vcols] = mcol;
                            ++i_vcols;

                            ++i_vnorms;

                            if (iu < shell_resolution - 1) {
                                ChVector<int> ivert_offset(ivert_el, ivert_el, ivert_el);
                                ChVector<int> inorm_offset(inorm_el, inorm_el, inorm_el);

                                if (iv > 0) {
                                    trianglemesh->getIndicesVertexes()[i_triindex] =
                                        ChVector<int>(triangle_pt, triangle_pt - 1,
                                                      triangle_pt + shell_resolution - iu - 1) +
                                        ivert_offset;
                                    trianglemesh->getIndicesVertexes()[i_triindex + 1] =
                                        ChVector<int>(triangle_pt - 1, triangle_pt,
                                                      triangle_pt + shell_resolution - iu - 1) +
                                        ivert_offset;

                                    if (smooth_faces) {
                                        trianglemesh->getIndicesNormals()[i_triindex] =
                                            ChVector<int>(triangle_pt, triangle_pt - 1,
                                                          triangle_pt + shell_resolution - iu - 1) +
                                            inorm_offset;
                                        trianglemesh->getIndicesNormals()[i_triindex + 1] =
                                            ChVector<int>(triangle_pt - 1, triangle_pt,
                                                          triangle_pt + shell_resolution - iu - 1) +
                                            inorm_offset;
                                    }
                                    ++i_triindex;
                                    ++i_triindex;
                                }

                                if (iv > 1) {
                                    trianglemesh->getIndicesVertexes()[i_triindex] =
                                        ChVector<int>(triangle_pt - 1, triangle_pt + shell_resolution - iu - 2,
                                                      triangle_pt + shell_resolution - iu - 1) +
                                        ivert_offset;
                                    trianglemesh->getIndicesVertexes()[i_triindex + 1] =
                                        ChVector<int>(triangle_pt - 1, triangle_pt + shell_resolution - iu - 1,
                                                      triangle_pt + shell_resolution - iu - 2) +
                                        ivert_offset;

                                    if (smooth_faces) {
                                        trianglemesh->getIndicesNormals()[i_triindex] =
                                            ChVector<int>(triangle_pt - 1, triangle_pt + shell_resolution - iu - 2,
                                                          triangle_pt + shell_resolution - iu - 1) +
                                            inorm_offset;
                                        trianglemesh->getIndicesNormals()[i_triindex + 1] =
                                            ChVector<int>(triangle_pt - 1, triangle_pt + shell_resolution - iu - 1,
                                                          triangle_pt + shell_resolution - iu - 2) +
                                            inorm_offset;
                                    }
                                    ++i_triindex;
                                    ++i_triindex;
                                }
                            }
                            ++triangle_pt;

                        }  // end V loop on triangle tesselated points
                    }      // end U loop on triangle tesselated points

                }  // end if triangular shell
            }

            // ------------***TO DO*** other types of elements...

        }  // End of loop on elements
    }      //  End of case of colormap drawing:

    //   In case mesh surfaces for pressure loads etc.:
    //
    if (fem_data_type == DataType::LOADSURFACES) {
        for (unsigned int isu = 0; isu < FEMmesh->GetNmeshSurfaces(); ++isu) {
            std::shared_ptr<ChMeshSurface> msurface = FEMmesh->GetMeshSurface(isu);
            for (unsigned int ifa = 0; ifa < msurface->GetFacesList().size(); ++ifa) {
                std::shared_ptr<ChLoadableUV> mface = msurface->GetFacesList()[ifa];
                // FACE ELEMENT IS A TETRAHEDRON FACE
                if (auto mfacetetra = std::dynamic_pointer_cast<ChTetrahedronFace>(mface)) {
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
                    trianglemesh->getCoordsColors()[i_vcols] = meshcolor;
                    ++i_vcols;
                    trianglemesh->getCoordsColors()[i_vcols] = meshcolor;
                    ++i_vcols;
                    trianglemesh->getCoordsColors()[i_vcols] = meshcolor;
                    ++i_vcols;

                    // faces indexes
                    ChVector<int> ivert_offset(ivert_el, ivert_el, ivert_el);
                    trianglemesh->getIndicesVertexes()[i_triindex] = ChVector<int>(0, 1, 2) + ivert_offset;
                    ++i_triindex;

                    // normals indices (if not defaulting to flat triangles)
                    if (smooth_faces) {
                        ChVector<int> inorm_offset = ChVector<int>(inorm_el, inorm_el, inorm_el);
                        trianglemesh->getIndicesNormals()[i_triindex - 4] = ChVector<int>(0, 0, 0) + inorm_offset;
                        i_vnorms += 1;
                    }
                }

                // FACE ELEMENT IS A SHELL
                if (auto mfacetetra = std::dynamic_pointer_cast<ChElementTetraCorot_4_P>(mface)) {
                    //***TODO***
                }
            }
        }  // end loop on load surfaces
    }      // End of case of load surfaces

    //   In case of contact surfaces:
    //
    if (fem_data_type == DataType::CONTACTSURFACES) {
        for (unsigned int isu = 0; isu < FEMmesh->GetNcontactSurfaces(); ++isu) {
            if (auto msurface = std::dynamic_pointer_cast<ChContactSurfaceMesh>(FEMmesh->GetContactSurface(isu))) {
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
                    trianglemesh->getCoordsColors()[i_vcols] = meshcolor;
                    ++i_vcols;
                    trianglemesh->getCoordsColors()[i_vcols] = meshcolor;
                    ++i_vcols;
                    trianglemesh->getCoordsColors()[i_vcols] = meshcolor;
                    ++i_vcols;

                    // faces indexes
                    ChVector<int> ivert_offset(ivert_el, ivert_el, ivert_el);
                    trianglemesh->getIndicesVertexes()[i_triindex] = ChVector<int>(0, 1, 2) + ivert_offset;
                    ++i_triindex;

                    // normals indices (if not defaulting to flat triangles)
                    if (smooth_faces) {
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
    m_trimesh_shape->SetWireframe(wireframe);
    m_trimesh_shape->SetBackfaceCull(backface_cull);

    // GLYPHS

    //***TEST***
    m_glyphs_shape->Reserve(0);  // unoptimal, should reuse buffers as much as possible

    m_glyphs_shape->SetGlyphsSize(symbols_thickness);

    m_glyphs_shape->SetZbufferHide(zbuffer_hide);

    if (fem_glyph == GlyphType::NODE_DOT_POS) {
        m_glyphs_shape->SetDrawMode(ChGlyphs::GLYPH_POINT);
        for (unsigned int inode = 0; inode < FEMmesh->GetNnodes(); ++inode) {
            if (auto mynode1 = std::dynamic_pointer_cast<ChNodeFEAxyz>(FEMmesh->GetNode(inode))) {
                m_glyphs_shape->SetGlyphPoint(inode, mynode1->GetPos(), symbolscolor);
            } else if (auto mynode2 = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(FEMmesh->GetNode(inode))) {
                m_glyphs_shape->SetGlyphPoint(inode, mynode2->GetPos(), symbolscolor);
            } else if (auto mynode3 = std::dynamic_pointer_cast<ChNodeFEAxyzD>(FEMmesh->GetNode(inode))) {
                m_glyphs_shape->SetGlyphPoint(inode, mynode3->GetPos(), symbolscolor);
            } else if (auto mynode4 = std::dynamic_pointer_cast<ChNodeFEAxyzDD>(FEMmesh->GetNode(inode))) {
                m_glyphs_shape->SetGlyphPoint(inode, mynode4->GetPos(), symbolscolor);
            }
        }
    }
    if (fem_glyph == GlyphType::NODE_CSYS) {
        m_glyphs_shape->SetDrawMode(ChGlyphs::GLYPH_COORDSYS);
        for (unsigned int inode = 0; inode < FEMmesh->GetNnodes(); ++inode) {
            if (auto mynode = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(FEMmesh->GetNode(inode))) {
                m_glyphs_shape->SetGlyphCoordsys(inode, mynode->Frame().GetCoord());
            }
            // else if (auto mynode = std::dynamic_pointer_cast<ChNodeFEAxyzD>(FEMmesh->GetNode(inode))) {
            //	m_glyphs_shape->SetGlyphVector(inode, mynode->GetPos(), mynode->GetD() * symbols_scale,
            // symbolscolor );
            //}
        }
    }
    if (fem_glyph == GlyphType::NODE_VECT_SPEED) {
        m_glyphs_shape->SetDrawMode(ChGlyphs::GLYPH_VECTOR);
        for (unsigned int inode = 0; inode < FEMmesh->GetNnodes(); ++inode)
            if (auto mynode = std::dynamic_pointer_cast<ChNodeFEAxyz>(FEMmesh->GetNode(inode))) {
                m_glyphs_shape->SetGlyphVector(inode, mynode->GetPos(), mynode->GetPos_dt() * symbols_scale,
                                               symbolscolor);
            }
    }
    if (fem_glyph == GlyphType::NODE_VECT_ACCEL) {
        m_glyphs_shape->SetDrawMode(ChGlyphs::GLYPH_VECTOR);
        for (unsigned int inode = 0; inode < FEMmesh->GetNnodes(); ++inode)
            if (auto mynode = std::dynamic_pointer_cast<ChNodeFEAxyz>(FEMmesh->GetNode(inode))) {
                m_glyphs_shape->SetGlyphVector(inode, mynode->GetPos(), mynode->GetPos_dtdt() * symbols_scale,
                                               symbolscolor);
            }
    }
    if (fem_glyph == GlyphType::ELEM_VECT_DP) {
        m_glyphs_shape->SetDrawMode(ChGlyphs::GLYPH_VECTOR);
        for (unsigned int iel = 0; iel < FEMmesh->GetNelements(); ++iel)
            if (auto myelement = std::dynamic_pointer_cast<ChElementTetraCorot_4_P>(FEMmesh->GetElement(iel))) {
                ChVector<> mvP(myelement->GetPgradient());
                auto n0 = std::static_pointer_cast<ChNodeFEAxyzP>(myelement->GetNodeN(0));
                auto n1 = std::static_pointer_cast<ChNodeFEAxyzP>(myelement->GetNodeN(1));
                auto n2 = std::static_pointer_cast<ChNodeFEAxyzP>(myelement->GetNodeN(2));
                auto n3 = std::static_pointer_cast<ChNodeFEAxyzP>(myelement->GetNodeN(3));
                ChVector<> mPoint = (n0->GetPos() + n1->GetPos() + n2->GetPos() + n3->GetPos()) *
                                    0.25;  // to do: better placement in Gauss point
                m_glyphs_shape->SetGlyphVector(iel, mPoint, mvP * symbols_scale, symbolscolor);
            }
    }
    if (fem_glyph == GlyphType::ELEM_TENS_STRAIN) {
        m_glyphs_shape->SetDrawMode(ChGlyphs::GLYPH_VECTOR);
        int nglyvect = 0;
        for (unsigned int iel = 0; iel < FEMmesh->GetNelements(); ++iel)
            if (auto myelement = std::dynamic_pointer_cast<ChElementTetraCorot_4>(FEMmesh->GetElement(iel))) {
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
                m_glyphs_shape->SetGlyphVector(nglyvect, mPoint, myelement->Rotation() * v1 * e1 * symbols_scale,
                                               ComputeFalseColor(e1));
                ++nglyvect;
                m_glyphs_shape->SetGlyphVector(nglyvect, mPoint, myelement->Rotation() * v2 * e2 * symbols_scale,
                                               ComputeFalseColor(e2));
                ++nglyvect;
                m_glyphs_shape->SetGlyphVector(nglyvect, mPoint, myelement->Rotation() * v3 * e3 * symbols_scale,
                                               ComputeFalseColor(e3));
                ++nglyvect;
            }
    }
    if (fem_glyph == GlyphType::ELEM_TENS_STRESS) {
        m_glyphs_shape->SetDrawMode(ChGlyphs::GLYPH_VECTOR);
        int nglyvect = 0;
        for (unsigned int iel = 0; iel < FEMmesh->GetNelements(); ++iel)
            if (auto myelement = std::dynamic_pointer_cast<ChElementTetraCorot_4>(FEMmesh->GetElement(iel))) {
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
                m_glyphs_shape->SetGlyphVector(nglyvect, mPoint, myelement->Rotation() * v1 * e1 * symbols_scale,
                                               ComputeFalseColor(e1));
                ++nglyvect;
                m_glyphs_shape->SetGlyphVector(nglyvect, mPoint, myelement->Rotation() * v2 * e2 * symbols_scale,
                                               ComputeFalseColor(e2));
                ++nglyvect;
                m_glyphs_shape->SetGlyphVector(nglyvect, mPoint, myelement->Rotation() * v3 * e3 * symbols_scale,
                                               ComputeFalseColor(e3));
                ++nglyvect;
            }
    }

    //***TEST***
    if (false)
        for (unsigned int iel = 0; iel < FEMmesh->GetNelements(); ++iel) {
            // ------------ELEMENT IS A ChElementShellReissner4?
            if (auto myshell = std::dynamic_pointer_cast<ChElementShellReissner4>(FEMmesh->GetElement(iel))) {
                m_glyphs_shape->SetGlyphsSize(0.4);
                // average rotation
                if (false) {
                    m_glyphs_shape->SetDrawMode(ChGlyphs::GLYPH_COORDSYS);
                    m_glyphs_shape->GetNumberOfGlyphs();
                    m_glyphs_shape->SetGlyphCoordsys(
                        (unsigned int)m_glyphs_shape->GetNumberOfGlyphs(),
                        ChCoordsys<>((myshell->GetNodeA()->GetPos() + myshell->GetNodeB()->GetPos() +
                                      myshell->GetNodeC()->GetPos() + myshell->GetNodeD()->GetPos()) *
                                         0.25,
                                     myshell->GetAvgRot()));
                }
                // gauss point coordsys
                if (false) {
                    m_glyphs_shape->SetDrawMode(ChGlyphs::GLYPH_COORDSYS);
                    for (int igp = 0; igp < 4; ++igp) {
                        m_glyphs_shape->GetNumberOfGlyphs();
                        m_glyphs_shape->SetGlyphCoordsys(
                            (unsigned int)m_glyphs_shape->GetNumberOfGlyphs(),
                            ChCoordsys<>(myshell->EvaluateGP(igp), myshell->T_i[igp].Get_A_quaternion()));
                    }
                }
                // gauss point weights
                if (false) {
                    m_glyphs_shape->SetDrawMode(ChGlyphs::GLYPH_VECTOR);
                    for (int igp = 0; igp < 4; ++igp) {
                        m_glyphs_shape->GetNumberOfGlyphs();
                        m_glyphs_shape->SetGlyphVector((unsigned int)m_glyphs_shape->GetNumberOfGlyphs(),
                                                       myshell->EvaluateGP(igp),
                                                       ChVector<>(0, myshell->alpha_i[igp] * 4, 0));
                    }
                }
                // gauss curvatures
                if (false) {
                    for (int igp = 0; igp < 4; ++igp) {
                        m_glyphs_shape->SetDrawMode(ChGlyphs::GLYPH_VECTOR);
                        /*
                        m_glyphs_shape->GetNumberOfGlyphs();
                        m_glyphs_shape->SetGlyphVector(m_glyphs_shape->GetNumberOfGlyphs(),
                            myshell->EvaluateGP(igp),
                            myshell->T_i[igp].Rotate(myshell->kur_u_tilde[igp]*50), ChColor(1,0,0) );
                        m_glyphs_shape->GetNumberOfGlyphs();
                        m_glyphs_shape->SetGlyphVector(m_glyphs_shape->GetNumberOfGlyphs(),
                            myshell->EvaluateGP(igp),
                            myshell->T_i[igp].Rotate(myshell->kur_v_tilde[igp]*50), ChColor(0,1,0) );
                        */
                        m_glyphs_shape->GetNumberOfGlyphs();
                        m_glyphs_shape->SetGlyphVector(
                            (unsigned int)m_glyphs_shape->GetNumberOfGlyphs(), myshell->EvaluateGP(igp),
                            myshell->T_i[igp] * ((myshell->k_tilde_1_i[igp] + myshell->k_tilde_2_i[igp]) * 50),
                            ChColor(0, 0, 0));
                    }
                }
                // gauss strains
                if (true) {
                    for (int igp = 0; igp < 4; ++igp) {
                        m_glyphs_shape->SetDrawMode(ChGlyphs::GLYPH_VECTOR);
                        double scale = 1;
                        m_glyphs_shape->GetNumberOfGlyphs();
                        m_glyphs_shape->SetGlyphVector(
                            (unsigned int)m_glyphs_shape->GetNumberOfGlyphs(), myshell->EvaluateGP(igp),
                            myshell->T_i[igp] * (myshell->eps_tilde_1_i[igp] * scale), ChColor(1, 0, 0));
                        m_glyphs_shape->GetNumberOfGlyphs();
                        m_glyphs_shape->SetGlyphVector(
                            (unsigned int)m_glyphs_shape->GetNumberOfGlyphs(), myshell->EvaluateGP(igp),
                            myshell->T_i[igp] * (myshell->eps_tilde_2_i[igp] * scale), ChColor(0, 0, 1));
                        m_glyphs_shape->GetNumberOfGlyphs();
                    }
                }
                // other...
            }
        }
}

}  // end namespace chrono
