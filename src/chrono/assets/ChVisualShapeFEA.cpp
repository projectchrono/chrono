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
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
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

ChVisualShapeFEA::ChVisualShapeFEA() : physics_item(nullptr) {
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
    shell_resolution = 2;

    meshcolor = ChColor(1, 1, 1);
    symbolscolor = ChColor(0, 0.5, 0.5);

    undeformed_reference = false;

    m_trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    m_glyphs_shape = chrono_types::make_shared<ChGlyphs>();
}

ChColor ChVisualShapeFEA::ComputeFalseColor(double mv) {
    if (fem_data_type == DataType::SURFACE)
        return meshcolor;

    return ChColor::ComputeFalseColor(mv, colorscale_min, colorscale_max, true);
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
            return mnode->GetPosDt().Length();
        case DataType::NODE_SPEED_X:
            return mnode->GetPosDt().x();
        case DataType::NODE_SPEED_Y:
            return mnode->GetPosDt().y();
        case DataType::NODE_SPEED_Z:
            return mnode->GetPosDt().z();
        case DataType::NODE_ACCEL_NORM:
            return mnode->GetPosDt2().Length();
        case DataType::NODE_ACCEL_X:
            return mnode->GetPosDt2().x();
        case DataType::NODE_ACCEL_Y:
            return mnode->GetPosDt2().y();
        case DataType::NODE_ACCEL_Z:
            return mnode->GetPosDt2().z();
        case DataType::ELEM_STRAIN_VONMISES:
            if (auto mytetra = std::dynamic_pointer_cast<ChElementTetraCorot_4>(melement)) {
                return mytetra->GetStrain().GetEquivalentVonMises();
            }
            break;
        case DataType::ELEM_STRESS_VONMISES:
            if (auto mytetra = std::dynamic_pointer_cast<ChElementTetraCorot_4>(melement)) {
                return mytetra->GetStress().GetEquivalentVonMises();
            }
            break;
        case DataType::ELEM_STRAIN_HYDROSTATIC:
            if (auto mytetra = std::dynamic_pointer_cast<ChElementTetraCorot_4>(melement)) {
                return mytetra->GetStrain().GetEquivalentMeanHydrostatic();
            }
            break;
        case DataType::ELEM_STRESS_HYDROSTATIC:
            if (auto mytetra = std::dynamic_pointer_cast<ChElementTetraCorot_4>(melement)) {
                return mytetra->GetStress().GetEquivalentMeanHydrostatic();
            }
            break;
        default:
            return 1e30;
    }
    //// TODO  other types of scalar outputs
    return 0;
}

double ChVisualShapeFEA::ComputeScalarOutput(std::shared_ptr<ChNodeFEAxyzP> mnode,
                                             int nodeID,
                                             std::shared_ptr<ChElementBase> melement) {
    switch (fem_data_type) {
        case DataType::SURFACE:
            return 1e30;  // to force 'white' in false color scale. Hack, to be improved.
        case DataType::NODE_FIELD_VALUE:
            return (mnode->GetFieldVal());
        default:
            return 1e30;
    }
    //// TODO  other types of scalar outputs
    return 0;
}

ChVector3f& FetchOrAllocate(std::vector<ChVector3f>& mvector, unsigned int& id) {
    if (id > mvector.size()) {
        id = 0;
        return mvector[0];  // error
    }
    if (id == mvector.size()) {
        mvector.push_back(ChVector3f(0, 0, 0));
    }
    ++id;
    return mvector[id - 1];
}

void TriangleNormalsReset(std::vector<ChVector3d>& normals, std::vector<int>& accumul) {
    for (unsigned int nn = 0; nn < normals.size(); ++nn) {
        normals[nn] = ChVector3d(0, 0, 0);
        accumul[nn] = 0;
    }
}

void TriangleNormalsCompute(ChVector3i norm_indexes,
                            ChVector3i vert_indexes,
                            std::vector<ChVector3d>& vertexes,
                            std::vector<ChVector3d>& normals,
                            std::vector<int>& accumul) {
    ChVector3d tnorm = Vcross(vertexes[vert_indexes.y()] - vertexes[vert_indexes.x()],
                              vertexes[vert_indexes.z()] - vertexes[vert_indexes.x()])
                           .GetNormalized();
    normals[norm_indexes.x()] += tnorm;
    normals[norm_indexes.y()] += tnorm;
    normals[norm_indexes.z()] += tnorm;
    accumul[norm_indexes.x()] += 1;
    accumul[norm_indexes.y()] += 1;
    accumul[norm_indexes.z()] += 1;
}

void TriangleNormalsSmooth(std::vector<ChVector3d>& normals, std::vector<int>& accumul) {
    for (unsigned int nn = 0; nn < normals.size(); ++nn) {
        normals[nn] = normals[nn] * (1.0 / (double)accumul[nn]);
    }
}

void ChVisualShapeFEA::UpdateBuffers_Tetrahedron(std::shared_ptr<fea::ChElementBase> element,
                                                 ChTriangleMeshConnected& trianglemesh,
                                                 unsigned int& i_verts,
                                                 unsigned int& i_vnorms,
                                                 unsigned int& i_vcols,
                                                 unsigned int& i_triindex,
                                                 bool& need_automatic_smoothing) {
    auto node0 = std::static_pointer_cast<ChNodeFEAxyz>(element->GetNode(0));
    auto node1 = std::static_pointer_cast<ChNodeFEAxyz>(element->GetNode(1));
    auto node2 = std::static_pointer_cast<ChNodeFEAxyz>(element->GetNode(2));
    auto node3 = std::static_pointer_cast<ChNodeFEAxyz>(element->GetNode(3));

    unsigned int ivert_el = i_verts;
    unsigned int inorm_el = i_vnorms;

    // vertexes
    ChVector3d p0 = node0->GetPos();
    ChVector3d p1 = node1->GetPos();
    ChVector3d p2 = node2->GetPos();
    ChVector3d p3 = node3->GetPos();
    if (undeformed_reference) {
        p0 = node0->GetX0();
        p1 = node1->GetX0();
        p2 = node2->GetX0();
        p3 = node3->GetX0();
    }

    if (shrink_elements) {
        ChVector3d vc = (p0 + p1 + p2 + p3) * (0.25);
        p0 = vc + shrink_factor * (p0 - vc);
        p1 = vc + shrink_factor * (p1 - vc);
        p2 = vc + shrink_factor * (p2 - vc);
        p3 = vc + shrink_factor * (p3 - vc);
    }
    trianglemesh.GetCoordsVertices()[i_verts] = p0;
    ++i_verts;
    trianglemesh.GetCoordsVertices()[i_verts] = p1;
    ++i_verts;
    trianglemesh.GetCoordsVertices()[i_verts] = p2;
    ++i_verts;
    trianglemesh.GetCoordsVertices()[i_verts] = p3;
    ++i_verts;

    // color
    trianglemesh.GetCoordsColors()[i_vcols] = ComputeFalseColor(ComputeScalarOutput(node0, 0, element));
    ++i_vcols;
    trianglemesh.GetCoordsColors()[i_vcols] = ComputeFalseColor(ComputeScalarOutput(node1, 1, element));
    ++i_vcols;
    trianglemesh.GetCoordsColors()[i_vcols] = ComputeFalseColor(ComputeScalarOutput(node2, 2, element));
    ++i_vcols;
    trianglemesh.GetCoordsColors()[i_vcols] = ComputeFalseColor(ComputeScalarOutput(node3, 3, element));
    ++i_vcols;

    // faces indexes
    ChVector3i ivert_offset(ivert_el, ivert_el, ivert_el);
    trianglemesh.GetIndicesVertexes()[i_triindex] = ChVector3i(0, 1, 2) + ivert_offset;
    ++i_triindex;
    trianglemesh.GetIndicesVertexes()[i_triindex] = ChVector3i(1, 3, 2) + ivert_offset;
    ++i_triindex;
    trianglemesh.GetIndicesVertexes()[i_triindex] = ChVector3i(2, 3, 0) + ivert_offset;
    ++i_triindex;
    trianglemesh.GetIndicesVertexes()[i_triindex] = ChVector3i(3, 1, 0) + ivert_offset;
    ++i_triindex;

    // normals indices (if not defaulting to flat triangles)
    if (smooth_faces) {
        ChVector3i inorm_offset = ChVector3i(inorm_el, inorm_el, inorm_el);
        trianglemesh.GetIndicesNormals()[i_triindex - 4] = ChVector3i(0, 0, 0) + inorm_offset;
        trianglemesh.GetIndicesNormals()[i_triindex - 3] = ChVector3i(1, 1, 1) + inorm_offset;
        trianglemesh.GetIndicesNormals()[i_triindex - 2] = ChVector3i(2, 2, 2) + inorm_offset;
        trianglemesh.GetIndicesNormals()[i_triindex - 1] = ChVector3i(3, 3, 3) + inorm_offset;
        i_vnorms += 4;
    }
}

void ChVisualShapeFEA::UpdateBuffers_Tetra_4_P(std::shared_ptr<fea::ChElementBase> element,
                                               ChTriangleMeshConnected& trianglemesh,
                                               unsigned int& i_verts,
                                               unsigned int& i_vnorms,
                                               unsigned int& i_vcols,
                                               unsigned int& i_triindex,
                                               bool& need_automatic_smoothing) {
    auto node0 = std::dynamic_pointer_cast<ChNodeFEAxyzP>(element->GetNode(0));
    auto node1 = std::dynamic_pointer_cast<ChNodeFEAxyzP>(element->GetNode(1));
    auto node2 = std::dynamic_pointer_cast<ChNodeFEAxyzP>(element->GetNode(2));
    auto node3 = std::dynamic_pointer_cast<ChNodeFEAxyzP>(element->GetNode(3));

    unsigned int ivert_el = i_verts;
    unsigned int inorm_el = i_vnorms;

    // vertexes
    ChVector3d p0 = node0->GetPos();
    ChVector3d p1 = node1->GetPos();
    ChVector3d p2 = node2->GetPos();
    ChVector3d p3 = node3->GetPos();

    if (shrink_elements) {
        ChVector3d vc = (p0 + p1 + p2 + p3) * (0.25);
        p0 = vc + shrink_factor * (p0 - vc);
        p1 = vc + shrink_factor * (p1 - vc);
        p2 = vc + shrink_factor * (p2 - vc);
        p3 = vc + shrink_factor * (p3 - vc);
    }
    trianglemesh.GetCoordsVertices()[i_verts] = p0;
    ++i_verts;
    trianglemesh.GetCoordsVertices()[i_verts] = p1;
    ++i_verts;
    trianglemesh.GetCoordsVertices()[i_verts] = p2;
    ++i_verts;
    trianglemesh.GetCoordsVertices()[i_verts] = p3;
    ++i_verts;

    // color
    trianglemesh.GetCoordsColors()[i_vcols] = ComputeFalseColor(ComputeScalarOutput(node0, 0, element));
    ++i_vcols;
    trianglemesh.GetCoordsColors()[i_vcols] = ComputeFalseColor(ComputeScalarOutput(node1, 1, element));
    ++i_vcols;
    trianglemesh.GetCoordsColors()[i_vcols] = ComputeFalseColor(ComputeScalarOutput(node2, 2, element));
    ++i_vcols;
    trianglemesh.GetCoordsColors()[i_vcols] = ComputeFalseColor(ComputeScalarOutput(node3, 3, element));
    ++i_vcols;

    // faces indexes
    ChVector3i ivert_offset(ivert_el, ivert_el, ivert_el);
    trianglemesh.GetIndicesVertexes()[i_triindex] = ChVector3i(0, 1, 2) + ivert_offset;
    ++i_triindex;
    trianglemesh.GetIndicesVertexes()[i_triindex] = ChVector3i(1, 3, 2) + ivert_offset;
    ++i_triindex;
    trianglemesh.GetIndicesVertexes()[i_triindex] = ChVector3i(2, 3, 0) + ivert_offset;
    ++i_triindex;
    trianglemesh.GetIndicesVertexes()[i_triindex] = ChVector3i(3, 1, 0) + ivert_offset;
    ++i_triindex;

    // normals indices (if not defaulting to flat triangles)
    if (smooth_faces) {
        ChVector3i inorm_offset = ChVector3i(inorm_el, inorm_el, inorm_el);
        trianglemesh.GetIndicesNormals()[i_triindex - 4] = ChVector3i(0, 0, 0) + inorm_offset;
        trianglemesh.GetIndicesNormals()[i_triindex - 3] = ChVector3i(1, 1, 1) + inorm_offset;
        trianglemesh.GetIndicesNormals()[i_triindex - 2] = ChVector3i(2, 2, 2) + inorm_offset;
        trianglemesh.GetIndicesNormals()[i_triindex - 1] = ChVector3i(3, 3, 3) + inorm_offset;
        i_vnorms += 4;
    }
}

// Helper function for updating visualization mesh buffers for hex elements.
void ChVisualShapeFEA::UpdateBuffers_Hex(std::shared_ptr<ChElementBase> element,
                                         ChTriangleMeshConnected& trianglemesh,
                                         unsigned int& i_verts,
                                         unsigned int& i_vnorms,
                                         unsigned int& i_vcols,
                                         unsigned int& i_triindex,
                                         bool& need_automatic_smoothing) {
    unsigned int ivert_el = i_verts;
    unsigned int inorm_el = i_vnorms;

    std::shared_ptr<ChNodeFEAxyz> nodes[8];
    ChVector3d pt[8];

    for (int in = 0; in < 8; ++in) {
        nodes[in] = std::static_pointer_cast<ChNodeFEAxyz>(element->GetNode(in));
        if (!undeformed_reference)
            pt[in] = nodes[in]->GetPos();
        else
            pt[in] = nodes[in]->GetX0();
    }

    // vertexes

    if (shrink_elements) {
        ChVector3d vc(0, 0, 0);
        for (int in = 0; in < 8; ++in)
            vc += pt[in];
        vc = vc * (1.0 / 8.0);  // average, center of element
        for (int in = 0; in < 8; ++in)
            pt[in] = vc + shrink_factor * (pt[in] - vc);
    }

    for (int in = 0; in < 8; ++in) {
        trianglemesh.GetCoordsVertices()[i_verts] = pt[in];
        ++i_verts;
    }

    // colours and colours indexes
    for (int in = 0; in < 8; ++in) {
        trianglemesh.GetCoordsColors()[i_vcols] = ComputeFalseColor(ComputeScalarOutput(nodes[in], in, element));
        ++i_vcols;
    }

    // faces indexes
    ChVector3i ivert_offset(ivert_el, ivert_el, ivert_el);
    trianglemesh.GetIndicesVertexes()[i_triindex] = ChVector3i(0, 2, 1) + ivert_offset;
    ++i_triindex;
    trianglemesh.GetIndicesVertexes()[i_triindex] = ChVector3i(0, 3, 2) + ivert_offset;
    ++i_triindex;
    trianglemesh.GetIndicesVertexes()[i_triindex] = ChVector3i(4, 5, 6) + ivert_offset;
    ++i_triindex;
    trianglemesh.GetIndicesVertexes()[i_triindex] = ChVector3i(4, 6, 7) + ivert_offset;
    ++i_triindex;
    trianglemesh.GetIndicesVertexes()[i_triindex] = ChVector3i(0, 7, 3) + ivert_offset;
    ++i_triindex;
    trianglemesh.GetIndicesVertexes()[i_triindex] = ChVector3i(0, 4, 7) + ivert_offset;
    ++i_triindex;
    trianglemesh.GetIndicesVertexes()[i_triindex] = ChVector3i(0, 5, 4) + ivert_offset;
    ++i_triindex;
    trianglemesh.GetIndicesVertexes()[i_triindex] = ChVector3i(0, 1, 5) + ivert_offset;
    ++i_triindex;
    trianglemesh.GetIndicesVertexes()[i_triindex] = ChVector3i(3, 7, 6) + ivert_offset;
    ++i_triindex;
    trianglemesh.GetIndicesVertexes()[i_triindex] = ChVector3i(3, 6, 2) + ivert_offset;
    ++i_triindex;
    trianglemesh.GetIndicesVertexes()[i_triindex] = ChVector3i(2, 5, 1) + ivert_offset;
    ++i_triindex;
    trianglemesh.GetIndicesVertexes()[i_triindex] = ChVector3i(2, 6, 5) + ivert_offset;
    ++i_triindex;

    // normals indices (if not defaulting to flat triangles)
    if (smooth_faces) {
        ChVector3i inorm_offset = ChVector3i(inorm_el, inorm_el, inorm_el);
        trianglemesh.GetIndicesNormals()[i_triindex - 12] = ChVector3i(0, 2, 1) + inorm_offset;
        trianglemesh.GetIndicesNormals()[i_triindex - 11] = ChVector3i(0, 3, 2) + inorm_offset;
        trianglemesh.GetIndicesNormals()[i_triindex - 10] = ChVector3i(4, 5, 6) + inorm_offset;
        trianglemesh.GetIndicesNormals()[i_triindex - 9] = ChVector3i(4, 6, 7) + inorm_offset;
        trianglemesh.GetIndicesNormals()[i_triindex - 8] = ChVector3i(8, 9, 10) + inorm_offset;
        trianglemesh.GetIndicesNormals()[i_triindex - 7] = ChVector3i(8, 11, 9) + inorm_offset;
        trianglemesh.GetIndicesNormals()[i_triindex - 6] = ChVector3i(12, 13, 14) + inorm_offset;
        trianglemesh.GetIndicesNormals()[i_triindex - 5] = ChVector3i(12, 15, 13) + inorm_offset;
        trianglemesh.GetIndicesNormals()[i_triindex - 4] = ChVector3i(16, 18, 17) + inorm_offset;
        trianglemesh.GetIndicesNormals()[i_triindex - 3] = ChVector3i(16, 17, 19) + inorm_offset;
        trianglemesh.GetIndicesNormals()[i_triindex - 2] = ChVector3i(20, 21, 23) + inorm_offset;
        trianglemesh.GetIndicesNormals()[i_triindex - 1] = ChVector3i(20, 22, 21) + inorm_offset;
        i_vnorms += 24;
    }
}

void ChVisualShapeFEA::UpdateBuffers_Beam(std::shared_ptr<fea::ChElementBase> element,
                                          ChTriangleMeshConnected& trianglemesh,
                                          unsigned int& i_verts,
                                          unsigned int& i_vnorms,
                                          unsigned int& i_vcols,
                                          unsigned int& i_triindex,
                                          bool& need_automatic_smoothing) {
    auto beam = std::static_pointer_cast<ChElementBeam>(element);

    std::shared_ptr<ChBeamSectionShape> sectionshape;
    if (auto beamEuler = std::dynamic_pointer_cast<ChElementBeamEuler>(beam)) {
        sectionshape = beamEuler->GetSection()->GetDrawShape();
    } else if (auto cableANCF = std::dynamic_pointer_cast<ChElementCableANCF>(beam)) {
        sectionshape = cableANCF->GetSection()->GetDrawShape();
    } else if (auto beamIGA = std::dynamic_pointer_cast<ChElementBeamIGA>(beam)) {
        sectionshape = beamIGA->GetSection()->GetDrawShape();
    } else if (auto beamTimoshenko = std::dynamic_pointer_cast<ChElementBeamTaperedTimoshenko>(beam)) {
        sectionshape = beamTimoshenko->GetTaperedSection()->GetSectionA()->GetDrawShape();
    } else if (auto beamTimoshenkoFPM = std::dynamic_pointer_cast<ChElementBeamTaperedTimoshenkoFPM>(beam)) {
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
        unsigned int n_section_pts = 0;
        for (unsigned int i = 0; i < sectionshape->GetNumLines(); ++i)
            n_section_pts += sectionshape->GetNumPoints(i);

        for (int in = 0; in < beam_resolution; ++in) {
            double eta = -1.0 + (2.0 * in / (beam_resolution - 1));

            // compute abs. pos and rot of section plane
            ChVector3d P;
            ChQuaternion<> msectionrot;
            beam->EvaluateSectionFrame(eta, P, msectionrot);

            ChVector3d vresult;
            ChVector3d vresultB;
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

            for (unsigned int il = 0; il < sectionshape->GetNumLines(); ++il) {
                std::vector<ChVector3d> msubline_pts(
                    sectionshape->GetNumPoints(il));  // suboptimal temp - better store&transform in place
                std::vector<ChVector3d> msubline_normals(
                    sectionshape->GetNumPoints(il));  // suboptimal temp - better store&transform in place

                // compute the point yz coords and yz normals in sectional frame
                sectionshape->GetPoints(il, msubline_pts);
                sectionshape->GetNormals(il, msubline_normals);

                // store rotated vertexes, colors, normals
                for (int is = 0; is < msubline_pts.size(); ++is) {
                    ChVector3d Rw = msectionrot.Rotate(msubline_pts[is]);
                    ChVector3d Rn = msectionrot.Rotate(msubline_normals[is]);
                    trianglemesh.GetCoordsVertices()[i_verts] = P + Rw;
                    ++i_verts;
                    trianglemesh.GetCoordsColors()[i_vcols] = mcol;
                    ++i_vcols;
                    if (smooth_faces) {
                        trianglemesh.GetCoordsNormals()[i_vnorms] = Rn;
                        ++i_vnorms;
                    }
                }
                // store face connectivity
                if (in > 0) {
                    ChVector3i ivert_offset(ivert_el, ivert_el, ivert_el);
                    ChVector3i islice_offset((in - 1) * n_section_pts, (in - 1) * n_section_pts,
                                             (in - 1) * n_section_pts);
                    for (size_t is = 0; is < msubline_pts.size() - 1; ++is) {
                        int ipa = int(is) + subline_stride;
                        int ipb = int(is + 1) + subline_stride;  // % n_section_pts; if wrapped - not needed here;
                        int ipaa = ipa + n_section_pts;
                        int ipbb = ipb + n_section_pts;

                        trianglemesh.GetIndicesVertexes()[i_triindex] =
                            ChVector3i(ipa, ipbb, ipaa) + islice_offset + ivert_offset;
                        if (smooth_faces) {
                            trianglemesh.GetIndicesNormals()[i_triindex] =
                                ChVector3i(ipa, ipbb, ipaa) + islice_offset + ivert_offset;
                        }
                        ++i_triindex;

                        trianglemesh.GetIndicesVertexes()[i_triindex] =
                            ChVector3i(ipa, ipb, ipbb) + islice_offset + ivert_offset;
                        if (smooth_faces) {
                            trianglemesh.GetIndicesNormals()[i_triindex] =
                                ChVector3i(ipa, ipb, ipbb) + islice_offset + ivert_offset;
                        }
                        ++i_triindex;
                    }
                }  // end if not first section

                subline_stride += int(msubline_pts.size());

            }  // end sublines loop

        }  // end sections loop
    }

    need_automatic_smoothing = false;  // normals are already computed in the best way
}

void ChVisualShapeFEA::UpdateBuffers_Shell(std::shared_ptr<fea::ChElementBase> element,
                                           ChTriangleMeshConnected& trianglemesh,
                                           unsigned int& i_verts,
                                           unsigned int& i_vnorms,
                                           unsigned int& i_vcols,
                                           unsigned int& i_triindex,
                                           bool& need_automatic_smoothing) {
    auto shell = std::static_pointer_cast<ChElementShell>(element);

    // Cache initial values
    ChVector3i ivert_offset(i_verts, i_verts, i_verts);
    ChVector3i inorm_offset(i_vnorms, i_vnorms, i_vnorms);

    if (shell->IsTriangleShell()) {
        // Triangular shell

        need_automatic_smoothing = false;

        int triangle_pt = 0;
        for (int iu = 0; iu < shell_resolution; ++iu) {
            for (int iv = 0; iv + iu < shell_resolution; ++iv) {
                double u = ((double)iu / (double)(shell_resolution - 1));
                double v = ((double)iv / (double)(shell_resolution - 1));

                ChVector3d P;
                shell->EvaluateSectionPoint(u, v, P);  // compute abs. pos and rot of section plane

                ChColor mcol(1, 1, 1);
                if (fem_data_type == DataType::NODE_SPEED_NORM) {
                    ChVector3d vresult;
                    shell->EvaluateSectionVelNorm(u, v, vresult);
                    mcol = ComputeFalseColor(vresult.Length());
                }

                trianglemesh.GetCoordsVertices()[i_verts] = P;
                ++i_verts;

                trianglemesh.GetCoordsColors()[i_vcols] = mcol;
                ++i_vcols;

                if (smooth_faces)
                    ++i_vnorms;

                if (iu < shell_resolution - 1) {
                    if (iv > 0) {
                        trianglemesh.GetIndicesVertexes()[i_triindex] =
                            ChVector3i(triangle_pt, triangle_pt - 1, triangle_pt + shell_resolution - iu - 1) +
                            ivert_offset;
                        trianglemesh.GetIndicesVertexes()[i_triindex + 1] =
                            ChVector3i(triangle_pt - 1, triangle_pt, triangle_pt + shell_resolution - iu - 1) +
                            ivert_offset;

                        if (smooth_faces) {
                            trianglemesh.GetIndicesNormals()[i_triindex] =
                                ChVector3i(triangle_pt, triangle_pt - 1, triangle_pt + shell_resolution - iu - 1) +
                                inorm_offset;
                            trianglemesh.GetIndicesNormals()[i_triindex + 1] =
                                ChVector3i(triangle_pt - 1, triangle_pt, triangle_pt + shell_resolution - iu - 1) +
                                inorm_offset;
                        }
                        ++i_triindex;
                        ++i_triindex;
                    }

                    if (iv > 1) {
                        trianglemesh.GetIndicesVertexes()[i_triindex] =
                            ivert_offset + ChVector3i(triangle_pt - 1, triangle_pt + shell_resolution - iu - 2,
                                                      triangle_pt + shell_resolution - iu - 1);
                        trianglemesh.GetIndicesVertexes()[i_triindex + 1] =
                            ivert_offset + ChVector3i(triangle_pt - 1, triangle_pt + shell_resolution - iu - 1,
                                                      triangle_pt + shell_resolution - iu - 2);

                        if (smooth_faces) {
                            trianglemesh.GetIndicesNormals()[i_triindex] =
                                inorm_offset + ChVector3i(triangle_pt - 1, triangle_pt + shell_resolution - iu - 2,
                                                          triangle_pt + shell_resolution - iu - 1);
                            trianglemesh.GetIndicesNormals()[i_triindex + 1] =
                                inorm_offset + ChVector3i(triangle_pt - 1, triangle_pt + shell_resolution - iu - 1,
                                                          triangle_pt + shell_resolution - iu - 2);
                        }

                        i_triindex += 2;
                    }
                }
                ++triangle_pt;

            }  // end for(iv)
        }      // end for(iu)

    } else {
        // Non-triangular shell

        for (int iu = 0; iu < shell_resolution; ++iu) {
            for (int iv = 0; iv < shell_resolution; ++iv) {
                double u = -1.0 + (2.0 * iu / (shell_resolution - 1));
                double v = -1.0 + (2.0 * iv / (shell_resolution - 1));

                ChVector3d P;
                shell->EvaluateSectionPoint(u, v, P);

                ChColor mcol(1, 1, 1);
                if (fem_data_type == DataType::NODE_SPEED_NORM) {
                    ChVector3d vresult;
                    shell->EvaluateSectionVelNorm(u, v, vresult);
                    mcol = ComputeFalseColor(vresult.Length());
                }

                trianglemesh.GetCoordsVertices()[i_verts] = P;
                ++i_verts;

                trianglemesh.GetCoordsColors()[i_vcols] = mcol;
                ++i_vcols;

                if (smooth_faces)
                    ++i_vnorms;

                if (iu > 0 && iv > 0) {
                    trianglemesh.GetIndicesVertexes()[i_triindex] =
                        ivert_offset + ChVector3i(iu * shell_resolution + iv, (iu - 1) * shell_resolution + iv,
                                                  iu * shell_resolution + iv - 1);
                    trianglemesh.GetIndicesVertexes()[i_triindex + 1] =
                        ivert_offset + ChVector3i(iu * shell_resolution + iv - 1, (iu - 1) * shell_resolution + iv,
                                                  (iu - 1) * shell_resolution + iv - 1);

                    if (smooth_faces) {
                        trianglemesh.GetIndicesNormals()[i_triindex] =
                            inorm_offset + ChVector3i(iu * shell_resolution + iv, (iu - 1) * shell_resolution + iv,
                                                      iu * shell_resolution + iv - 1);
                        trianglemesh.GetIndicesNormals()[i_triindex + 1] =
                            inorm_offset + ChVector3i(iu * shell_resolution + iv - 1, (iu - 1) * shell_resolution + iv,
                                                      (iu - 1) * shell_resolution + iv - 1);
                    }

                    i_triindex += 2;
                }

            }  // end for(iv)
        }      // end for(iu)
    }          // end triangular shell
}

void ChVisualShapeFEA::UpdateBuffers_LoadSurface(std::shared_ptr<ChMeshSurface> surface,
                                                 ChTriangleMeshConnected& trianglemesh,
                                                 unsigned int& i_verts,
                                                 unsigned int& i_vnorms,
                                                 unsigned int& i_vcols,
                                                 unsigned int& i_triindex,
                                                 bool& need_automatic_smoothing) {
    for (const auto& face : surface->GetFaces()) {
        if (auto face_tetra = std::dynamic_pointer_cast<ChTetrahedronFace>(face)) {
            auto node0 = std::static_pointer_cast<ChNodeFEAxyz>(face_tetra->GetNode(0));
            auto node1 = std::static_pointer_cast<ChNodeFEAxyz>(face_tetra->GetNode(1));
            auto node2 = std::static_pointer_cast<ChNodeFEAxyz>(face_tetra->GetNode(2));

            unsigned int ivert_el = i_verts;
            unsigned int inorm_el = i_vnorms;

            // vertexes
            ChVector3d p0 = node0->GetPos();
            ChVector3d p1 = node1->GetPos();
            ChVector3d p2 = node2->GetPos();

            // debug: offset 1 m to show it better..
            //    p0.x() +=1;
            //    p1.x() +=1;
            //    p2.x() +=1;

            trianglemesh.GetCoordsVertices()[i_verts] = p0;
            ++i_verts;
            trianglemesh.GetCoordsVertices()[i_verts] = p1;
            ++i_verts;
            trianglemesh.GetCoordsVertices()[i_verts] = p2;
            ++i_verts;

            // color
            trianglemesh.GetCoordsColors()[i_vcols] = meshcolor;
            ++i_vcols;
            trianglemesh.GetCoordsColors()[i_vcols] = meshcolor;
            ++i_vcols;
            trianglemesh.GetCoordsColors()[i_vcols] = meshcolor;
            ++i_vcols;

            // faces indexes
            ChVector3i ivert_offset(ivert_el, ivert_el, ivert_el);
            trianglemesh.GetIndicesVertexes()[i_triindex] = ChVector3i(0, 1, 2) + ivert_offset;
            ++i_triindex;

            // normals indices (if not defaulting to flat triangles)
            if (smooth_faces) {
                ChVector3i inorm_offset = ChVector3i(inorm_el, inorm_el, inorm_el);
                trianglemesh.GetIndicesNormals()[i_triindex - 4] = ChVector3i(0, 0, 0) + inorm_offset;
                i_vnorms += 1;
            }
        }
        //// TODO: other types of elements
    }
}

void ChVisualShapeFEA::UpdateBuffers_ContactSurfaceMesh(std::shared_ptr<ChContactSurface> surface,
                                                        ChTriangleMeshConnected& trianglemesh,
                                                        unsigned int& i_verts,
                                                        unsigned int& i_vnorms,
                                                        unsigned int& i_vcols,
                                                        unsigned int& i_triindex,
                                                        bool& need_automatic_smoothing) {
    auto msurface = std::static_pointer_cast<ChContactSurfaceMesh>(surface);

    for (const auto& face : msurface->GetTrianglesXYZ()) {
        unsigned int ivert_el = i_verts;
        unsigned int inorm_el = i_vnorms;

        // vertexes
        ChVector3d p0 = face->GetNode(0)->pos;
        ChVector3d p1 = face->GetNode(1)->pos;
        ChVector3d p2 = face->GetNode(2)->pos;

        trianglemesh.GetCoordsVertices()[i_verts] = p0;
        ++i_verts;
        trianglemesh.GetCoordsVertices()[i_verts] = p1;
        ++i_verts;
        trianglemesh.GetCoordsVertices()[i_verts] = p2;
        ++i_verts;

        // color
        trianglemesh.GetCoordsColors()[i_vcols] = meshcolor;
        ++i_vcols;
        trianglemesh.GetCoordsColors()[i_vcols] = meshcolor;
        ++i_vcols;
        trianglemesh.GetCoordsColors()[i_vcols] = meshcolor;
        ++i_vcols;

        // faces indexes
        ChVector3i ivert_offset(ivert_el, ivert_el, ivert_el);
        trianglemesh.GetIndicesVertexes()[i_triindex] = ChVector3i(0, 1, 2) + ivert_offset;
        ++i_triindex;

        // normals indices (if not defaulting to flat triangles)
        if (smooth_faces) {
            ChVector3i inorm_offset(inorm_el, inorm_el, inorm_el);
            trianglemesh.GetIndicesNormals()[i_triindex - 4] = ChVector3i(0, 0, 0) + inorm_offset;
            i_vnorms += 1;
        }
    }
}

void ChVisualShapeFEA::Update(ChPhysicsItem* updater, const ChFrame<>& frame) {
    if (!physics_item)
        return;

    assert(dynamic_cast<ChMesh*>(physics_item));
    auto FEMmesh = static_cast<ChMesh*>(physics_item);

    auto trianglemesh = m_trimesh_shape->GetMesh();

    size_t n_verts = 0;
    size_t n_vcols = 0;
    size_t n_vnorms = 0;
    size_t n_triangles = 0;

    // A - Count the needed vertexes and faces

    switch (fem_data_type) {
        case DataType::NONE:
            break;
        case DataType::LOADSURFACES:
            for (const auto& surface : FEMmesh->GetMeshSurfaces()) {
                for (const auto& face : surface->GetFaces()) {
                    if (std::dynamic_pointer_cast<ChTetrahedronFace>(face)) {
                        n_verts += 3;
                        n_vcols += 3;
                        n_vnorms += 1;
                        n_triangles += 1;
                    }
                    //// TODO: other types of element faces
                }
            }
            break;
        case DataType::CONTACTSURFACES:
            for (const auto& surface : FEMmesh->GetContactSurfaces()) {
                if (auto msurface = std::dynamic_pointer_cast<ChContactSurfaceMesh>(surface)) {
                    n_verts += 3 * msurface->GetTrianglesXYZ().size();
                    n_vcols += 3 * msurface->GetTrianglesXYZ().size();
                    n_vnorms += msurface->GetTrianglesXYZ().size();
                    n_triangles += msurface->GetTrianglesXYZ().size();
                }
                //// TODO: other types of contact surfaces
            }
            break;
        default:
            // Colormap drawing
            for (const auto& element : FEMmesh->GetElements()) {
                if (std::dynamic_pointer_cast<ChElementTetrahedron>(element)) {
                    n_verts += 4;
                    n_vcols += 4;
                    n_vnorms += 4;
                    n_triangles += 4;
                } else if (std::dynamic_pointer_cast<ChElementTetraCorot_4_P>(element)) {
                    n_verts += 4;
                    n_vcols += 4;
                    n_vnorms += 4;
                    n_triangles += 4;
                } else if (std::dynamic_pointer_cast<ChElementHexahedron>(element)) {
                    n_verts += 8;
                    n_vcols += 8;
                    n_vnorms += 24;
                    n_triangles += 12;
                } else if (auto beam = std::dynamic_pointer_cast<ChElementBeam>(element)) {
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
                        // TODO use ChBeamSection also in ANCF beam
                        sectionshape = chrono_types::make_shared<ChBeamSectionShapeRectangular>(
                            beam3243->GetThicknessY(), beam3243->GetThicknessZ());
                    } else if (auto beam3333 = std::dynamic_pointer_cast<ChElementBeamANCF_3333>(beam)) {
                        // TODO use ChBeamSection also in ANCF beam
                        sectionshape = chrono_types::make_shared<ChBeamSectionShapeRectangular>(
                            beam3333->GetThicknessY(), beam3333->GetThicknessZ());
                    }
                    if (sectionshape) {
                        for (unsigned int il = 0; il < sectionshape->GetNumLines(); ++il) {
                            n_verts += sectionshape->GetNumPoints(il) * beam_resolution;
                            n_vcols += sectionshape->GetNumPoints(il) * beam_resolution;
                            n_vnorms += sectionshape->GetNumPoints(il) * beam_resolution;
                            n_triangles += 2 * (sectionshape->GetNumPoints(il) - 1) * (beam_resolution - 1);
                        }
                    }
                } else if (auto shell = std::dynamic_pointer_cast<ChElementShell>(element)) {
                    if (shell->IsTriangleShell()) {
                        for (int idp = 1; idp <= shell_resolution; ++idp) {
                            n_verts += idp;
                            n_vcols += idp;
                            n_vnorms += idp;
                        }
                        n_triangles += 2 * (shell_resolution - 1) * (shell_resolution - 1);
                    } else {
                        n_verts += shell_resolution * shell_resolution;
                        n_vcols += shell_resolution * shell_resolution;
                        n_vnorms += shell_resolution * shell_resolution;
                        n_triangles += 2 * (shell_resolution - 1) * (shell_resolution - 1);
                    }
                }
                //// TODO: other types of elements
            }
            break;
    }

    // B - resize mesh buffers if needed

    if (trianglemesh->GetCoordsVertices().size() != n_verts)
        trianglemesh->GetCoordsVertices().resize(n_verts);
    if (trianglemesh->GetCoordsColors().size() != n_vcols)
        trianglemesh->GetCoordsColors().resize(n_vcols);
    if (trianglemesh->GetIndicesVertexes().size() != n_triangles)
        trianglemesh->GetIndicesVertexes().resize(n_triangles);

    if (smooth_faces) {
        if (trianglemesh->GetCoordsNormals().size() != n_vnorms)
            trianglemesh->GetCoordsNormals().resize(n_vnorms);
        if (trianglemesh->GetIndicesNormals().size() != n_triangles)
            trianglemesh->GetIndicesNormals().resize(n_triangles);
        if (normal_accumulators.size() != n_vnorms)
            normal_accumulators.resize(n_vnorms);

        TriangleNormalsReset(trianglemesh->GetCoordsNormals(), normal_accumulators);
    }

    // C - update mesh buffers

    unsigned int i_verts = 0;
    unsigned int i_vcols = 0;
    unsigned int i_vnorms = 0;
    unsigned int i_triindex = 0;
    bool need_automatic_smoothing = smooth_faces;

    switch (fem_data_type) {
        case DataType::NONE:
            break;
        case DataType::LOADSURFACES:
            for (const auto& surface : FEMmesh->GetMeshSurfaces()) {
                UpdateBuffers_LoadSurface(surface, *trianglemesh, i_verts, i_vnorms, i_vcols, i_triindex,
                                          need_automatic_smoothing);
            }
            break;
        case DataType::CONTACTSURFACES:
            for (const auto& surface : FEMmesh->GetContactSurfaces()) {
                if (std::dynamic_pointer_cast<ChContactSurfaceMesh>(surface))
                    UpdateBuffers_ContactSurfaceMesh(surface, *trianglemesh, i_verts, i_vnorms, i_vcols, i_triindex,
                                                     need_automatic_smoothing);
                //// TODO: other types of contact surfaces
            }
            break;
        default:
            // Colormap drawing
            for (const auto& element : FEMmesh->GetElements()) {
                if (std::dynamic_pointer_cast<ChElementTetrahedron>(element))
                    UpdateBuffers_Tetrahedron(element, *trianglemesh, i_verts, i_vnorms, i_vcols, i_triindex,
                                              need_automatic_smoothing);
                else if (std::dynamic_pointer_cast<ChElementTetraCorot_4_P>(element))
                    UpdateBuffers_Tetra_4_P(element, *trianglemesh, i_verts, i_vnorms, i_vcols, i_triindex,
                                            need_automatic_smoothing);
                else if (std::dynamic_pointer_cast<ChElementHexahedron>(element))
                    UpdateBuffers_Hex(element, *trianglemesh, i_verts, i_vnorms, i_vcols, i_triindex,
                                      need_automatic_smoothing);
                else if (std::dynamic_pointer_cast<ChElementBeam>(element))
                    UpdateBuffers_Beam(element, *trianglemesh, i_verts, i_vnorms, i_vcols, i_triindex,
                                       need_automatic_smoothing);
                else if (std::dynamic_pointer_cast<ChElementShell>(element))
                    UpdateBuffers_Shell(element, *trianglemesh, i_verts, i_vnorms, i_vcols, i_triindex,
                                        need_automatic_smoothing);
                //// TODO: other types of elements
            }
            break;
    }

    if (need_automatic_smoothing) {
        for (unsigned int itri = 0; itri < trianglemesh->GetIndicesVertexes().size(); ++itri)
            TriangleNormalsCompute(trianglemesh->GetIndicesNormals()[itri], trianglemesh->GetIndicesVertexes()[itri],
                                   trianglemesh->GetCoordsVertices(), trianglemesh->GetCoordsNormals(),
                                   normal_accumulators);

        TriangleNormalsSmooth(trianglemesh->GetCoordsNormals(), normal_accumulators);
    }

    // other flags
    m_trimesh_shape->SetWireframe(wireframe);
    m_trimesh_shape->SetBackfaceCull(backface_cull);

    // D - GLYPHS

    m_glyphs_shape->Reserve(0);  // unoptimal, should reuse buffers as much as possible
    m_glyphs_shape->SetGlyphsSize(symbols_thickness);
    m_glyphs_shape->SetZbufferHide(zbuffer_hide);

    switch (fem_glyph) {
        case GlyphType::NONE:
            break;
        case GlyphType::NODE_DOT_POS:
            m_glyphs_shape->SetDrawMode(ChGlyphs::GLYPH_POINT);
            for (unsigned int inode = 0; inode < FEMmesh->GetNumNodes(); ++inode) {
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
            break;
        case GlyphType::NODE_CSYS:
            m_glyphs_shape->SetDrawMode(ChGlyphs::GLYPH_COORDSYS);
            for (unsigned int inode = 0; inode < FEMmesh->GetNumNodes(); ++inode) {
                if (auto mynode = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(FEMmesh->GetNode(inode))) {
                    m_glyphs_shape->SetGlyphCoordsys(inode, mynode->Frame().GetCoordsys());
                }
                // else if (auto mynode = std::dynamic_pointer_cast<ChNodeFEAxyzD>(FEMmesh->GetNode(inode))) {
                //	m_glyphs_shape->SetGlyphVector(inode, mynode->GetPos(), mynode->GetSlope1() * symbols_scale,
                // symbolscolor );
                //}
            }
            break;
        case GlyphType::NODE_VECT_SPEED:
            m_glyphs_shape->SetDrawMode(ChGlyphs::GLYPH_VECTOR);
            for (unsigned int inode = 0; inode < FEMmesh->GetNumNodes(); ++inode)
                if (auto mynode = std::dynamic_pointer_cast<ChNodeFEAxyz>(FEMmesh->GetNode(inode))) {
                    m_glyphs_shape->SetGlyphVector(inode, mynode->GetPos(), mynode->GetPosDt() * symbols_scale,
                                                   symbolscolor);
                }
            break;
        case GlyphType::NODE_VECT_ACCEL:
            m_glyphs_shape->SetDrawMode(ChGlyphs::GLYPH_VECTOR);
            for (unsigned int inode = 0; inode < FEMmesh->GetNumNodes(); ++inode)
                if (auto mynode = std::dynamic_pointer_cast<ChNodeFEAxyz>(FEMmesh->GetNode(inode))) {
                    m_glyphs_shape->SetGlyphVector(inode, mynode->GetPos(), mynode->GetPosDt2() * symbols_scale,
                                                   symbolscolor);
                }
            break;
        case GlyphType::ELEM_VECT_DP:
            m_glyphs_shape->SetDrawMode(ChGlyphs::GLYPH_VECTOR);
            for (unsigned int iel = 0; iel < FEMmesh->GetNumElements(); ++iel)
                if (auto myelement = std::dynamic_pointer_cast<ChElementTetraCorot_4_P>(FEMmesh->GetElement(iel))) {
                    ChVector3d mvP(myelement->GetPgradient());
                    auto n0 = std::static_pointer_cast<ChNodeFEAxyzP>(myelement->GetNode(0));
                    auto n1 = std::static_pointer_cast<ChNodeFEAxyzP>(myelement->GetNode(1));
                    auto n2 = std::static_pointer_cast<ChNodeFEAxyzP>(myelement->GetNode(2));
                    auto n3 = std::static_pointer_cast<ChNodeFEAxyzP>(myelement->GetNode(3));
                    ChVector3d mPoint = (n0->GetPos() + n1->GetPos() + n2->GetPos() + n3->GetPos()) *
                                        0.25;  // to do: better placement in Gauss point
                    m_glyphs_shape->SetGlyphVector(iel, mPoint, mvP * symbols_scale, symbolscolor);
                }
            break;
        case GlyphType::ELEM_TENS_STRAIN:
            m_glyphs_shape->SetDrawMode(ChGlyphs::GLYPH_VECTOR);
            for (unsigned int iel = 0, nglyvect = 0; iel < FEMmesh->GetNumElements(); ++iel)
                if (auto myelement = std::dynamic_pointer_cast<ChElementTetraCorot_4>(FEMmesh->GetElement(iel))) {
                    ChStrainTensor<> mstrain = myelement->GetStrain();
                    // mstrain.Rotate(myelement->Rotation());
                    double e1, e2, e3;
                    ChVector3d v1, v2, v3;
                    mstrain.ComputePrincipalStrains(e1, e2, e3);
                    mstrain.ComputePrincipalStrainsDirections(e1, e2, e3, v1, v2, v3);
                    v1.Normalize();
                    v2.Normalize();
                    v3.Normalize();
                    auto n0 = std::static_pointer_cast<ChNodeFEAxyz>(myelement->GetNode(0));
                    auto n1 = std::static_pointer_cast<ChNodeFEAxyz>(myelement->GetNode(1));
                    auto n2 = std::static_pointer_cast<ChNodeFEAxyz>(myelement->GetNode(2));
                    auto n3 = std::static_pointer_cast<ChNodeFEAxyz>(myelement->GetNode(3));
                    //// TODO: better placement in Gauss point
                    ChVector3d mPoint = (n0->GetPos() + n1->GetPos() + n2->GetPos() + n3->GetPos()) * 0.25;
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
            break;
        case GlyphType::ELEM_TENS_STRESS:
            m_glyphs_shape->SetDrawMode(ChGlyphs::GLYPH_VECTOR);
            for (unsigned int iel = 0, nglyvect = 0; iel < FEMmesh->GetNumElements(); ++iel)
                if (auto myelement = std::dynamic_pointer_cast<ChElementTetraCorot_4>(FEMmesh->GetElement(iel))) {
                    ChStressTensor<> mstress = myelement->GetStress();
                    mstress.Rotate(myelement->Rotation());
                    double e1, e2, e3;
                    ChVector3d v1, v2, v3;
                    mstress.ComputePrincipalStresses(e1, e2, e3);
                    mstress.ComputePrincipalStressesDirections(e1, e2, e3, v1, v2, v3);
                    v1.Normalize();
                    v2.Normalize();
                    v3.Normalize();
                    auto n0 = std::static_pointer_cast<ChNodeFEAxyz>(myelement->GetNode(0));
                    auto n1 = std::static_pointer_cast<ChNodeFEAxyz>(myelement->GetNode(1));
                    auto n2 = std::static_pointer_cast<ChNodeFEAxyz>(myelement->GetNode(2));
                    auto n3 = std::static_pointer_cast<ChNodeFEAxyz>(myelement->GetNode(3));
                    //// TODO: better placement in Gauss point
                    ChVector3d mPoint = (n0->GetPos() + n1->GetPos() + n2->GetPos() + n3->GetPos()) * 0.25;
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
            break;
    }

    //// TEST
    if (false)
        for (unsigned int iel = 0; iel < FEMmesh->GetNumElements(); ++iel) {
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
                            ChCoordsys<>(myshell->EvaluateGP(igp), myshell->T_i[igp].GetQuaternion()));
                    }
                }
                // gauss point weights
                if (false) {
                    m_glyphs_shape->SetDrawMode(ChGlyphs::GLYPH_VECTOR);
                    for (int igp = 0; igp < 4; ++igp) {
                        m_glyphs_shape->GetNumberOfGlyphs();
                        m_glyphs_shape->SetGlyphVector((unsigned int)m_glyphs_shape->GetNumberOfGlyphs(),
                                                       myshell->EvaluateGP(igp),
                                                       ChVector3d(0, myshell->alpha_i[igp] * 4, 0));
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
            }
        }
}

}  // end namespace chrono
