// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2022 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Rainer Gericke, Radu Serban
// =============================================================================
//
// Terrain defined by an OpenCRG file (http://opencrg.org)
//
// OpenCRG® (up to v1.1.2) was managed by
//	VIRES Simulationstechnologie GmbH
//
// OpenCRG® (>= v1.2) is now managed by
// ASAM e.V.
// https://www.asam.net/standards/detail/opencrg/
//
// v1.1.2 is still available. Both versions work with chrono.
// =============================================================================
//
// Limits:	Options and modifiers are ignored
//
// Roads start at {0,0,0}, first driving direction is {1,0,0}
//
//==============================================================================

#include <algorithm>

#include "chrono/assets/ChVisualShapePath.h"
#include "chrono/physics/ChBodyEasy.h"

#include "chrono/utils/ChFilters.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono/geometry/ChLineBezier.h"
#include "chrono/geometry/ChLineSegment.h"

#include "chrono/assets/ChVisualShapeLine.h"
#include "chrono/assets/ChVisualShapePath.h"

#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualShapeModelFile.h"


#include "chrono_vehicle/ChWorldFrame.h"
#include "chrono_vehicle/terrain/CRGTerrain.h"

#include "chrono_thirdparty/filesystem/path.h"

extern "C" {
#include "crgBaseLib.h"
}

namespace chrono {
namespace vehicle {

CRGTerrain::CRGTerrain(ChSystem* system)
    : m_use_vis_mesh(true),
      m_use_diffuseTexture(false),
      m_post_distance(0.0),
      m_friction(0.8f),
      m_dataSetId(0),
      m_simplified_mesh(false),
      m_cpId(0),
      m_isClosed(false) {
    m_ground = chrono_types::make_shared<ChBody>();
    m_ground->SetName("ground");
    m_ground->SetPos(ChVector3d(0, 0, 0));
    m_ground->SetFixed(true);
    m_ground->EnableCollision(false);
    system->Add(m_ground);

    crgMsgSetLevel(dCrgMsgLevelNone);
}

CRGTerrain::~CRGTerrain() {
    crgContactPointDelete(m_cpId);
    crgDataSetRelease(m_dataSetId);
    crgMemRelease();
}

void CRGTerrain::EnableVerbose(bool val) {
    if (val)
        crgMsgSetLevel(dCrgMsgLevelInfo);
    else
        crgMsgSetLevel(dCrgMsgLevelNone);
}

void CRGTerrain::SetRoadDiffuseTextureFile(std::string texFile) {
    m_diffuse_texture_filename = GetChronoDataFile(texFile);
    filesystem::path path(m_diffuse_texture_filename);
    if (path.is_file() && path.exists()) {
        m_use_diffuseTexture = true;
        ////std::cout << "Diffuse Texture file " << m_diffuse_texture_filename << " can be used.\n";
    }
}

void CRGTerrain::SetRoadNormalTextureFile(std::string texFile) {
    m_normal_texture_filename = GetChronoDataFile(texFile);
    filesystem::path path(m_normal_texture_filename);
    if (path.is_file() && path.exists()) {
        m_use_normalTexture = true;
        ////std::cout << "Normal Texture file " << m_normal_texture_filename << " can be used.\n";
    }
}

void CRGTerrain::SetRoadRoughnessTextureFile(std::string texFile) {
    m_rough_texture_filename = GetChronoDataFile(texFile);
    filesystem::path path(m_rough_texture_filename);
    if (path.is_file() && path.exists()) {
        m_use_roughTexture = true;
        ////std::cout << "Roughness Texture file " << m_rough_texture_filename << " can be used.\n";
    }
}

void CRGTerrain::Initialize(const std::string& crg_file) {
    m_v.clear();

    // Read the crg-file
    m_dataSetId = crgLoaderReadFile(crg_file.c_str());
    if (m_dataSetId <= 0) {
        std::cerr << "CRGTerrain::CRGTTerrain(): error reading data file " << crg_file << std::endl;
        return;
    }
    // Check it
    if (!crgCheck(m_dataSetId)) {
        std::cerr << "CRGTerrain::CRGTTerrain(): could not validate crg data." << std::endl;
        return;
    }
    // Create a contact point
    m_cpId = crgContactPointCreate(m_dataSetId);
    if (m_cpId < 0) {
        // crgMsgPrint( dCrgMsgLevelFatal, "main: could not create contact point.\n" );
        std::cerr << "CRGTerrain::CRGTTerrain(): could not create contact point!" << std::endl;
        return;
    }

    int urange_ok = crgDataSetGetURange(m_dataSetId, &m_ubeg, &m_uend);
    if (urange_ok != 1) {
        std::cerr << "CRGTerrain::CRGTTerrain(): error with urange in data file " << crg_file << std::endl;
        return;
    }
    int vrange_ok = crgDataSetGetVRange(m_dataSetId, &m_vbeg, &m_vend);
    if (vrange_ok != 1) {
        std::cerr << "CRGTerrain::CRGTTerrain(): error with vrange in data file " << crg_file << std::endl;
        return;
    }
    int incr_ok = crgDataSetGetIncrements(m_dataSetId, &m_uinc, &m_vinc);
    if (incr_ok != 1) {
        std::cerr << "CRGTerrain::CRGTTerrain(): could not get increments from data file " << crg_file << std::endl;
        return;
    }
    // for left/right excitation surface, also good for flat surfaces
    if (m_simplified_mesh && m_use_vis_mesh) {
        m_v.push_back(m_vbeg);
        m_v.push_back(-0.05);
        m_v.push_back(0.0);
        m_v.push_back(0.05);
        m_v.push_back(m_vend);
    }

    int uIsClosed;
    double uCloseMin, uCloseMax;
    int cl_ok = crgDataSetGetUtilityDataClosedTrack(m_dataSetId, &uIsClosed, &uCloseMin, &uCloseMax);
    if (cl_ok != 1) {
        std::cerr << "CRGTerrain::CRGTerrain(): could not get closedness from data file " << crg_file << std::endl;
        return;
    } else {
        m_isClosed = (uIsClosed != 0);
    }

    // Set mesh and curve names based on name of CRG input file.
    auto stem = filesystem::path(crg_file).stem();
    m_mesh_name = stem + "_mesh";
    m_curve_left_name = stem + "_left";
    m_curve_right_name = stem + "_right";

    GenerateMesh();
    GenerateCurves();
    m_ground->AddVisualModel(chrono_types::make_shared<ChVisualModel>());
    if (m_use_vis_mesh) {
        SetupMeshGraphics();
    } else {
        SetupLineGraphics();
    }
    SetRoadsidePosts();
}

void CRGTerrain::SetRoadsidePosts() {
    if (m_post_distance < 10.0)
        return;
    double ustep = m_post_distance;
    int nu = GetLength() / ustep;
    double vl = GetWidth() / 2.0 + 0.1, vr = -GetWidth() / 2.0 - 0.1;
    for (int iu = 0; iu < nu; iu++) {
        double u = ustep * double(iu);
        double xl, yl, zl;
        double xr, yr, zr;
        if (crgEvaluv2xy(m_cpId, u, vl, &xl, &yl) != 1) {
            std::cerr << "could not get xl,yl in CRGTerrain::SetRoadsidePosts" << std::endl;
        }
        if (crgEvaluv2xy(m_cpId, u, vr, &xr, &yr) != 1) {
            std::cerr << "could not get r,yr in CRGTerrain::SetRoadsidePosts" << std::endl;
        }
        if (crgEvaluv2z(m_cpId, u, vl, &zl) != 1) {
            std::cerr << "could not get zl in CRGTerrain::SetRoadsidePosts" << std::endl;
        }
        if (crgEvaluv2z(m_cpId, u, vr, &zr) != 1) {
            std::cerr << "could not get zr in CRGTerrain::SetRoadsidePosts" << std::endl;
        }
        if(iu == 0) {
            auto shape_l = chrono_types::make_shared<ChVisualShapeCylinder>(0.07, 1.0);
            shape_l->SetTexture(GetChronoDataFile("textures/greenwhite.png"), 2.0, 2.0);
            m_ground->AddVisualShape(shape_l, ChFrame<>(ChVector3d(xl, yl, zl + 0.5), QUNIT));
            
            auto shape_r = chrono_types::make_shared<ChVisualShapeCylinder>(0.07, 1.0);
            shape_r->SetTexture(GetChronoDataFile("textures/greenwhite.png"), 2.0, 2.0);
            m_ground->AddVisualShape(shape_r, ChFrame<>(ChVector3d(xr, yr, zr + 0.5), QUNIT));
        } else {
            auto shape_l = chrono_types::make_shared<ChVisualShapeCylinder>(0.07, 1.0);
            shape_l->SetTexture(GetChronoDataFile("textures/redwhite.png"), 2.0, 2.0);
            m_ground->AddVisualShape(shape_l, ChFrame<>(ChVector3d(xl, yl, zl + 0.5), QUNIT));
            
            auto shape_r = chrono_types::make_shared<ChVisualShapeCylinder>(0.07, 1.0);
            shape_r->SetTexture(GetChronoDataFile("textures/redwhite.png"), 2.0, 2.0);
            m_ground->AddVisualShape(shape_r, ChFrame<>(ChVector3d(xr, yr, zr + 0.5), QUNIT));
        }
    }
}

double CRGTerrain::GetStartHeading() {
    double phi, curv;

    if (crgEvaluv2pk(m_cpId, 0, 0, &phi, &curv) != 1) {
        std::cerr << "CRGTerrain::GetStartHeading(): error during uv -> pk coordinate transformation" << std::endl;
        return 0;
    }

    return phi;
}

ChCoordsys<> CRGTerrain::GetStartPosition() {
    double x, y, z;

    if (crgEvaluv2xy(m_cpId, 0, 0, &x, &y) != 1) {
        std::cerr << "CRGTerrain::GetStartPosition(): error during uv -> xy coordinate transformation" << std::endl;
        x = 0;
        y = 0;
    }

    if (crgEvaluv2z(m_cpId, 0, 0, &z) != 1) {
        std::cerr << "CRGTerrain::GetStartPosition(): error during uv -> z coordinate transformation" << std::endl;
        z = 0;
    }

    return ChCoordsys<>(ChVector3d(x, y, z), QuatFromAngleZ(GetStartHeading()));
}

double CRGTerrain::GetHeight(const ChVector3d& loc) const {
    ChVector3d loc_ISO = ChWorldFrame::ToISO(loc);
    double u, v, z;
    int uv_ok = crgEvalxy2uv(m_cpId, loc_ISO.x(), loc_ISO.y(), &u, &v);
    if (uv_ok != 1) {
        std::cerr << "CRGTerrain::GetHeight(): error during xy -> uv coordinate transformation" << std::endl;
        return 0;
    }

    // when leaving the road the vehicle should not fall into an abyss
    ChClampValue(u, m_ubeg, m_uend);
    ChClampValue(v, m_vbeg, m_vend);

    int z_ok = crgEvaluv2z(m_cpId, u, v, &z);
    if (z_ok != 1) {
        std::cerr << "CRGTerrain::GetHeight(): error during uv -> z coordinate transformation" << std::endl;
        return 0;
    }

    return z;
}

ChVector3d CRGTerrain::GetNormal(const ChVector3d& loc) const {
    ChVector3d loc_ISO = ChWorldFrame::ToISO(loc);
    // to avoid 'jumping' of the normal vector, we take this smoothing approach
    const double delta = 0.05;
    double z0, zfront, zleft;
    z0 = GetHeight(loc);
    zfront = GetHeight(ChWorldFrame::FromISO(loc_ISO + ChVector3d(delta, 0, 0)));
    zleft = GetHeight(ChWorldFrame::FromISO(loc_ISO + ChVector3d(0, delta, 0)));
    ChVector3d p0(loc_ISO.x(), loc_ISO.y(), z0);
    ChVector3d pfront(loc_ISO.x() + delta, loc_ISO.y(), zfront);
    ChVector3d pleft(loc_ISO.x(), loc_ISO.y() + delta, zleft);
    ChVector3d normal_ISO;
    ChVector3d r1, r2;
    r1 = pfront - p0;
    r2 = pleft - p0;
    normal_ISO = Vcross(r1, r2);
    if (normal_ISO.z() <= 0.0) {
        std::cerr << "Fatal: wrong surface normal!" << std::endl;
        throw std::runtime_error("Fatal: wrong surface normal!");
    }
    ChVector3d normal = ChWorldFrame::FromISO(normal_ISO);
    normal.Normalize();

    return normal;
}

float CRGTerrain::GetCoefficientFriction(const ChVector3d& loc) const {
    return m_friction_fun ? (*m_friction_fun)(loc) : m_friction;
}

std::shared_ptr<ChBezierCurve> CRGTerrain::GetRoadCenterLine() {
    std::vector<ChVector3d> pathpoints;

    // damp z oscillation for the path definition
    utils::ChRunningAverage avg(5);

    double dp = 3.0;
    size_t np = static_cast<size_t>(m_uend / dp);

    double du = (m_uend - m_ubeg) / double(np - 1);

    double vm = (m_vbeg + m_vend) / 2.0;

    for (size_t i = 0; i < np; i++) {
        double u = m_ubeg + double(i) * du;
        double xm, ym, zm;
        int xy_ok = crgEvaluv2xy(m_cpId, u, vm, &xm, &ym);
        if (xy_ok != 1) {
            std::cerr << "CRGTerrain::SetupGraphics(): error during uv -> xy coordinate transformation" << std::endl;
        }
        int z_ok = crgEvaluv2z(m_cpId, u, vm, &zm);
        if (z_ok != 1) {
            std::cerr << "CRGTerrain::SetupGraphics(): error during uv -> z coordinate transformation" << std::endl;
        }
        ////zm = avg.Add(zm);
        pathpoints.push_back(ChWorldFrame::FromISO(ChVector3d(xm, ym, zm + 0.2)));
    }

    if (m_isClosed) {
        pathpoints.back() = pathpoints[0];
    }

    return chrono_types::make_shared<ChBezierCurve>(pathpoints, m_isClosed);
}

void CRGTerrain::GenerateCurves() {
    double dp = 3.0;
    size_t np = static_cast<size_t>(m_uend / dp);
    double du = (m_uend - m_ubeg) / double(np - 1);
    std::vector<ChVector3d> pl, pr;

    for (size_t i = 0; i < np; i++) {
        double u = m_ubeg + i * du;
        double xl, yl, zl;
        double xr, yr, zr;

        int xy_ok = crgEvaluv2xy(m_cpId, u, m_vbeg, &xl, &yl);
        if (xy_ok != 1) {
            std::cerr << "CRGTerrain::SetupGraphics(): error during uv -> xy coordinate transformation" << std::endl;
        }
        xy_ok = crgEvaluv2xy(m_cpId, u, m_vend, &xr, &yr);
        if (xy_ok != 1) {
            std::cerr << "CRGTerrain::SetupGraphics(): error during uv -> xy coordinate transformation" << std::endl;
        }
        int z_ok = crgEvaluv2z(m_cpId, u, m_vbeg, &zl);
        if (z_ok != 1) {
            std::cerr << "CRGTerrain::SetupGraphics(): error during uv -> z coordinate transformation" << std::endl;
        }
        z_ok = crgEvaluv2z(m_cpId, u, m_vend, &zr);
        if (z_ok != 1) {
            std::cerr << "CRGTerrain::SetupGraphics(): error during uv -> z coordinate transformation" << std::endl;
        }
        pl.push_back(ChWorldFrame::FromISO(ChVector3d(xl, yl, zl)));
        pr.push_back(ChWorldFrame::FromISO(ChVector3d(xr, yr, zr)));
    }

    if (m_isClosed) {
        pl.back() = pl[0];
        pr.back() = pr[0];
    }

    // Create the two road boundary Bezier curves
    m_road_left = chrono_types::make_shared<ChBezierCurve>(pl);
    m_road_right = chrono_types::make_shared<ChBezierCurve>(pr);
}

void CRGTerrain::SetupLineGraphics() {
    auto mat = chrono_types::make_shared<ChVisualMaterial>();
    mat->SetDiffuseColor({0.3f, 0.3f, 0.6f});

    auto np = m_road_left->GetNumPoints();
    unsigned int num_render_points = std::max<unsigned int>(static_cast<unsigned int>(3 * np), 400);

    auto bezier_line_left = chrono_types::make_shared<ChLineBezier>(m_road_left);
    auto bezier_asset_left = chrono_types::make_shared<ChVisualShapeLine>();
    bezier_asset_left->SetLineGeometry(bezier_line_left);
    bezier_asset_left->SetNumRenderPoints(num_render_points);
    bezier_asset_left->SetName(m_curve_left_name);
    bezier_asset_left->AddMaterial(mat);
    m_ground->AddVisualShape(bezier_asset_left);

    auto bezier_line_right = chrono_types::make_shared<ChLineBezier>(m_road_right);
    auto bezier_asset_right = chrono_types::make_shared<ChVisualShapeLine>();
    bezier_asset_right->SetLineGeometry(bezier_line_right);
    bezier_asset_right->SetNumRenderPoints(num_render_points);
    bezier_asset_right->SetName(m_curve_right_name);
    bezier_asset_right->AddMaterial(mat);
    m_ground->AddVisualShape(bezier_asset_right);
}

void CRGTerrain::GenerateMesh() {
    m_mesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    auto& coords = m_mesh->GetCoordsVertices();
    auto& indices = m_mesh->GetIndicesVertexes();
    auto& coords_uv = m_mesh->GetCoordsUV();
    auto& indices_uv = m_mesh->GetIndicesUV();

    int nu = static_cast<int>((m_uend - m_ubeg) / m_uinc) + 1;
    int nv;

    std::vector<double> x0, y0, z0;
    std::vector<double> tu0, tv0;
    // Define the vertices
    if (m_simplified_mesh) {
        // we use m_v[] for a simpler graphics
        nv = static_cast<int>(m_v.size());
        for (auto i = 0; i < nu; i++) {
            double u = m_ubeg + m_uinc * double(i);
            double tu = (u - m_ubeg) / (m_uend - m_ubeg);
            for (auto j = 0; j < nv; j++) {
                double x, y, z, v;
                v = m_v[j];
                double tv = (v - m_vbeg) / (m_vend - m_vbeg);
                int uv_ok = crgEvaluv2xy(m_cpId, u, v, &x, &y);
                if (uv_ok != 1) {
                    std::cerr << "Error during uv -> xy coordinate transformation in crg file" << std::endl;
                    throw std::runtime_error("Error during uv -> xy coordinate transformation in crg file");
                }
                int z_ok = crgEvaluv2z(m_cpId, u, v, &z);
                if (z_ok != 1) {
                    std::cerr << "Error during uv -> z coordinate transformation in crg file" << std::endl;
                    throw std::runtime_error("Error during uv -> z coordinate transformation in crg file");
                }
                if (i == 0) {
                    x0.push_back(x);
                    y0.push_back(y);
                    z0.push_back(z);
                    tu0.push_back(tu);
                    tv0.push_back(tv);
                }
                if (i == nu - 1 && m_isClosed) {
                    coords.push_back(ChWorldFrame::FromISO(ChVector3d(x0[j], y0[j], z0[j])));
                    coords_uv.push_back(ChVector2d(tu0[j], tv0[j]));
                } else {
                    coords.push_back(ChWorldFrame::FromISO(ChVector3d(x, y, z)));
                    coords_uv.push_back(ChVector2d(tu, tv));
                }
            }
        }
    } else {
        // v is equidistant, we use m_vinc
        nv = static_cast<int>((m_vend - m_vbeg) / m_vinc) + 1;
        for (auto i = 0; i < nu; i++) {
            double u = m_ubeg + m_uinc * double(i);
            double tu = (u - m_ubeg) / (m_uend - m_ubeg);
            for (auto j = 0; j < nv; j++) {
                double v = m_vbeg + m_vinc * double(j);
                double tv = (v - m_vbeg) / (m_vend - m_vbeg);
                double x, y, z;
                int uv_ok = crgEvaluv2xy(m_cpId, u, v, &x, &y);
                if (uv_ok != 1) {
                    std::cerr << "Error during uv -> xy coordinate transformation in crg file" << std::endl;
                    throw std::runtime_error("Error during uv -> xy coordinate transformation in crg file");
                }
                int z_ok = crgEvaluv2z(m_cpId, u, v, &z);
                if (z_ok != 1) {
                    std::cerr << "Error during uv -> z coordinate transformation in crg file" << std::endl;
                    throw std::runtime_error("Error during uv -> z coordinate transformation in crg file");
                }
                if (i == 0) {
                    x0.push_back(x);
                    y0.push_back(y);
                    z0.push_back(z);
                    tu0.push_back(tu);
                    tv0.push_back(tv);
                }
                if (i == nu - 1 && m_isClosed) {
                    coords.push_back(ChWorldFrame::FromISO(ChVector3d(x0[j], y0[j], z0[j])));
                    coords_uv.push_back(ChVector2d(tu0[j], tv0[j]));
                } else {
                    coords.push_back(ChWorldFrame::FromISO(ChVector3d(x, y, z)));
                    coords_uv.push_back(ChVector2d(tu, tv));
                }
            }
        }
    }

    // Define the faces
    for (int i = 0; i < nu - 1; i++) {
        int ofs = nv * i;
        for (int j = 0; j < nv - 1; j++) {
            indices.push_back(ChVector3i(j + ofs, j + nv + ofs, j + 1 + ofs));
            indices.push_back(ChVector3i(j + 1 + ofs, j + nv + ofs, j + 1 + nv + ofs));
            indices_uv.push_back(ChVector3i(j + ofs, j + nv + ofs, j + 1 + ofs));
            indices_uv.push_back(ChVector3i(j + 1 + ofs, j + nv + ofs, j + 1 + nv + ofs));
        }
    }
}

void CRGTerrain::SetupMeshGraphics() {
    if (m_use_diffuseTexture) {
        auto vmesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        vmesh->SetMesh(m_mesh);
        vmesh->SetName(m_mesh_name);
        auto material = chrono_types::make_shared<ChVisualMaterial>();
        material->SetDiffuseColor(ChColor(1.0f, 1.0f, 1.0f));
        material->SetAmbientColor(ChColor(1.0f, 1.0f, 1.0f));
        double scale_u = 0.5 * GetLength() / GetWidth();
        double scale_w = 1.0;
        material->SetTextureScale(scale_u, scale_w);
        if (m_use_diffuseTexture)
            material->SetKdTexture(m_diffuse_texture_filename);
        if (m_use_normalTexture)
            material->SetNormalMapTexture(m_normal_texture_filename);
        if (m_use_roughTexture)
            material->SetRoughnessTexture(m_rough_texture_filename);
        vmesh->SetMaterial(0, material);
        m_ground->AddVisualShape(vmesh);
    } else {
        auto vmesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        vmesh->SetMesh(m_mesh);
        vmesh->SetName(m_mesh_name);
        vmesh->SetColor(ChColor(1.0f, 1.0f, 1.0f));

        m_ground->AddVisualShape(vmesh);
    }
}

void CRGTerrain::ExportMeshWavefront(const std::string& out_dir) {
    std::vector<ChTriangleMeshConnected> meshes = {*m_mesh};
    ChTriangleMeshConnected::WriteWavefront(out_dir + "/" + m_mesh_name + ".obj", meshes);
}

void CRGTerrain::ExportMeshPovray(const std::string& out_dir) {
    utils::WriteMeshPovray(*m_mesh, m_mesh_name, out_dir, ChColor(1, 1, 1));
}

void CRGTerrain::ExportCurvesPovray(const std::string& out_dir) {
    if (m_use_vis_mesh)
        return;
    utils::WriteCurvePovray(*m_road_left, m_curve_left_name, out_dir, 0.04, ChColor(0.5f, 0.8f, 0.0f));
    utils::WriteCurvePovray(*m_road_right, m_curve_right_name, out_dir, 0.04, ChColor(0.5f, 0.8f, 0.0f));
}

}  // end namespace vehicle
}  // end namespace chrono
