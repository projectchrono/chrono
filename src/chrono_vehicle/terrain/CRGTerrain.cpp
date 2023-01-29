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

#include "chrono/core/ChLog.h"
#include "chrono/assets/ChPathShape.h"
#include "chrono/physics/ChBodyEasy.h"

#include "chrono/utils/ChFilters.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono/geometry/ChLineBezier.h"
#include "chrono/geometry/ChLineSegment.h"

#include "chrono/assets/ChLineShape.h"
#include "chrono/assets/ChPathShape.h"

#include "chrono/assets/ChCylinderShape.h"

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
      m_cpId(0),
      m_isClosed(false) {
    m_ground = std::shared_ptr<ChBody>(system->NewBody());
    m_ground->SetName("ground");
    m_ground->SetPos(ChVector<>(0, 0, 0));
    m_ground->SetBodyFixed(true);
    m_ground->SetCollide(false);
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
        GetLog() << "Diffuse Texture file " << m_diffuse_texture_filename << " can be used.\n";
    }
}

void CRGTerrain::SetRoadNormalTextureFile(std::string texFile) {
    m_normal_texture_filename = GetChronoDataFile(texFile);
    filesystem::path path(m_normal_texture_filename);
    if (path.is_file() && path.exists()) {
        m_use_normalTexture = true;
        GetLog() << "Normal Texture file " << m_normal_texture_filename << " can be used.\n";
    }
}

void CRGTerrain::Initialize(const std::string& crg_file) {
    m_v.clear();

    // Read the crg-file
    m_dataSetId = crgLoaderReadFile(crg_file.c_str());
    if (m_dataSetId <= 0) {
        GetLog() << "CRGTerrain::CRGTTerrain(): error reading data file " << crg_file << "\n";
        return;
    }
    // Check it
    if (!crgCheck(m_dataSetId)) {
        GetLog() << "CRGTerrain::CRGTTerrain(): could not validate crg data.\n";
        return;
    }
    // Create a contact point
    m_cpId = crgContactPointCreate(m_dataSetId);
    if (m_cpId < 0) {
        // crgMsgPrint( dCrgMsgLevelFatal, "main: could not create contact point.\n" );
        GetLog() << "CRGTerrain::CRGTTerrain(): could not create contact point!\n";
        return;
    }

    int urange_ok = crgDataSetGetURange(m_dataSetId, &m_ubeg, &m_uend);
    if (urange_ok != 1) {
        GetLog() << "CRGTerrain::CRGTTerrain(): error with urange in data file " << crg_file << "\n";
        return;
    }
    int vrange_ok = crgDataSetGetVRange(m_dataSetId, &m_vbeg, &m_vend);
    if (vrange_ok != 1) {
        GetLog() << "CRGTerrain::CRGTTerrain(): error with vrange in data file " << crg_file << "\n";
        return;
    }
    int incr_ok = crgDataSetGetIncrements(m_dataSetId, &m_uinc, &m_vinc);
    if (incr_ok != 1) {
        GetLog() << "CRGTerrain::CRGTTerrain(): could not get increments from data file " << crg_file << "\n";
        return;
    }
    // dirty hack:
    if (m_vinc <= 0.01 && m_use_vis_mesh) {
        m_v.push_back(m_vbeg);
        m_v.push_back(-0.05);
        m_v.push_back(0.0);
        m_v.push_back(0.05);
        m_v.push_back(m_vend);
        GetLog() << "\n";
        GetLog() << " Caution:\n";
        GetLog() << " Mesh seemes to be nonequidistant in v direction (vinc = " << m_vinc << "m).\n";
        GetLog() << " We use 5 distinct v values [";
        for (size_t i = 0; i < m_v.size(); i++) {
            GetLog() << " " << m_v[i];
        }
        GetLog() << " ] as fallback values.\n";
        GetLog() << " If it does not work for you, only use v-eqidistant crg files with vinc > 0.01 m.\n\n";
    }

    int uIsClosed;
    double uCloseMin, uCloseMax;
    int cl_ok = crgDataSetGetUtilityDataClosedTrack(m_dataSetId, &uIsClosed, &uCloseMin, &uCloseMax);
    if (cl_ok != 1) {
        GetLog() << "CRGTerrain::CRGTerrain(): could not get closedness from data file " << crg_file << "\n";
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
            GetLog() << "could not get xl,yl in " << __func__ << "\n";
        }
        if (crgEvaluv2xy(m_cpId, u, vr, &xr, &yr) != 1) {
            GetLog() << "could not get r,yr in " << __func__ << "\n";
        }
        if (crgEvaluv2z(m_cpId, u, vl, &zl) != 1) {
            GetLog() << "could not get zl in " << __func__ << "\n";
        }
        if (crgEvaluv2z(m_cpId, u, vr, &zr) != 1) {
            GetLog() << "could not get zr in " << __func__ << "\n";
        }
        geometry::ChCylinder cyl;
        cyl.p1 = ChVector<>(0, 0, 0);
        if (iu == 0)
            cyl.p2 = ChVector<>(0, 0, 2.0);
        else
            cyl.p2 = ChVector<>(0, 0, 1.0);
        cyl.rad = 0.07;
        auto shape_l = chrono_types::make_shared<ChCylinderShape>(cyl);
        ChFrame<> frame_l(ChVector<>(xl, yl, zl), QUNIT);
        shape_l->SetTexture(GetChronoDataFile("textures/redwhite.png"), 2.0, 2.0);
        m_ground->AddVisualShape(shape_l, frame_l);
        auto shape_r = chrono_types::make_shared<ChCylinderShape>(cyl);
        ChFrame<> frame_r(ChVector<>(xr, yr, zr), QUNIT);
        shape_r->SetTexture(GetChronoDataFile("textures/redwhite.png"), 2.0, 2.0);
        m_ground->AddVisualShape(shape_r, frame_r);
    }
}

double CRGTerrain::GetStartHeading() {
    double phi, curv;

    if (crgEvaluv2pk(m_cpId, 0, 0, &phi, &curv) != 1) {
        GetLog() << "CRGTerrain::GetStartHeading(): error during uv -> pk coordinate transformation\n";
        phi = 0;
    }

    return phi;
}

ChCoordsys<> CRGTerrain::GetStartPosition() {
    double x, y, z;

    if (crgEvaluv2xy(m_cpId, 0, 0, &x, &y) != 1) {
        GetLog() << "CRGTerrain::GetStartPosition(): error during uv -> xy coordinate transformation\n";
        x = 0;
        y = 0;
    }

    if (crgEvaluv2z(m_cpId, 0, 0, &z) != 1) {
        GetLog() << "CRGTerrain::GetStartPosition(): error during uv -> z coordinate transformation\n";
        z = 0;
    }

    return ChCoordsys<>(ChVector<>(x, y, z), Q_from_AngZ(GetStartHeading()));
}

double CRGTerrain::GetHeight(const ChVector<>& loc) const {
    ChVector<> loc_ISO = ChWorldFrame::ToISO(loc);
    double u, v, z;
    int uv_ok = crgEvalxy2uv(m_cpId, loc_ISO.x(), loc_ISO.y(), &u, &v);
    if (uv_ok != 1) {
        GetLog() << "CRGTerrain::GetHeight(): error during xy -> uv coordinate transformation\n";
    }

    // when leaving the road the vehicle should not fall into an abyss
    ChClampValue(u, m_ubeg, m_uend);
    ChClampValue(v, m_vbeg, m_vend);

    int z_ok = crgEvaluv2z(m_cpId, u, v, &z);
    if (z_ok != 1) {
        GetLog() << "CRGTerrain::GetHeight(): error during uv -> z coordinate transformation\n";
    }

    return z;
}

ChVector<> CRGTerrain::GetNormal(const ChVector<>& loc) const {
    ChVector<> loc_ISO = ChWorldFrame::ToISO(loc);
    // to avoid 'jumping' of the normal vector, we take this smoothing approach
    const double delta = 0.05;
    double z0, zfront, zleft;
    z0 = GetHeight(loc);
    zfront = GetHeight(ChWorldFrame::FromISO(loc_ISO + ChVector<>(delta, 0, 0)));
    zleft = GetHeight(ChWorldFrame::FromISO(loc_ISO + ChVector<>(0, delta, 0)));
    ChVector<> p0(loc_ISO.x(), loc_ISO.y(), z0);
    ChVector<> pfront(loc_ISO.x() + delta, loc_ISO.y(), zfront);
    ChVector<> pleft(loc_ISO.x(), loc_ISO.y() + delta, zleft);
    ChVector<> normal_ISO;
    ChVector<> r1, r2;
    r1 = pfront - p0;
    r2 = pleft - p0;
    normal_ISO = Vcross(r1, r2);
    if (normal_ISO.z() <= 0.0) {
        GetLog() << "Fatal: wrong surface normal!\n";
        exit(99);
    }
    ChVector<> normal = ChWorldFrame::FromISO(normal_ISO);
    normal.Normalize();

    return normal;
}

float CRGTerrain::GetCoefficientFriction(const ChVector<>& loc) const {
    return m_friction_fun ? (*m_friction_fun)(loc) : m_friction;
}

std::shared_ptr<ChBezierCurve> CRGTerrain::GetRoadCenterLine() {
    std::vector<ChVector<>> pathpoints;

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
            GetLog() << "CRGTerrain::SetupGraphics(): error during uv -> xy coordinate transformation\n";
        }
        int z_ok = crgEvaluv2z(m_cpId, u, vm, &zm);
        if (z_ok != 1) {
            GetLog() << "CRGTerrain::SetupGraphics(): error during uv -> z coordinate transformation\n";
        }
        ////zm = avg.Add(zm);
        pathpoints.push_back(ChWorldFrame::FromISO(ChVector<>(xm, ym, zm + 0.2)));
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
    std::vector<ChVector<>> pl, pr;

    for (size_t i = 0; i < np; i++) {
        double u = m_ubeg + i * du;
        double xl, yl, zl;
        double xr, yr, zr;

        int xy_ok = crgEvaluv2xy(m_cpId, u, m_vbeg, &xl, &yl);
        if (xy_ok != 1) {
            GetLog() << "CRGTerrain::SetupGraphics(): error during uv -> xy coordinate transformation\n";
        }
        xy_ok = crgEvaluv2xy(m_cpId, u, m_vend, &xr, &yr);
        if (xy_ok != 1) {
            GetLog() << "CRGTerrain::SetupGraphics(): error during uv -> xy coordinate transformation\n";
        }
        int z_ok = crgEvaluv2z(m_cpId, u, m_vbeg, &zl);
        if (z_ok != 1) {
            GetLog() << "CRGTerrain::SetupGraphics(): error during uv -> z coordinate transformation\n";
        }
        z_ok = crgEvaluv2z(m_cpId, u, m_vend, &zr);
        if (z_ok != 1) {
            GetLog() << "CRGTerrain::SetupGraphics(): error during uv -> z coordinate transformation\n";
        }
        pl.push_back(ChWorldFrame::FromISO(ChVector<>(xl, yl, zl)));
        pr.push_back(ChWorldFrame::FromISO(ChVector<>(xr, yr, zr)));
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

    auto np = m_road_left->getNumPoints();
    unsigned int num_render_points = std::max<unsigned int>(static_cast<unsigned int>(3 * np), 400);

    auto bezier_line_left = chrono_types::make_shared<geometry::ChLineBezier>(m_road_left);
    auto bezier_asset_left = chrono_types::make_shared<ChLineShape>();
    bezier_asset_left->SetLineGeometry(bezier_line_left);
    bezier_asset_left->SetNumRenderPoints(num_render_points);
    bezier_asset_left->SetName(m_curve_left_name);
    bezier_asset_left->AddMaterial(mat);
    m_ground->AddVisualShape(bezier_asset_left);

    auto bezier_line_right = chrono_types::make_shared<geometry::ChLineBezier>(m_road_right);
    auto bezier_asset_right = chrono_types::make_shared<ChLineShape>();
    bezier_asset_right->SetLineGeometry(bezier_line_right);
    bezier_asset_right->SetNumRenderPoints(num_render_points);
    bezier_asset_right->SetName(m_curve_right_name);
    bezier_asset_right->AddMaterial(mat);
    m_ground->AddVisualShape(bezier_asset_right);
}

void CRGTerrain::GenerateMesh() {
    m_mesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
    auto& coords = m_mesh->getCoordsVertices();
    auto& indices = m_mesh->getIndicesVertexes();
    auto& coords_uv = m_mesh->getCoordsUV();
    auto& indices_uv = m_mesh->getIndicesUV();

    int nu = static_cast<int>((m_uend - m_ubeg) / m_uinc) + 1;
    int nv;

    std::vector<double> x0, y0, z0;
    std::vector<double> tu0, tv0;
    // Define the vertices
    if (m_v.size() == 5) {
        // v is nonequidistant, we use m_v[]
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
                    GetLog() << "main: error during uv -> xy coordinate transformation in crg file\n";
                    exit(99);
                }
                int z_ok = crgEvaluv2z(m_cpId, u, v, &z);
                if (z_ok != 1) {
                    GetLog() << "main: error during uv -> z coordinate transformation in crg file\n";
                    exit(99);
                }
                if (i == 0) {
                    x0.push_back(x);
                    y0.push_back(y);
                    z0.push_back(z);
                    tu0.push_back(tu);
                    tv0.push_back(tv);
                }
                if (i == nu - 1 && m_isClosed) {
                    coords.push_back(ChWorldFrame::FromISO(ChVector<>(x0[j], y0[j], z0[j])));
                    coords_uv.push_back(ChVector2<>(tu0[j], tv0[j]));
                } else {
                    coords.push_back(ChWorldFrame::FromISO(ChVector<>(x, y, z)));
                    coords_uv.push_back(ChVector2<>(tu, tv));
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
                    GetLog() << "main: error during uv -> xy coordinate transformation in crg file\n";
                    exit(99);
                }
                int z_ok = crgEvaluv2z(m_cpId, u, v, &z);
                if (z_ok != 1) {
                    GetLog() << "main: error during uv -> z coordinate transformation in crg file\n";
                    exit(99);
                }
                if (i == 0) {
                    x0.push_back(x);
                    y0.push_back(y);
                    z0.push_back(z);
                    tu0.push_back(tu);
                    tv0.push_back(tv);
                }
                if (i == nu - 1 && m_isClosed) {
                    coords.push_back(ChWorldFrame::FromISO(ChVector<>(x0[j], y0[j], z0[j])));
                    coords_uv.push_back(ChVector2<>(tu0[j], tv0[j]));
                } else {
                    coords.push_back(ChWorldFrame::FromISO(ChVector<>(x, y, z)));
                    coords_uv.push_back(ChVector2<>(tu, tv));
                }
            }
        }
    }

    // Define the faces
    for (int i = 0; i < nu - 1; i++) {
        int ofs = nv * i;
        for (int j = 0; j < nv - 1; j++) {
            indices.push_back(ChVector<int>(j + ofs, j + nv + ofs, j + 1 + ofs));
            indices.push_back(ChVector<int>(j + 1 + ofs, j + nv + ofs, j + 1 + nv + ofs));
            indices_uv.push_back(ChVector<int>(j + ofs, j + nv + ofs, j + 1 + ofs));
            indices_uv.push_back(ChVector<int>(j + 1 + ofs, j + nv + ofs, j + 1 + nv + ofs));
        }
    }
}

void CRGTerrain::SetupMeshGraphics() {
    if (m_use_diffuseTexture) {
        auto vmesh = chrono_types::make_shared<ChTriangleMeshShape>();
        vmesh->SetMesh(m_mesh);
        vmesh->SetName(m_mesh_name);
        auto material = chrono_types::make_shared<ChVisualMaterial>();
        material->SetDiffuseColor(ChColor(1.0f, 1.0f, 1.0f));
        material->SetAmbientColor(ChColor(1.0f, 1.0f, 1.0f));
        material->SetEmissiveColor(ChColor(0.1f, 0.1f, 0.1f));
        if(m_use_diffuseTexture)
            material->SetKdTexture(m_diffuse_texture_filename, 0.5 * GetLength() / GetWidth(), 1.0);
        if (m_use_normalTexture)
            material->SetNormalMapTexture(m_normal_texture_filename, 0.5 * GetLength() / GetWidth(), 1.0);
        vmesh->SetMaterial(0, material);
        m_ground->AddVisualShape(vmesh);
        GetLog() << "Texture?\n";
    } else {
        auto vmesh = chrono_types::make_shared<ChTriangleMeshShape>();
        vmesh->SetMesh(m_mesh);
        vmesh->SetName(m_mesh_name);
        vmesh->SetColor(ChColor(1.0f, 1.0f, 1.0f));

        m_ground->AddVisualShape(vmesh);
    }
}

void CRGTerrain::ExportMeshWavefront(const std::string& out_dir) {
    std::vector<geometry::ChTriangleMeshConnected> meshes = {*m_mesh};
    geometry::ChTriangleMeshConnected::WriteWavefront(out_dir + "/" + m_mesh_name + ".obj", meshes);
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
