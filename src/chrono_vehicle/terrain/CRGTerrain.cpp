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
// Authors: Rainer Gericke, Radu Serban
// =============================================================================
//
// Terrain defined by an OpenCRG file (http://opencrg.org)
//
// OpenCRG® is managed by
//	VIRES Simulationstechnologie GmbH
//	Grassinger Strasse 8
//	83043 Bad Aibling
//	Germany
//	p: +49.8061.939093-0
//	f: +49.8061.939093-13
//	e: opencrg@opencrg.org
//
// =============================================================================
//
// Limits:	Options and modifiers are ignored
//
// Roads start at {0,0,0}, first driving direction is {1,0,0}
//
//==============================================================================

#include <algorithm>

#include "chrono/assets/ChPathShape.h"
#include "chrono/physics/ChBodyEasy.h"

#include "chrono/utils/ChFilters.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono/geometry/ChLineBezier.h"
#include "chrono/geometry/ChLineSegment.h"

#include "chrono/assets/ChLineShape.h"
#include "chrono/assets/ChPathShape.h"

#include "chrono_vehicle/terrain/CRGTerrain.h"

extern "C" {
#include "crgBaseLib.h"
}

namespace chrono {
namespace vehicle {

const std::string CRGTerrain::m_mesh_name = "crg_road";

CRGTerrain::CRGTerrain(ChSystem* system)
    : m_use_vis_mesh(true), m_friction(0.8f), m_dataSetId(0), m_cpId(0), m_isClosed(false) {
    m_ground = std::shared_ptr<ChBody>(system->NewBody());
    m_ground->SetName("ground");
    m_ground->SetPos(ChVector<>(0, 0, 0));
    m_ground->SetBodyFixed(true);
    m_ground->SetCollide(false);
    system->Add(m_ground);
}

CRGTerrain::~CRGTerrain() {
    crgContactPointDelete(m_cpId);
    crgDataSetRelease(m_dataSetId);
    crgMemRelease();
}

void CRGTerrain::Initialize(const std::string& crg_file) {
    m_v.clear();

    // Read the crg-file
    m_dataSetId = crgLoaderReadFile(crg_file.c_str());
    if (m_dataSetId <= 0) {
        std::cout << "CRGTerrain::CRGTTerrain(): error reading data file " << crg_file << std::endl;
        return;
    }
    // Check it
    if (!crgCheck(m_dataSetId)) {
        std::cout << "CRGTerrain::CRGTTerrain(): could not validate crg data." << std::endl;
        return;
    }
    // Create a contact point
    m_cpId = crgContactPointCreate(m_dataSetId);
    if (m_cpId < 0) {
        // crgMsgPrint( dCrgMsgLevelFatal, "main: could not create contact point.\n" );
        std::cout << "CRGTerrain::CRGTTerrain(): could not create contact point!" << std::endl;
        return;
    }

    int urange_ok = crgDataSetGetURange(m_dataSetId, &m_ubeg, &m_uend);
    if (urange_ok != 1) {
        std::cout << "CRGTerrain::CRGTTerrain(): error with urange in data file " << crg_file << std::endl;
        return;
    }
    int vrange_ok = crgDataSetGetVRange(m_dataSetId, &m_vbeg, &m_vend);
    if (vrange_ok != 1) {
        std::cout << "CRGTerrain::CRGTTerrain(): error with vrange in data file " << crg_file << std::endl;
        return;
    }
    int incr_ok = crgDataSetGetIncrements(m_dataSetId, &m_uinc, &m_vinc);
    if (incr_ok != 1) {
        std::cout << "CRGTerrain::CRGTTerrain(): could not get increments from data file " << crg_file << std::endl;
        return;
    }
    // dirty hack:
    if (m_vinc <= 0.01 && m_use_vis_mesh) {
        m_v.push_back(m_vbeg);
        m_v.push_back(-0.05);
        m_v.push_back(0.0);
        m_v.push_back(0.05);
        m_v.push_back(m_vend);
        std::cout << std::endl;
        std::cout << " Caution:" << std::endl;
        std::cout << " Mesh seemes to be nonequidistant in v direction (vinc = " << m_vinc << "m)." << std::endl;
        std::cout << " We use 5 distinct v values [";
        for (size_t i = 0; i < m_v.size(); i++) {
            std::cout << " " << m_v[i];
        }
        std::cout << " ] as fallback values." << std::endl;
        std::cout << " If it does not work for you, only use v-eqidistant crg files with vinc > 0.01 m." << std::endl;
        std::cout << std::endl;
    }

    int uIsClosed;
    double uCloseMin, uCloseMax;
    int cl_ok = crgDataSetGetUtilityDataClosedTrack(m_dataSetId, &uIsClosed, &uCloseMin, &uCloseMax);
    if (cl_ok != 1) {
        std::cout << "CRGTerrain::CRGTerrain(): could not get closedness from data file " << crg_file << std::endl;
        return;
    } else {
        m_isClosed = (uIsClosed != 0);
    }

    GenerateMesh();

    if (m_use_vis_mesh) {
        SetupMeshGraphics();
    } else {
        SetupLineGraphics();
    }
}

float CRGTerrain::GetCoefficientFriction(double x, double y) const {
    return m_friction_fun ? (*m_friction_fun)(x, y) : m_friction;
}

double CRGTerrain::GetHeight(double x, double y) const {
    double u, v, z;
    int uv_ok = crgEvalxy2uv(m_cpId, x, y, &u, &v);
    if (uv_ok != 1) {
        std::cout << "CRGTerrain::GetHeight(): error during xy -> uv coordinate transformation" << std::endl;
    }

    // when leaving the road the vehicle should not fall into an abyss
    ChClampValue(u, m_ubeg, m_uend);
    ChClampValue(v, m_vbeg, m_vend);

    int z_ok = crgEvaluv2z(m_cpId, u, v, &z);
    if (z_ok != 1) {
        std::cout << "CRGTerrain::GetHeight(): error during uv -> z coordinate transformation" << std::endl;
    }

    return z;
}

ChVector<> CRGTerrain::GetNormal(double x, double y) const {
    // to avoid 'jumping' of the normal vector, we take this smoothing approach
    const double delta = 0.05;
    double z0, zfront, zleft;
    z0 = GetHeight(x, y);
    zfront = GetHeight(x + delta, y);
    zleft = GetHeight(x, y + delta);
    ChVector<> p0(x, y, z0), pfront(x + delta, y, zfront), pleft(x, y + delta, zleft), normal;
    ChVector<> r1, r2;
    r1 = pfront - p0;
    r2 = pleft - p0;
    normal = Vcross(r1, r2);
    if (normal.z() <= 0.0) {
        std::cout << "Fatal: wrong surface normal!" << std::endl;
        exit(99);
    }
    normal.Normalize();
    return normal;
}

std::shared_ptr<ChBezierCurve> CRGTerrain::GetPath() {
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
            std::cout << "CRGTerrain::SetupGraphics(): error during uv -> xy coordinate transformation" << std::endl;
        }
        int z_ok = crgEvaluv2z(m_cpId, u, vm, &zm);
        if (z_ok != 1) {
            std::cout << "CRGTerrain::SetupGraphics(): error during uv -> z coordinate transformation" << std::endl;
        }
        ////zm = avg.Add(zm);
        pathpoints.push_back(ChVector<>(xm, ym, zm + 0.2));
    }

    if (m_isClosed) {
        pathpoints.back() = pathpoints[0];
    }

    return std::make_shared<ChBezierCurve>(pathpoints);
}

void CRGTerrain::SetupLineGraphics() {
    double dp = 3.0;
    size_t np = static_cast<size_t>(m_uend / dp);
    std::vector<ChVector<>> pl, pr;
    unsigned int num_render_points = std::max<unsigned int>(static_cast<unsigned int>(3 * np), 400);

    double du = (m_uend - m_ubeg) / double(np - 1);

    for (size_t i = 0; i < np; i++) {
        double u = m_ubeg + double(i) * du;
        double xl, yl, zl;
        double xr, yr, zr;

        int xy_ok = crgEvaluv2xy(m_cpId, u, m_vbeg, &xl, &yl);
        if (xy_ok != 1) {
            std::cout << "CRGTerrain::SetupGraphics(): error during uv -> xy coordinate transformation" << std::endl;
        }
        xy_ok = crgEvaluv2xy(m_cpId, u, m_vend, &xr, &yr);
        if (xy_ok != 1) {
            std::cout << "CRGTerrain::SetupGraphics(): error during uv -> xy coordinate transformation" << std::endl;
        }
        int z_ok = crgEvaluv2z(m_cpId, u, m_vbeg, &zl);
        if (z_ok != 1) {
            std::cout << "CRGTerrain::SetupGraphics(): error during uv -> z coordinate transformation" << std::endl;
        }
        z_ok = crgEvaluv2z(m_cpId, u, m_vend, &zr);
        if (z_ok != 1) {
            std::cout << "CRGTerrain::SetupGraphics(): error during uv -> z coordinate transformation" << std::endl;
        }
        pl.push_back(ChVector<>(xl, yl, zl));
        pr.push_back(ChVector<>(xr, yr, zr));
    }

    if (m_isClosed) {
        pl.back() = pl[0];
        pr.back() = pr[0];
    }

    auto mfloorcolor = std::make_shared<ChColorAsset>();
    mfloorcolor->SetColor(ChColor(0.3f, 0.3f, 0.6f));
    m_ground->AddAsset(mfloorcolor);

    // Create a Bezier curve asset, reusing the points
    auto bezier_curve_left = std::make_shared<ChBezierCurve>(pl);
    auto bezier_line_left = std::make_shared<geometry::ChLineBezier>(bezier_curve_left);
    auto bezier_asset_left = std::make_shared<ChLineShape>();
    bezier_asset_left->SetLineGeometry(bezier_line_left);
    bezier_asset_left->SetNumRenderPoints(num_render_points);
    m_ground->AddAsset(bezier_asset_left);

    // Create a Bezier curve asset, reusing the points
    auto bezier_curve_right = std::make_shared<ChBezierCurve>(pr);
    auto bezier_line_right = std::make_shared<geometry::ChLineBezier>(bezier_curve_right);
    auto bezier_asset_right = std::make_shared<ChLineShape>();
    bezier_asset_right->SetLineGeometry(bezier_line_right);
    bezier_asset_right->SetNumRenderPoints(num_render_points);
    m_ground->AddAsset(bezier_asset_right);
}

void CRGTerrain::GenerateMesh() {
    m_mesh = std::make_shared<geometry::ChTriangleMeshConnected>();
    auto& coords = m_mesh->getCoordsVertices();
    auto& indices = m_mesh->getIndicesVertexes();

    int nu = static_cast<int>((m_uend - m_ubeg) / m_uinc) + 1;
    int nv;

    std::vector<double> x0, y0, z0;
    // Define the vertices
    if (m_v.size() == 5) {
        // v is nonequidistant, we use m_v[]
        nv = static_cast<int>(m_v.size());
        for (auto i = 0; i < nu; i++) {
            double u = m_ubeg + m_uinc * double(i);
            for (auto j = 0; j < nv; j++) {
                double x, y, z, v;
                v = m_v[j];
                int uv_ok = crgEvaluv2xy(m_cpId, u, v, &x, &y);
                if (uv_ok != 1) {
                    std::cout << "main: error during uv -> xy coordinate transformation in crg file " << std::endl;
                    exit(99);
                }
                int z_ok = crgEvaluv2z(m_cpId, u, v, &z);
                if (z_ok != 1) {
                    std::cout << "main: error during uv -> z coordinate transformation in crg file " << std::endl;
                    exit(99);
                }
                if (i == 0) {
                    x0.push_back(x);
                    y0.push_back(y);
                    z0.push_back(z);
                }
                if (i == nu - 1 && m_isClosed) {
                    coords.push_back(ChVector<>(x0[j], y0[j], z0[j]));
                } else {
                    coords.push_back(ChVector<>(x, y, z));
                }
            }
        }
    } else {
        // v is equidistant, we use m_vinc
        nv = static_cast<int>((m_vend - m_vbeg) / m_vinc) + 1;
        for (auto i = 0; i < nu; i++) {
            double u = m_ubeg + m_uinc * double(i);
            for (auto j = 0; j < nv; j++) {
                double v = m_vbeg + m_vinc * double(j);
                double x, y, z;
                int uv_ok = crgEvaluv2xy(m_cpId, u, v, &x, &y);
                if (uv_ok != 1) {
                    std::cout << "main: error during uv -> xy coordinate transformation in crg file " << std::endl;
                    exit(99);
                }
                int z_ok = crgEvaluv2z(m_cpId, u, v, &z);
                if (z_ok != 1) {
                    std::cout << "main: error during uv -> z coordinate transformation in crg file " << std::endl;
                    exit(99);
                }
                if (i == 0) {
                    x0.push_back(x);
                    y0.push_back(y);
                    z0.push_back(z);
                }
                if (i == nu - 1 && m_isClosed) {
                    coords.push_back(ChVector<>(x0[j], y0[j], z0[j]));
                } else {
                    coords.push_back(ChVector<>(x, y, z));
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
        }
    }
}

void CRGTerrain::SetupMeshGraphics() {
    auto vmesh = std::make_shared<ChTriangleMeshShape>();
    vmesh->SetMesh(m_mesh);
    vmesh->SetName(m_mesh_name);

    auto vcolor = std::make_shared<ChColorAsset>();
    vcolor->SetColor(ChColor(0.6f, 0.6f, 0.8f));

    m_ground->AddAsset(vcolor);
    m_ground->AddAsset(vmesh);
}

void CRGTerrain::ExportMeshWavefront(const std::string& out_dir) {
    std::vector<geometry::ChTriangleMeshConnected> meshes = { *m_mesh };
    geometry::ChTriangleMeshConnected::WriteWavefront(out_dir + "/" + m_mesh_name + ".obj", meshes);
}

void CRGTerrain::ExportMeshPovray(const std::string& out_dir) {
    utils::WriteMeshPovray(*m_mesh, m_mesh_name, out_dir, ChColor(1, 1, 1));
}

}  // end namespace vehicle
}  // end namespace chrono
