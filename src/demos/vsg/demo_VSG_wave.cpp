// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Deformable mesh example
//
// =============================================================================

#include <cmath>

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChBodyGeometry.h"
#include "chrono/utils/ChUtilsCreators.h"

#include "chrono_vsg/ChVisualSystemVSG.h"

using namespace chrono;
using namespace chrono::vsg3d;

using std::cout;
using std::endl;

// -----------------------------------------------------------------------------

std::shared_ptr<ChTriangleMeshConnected> CreateMesh(double L, int n) {
    // Create triangular mesh
    auto trimesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    std::vector<ChVector3d>& vertices = trimesh->GetCoordsVertices();
    std::vector<ChVector3d>& normals = trimesh->GetCoordsNormals();
    std::vector<ChVector3i>& idx_vertices = trimesh->GetIndicesVertexes();
    std::vector<ChVector3i>& idx_normals = trimesh->GetIndicesNormals();
    std::vector<ChVector2d>& uv_coords = trimesh->GetCoordsUV();
    std::vector<ChColor>& colors = trimesh->GetCoordsColors();

    // Resize mesh arrays
    int n_verts = n * n;
    int n_faces = 2 * (n - 1) * (n - 1);
    double uv_scale = 1.0 / n;

    vertices.resize(n_verts);
    normals.resize(n_verts);
    uv_coords.resize(n_verts);
    colors.resize(n_verts);
    idx_vertices.resize(n_faces);
    idx_normals.resize(n_faces);

    // Create mesh vertices (in x-y plane, centered at origin)
    double delta = L / (n - 1);
    int iv = 0;
    for (int iy = 0; iy < n; iy++) {
        double y = iy * delta - 0.5 * L;
        for (int ix = 0; ix < n; ix++) {
            double x = ix * delta - 0.5 * L;
            vertices[iv] = ChVector3d(x, y, std::sin(CH_2PI * x/L) * std::sin(CH_2PI * y/L));
            colors[iv] = ChColor(1, 1, 1);
            uv_coords[iv] = ChVector2d(ix * uv_scale, iy * uv_scale);
            ++iv;
        }
    }

    // Specify triangular faces (two at a time)
    // Specify the face vertices counter-clockwise
    // Set the normal indices same as the vertex indices
    int it = 0;
    for (int iy = 0; iy < n - 1; iy++) {
        for (int ix = 0; ix < n - 1; ix++) {
            int v0 = ix + n * iy;
            idx_vertices[it] = ChVector3i(v0, v0 + 1, v0 + n + 1);
            idx_normals[it] = ChVector3i(v0, v0 + 1, v0 + n + 1);
            ++it;
            idx_vertices[it] = ChVector3i(v0, v0 + n + 1, v0 + n);
            idx_normals[it] = ChVector3i(v0, v0 + n + 1, v0 + n);
            ++it;
        }
    }

    return trimesh;
}

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2025 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Create the system
    ChSystemSMC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.8));
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    
    // Create wave mesh
    int n = 51;
    double L = 10.0;
    auto trimesh = CreateMesh(L, n);
    std::vector<ChVector3d> u0 = trimesh->GetCoordsVertices();
    
    // Create a material (will be used by all collision shapes)
    ChContactMaterialData mat_data;
    mat_data.mu = 0.4f;
    mat_data.cr = 0.1f;
    auto mat = mat_data.CreateMaterial(sys.GetContactMethod());

    // Create the "wave" body
    auto wave = chrono_types::make_shared<ChBody>();
    wave->SetName("ground");
    wave->SetFixed(true);
    wave->SetMass(1);
    wave->SetPos(VNULL);
    wave->SetRot(QUNIT);
    sys.AddBody(wave);

    // Attach a deformable mesh (visualization and collision)
    utils::ChBodyGeometry geometry;
    geometry.materials.push_back(mat_data);
    auto mesh_shape = utils::ChBodyGeometry::TrimeshShape(VNULL, QUNIT, trimesh, VNULL, 1.0, 0.01, 0);
    mesh_shape.is_mutable = true;
    //mesh_shape.color = ChColor(0.4f, 0.8f, 0.2f);
    geometry.coll_meshes.push_back(mesh_shape);
    geometry.vis_meshes.push_back(mesh_shape);
    geometry.CreateCollisionShapes(wave, 0, sys.GetContactMethod());
    geometry.CreateVisualizationAssets(wave, VisualizationType::COLLISION, false);

    // Create visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemVSG>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(1280, 800);
    vis->SetWindowPosition(100, 100);
    vis->SetWindowTitle("2D wave");
    vis->SetBackgroundColor(ChColor(18.0f / 255, 26.0f / 255, 32.0f / 255));
    vis->SetCameraVertical(CameraVerticalDir::Z);
    vis->AddCamera(ChVector3d(0, -15, 3));
    vis->SetCameraAngleDeg(40);
    vis->SetLightIntensity(1.0f);
    vis->SetLightDirection(-1.5 * CH_PI_2, CH_PI_4);
    vis->EnableShadows();
    vis->Initialize();

    // Simulation loop
    double time_step = 1e-4;
    double render_fps = 200;

    ChRealtimeStepTimer rt_timer;
    double time = 0.0;
    int render_frame = 0;

    while (vis->Run()) {
        if (time > render_frame / render_fps) {
            vis->BeginScene();
            vis->Render();
            vis->EndScene();
            render_frame++;
        }

        sys.DoStepDynamics(time_step);

        double modulation = std::sin(CH_2PI * (time / 10.0));
        for (size_t i = 0; i < n * n; i++) {
            double& z = trimesh->GetCoordsVertices()[i].z();
            z = modulation * u0[i].z();
            trimesh->GetCoordsColors()[i] =
                ChColor::Interp(ChColor(0.1f, 0.0f, 0.9f), ChColor(0.8f, 0.1f, 0.1f), (1 + z) / 2);
        }

        rt_timer.Spin(time_step);
        time += time_step;
    }
}
