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
// Authors: Radu Serban, Hammad Mazhar
// =============================================================================
//
// =============================================================================

#include "chrono_utils/ChUtilsCreators.h"
#include "collision/ChCConvexDecomposition.h"
namespace chrono {
using namespace geometry;
using namespace collision;
namespace utils {

// -----------------------------------------------------------------------------

void AddSphereGeometry(ChBody* body,
                       double radius,
                       const ChVector<>& pos,
                       const ChQuaternion<>& rot,
                       bool visualization) {
  body->GetCollisionModel()->AddSphere(radius, pos);

  if (visualization) {
    ChSharedPtr<ChSphereShape> sphere(new ChSphereShape);
    sphere->GetSphereGeometry().rad = radius;
    sphere->Pos = pos;
    sphere->Rot = rot;
    body->GetAssets().push_back(sphere);
  }
}

// -----------------------------------------------------------------------------

void AddEllipsoidGeometry(ChBody* body,
                          const ChVector<>& size,
                          const ChVector<>& pos,
                          const ChQuaternion<>& rot,
                          bool visualization) {
  body->GetCollisionModel()->AddEllipsoid(size.x, size.y, size.z, pos, rot);

  if (visualization) {
    ChSharedPtr<ChEllipsoidShape> ellipsoid(new ChEllipsoidShape);
    ellipsoid->GetEllipsoidGeometry().rad = size;
    ellipsoid->Pos = pos;
    ellipsoid->Rot = rot;
    body->GetAssets().push_back(ellipsoid);
  }
}

// -----------------------------------------------------------------------------

void AddBoxGeometry(ChBody* body,
                    const ChVector<>& size,
                    const ChVector<>& pos,
                    const ChQuaternion<>& rot,
                    bool visualization) {
  body->GetCollisionModel()->AddBox(size.x, size.y, size.z, pos, rot);

  if (visualization) {
    ChSharedPtr<ChBoxShape> box(new ChBoxShape);
    box->GetBoxGeometry().Size = size;
    box->Pos = pos;
    box->Rot = rot;
    body->GetAssets().push_back(box);
  }
}

// -----------------------------------------------------------------------------

void AddCapsuleGeometry(ChBody* body,
                        double radius,
                        double hlen,
                        const ChVector<>& pos,
                        const ChQuaternion<>& rot,
                        bool visualization) {
  body->GetCollisionModel()->AddCapsule(radius, hlen, pos, rot);

  if (visualization) {
    ChSharedPtr<ChCapsuleShape> capsule(new ChCapsuleShape);
    capsule->GetCapsuleGeometry().rad = radius;
    capsule->GetCapsuleGeometry().hlen = hlen;
    capsule->Pos = pos;
    capsule->Rot = rot;
    body->GetAssets().push_back(capsule);
  }
}

// -----------------------------------------------------------------------------

void AddCylinderGeometry(ChBody* body,
                         double radius,
                         double hlen,
                         const ChVector<>& pos,
                         const ChQuaternion<>& rot,
                         bool visualization) {
  body->GetCollisionModel()->AddCylinder(radius, radius, hlen, pos, rot);

  if (visualization) {
    ChSharedPtr<ChCylinderShape> cylinder(new ChCylinderShape);
    cylinder->GetCylinderGeometry().rad = radius;
    cylinder->GetCylinderGeometry().p1 = ChVector<>(0, hlen, 0);
    cylinder->GetCylinderGeometry().p2 = ChVector<>(0, -hlen, 0);
    cylinder->Pos = pos;
    cylinder->Rot = rot;
    body->GetAssets().push_back(cylinder);
  }
}

// -----------------------------------------------------------------------------

void AddConeGeometry(ChBody* body,
                     double radius,
                     double height,
                     const ChVector<>& pos,
                     const ChQuaternion<>& rot,
                     bool visualization) {
  body->GetCollisionModel()->AddCone(radius, radius, height, pos, rot);

  if (visualization) {
    ChSharedPtr<ChConeShape> cone(new ChConeShape);
    cone->GetConeGeometry().rad = ChVector<>(radius, height, radius);
    cone->Pos = pos;
    cone->Rot = rot;
    body->GetAssets().push_back(cone);
  }
}

// -----------------------------------------------------------------------------

void AddTriangleMeshGeometry(ChBody* body,
                             const std::string& obj_filename,
                             const std::string& name,
                             const ChVector<>& pos,
                             const ChQuaternion<>& rot,
                             bool visualization) {
  geometry::ChTriangleMeshConnected trimesh;
  trimesh.LoadWavefrontMesh(obj_filename, false, false);

  for (int i = 0; i < trimesh.m_vertices.size(); i++)
    trimesh.m_vertices[i] = pos + rot.Rotate(trimesh.m_vertices[i]);

  body->GetCollisionModel()->AddTriangleMesh(trimesh, false, false);

  if (visualization) {
    ChSharedPtr<ChTriangleMeshShape> trimesh_shape(new ChTriangleMeshShape);
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->SetName(name);
    trimesh_shape->Pos = ChVector<>(0, 0, 0);
    trimesh_shape->Rot = ChQuaternion<>(1, 0, 0, 0);
    body->GetAssets().push_back(trimesh_shape);
  }
}

// -----------------------------------------------------------------------------

void AddTriangleMeshConvexDecomposition(ChBody* body,
                                        const std::string& obj_filename,
                                        const std::string& name,
                                        const ChVector<>& pos,
                                        const ChQuaternion<>& rot,
                                        double skin_thickness,
                                        bool use_original_asset) {
  int decompdepth = 100;
  int maxhullvert = 50;
  float concavity = 0.1f;
  float merge = 30.f;
  float volumep = 0.1f;
  bool useinitialislands = true;

  geometry::ChTriangleMeshConnected trimesh;
  trimesh.LoadWavefrontMesh(obj_filename, true, false);
  for (int i = 0; i < trimesh.m_vertices.size(); i++) {
    trimesh.m_vertices[i] = pos + rot.Rotate(trimesh.m_vertices[i]);
  }
  collision::ChConvexDecompositionJR mydecompositionJR;

  mydecompositionJR.Reset();
  mydecompositionJR.AddTriangleMesh(trimesh);
  mydecompositionJR.SetParameters(skin_thickness,     // skin width
                                  decompdepth,        // decomp.depth
                                  maxhullvert,        // max hull vertexes
                                  concavity,          // concavity threshold percent
                                  merge,              // merge threshold percent
                                  volumep,            // volume split percent
                                  useinitialislands,  // initial islands
                                  false);
  mydecompositionJR.ComputeConvexDecomposition();
  collision::ChConvexDecomposition* used_decomposition = &mydecompositionJR;

  int hull_count = used_decomposition->GetHullCount();
  std::vector<ChVector<double> > convexhull;
  for (int c = 0; c < hull_count; c++) {
    used_decomposition->GetConvexHullResult(c, convexhull);

    ((collision::ChCollisionModelParallel*)body->GetCollisionModel())->AddConvexHull(convexhull, pos, rot);
    if (!use_original_asset) {
      std::stringstream ss;
      ss << name << "_" << c;
      geometry::ChTriangleMeshConnected trimesh_convex;
      used_decomposition->GetConvexHullResult(c, trimesh_convex);

      ChSharedPtr<ChTriangleMeshShape> trimesh_shape(new ChTriangleMeshShape);
      trimesh_shape->SetMesh(trimesh_convex);
      trimesh_shape->SetName(ss.str());
      trimesh_shape->Pos = ChVector<>(0, 0, 0);
      trimesh_shape->Rot = ChQuaternion<>(1, 0, 0, 0);

      body->GetAssets().push_back(trimesh_shape);
    }
  }
  if (use_original_asset) {
    ChSharedPtr<ChTriangleMeshShape> trimesh_shape(new ChTriangleMeshShape);
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->SetName(name);
    trimesh_shape->Pos = ChVector<>(0, 0, 0);
    trimesh_shape->Rot = ChQuaternion<>(1, 0, 0, 0);
    body->GetAssets().push_back(trimesh_shape);
  }
}

// -----------------------------------------------------------------------------

void AddTriangleMeshConvexDecompositionV2(ChBody* body,
                                          const std::string& obj_filename,
                                          const std::string& name,
                                          const ChVector<>& pos,
                                          const ChQuaternion<>& rot,
                                          bool use_original_asset) {
  geometry::ChTriangleMeshConnected trimesh;
  trimesh.LoadWavefrontMesh(obj_filename, true, false);

  for (int i = 0; i < trimesh.m_vertices.size(); i++) {
    trimesh.m_vertices[i] = pos + rot.Rotate(trimesh.m_vertices[i]);
  }
  collision::ChConvexDecompositionHACDv2 mydecompositionHACDv2;

  int hacd_maxhullcount = 1024;
  int hacd_maxhullmerge = 256;
  int hacd_maxhullvertexes = 64;
  double hacd_concavity = 0.01;
  double hacd_smallclusterthreshold = 0.0;
  double hacd_fusetolerance = 1e-6;

  mydecompositionHACDv2.Reset();
  mydecompositionHACDv2.AddTriangleMesh(trimesh);

  mydecompositionHACDv2.SetParameters(hacd_maxhullcount, hacd_maxhullmerge, hacd_maxhullvertexes, (float)hacd_concavity,
                                      (float)hacd_smallclusterthreshold, (float)hacd_fusetolerance);
  mydecompositionHACDv2.ComputeConvexDecomposition();
  collision::ChConvexDecomposition* used_decomposition = &mydecompositionHACDv2;

  int hull_count = used_decomposition->GetHullCount();

  for (int c = 0; c < hull_count; c++) {
    std::vector<ChVector<double> > convexhull;
    used_decomposition->GetConvexHullResult(c, convexhull);

    ((collision::ChCollisionModelParallel*)body->GetCollisionModel())->AddConvexHull(convexhull, pos, rot);
    if (!use_original_asset) {
      std::stringstream ss;
      ss << name << "_" << c;
      geometry::ChTriangleMeshConnected trimesh_convex;
      used_decomposition->GetConvexHullResult(c, trimesh_convex);

      ChSharedPtr<ChTriangleMeshShape> trimesh_shape(new ChTriangleMeshShape);
      trimesh_shape->SetMesh(trimesh_convex);
      trimesh_shape->SetName(ss.str());
      trimesh_shape->Pos = ChVector<>(0, 0, 0);
      trimesh_shape->Rot = ChQuaternion<>(1, 0, 0, 0);
      body->GetAssets().push_back(trimesh_shape);
    }
  }
  if (use_original_asset) {
    ChSharedPtr<ChTriangleMeshShape> trimesh_shape(new ChTriangleMeshShape);
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->SetName(name);
    trimesh_shape->Pos = ChVector<>(0, 0, 0);
    trimesh_shape->Rot = ChQuaternion<>(1, 0, 0, 0);
    body->GetAssets().push_back(trimesh_shape);
  }
}

// -----------------------------------------------------------------------------

//// TODO: extend this to also work for DEM systems.

void AddTriangleMeshConvexDecompositionSplit(ChSystemParallel* system,
                                             const std::string& obj_filename,
                                             const std::string& name,
                                             const ChVector<>& pos,
                                             const ChQuaternion<>& rot,
                                             ChSharedPtr<ChMaterialSurface>& material,
                                             double total_mass) {
  geometry::ChTriangleMeshConnected trimesh;
  trimesh.LoadWavefrontMesh(obj_filename, true, false);

  for (int i = 0; i < trimesh.m_vertices.size(); i++) {
    trimesh.m_vertices[i] = pos + rot.Rotate(trimesh.m_vertices[i]);
  }
  collision::ChConvexDecompositionHACDv2 mydecompositionHACDv2;

  int hacd_maxhullcount = 1024;
  int hacd_maxhullmerge = 256;
  int hacd_maxhullvertexes = 64;
  double hacd_concavity = 0.01;
  double hacd_smallclusterthreshold = 0.1;
  double hacd_fusetolerance = 1e-6;

  mydecompositionHACDv2.Reset();
  mydecompositionHACDv2.AddTriangleMesh(trimesh);

  mydecompositionHACDv2.SetParameters(hacd_maxhullcount, hacd_maxhullmerge, hacd_maxhullvertexes, (float)hacd_concavity,
                                      (float)hacd_smallclusterthreshold, (float)hacd_fusetolerance);
  mydecompositionHACDv2.ComputeConvexDecomposition();
  collision::ChConvexDecomposition* used_decomposition = &mydecompositionHACDv2;

  int hull_count = used_decomposition->GetHullCount();

  ChSharedBodyPtr body;
  double mass;
  ChVector<> center;
  ChMatrix33<> inertia;
  real sum = 0;
  for (int c = 0; c < hull_count; c++) {
    geometry::ChTriangleMeshConnected trimesh_convex;
    used_decomposition->GetConvexHullResult(c, trimesh_convex);
    trimesh_convex.ComputeMassProperties(true, mass, center, inertia);
    sum += mass;
  }

  real scale = 1.0 / sum;

  for (int c = 0; c < hull_count; c++) {
    geometry::ChTriangleMeshConnected trimesh_convex;
    used_decomposition->GetConvexHullResult(c, trimesh_convex);
    trimesh_convex.ComputeMassProperties(true, mass, center, inertia);

    body = ChSharedPtr<ChBody>(new ChBody(new collision::ChCollisionModelParallel));

    InitializeObject(body, scale * mass * total_mass, material, center, Quaternion(1, 0, 0, 0), true, false, 0, 2);

    std::vector<ChVector<double> > convexhull;
    used_decomposition->GetConvexHullResult(c, convexhull);
    for (size_t v = 0; v < convexhull.size(); v++) {
      convexhull[v] = convexhull[v] - center;
    }

    ((collision::ChCollisionModelParallel*)body->GetCollisionModel())->AddConvexHull(convexhull, pos, rot);

    std::stringstream ss;
    ss << name << "_" << c;
    //      geometry::ChTriangleMeshConnected trimesh_convex;
    //      used_decomposition->GetConvexHullResult(c, trimesh_convex);

    ChSharedPtr<ChTriangleMeshShape> trimesh_shape(new ChTriangleMeshShape);
    trimesh_shape->SetMesh(trimesh_convex);
    trimesh_shape->SetName(ss.str());
    trimesh_shape->Pos = -center;
    trimesh_shape->Rot = ChQuaternion<>(1, 0, 0, 0);

    body->GetAssets().push_back(trimesh_shape);
    // std::cout << mass << " " << scale * mass* total_mass << " " <<
    // inertia.GetElement(0, 0) << " " << inertia.GetElement(1, 1) << " " <<
    // inertia.GetElement(2, 2) << std::endl;
    FinalizeObject(body, system);
    body->SetInertiaXX(ChVector<>(inertia.GetElement(0, 0) * scale * total_mass,
                                  inertia.GetElement(1, 1) * scale * total_mass,
                                  inertia.GetElement(2, 2) * scale * total_mass));
  }
}

// -----------------------------------------------------------------------------

void AddRoundedBoxGeometry(ChBody* body,
                           const ChVector<>& size,
                           double srad,
                           const ChVector<>& pos,
                           const ChQuaternion<>& rot,
                           bool visualization) {
  body->GetCollisionModel()->AddRoundedBox(size.x, size.y, size.z, srad, pos, rot);

  if (visualization) {
    ChSharedPtr<ChRoundedBoxShape> box(new ChRoundedBoxShape);
    box->GetRoundedBoxGeometry().Size = size;
    box->GetRoundedBoxGeometry().radsphere = srad;
    box->Pos = pos;
    box->Rot = rot;
    body->GetAssets().push_back(box);
  }
}

// -----------------------------------------------------------------------------

void AddRoundedCylinderGeometry(ChBody* body,
                                double radius,
                                double hlen,
                                double srad,
                                const ChVector<>& pos,
                                const ChQuaternion<>& rot,
                                bool visualization) {
  body->GetCollisionModel()->AddRoundedCylinder(radius, radius, hlen, srad, pos, rot);

  if (visualization) {
    ChSharedPtr<ChRoundedCylinderShape> rcyl(new ChRoundedCylinderShape);
    rcyl->GetRoundedCylinderGeometry().rad = radius;
    rcyl->GetRoundedCylinderGeometry().hlen = hlen;
    rcyl->GetRoundedCylinderGeometry().radsphere = srad;
    rcyl->Pos = pos;
    rcyl->Rot = rot;
    body->GetAssets().push_back(rcyl);
  }
}

// -----------------------------------------------------------------------------

void AddTorusGeometry(ChBody* body,
                      double radius,
                      double thickness,
                      int segments,
                      int angle,
                      const ChVector<>& pos,
                      const ChQuaternion<>& rot,
                      bool visualization) {
  for (int i = 0; i < angle; i += angle / segments) {
    double angle = i * CH_C_PI / 180.0;
    double x = cos(angle) * radius;
    double z = sin(angle) * radius;
    Quaternion q = chrono::Q_from_AngAxis(-angle, VECT_Y) % chrono::Q_from_AngAxis(CH_C_PI / 2.0, VECT_X);
    double outer_circ = 2 * CH_C_PI * (radius + thickness);

    AddCapsuleGeometry(body, thickness, outer_circ / segments * .5, ChVector<>(x, 0, z) + pos, q, visualization);
  }
}

// -----------------------------------------------------------------------------
// CreateBoxContainer
//
// Create a fixed body with contact and asset geometry representing a box with 5
// walls (no top).
// -----------------------------------------------------------------------------
ChSharedPtr<ChBody> CreateBoxContainer(ChSystem* system,
                                       int id,
                                       ChSharedPtr<ChMaterialSurfaceBase> mat,
                                       const ChVector<>& hdim,
                                       double hthick,
                                       const ChVector<>& pos,
                                       const ChQuaternion<>& rot,
                                       bool collide,
                                       bool y_up,
                                       bool overlap,
                                       bool closed) {
  // Infer system type and collision type.
  SystemType sysType = GetSystemType(system);
  CollisionType cdType = GetCollisionType(system);

  // Infer contact method from the specified material properties object.
  ChBody::ContactMethod contact_method = GetContactMethod(mat);

  // Verify consistency of input arguments.
  assert(((sysType == SEQUENTIAL_DVI || sysType == PARALLEL_DVI) && contact_method == ChBody::DVI) ||
         ((sysType == SEQUENTIAL_DEM || sysType == PARALLEL_DEM) && contact_method == ChBody::DEM));

  // Create the body and set material
  ChSharedPtr<ChBody> body;

  if (sysType == SEQUENTIAL_DVI || sysType == SEQUENTIAL_DEM || cdType == BULLET_CD)
    body = ChSharedPtr<ChBody>(new ChBody(contact_method));
  else
    body = ChSharedPtr<ChBody>(new ChBody(new collision::ChCollisionModelParallel, contact_method));

  body->SetMaterialSurface(mat);

  // Set body properties and geometry.
  body->SetIdentifier(id);
  body->SetMass(1);
  body->SetPos(pos);
  body->SetRot(rot);
  body->SetCollide(collide);
  body->SetBodyFixed(true);
  double o_lap = 0;
  if (overlap) {
    o_lap = hthick * 2;
  }
  body->GetCollisionModel()->ClearModel();
  if (y_up) {
    AddBoxGeometry(body.get_ptr(), ChVector<>(hdim.x + o_lap, hthick, hdim.y + o_lap), ChVector<>(0, -hthick, 0));
    AddBoxGeometry(body.get_ptr(), ChVector<>(hthick, hdim.z + o_lap, hdim.y + o_lap),
                   ChVector<>(-hdim.x - hthick, hdim.z, 0));
    AddBoxGeometry(body.get_ptr(), ChVector<>(hthick, hdim.z + o_lap, hdim.y + o_lap),
                   ChVector<>(hdim.x + hthick, hdim.z, 0));
    AddBoxGeometry(body.get_ptr(), ChVector<>(hdim.x + o_lap, hdim.z + o_lap, hthick),
                   ChVector<>(0, hdim.z, -hdim.y - hthick));
    AddBoxGeometry(body.get_ptr(), ChVector<>(hdim.x + o_lap, hdim.z + o_lap, hthick),
                   ChVector<>(0, hdim.z, hdim.y + hthick));
    if (closed) {
      AddBoxGeometry(body.get_ptr(), ChVector<>(hdim.x + o_lap, hthick, hdim.y + o_lap),
                     ChVector<>(0, hdim.z * 2 + hthick, 0));
    }
  } else {
    AddBoxGeometry(body.get_ptr(), ChVector<>(hdim.x + o_lap, hdim.y + o_lap, hthick), ChVector<>(0, 0, -hthick));
    AddBoxGeometry(body.get_ptr(), ChVector<>(hthick, hdim.y + o_lap, hdim.z + o_lap),
                   ChVector<>(-hdim.x - hthick, 0, hdim.z));
    AddBoxGeometry(body.get_ptr(), ChVector<>(hthick, hdim.y + o_lap, hdim.z + o_lap),
                   ChVector<>(hdim.x + hthick, 0, hdim.z));
    AddBoxGeometry(body.get_ptr(), ChVector<>(hdim.x + o_lap, hthick, hdim.z + o_lap),
                   ChVector<>(0, -hdim.y - hthick, hdim.z));
    AddBoxGeometry(body.get_ptr(), ChVector<>(hdim.x + o_lap, hthick, hdim.z + o_lap),
                   ChVector<>(0, hdim.y + hthick, hdim.z));
    if (closed) {
      AddBoxGeometry(body.get_ptr(), ChVector<>(hdim.x + o_lap, hdim.y + o_lap, hthick),
                     ChVector<>(0, 0, hdim.z * 2 + hthick));
    }
  }
  body->GetCollisionModel()->BuildModel();

  // Attach the body to the system.
  system->AddBody(body);
  return body;
}

// -----------------------------------------------------------------------------
void InitializeObject(ChSharedPtr<ChBody> body,
                      double mass,
                      ChSharedPtr<ChMaterialSurfaceBase> mat,
                      const ChVector<>& pos,
                      const ChQuaternion<>& rot,
                      bool collide,
                      bool fixed,
                      int collision_family,
                      int do_not_collide_with) {
  body->SetMass(mass);
  body->SetPos(pos);
  body->SetRot(rot);
  body->SetCollide(collide);
  body->SetBodyFixed(fixed);
  body->SetMaterialSurface(mat);
  body->GetCollisionModel()->ClearModel();
  body->GetCollisionModel()->SetFamily(collision_family);
  body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(do_not_collide_with);
}

// -----------------------------------------------------------------------------

void FinalizeObject(ChSharedPtr<ChBody> body, ChSystem* system) {
  // Infer system type and contact method.
  SystemType sysType = GetSystemType(system);
  ChBody::ContactMethod contact_method = body->GetContactMethod();

  // Verify consistency of input arguments.
  assert(((sysType == SEQUENTIAL_DVI || sysType == PARALLEL_DVI) && contact_method == ChBody::DVI) ||
    ((sysType == SEQUENTIAL_DEM || sysType == PARALLEL_DEM) && contact_method == ChBody::DEM));

  body->GetCollisionModel()->BuildModel();
  system->AddBody(body);
}

// -----------------------------------------------------------------------------

void LoadConvexMesh(const std::string& file_name,
                    ChTriangleMeshConnected& convex_mesh,
                    ChConvexDecompositionHACDv2& convex_shape,
                    const ChVector<>& pos,
                    const ChQuaternion<>& rot,
                    int hacd_maxhullcount,
                    int hacd_maxhullmerge,
                    int hacd_maxhullvertexes,
                    double hacd_concavity,
                    double hacd_smallclusterthreshold,
                    double hacd_fusetolerance) {
  convex_mesh.LoadWavefrontMesh(file_name, true, false);

  for (int i = 0; i < convex_mesh.m_vertices.size(); i++) {
    convex_mesh.m_vertices[i] = pos + rot.Rotate(convex_mesh.m_vertices[i]);
  }

  convex_shape.Reset();
  convex_shape.AddTriangleMesh(convex_mesh);
  convex_shape.SetParameters(hacd_maxhullcount, hacd_maxhullmerge, hacd_maxhullvertexes, hacd_concavity,
                             hacd_smallclusterthreshold, hacd_fusetolerance);
  convex_shape.ComputeConvexDecomposition();
}

// -----------------------------------------------------------------------------

void AddConvexCollisionModel(ChSharedPtr<ChBody> body,
                             ChTriangleMeshConnected& convex_mesh,
                             ChConvexDecompositionHACDv2& convex_shape,
                             const ChVector<>& pos,
                             const ChQuaternion<>& rot,
                             bool use_original_asset) {
  ChConvexDecomposition* used_decomposition = &convex_shape;

  int hull_count = used_decomposition->GetHullCount();

  for (int c = 0; c < hull_count; c++) {
    std::vector<ChVector<double> > convexhull;
    used_decomposition->GetConvexHullResult(c, convexhull);

    ((collision::ChCollisionModelParallel*)body->GetCollisionModel())->AddConvexHull(convexhull, pos, rot);
    // Add each convex chunk as a new asset
    if (!use_original_asset) {
      std::stringstream ss;
      ss << convex_mesh.GetFileName() << "_" << c;
      geometry::ChTriangleMeshConnected trimesh_convex;
      used_decomposition->GetConvexHullResult(c, trimesh_convex);

      ChSharedPtr<ChTriangleMeshShape> trimesh_shape(new ChTriangleMeshShape);
      trimesh_shape->SetMesh(trimesh_convex);
      trimesh_shape->SetName(ss.str());
      trimesh_shape->Pos = pos;
      trimesh_shape->Rot = rot;
      body->GetAssets().push_back(trimesh_shape);
    }
  }
  // Add the original triangle mesh as asset
  if (use_original_asset) {
    ChSharedPtr<ChTriangleMeshShape> trimesh_shape(new ChTriangleMeshShape);
    trimesh_shape->SetMesh(convex_mesh);
    trimesh_shape->SetName(convex_mesh.GetFileName());
    trimesh_shape->Pos = VNULL;
    trimesh_shape->Rot = QUNIT;
    body->GetAssets().push_back(trimesh_shape);
  }
}

}  // namespace utils
}  // namespace chrono
