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
// Authors: Radu Serban
// =============================================================================
//
// A set of utility classes and functions for I/O.
//
// =============================================================================

#include "assets/ChColorAsset.h"

#include "utils/ChUtilsInputOutput.h"

namespace chrono {
namespace utils {


// -----------------------------------------------------------------------------
// WriteBodies
//
// Write to a CSV file pody position, orientation, and (optionally) linear and
// angular velocity. Optionally, only active bodies are processed.
// -----------------------------------------------------------------------------
void WriteBodies(ChSystem*          system,
                 const std::string& filename,
                 bool               active_only,
                 bool               dump_vel,
                 const std::string& delim)
{
  CSV_writer csv(delim);

  for (int i = 0; i < system->Get_bodylist()->size(); i++) {
    ChBody* body = system->Get_bodylist()->at(i);
    if (active_only && !body->IsActive())
      continue;
    csv << body->GetPos() << body->GetRot();
    if (dump_vel)
      csv << body->GetPos_dt() << body->GetWvel_loc();
    csv << std::endl;
  }

  csv.write_to_file(filename);
}


// -----------------------------------------------------------------------------
// WriteCheckpoint
//
// Create a CSV file with a checkpoint ...
// -----------------------------------------------------------------------------
bool WriteCheckpoint(ChSystem*          system,
                     const std::string& filename)
{
  CSV_writer csv(" ");

  std::vector<ChBody*>::iterator ibody = system->Get_bodylist()->begin();
  for (; ibody != system->Get_bodylist()->end(); ++ibody)
  {
    ChBody* body = *ibody;

    // Infer body type (0: DVI, 1:DEM)
    int btype = (dynamic_cast<ChBodyDEM*>(body)) ? 1 : 0;

    // Write body type, body identifier, the body fixed flag, and the collide flag
    csv << btype << body->GetIdentifier() << body->GetBodyFixed() << body->GetCollide();
    
    // Write body mass and inertia
    csv << body->GetMass() << body->GetInertiaXX();

    // Write body position, orientation, and their time derivatives
    csv << body->GetPos() << body->GetRot();
    csv << body->GetPos_dt() << body->GetRot_dt();

    csv << std::endl;

    // Write material information
    if (btype == 0) {
      // Write DVI material surface information
      ChSharedPtr<ChMaterialSurface>& mat = body->GetMaterialSurface();
      csv << mat->static_friction << mat->sliding_friction << mat->rolling_friction << mat->spinning_friction;
      csv << mat->restitution << mat->cohesion << mat->dampingf;
      csv << mat->compliance << mat->complianceT << mat->complianceRoll << mat->complianceSpin;
    } else {
      // Write DEM material surface information
      ChSharedPtr<ChMaterialSurfaceDEM>& mat = static_cast<ChBodyDEM*>(body)->GetMaterialSurfaceDEM();
      csv << mat->young_modulus << mat->poisson_ratio;
      csv << mat->static_friction << mat->sliding_friction;
      csv << mat->restitution << mat->dissipation_factor;
      csv << mat->cohesion;
    }

    csv << std::endl;

    // Count and write all visual assets.
    int num_visual_assets = 0;
    std::vector<ChSharedPtr<ChAsset> >::iterator iasset = (*ibody)->GetAssets().begin();
    for (; iasset != (*ibody)->GetAssets().end(); ++iasset)
    {
      if ((*iasset).IsType<ChVisualization>())
        num_visual_assets++;
    }
    csv << num_visual_assets << std::endl;

    // Loop over each asset and, for selected visual assets only, write its data
    // on a separate line. If we encounter an unsupported type, return false.
    iasset = (*ibody)->GetAssets().begin();
    for (; iasset != (*ibody)->GetAssets().end(); ++iasset)
    {
      ChSharedPtr<ChVisualization> visual_asset = (*iasset).DynamicCastTo<ChVisualization>();
      if (visual_asset.IsNull())
        continue;

      // Write relative position and rotation
      csv << visual_asset->Pos << visual_asset->Rot.Get_A_quaternion();

      // Write shape type and geometry data
      if (ChSharedPtr<ChSphereShape> sphere = visual_asset.DynamicCastTo<ChSphereShape>())
      {
        csv << collision::SPHERE << sphere->GetSphereGeometry().rad;
      }
      else if (ChSharedPtr<ChEllipsoidShape> ellipsoid = visual_asset.DynamicCastTo<ChEllipsoidShape>())
      {
        csv << collision::ELLIPSOID << ellipsoid->GetEllipsoidGeometry().rad;
      }
      else if (ChSharedPtr<ChBoxShape> box = visual_asset.DynamicCastTo<ChBoxShape>())
      {
        csv << collision::BOX << box->GetBoxGeometry().Size;
      }
      else if (ChSharedPtr<ChCapsuleShape> capsule = visual_asset.DynamicCastTo<ChCapsuleShape>())
      {
        const geometry::ChCapsule& geom = capsule->GetCapsuleGeometry();
        csv << collision::CAPSULE << geom.rad << geom.hlen;
      }
      ////else if (ChSharedPtr<ChCylinderShape> cylinder = visual_asset.DynamicCastTo<ChCylinderShape>())
      ////{
      ////  const geometry::ChCylinder& geom = cylinder->GetCylinderGeometry();
      ////  csv << collision::CYLINDER << geom.rad << (geom.p1.y - geom.p2.y) / 2;
      ////}
      else if (ChSharedPtr<ChConeShape> cone = visual_asset.DynamicCastTo<ChConeShape>())
      {
        const geometry::ChCone& geom = cone->GetConeGeometry();
        csv << collision::CONE << geom.rad.x << geom.rad.y;
      }
      else if (ChSharedPtr<ChRoundedBoxShape> rbox = visual_asset.DynamicCastTo<ChRoundedBoxShape>())
      {
        const geometry::ChRoundedBox& geom = rbox->GetRoundedBoxGeometry();
        csv << collision::ROUNDEDBOX << geom.Size << geom.radsphere;
      }
      else if (ChSharedPtr<ChRoundedCylinderShape> rcyl = visual_asset.DynamicCastTo<ChRoundedCylinderShape>())
      {
        const geometry::ChRoundedCylinder& geom = rcyl->GetRoundedCylinderGeometry();
        csv << collision::ROUNDEDCYL << geom.rad << geom.hlen << geom.radsphere;
      }
      else
      {
        // Unsupported visual asset type.
        return false;
      }

      csv << std::endl;
    }
  }

  csv.write_to_file(filename);

  return true;
}


// -----------------------------------------------------------------------------
// ReadCheckpoint
//
//
// -----------------------------------------------------------------------------
void ReadCheckpoint(ChSystem*          system,
                    const std::string& filename)
{
  // Open input file stream
  std::ifstream      ifile(filename.c_str());
  std::string        line;

  while (std::getline(ifile, line)) {
    std::istringstream iss1(line);

    // Read body type, Id, flags
    int btype, bid, bfixed, bcollide;
    iss1 >> btype >> bid >> bfixed >> bcollide;

    // Read body mass and inertia
    double     mass;
    ChVector<> inertiaXX;
    iss1 >> mass >> inertiaXX.x >> inertiaXX.y >> inertiaXX.z;

    // Read body position, orientation, and their time derivatives
    ChVector<>     bpos, bpos_dt;
    ChQuaternion<> brot, brot_dt;
    iss1 >> bpos.x >> bpos.y >> bpos.z >> brot.e0 >> brot.e1 >> brot.e2 >> brot.e3;
    iss1 >> bpos_dt.x >> bpos_dt.y >> bpos_dt.z >> brot_dt.e0 >> brot_dt.e1 >> brot_dt.e2 >> brot_dt.e3;

    // Get the next line in the file (material properties)
    std::getline(ifile, line);
    std::istringstream iss2(line);

    // Create a body of the appropriate type, read and apply material properties
    ChBody* body;
    if (btype == 0) {
      body = new ChBody();
      ChSharedPtr<ChMaterialSurface>& mat = body->GetMaterialSurface();
      iss2 >> mat->static_friction >> mat->sliding_friction >> mat->rolling_friction >> mat->spinning_friction;
      iss2 >> mat->restitution >> mat->cohesion >> mat->dampingf;
      iss2 >> mat->compliance >> mat->complianceT >> mat->complianceRoll >> mat->complianceSpin;
    } else {
      body = new ChBodyDEM();
      ChSharedPtr<ChMaterialSurfaceDEM>& mat = static_cast<ChBodyDEM*>(body)->GetMaterialSurfaceDEM();
      iss2 >> mat->young_modulus >> mat->poisson_ratio;
      iss2 >> mat->static_friction >> mat->sliding_friction;
      iss2 >> mat->restitution >> mat->dissipation_factor;
      iss2 >> mat->cohesion;
    }

    // Set body properties and state
    body->SetPos(bpos);
    body->SetRot(brot);
    body->SetPos_dt(bpos_dt);
    body->SetRot_dt(brot_dt);

    body->SetIdentifier(bid);
    body->SetBodyFixed(bfixed != 0);
    body->SetCollide(bcollide != 0);

    body->SetMass(mass);
    body->SetInertiaXX(inertiaXX);

    // Get next line in the file (number of visualization assets)
    std::getline(ifile, line);
    std::istringstream iss3(line);

    int numAssets;
    iss3 >> numAssets;

    // In a loop, read information about each asset and add geometry to the body
    body->GetCollisionModel()->ClearModel();

    for (int j = 0; j < numAssets; j++) {
      std::getline(ifile, line);
      std::istringstream iss(line);

      // Get relative position and rotation
      ChVector<>     apos;
      ChQuaternion<> arot;
      iss >> apos.x >> apos.y >> apos.z >> arot.e0 >> arot.e1 >> arot.e2 >> arot.e3;

      // Get visualization asset type and geometry data.
      // Create the appropriate 
      int atype;
      iss >> atype;

      switch (collision::ShapeType(atype)) {
      case collision::SPHERE:
        {
          double radius;
          iss >> radius;
          AddSphereGeometry(body, radius, apos, arot);
        }
        break;
      case collision::ELLIPSOID:
        {
          ChVector<> size;
          iss >> size.x >> size.y >> size.z;
          AddEllipsoidGeometry(body, size, apos, arot);
        }
        break;
      case collision::BOX:
        {
          ChVector<> size;
          iss >> size.x >> size.y >> size.z;
          AddBoxGeometry(body, size, apos, arot);
        }
        break;
      case collision::CAPSULE:
        {
          double radius, hlen;
          iss >> radius >> hlen;
          AddCapsuleGeometry(body, radius, hlen, apos, arot);
        }
        break;
      ////case collision::CYLINDER:
      ////  {
      ////    double radius, hlen;
      ////    iss >> radius >> hlen;
      ////    AddCylinderGeometry(body, radius, hlen, apos, arot);
      ////  }
      ////  break;
      case collision::CONE:
        {
          double radius, height;
          iss >> radius >> height;
          AddConeGeometry(body, radius, height, apos, arot);
        }
        break;
      case collision::ROUNDEDBOX:
        {
          ChVector<> size;
          double srad;
          iss >> size.x >> size.y >> size.z >> srad;
          AddRoundedBoxGeometry(body, size, srad, apos, arot);
        }
        break;
        case collision::ROUNDEDCYL:
        {
          double radius, hlen, srad;
          iss >> radius >> hlen >> srad;
          AddRoundedCylinderGeometry(body, radius, hlen, srad, apos, arot);
        }
        break;
      }
    }

    body->GetCollisionModel()->BuildModel();

    // Attach the body to the system.
    system->AddBody(ChSharedPtr<ChBody>(body));
  }
}


// -----------------------------------------------------------------------------
// WriteShapesPovray
//
// Write CSV output file for PovRay.
// First line contains the number of visual assets and links to follow.
// A line with information about a visualization asset contains:
//    bodyId, bodyActive, x, y, z, e0, e1, e2, e3, shapeType, [shape Data]
// A line with information about a link contains:
//    linkType, [linkData]
//
// NOTE: we do not account for any transform specified for the ChGeometry of
// a visual asset (except for cylinders where that is implicit)!
// -----------------------------------------------------------------------------
void WriteShapesPovray(ChSystem*          system,
                       const std::string& filename,
                       bool               body_info,
                       const std::string& delim)
{
  CSV_writer csv(delim);

  // If requested, Loop over all bodies and write out their position and
  // orientation.  Otherwise, body count is left at 0.
  int b_count = 0;

  if (body_info) {
    std::vector<ChBody*>::iterator ibody = system->Get_bodylist()->begin();
    for (; ibody != system->Get_bodylist()->end(); ++ibody)
    {
      const ChVector<>& body_pos = (*ibody)->GetFrame_REF_to_abs().GetPos();
      const ChQuaternion<>& body_rot = (*ibody)->GetFrame_REF_to_abs().GetRot();

      csv << (*ibody)->GetIdentifier() << (*ibody)->IsActive() << body_pos << body_rot << std::endl;

      b_count++;
    }
  }

  // Loop over all bodies and over all their assets.
  int a_count = 0;
  std::vector<ChBody*>::iterator ibody = system->Get_bodylist()->begin();
  for (; ibody != system->Get_bodylist()->end(); ++ibody)
  {
    const ChVector<>& body_pos = (*ibody)->GetFrame_REF_to_abs().GetPos();
    const ChQuaternion<>& body_rot = (*ibody)->GetFrame_REF_to_abs().GetRot();

    ChColor color(0.8f, 0.8f, 0.8f);

    // First loop over assets -- search for a color asset
    std::vector<ChSharedPtr<ChAsset> >::iterator iasset = (*ibody)->GetAssets().begin();
    for (; iasset != (*ibody)->GetAssets().end(); ++iasset)
    {
      if (ChSharedPtr<ChColorAsset> color_asset = (*iasset).DynamicCastTo<ChColorAsset>())
        color = color_asset->GetColor();
    }

    // Loop over assets once again -- write information for supported types.
    iasset = (*ibody)->GetAssets().begin();
    for (; iasset != (*ibody)->GetAssets().end(); ++iasset)
    {
      ChSharedPtr<ChVisualization> visual_asset = (*iasset).DynamicCastTo<ChVisualization>();
      if (visual_asset.IsNull())
        continue;

      const Vector& asset_pos = visual_asset->Pos;
      Quaternion    asset_rot = visual_asset->Rot.Get_A_quaternion();

      Vector     pos = body_pos + body_rot.Rotate(asset_pos);
      Quaternion rot = body_rot % asset_rot;

      std::stringstream gss;

      if (ChSharedPtr<ChSphereShape> sphere = visual_asset.DynamicCastTo<ChSphereShape>())
      {
        gss << collision::SPHERE << delim << sphere->GetSphereGeometry().rad;
        a_count++;
      }
      else if (ChSharedPtr<ChEllipsoidShape> ellipsoid = visual_asset.DynamicCastTo<ChEllipsoidShape>()) {
        const Vector& size = ellipsoid->GetEllipsoidGeometry().rad;
        gss << collision::ELLIPSOID << delim << size.x << delim << size.y << delim << size.z;
        a_count++;
      }
      else if (ChSharedPtr<ChBoxShape> box = visual_asset.DynamicCastTo<ChBoxShape>())
      {
        const Vector& size = box->GetBoxGeometry().Size;
        gss << collision::BOX << delim << size.x << delim << size.y << delim << size.z;
        a_count++;
      }
      else if (ChSharedPtr<ChCapsuleShape> capsule = visual_asset.DynamicCastTo<ChCapsuleShape>())
      {
        const geometry::ChCapsule& geom = capsule->GetCapsuleGeometry();
        gss << collision::CAPSULE << delim << geom.rad << delim << geom.hlen;
        a_count++;
      }
      else if (ChSharedPtr<ChCylinderShape> cylinder = visual_asset.DynamicCastTo<ChCylinderShape>())
      {
        const geometry::ChCylinder& geom = cylinder->GetCylinderGeometry();
        gss << collision::CYLINDER << delim << geom.rad << delim
            << geom.p1.x << delim << geom.p1.y << delim << geom.p1.z << delim
            << geom.p2.x << delim << geom.p2.y << delim << geom.p2.z;
        a_count++;
      }
      else if (ChSharedPtr<ChConeShape> cone = visual_asset.DynamicCastTo<ChConeShape>())
      {
        const geometry::ChCone& geom = cone->GetConeGeometry();
        gss << collision::CONE << delim << geom.rad.x << delim << geom.rad.y;
        a_count++;
      }
      else if (ChSharedPtr<ChRoundedBoxShape> rbox = visual_asset.DynamicCastTo<ChRoundedBoxShape>())
      {
        const geometry::ChRoundedBox& geom = rbox->GetRoundedBoxGeometry();
        gss << collision::ROUNDEDBOX << delim << geom.Size.x << delim << geom.Size.y << delim << geom.Size.z << delim << geom.radsphere;
        a_count++;
      }
      else if (ChSharedPtr<ChRoundedCylinderShape> rcyl = visual_asset.DynamicCastTo<ChRoundedCylinderShape>())
      {
        const geometry::ChRoundedCylinder& geom = rcyl->GetRoundedCylinderGeometry();
        gss << collision::ROUNDEDCYL << delim << geom.rad << delim << geom.hlen << delim << geom.radsphere;
        a_count++;
      }
      else if (ChSharedPtr<ChTriangleMeshShape> mesh = visual_asset.DynamicCastTo<ChTriangleMeshShape>())
      {
        gss << collision::TRIANGLEMESH << delim << "\"" << mesh->GetName() << "\"";
        a_count++;
      }

      csv << (*ibody)->GetIdentifier() << (*ibody)->IsActive() 
          << pos << rot 
          << color
          << gss.str() << std::endl;
    }
  }

  // Loop over all links.  Write information on selected types of links.
  int l_count = 0;
  std::list<ChLink*>::iterator ilink = system->Get_linklist()->begin();
  for (; ilink != system->Get_linklist()->end(); ++ilink)
  {
    int type = (*ilink)->GetType();

    if (ChLinkLockRevolute* link = dynamic_cast<ChLinkLockRevolute*>(*ilink))
    {
      chrono::ChFrame<> frA_abs = *(link->GetMarker1()) >> *(link->GetBody1());
      chrono::ChFrame<> frB_abs = *(link->GetMarker2()) >> *(link->GetBody2());

      csv << type << frA_abs.GetPos() << frA_abs.GetA()->Get_A_Zaxis() << std::endl;
      l_count++;
    }
    else if (ChLinkLockSpherical* link = dynamic_cast<ChLinkLockSpherical*>(*ilink))
    {
      chrono::ChFrame<> frA_abs = *(link->GetMarker1()) >> *(link->GetBody1());
      chrono::ChFrame<> frB_abs = *(link->GetMarker2()) >> *(link->GetBody2());

      csv << type << frA_abs.GetPos() << std::endl;
      l_count++;
    }
    else if (ChLinkLockUniversal* link = dynamic_cast<ChLinkLockUniversal*>(*ilink))
    {
      chrono::ChFrame<> frA_abs = *(link->GetMarker1()) >> *(link->GetBody1());
      chrono::ChFrame<> frB_abs = *(link->GetMarker2()) >> *(link->GetBody2());

      csv << type << frA_abs.GetPos() << frA_abs.GetA()->Get_A_Xaxis() << frA_abs.GetA()->Get_A_Yaxis() << std::endl;
      l_count++;
    }
    else if (ChLinkSpring* link = dynamic_cast<ChLinkSpring*>(*ilink))
    {
      chrono::ChFrame<> frA_abs = *(link->GetMarker1()) >> *(link->GetBody1());
      chrono::ChFrame<> frB_abs = *(link->GetMarker2()) >> *(link->GetBody2());

      csv << type << frA_abs.GetPos() << frB_abs.GetPos() << std::endl;
      l_count++;
    }
    else if (ChLinkSpringCB* link = dynamic_cast<ChLinkSpringCB*>(*ilink))
    {
      chrono::ChFrame<> frA_abs = *(link->GetMarker1()) >> *(link->GetBody1());
      chrono::ChFrame<> frB_abs = *(link->GetMarker2()) >> *(link->GetBody2());

      csv << type << frA_abs.GetPos() << frB_abs.GetPos() << std::endl;
      l_count++;
    }
    else if (ChLinkDistance* link = dynamic_cast<ChLinkDistance*>(*ilink))
    {
      csv << type << link->GetEndPoint1Abs() << link->GetEndPoint2Abs() << std::endl;
      l_count++;
    }
    else if (ChLinkEngine* link = dynamic_cast<ChLinkEngine*>(*ilink))
    {
      chrono::ChFrame<> frA_abs = *(link->GetMarker1()) >> *(link->GetBody1());
      chrono::ChFrame<> frB_abs = *(link->GetMarker2()) >> *(link->GetBody2());

      csv << type << frA_abs.GetPos() << frA_abs.GetA()->Get_A_Zaxis() << std::endl;
      l_count++;
    }

  }

  // Write the output file, including a first line with number of bodies, visual
  // assets, and links.
  std::stringstream header;
  header << b_count << delim << a_count << delim << l_count << delim << std::endl;

  csv.write_to_file(filename, header.str());
}


// -----------------------------------------------------------------------------
// WriteMeshPovray
//
// Write the triangular mesh from the specified OBJ file as a macro in a PovRay
// include file.
// -----------------------------------------------------------------------------
void WriteMeshPovray(const std::string&    obj_filename,
                     const std::string&    mesh_name,
                     const std::string&    out_dir,
                     const ChColor&        col,
                     const ChVector<>&     pos,
                     const ChQuaternion<>& rot)
{
  // Read trimesh from OBJ file
  geometry::ChTriangleMeshConnected trimesh;
  trimesh.LoadWavefrontMesh(obj_filename, false, false);

  // Transform vertices.
  for (int i = 0; i < trimesh.m_vertices.size(); i++)
    trimesh.m_vertices[i] = pos + rot.Rotate(trimesh.m_vertices[i]);

  // Open output file.
  std::string pov_filename = out_dir + "/" + mesh_name + ".inc";
  std::ofstream  ofile(pov_filename.c_str());

  ofile << "#macro " << mesh_name << "()" << std::endl;

  // Write vertices.
  for (int i = 0; i < trimesh.m_vertices.size(); i++) {
    ChVector<> v = trimesh.m_vertices[i];
    ofile << "#local v" << i << " = <" << v.x << ", " << v.z << ", " << v.y << ">;" << std::endl;
  }

  // Write face connectivity.
  ofile << "mesh {" << std::endl;

  for (int i = 0; i < trimesh.m_face_v_indices.size(); i++) {
    ChVector<int> face = trimesh.m_face_v_indices[i];
    ofile << "   triangle {";
    ofile << "v" << face.x << ", v" << face.y << ", v" << face.z;
    ofile << "}" << std::endl;
  }

  ofile << "   texture {" << std::endl;
  ofile << "      pigment {color rgb<" << col.R << ", " << col.G << ", " << col.B << ">}" << std::endl;
  ofile << "      finish  {phong 0.2  diffuse 0.6}" << std::endl;
  ofile << "    }" << std::endl;
  ofile << "}" << std::endl;
  ofile << "#end" << std::endl;
}


}  // namespace utils
}  // namespace chrono