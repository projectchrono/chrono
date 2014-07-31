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
void WriteCheckpoint(ChSystem*          system,
                     const std::string& filename)
{
  CSV_writer csv(" ");

  for (int i = 0; i < system->Get_bodylist()->size(); i++) {
    ChBody* body = system->Get_bodylist()->at(i);

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

    // Write number of visual assets
    int num_visual_assets = 0;
    for (int j = 0; j < body->GetAssets().size(); j++) {
      ChSharedPtr<ChAsset> asset = body->GetAssets().at(j);
      if (asset.IsType<ChVisualization>())
        num_visual_assets++;
    }
    csv << num_visual_assets << std::endl;

    // Loop over each asset and, for visual assets, write its data on a separate line
    for (int j = 0; j < body->GetAssets().size(); j++) {
      ChSharedPtr<ChAsset> asset = body->GetAssets().at(j);
      ChSharedPtr<ChVisualization> visual_asset = asset.DynamicCastTo<ChVisualization>();
      if (visual_asset.IsNull())
        continue;

      // Write relative position and rotation
      csv << visual_asset->Pos << visual_asset->Rot.Get_A_quaternion();

      // Write shape type and geometry data
      if (ChSharedPtr<ChSphereShape> sphere = asset.DynamicCastTo<ChSphereShape>())
      {
        csv << collision::SPHERE << sphere->GetSphereGeometry().rad;
      }
      else if (ChSharedPtr<ChEllipsoidShape> ellipsoid = asset.DynamicCastTo<ChEllipsoidShape>())
      {
        csv << collision::ELLIPSOID << ellipsoid->GetEllipsoidGeometry().rad;
      }
      else if (ChSharedPtr<ChBoxShape> box = asset.DynamicCastTo<ChBoxShape>())
      {
        csv << collision::BOX << box->GetBoxGeometry().Size;
      }
      else if (ChSharedPtr<ChCapsuleShape> capsule = asset.DynamicCastTo<ChCapsuleShape>())
      {
        const geometry::ChCapsule& geom = capsule->GetCapsuleGeometry();
        csv << collision::CAPSULE << geom.rad << geom.hlen;
      }
      else if (ChSharedPtr<ChCylinderShape> cylinder = asset.DynamicCastTo<ChCylinderShape>())
      {
        const geometry::ChCylinder& geom = cylinder->GetCylinderGeometry();
        csv << collision::CYLINDER << geom.rad << (geom.p1.y - geom.p2.y) / 2;
      }
      else if (ChSharedPtr<ChConeShape> cone = asset.DynamicCastTo<ChConeShape>())
      {
        const geometry::ChCone& geom = cone->GetConeGeometry();
        csv << collision::CONE << geom.rad.x << geom.rad.y;
      }

      csv << std::endl;
    }
  }

  csv.write_to_file(filename);
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
    body->SetBodyFixed(bfixed);
    body->SetCollide(bcollide);

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
      case collision::CYLINDER:
        {
          double radius, hlen;
          iss >> radius >> hlen;
          AddCylinderGeometry(body, radius, hlen, apos, arot);
        }
        break;
      case collision::CONE:
        {
          double radius, height;
          iss >> radius >> height;
          AddConeGeometry(body, radius, height, apos, arot);
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
// Each line contains information about one visualization asset shape, as
// follows:
//    index, x, y, z, e0, e1, e2, e3, type, geometry
// where 'geometry' depends on 'type' (an enum).
// -----------------------------------------------------------------------------
void WriteShapesPovray(ChSystem*          system,
                       const std::string& filename,
                       const std::string& delim)
{
  CSV_writer csv(delim);

  int count = 0;

  for (int i = 0; i < system->Get_bodylist()->size(); i++) {
    ChBody* body = system->Get_bodylist()->at(i);
    const Vector&     body_pos = body->GetPos();
    const Quaternion& body_rot = body->GetRot();
    const Vector&     body_vel = body->GetPos_dt();

    for (int j = 0; j < body->GetAssets().size(); j++) {
      ChSharedPtr<ChAsset> asset = body->GetAssets().at(j);
      ChSharedPtr<ChVisualization> visual_asset = asset.DynamicCastTo<ChVisualization>();
      if (visual_asset.IsNull())
        continue;

      const Vector& asset_pos = visual_asset->Pos;
      Quaternion    asset_rot = visual_asset->Rot.Get_A_quaternion();

      Vector     pos = body_pos + body_rot.Rotate(asset_pos);
      Quaternion rot = body_rot % asset_rot;

      std::stringstream gss;

      if (ChSharedPtr<ChSphereShape> sphere = asset.DynamicCastTo<ChSphereShape>())
      {
        gss << collision::SPHERE << delim << sphere->GetSphereGeometry().rad;
        count++;
      }
      else if (ChSharedPtr<ChEllipsoidShape> ellipsoid = asset.DynamicCastTo<ChEllipsoidShape>()) {
        const Vector& size = ellipsoid->GetEllipsoidGeometry().rad;
        gss << collision::ELLIPSOID << delim << size.x << delim << size.y << delim << size.z;
        count++;
      }
      else if (ChSharedPtr<ChBoxShape> box = asset.DynamicCastTo<ChBoxShape>())
      {
        const Vector& size = box->GetBoxGeometry().Size;
        gss << collision::BOX << delim << size.x << delim << size.y << delim << size.z;
        count++;
      }
      else if (ChSharedPtr<ChCapsuleShape> capsule = asset.DynamicCastTo<ChCapsuleShape>())
      {
        const geometry::ChCapsule& geom = capsule->GetCapsuleGeometry();
        gss << collision::CAPSULE << delim << geom.rad << delim << geom.hlen;
        count++;
      }
      else if (ChSharedPtr<ChCylinderShape> cylinder = asset.DynamicCastTo<ChCylinderShape>())
      {
        const geometry::ChCylinder& geom = cylinder->GetCylinderGeometry();
        gss << collision::CYLINDER << delim << geom.rad << delim << (geom.p1.y - geom.p2.y) / 2;
        count++;
      }
      else if (ChSharedPtr<ChConeShape> cone = asset.DynamicCastTo<ChConeShape>())
      {
        const geometry::ChCone& geom = cone->GetConeGeometry();
        gss << collision::CONE << delim << geom.rad.x << delim << geom.rad.y;
        count++;
      }
      else if (ChSharedPtr<ChTriangleMeshShape> mesh = asset.DynamicCastTo<ChTriangleMeshShape>())
      {
        gss << collision::TRIANGLEMESH << delim << "\"" << mesh->GetName() << "\"";
        count++;
      }

      csv << body->GetIdentifier() << body->IsActive() << pos << rot << gss.str() << std::endl;

    }
  }

  std::stringstream header;
  header << count << delim << std::endl;

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

  ofile << "#macro " << mesh_name << "(col)" << std::endl;

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
  ofile << "      pigment {color col}" << std::endl;
  ofile << "      finish  {ambient 0.2  diffuse 0.7}" << std::endl;
  ofile << "    }" << std::endl;
  ofile << "}" << std::endl;
  ofile << "#end" << std::endl;
}


}  // namespace utils
}  // namespace chrono