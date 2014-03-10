#ifndef CH_UTILS_INOUT_H
#define CH_UTILS_INOUT_H

#include <string>
#include <iostream>
#include <sstream> 
#include <fstream>

#include "physics/ChSystem.h"

#include "ChParallelDefines.h"

#include "utils/creators.h"


namespace chrono {
namespace utils {


// -------------------------------------------------------------------------------
// CSV_writer
//
// Simple class to output to a Comma-Separated Values file.
// -------------------------------------------------------------------------------
class CSV_writer {
public:
	explicit CSV_writer(const std::string& delim = ",") : m_delim(delim) {}
	~CSV_writer() {}

	void write_to_file(const string& filename)
	{
		std::ofstream ofile(filename.c_str());
		ofile << m_ss.str();
		ofile.close();
	}

	const std::string&  delim() const {return m_delim;}
	std::ostringstream& stream() {return m_ss;}

	template <typename T>
	CSV_writer& operator<< (const T& t)                          {m_ss << t << m_delim; return *this;}
	
	CSV_writer& operator<<(std::ostream& (*t)(std::ostream&))    {m_ss << t; return *this;}
	CSV_writer& operator<<(std::ios& (*t)(std::ios&))            {m_ss << t; return *this;}
	CSV_writer& operator<<(std::ios_base& (*t)(std::ios_base&))  {m_ss << t; return *this;}

private:
	std::string m_delim;
	std::ostringstream m_ss;
};


CSV_writer& operator<< (CSV_writer& out, const ChVector<>& v)
{
   out << v.x << v.y << v.z;
   return out;
}

CSV_writer& operator<< (CSV_writer& out, const ChQuaternion<>& q)
{
   out << q.e0 << q.e1 << q.e2 << q.e3;
   return out;
}

CSV_writer& operator<< (CSV_writer& out, const real2& r)
{
   out << r.x << r.y;
   return out;
}

CSV_writer& operator<< (CSV_writer& out, const real3& r)
{
   out << r.x << r.y << r.z ;
   return out;
}

CSV_writer& operator<< (CSV_writer& out, const real4& r)
{
   out << r.w << r.x << r.y << r.z ;
   return out;
}


// -------------------------------------------------------------------------------
// WriteBodies
//
// This function dumps to a CSV file pody position, orientation, and (optionally)
// linear and angular velocity. Optionally, only active bodies are processed.
// -------------------------------------------------------------------------------
void WriteBodies(ChSystem*          system,
                 const std::string& filename,
                 bool               active_only = false,
                 bool               dump_vel = false,
                 const std::string& delim = ",")
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


// -------------------------------------------------------------------------------
// WriteCheckpoint
//
// Create a CSV file with a checkpoint ...
// -------------------------------------------------------------------------------
void WriteCheckpoint(ChSystem*          system,
                     const std::string& filename)
{
	CSV_writer csv(" ");

	for (int i = 0; i < system->Get_bodylist()->size(); i++) {
		ChBody* body = system->Get_bodylist()->at(i);

		// Infer body type (0: DVI, 1:DEM)
		int btype = 0;
		if (dynamic_cast<ChBodyDEM*>(body))  btype = 1;

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
			ChSharedPtr<ChMaterialSurfaceDEM>& mat = ((ChBodyDEM*) body)->GetMaterialSurfaceDEM();
			csv << mat->young_modulus << mat->poisson_ratio;
			csv << mat->static_friction << mat->sliding_friction;
			csv << mat->restitution << mat->dissipation_factor;
		}

		csv << std::endl;

		// Write number of assets
		csv << body->GetAssets().size() << std::endl;

		// Loop over each asset and, for visual assets, write its data on a separate line
		for (int j = 0; j < body->GetAssets().size(); j++) {
			ChSharedPtr<ChAsset> asset = body->GetAssets().at(j);
			ChVisualization* visual_asset = dynamic_cast<ChVisualization*>(asset.get_ptr());
			if (!visual_asset)  continue;

			// Write relative position and rotation
			csv << visual_asset->Pos << visual_asset->Rot.Get_A_quaternion();

			// Write shape type and geometry data
			if (asset.IsType<ChSphereShape>()) {
				ChSphereShape* sphere = (ChSphereShape*) asset.get_ptr();
				csv << chrono::SPHERE << sphere->GetSphereGeometry().rad;
			} else if (asset.IsType<ChEllipsoidShape>()) {
				ChEllipsoidShape* ellipsoid = (ChEllipsoidShape*) asset.get_ptr();
				csv << chrono::ELLIPSOID << ellipsoid->GetEllipsoidGeometry().rad;
			} else if (asset.IsType<ChBoxShape>()) {
				ChBoxShape* box = (ChBoxShape*) asset.get_ptr();
				csv << chrono::BOX << box->GetBoxGeometry().Size;
			} else if (asset.IsType<ChCylinderShape>()) {
				ChCylinderShape* cylinder = (ChCylinderShape*) asset.get_ptr();
				double rad = cylinder->GetCylinderGeometry().rad;
				double height = cylinder->GetCylinderGeometry().p1.y - cylinder->GetCylinderGeometry().p2.y;
				csv << chrono::CYLINDER << rad << height;
			} else if (asset.IsType<ChConeShape>()) {
				ChConeShape* cone = (ChConeShape*) asset.get_ptr();
				csv << chrono::CONE << cone->GetConeGeometry().rad.x << cone->GetConeGeometry().rad.y;
			}

			csv << std::endl;
		}
	}

	csv.write_to_file(filename);
}


// -------------------------------------------------------------------------------
// ReadCheckpoint
//
//
// -------------------------------------------------------------------------------
void ReadCheckpoint(ChSystem*          system,
                    const std::string& filename)
{
	// Infer system type (0: sequential, 1: parallel)
	int stype = 0;
	if (dynamic_cast<ChSystemParallelDVI*>(system) || dynamic_cast<ChSystemParallelDEM*>(system))
		stype = 1;

	// Open input file stream
	std::ifstream      ifile(filename.c_str());
	std::string        line;
	std::istringstream iss;

	while (std::getline(ifile, line)) {
		iss = std::istringstream(line);

		// Read body type, Id, flags
		int btype, bid, bfixed, bcollide;
		iss >> btype >> bid >> bfixed >> bcollide;

		// Read body mass and inertia
		double     mass;
		ChVector<> inertiaXX;
		iss >> mass >> inertiaXX.x >> inertiaXX.y >> inertiaXX.z;

		// Read body position, orientation, and their time derivatives
		ChVector<>     bpos, bpos_dt;
		ChQuaternion<> brot, brot_dt;
		iss >> bpos.x >> bpos.y >> bpos.z >> brot.e0 >> brot.e1 >> brot.e2 >> brot.e3;
		iss >> bpos_dt.x >> bpos_dt.y >> bpos_dt.z >> brot_dt.e0 >> brot_dt.e1 >> brot_dt.e2 >> brot_dt.e3;

		// Get the next line in the file (material properties)
		std::getline(ifile, line);
		iss = std::istringstream(line);

		// Create the body of the appropriate type, read and apply material properties
		ChBody* body;
		if (btype == 0) {
			body = (stype == 0) ? new ChBody() : new ChBody(new ChCollisionModelParallel);
			ChSharedPtr<ChMaterialSurface>& mat = body->GetMaterialSurface();
			iss >> mat->static_friction >> mat->sliding_friction >> mat->rolling_friction >> mat->spinning_friction;
			iss >> mat->restitution >> mat->cohesion >> mat->dampingf;
			iss >> mat->compliance >> mat->complianceT >> mat->complianceRoll >> mat->complianceSpin;
		} else {
			body = (stype == 0) ? new ChBodyDEM() : new ChBodyDEM(new ChCollisionModelParallel);
			ChSharedPtr<ChMaterialSurfaceDEM>& mat = ((ChBodyDEM*) body)->GetMaterialSurfaceDEM();
			iss >> mat->young_modulus >> mat->poisson_ratio;
			iss >> mat->static_friction >> mat->sliding_friction;
			iss >> mat->restitution >> mat->dissipation_factor;
		}

		// Set body properties and state
		body->SetPos(bpos);
		body->SetRot(brot);
		body->SetPos_dt(bpos_dt);
		body->SetRot_dt(brot_dt);

		body->SetBodyFixed(bfixed);
		body->SetCollide(bcollide);

		body->SetMass(mass);
		body->SetInertiaXX(inertiaXX);

		// Get next line in the file (number of visualization assets)
		std::getline(ifile, line);
		iss = std::istringstream(line);

		int numAssets;
		iss >> numAssets;

		// In a loop, read information about each asset and add geometry to the body.
		body->GetCollisionModel()->ClearModel();

		for (int j = 0; j < numAssets; j++) {
			std::getline(ifile, line);
			iss = std::istringstream(line);

			// Get relative position and rotation
			ChVector<>     apos;
			ChQuaternion<> arot;
			iss >> apos.x >> apos.y >> apos.z >> arot.e0 >> arot.e1 >> arot.e2 >> arot.e3;

			// Get visualization asset type and geometry data.
			// Create the appropriate 
			int atype;
			iss >> atype;

			switch (ShapeType(atype)) {
			case chrono::SPHERE:
				{
					double radius;
					iss >> radius;
					AddSphereGeometry(body, radius, apos, arot);
				}
				break;
			case chrono::ELLIPSOID:
				{
					ChVector<> size;
					iss >> size.x >> size.y >> size.z;
					AddEllipsoidGeometry(body, size, apos, arot);
				}
				break;
			case chrono::BOX:
				{
					ChVector<> size;
					iss >> size.x >> size.y >> size.z;
					AddBoxGeometry(body, size, apos, arot);
				}
				break;
			case chrono::CYLINDER:
				{
					double radius, height;
					iss >> radius >> height;
					AddCylinderGeometry(body, radius, height, apos, arot);
				}
				break;
			case chrono::CONE:
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
		ChSharedPtr<ChBody>  bodyPtr(body);
		if (stype == 0)
			system->AddBody(bodyPtr);
		else
			((ChSystemParallel*) system)->AddBody(bodyPtr);
	}
}


////  TODO:  figure out the Chrono inconsistent mess with relative transforms
////                body -> asset -> shape

// -------------------------------------------------------------------------------
// WriteShapesRender
//
// Write CSV output file for the Blender plugin.
// Each line contains information about one visualization asset shape, as follows:
//    group,index,p.x,p.y,p.z,q.e0,q.e1,q.e2,q.e3,type,geometry
// where 'geometry' depends on 'type' (a string).
// All shapes of the same 'type' are placed in the same 'group', unless the body
// has a negative identifier, in which case the shapes are tagged as 'individual'
// -------------------------------------------------------------------------------
void WriteShapesRender(ChSystem*          system,
                       const std::string& filename,
                       const std::string& delim = ",")
{
	CSV_writer csv(delim);

	int index = 0;

	for (int i = 0; i < system->Get_bodylist()->size(); i++) {
		ChBody* body = system->Get_bodylist()->at(i);
		const Vector& body_pos = body->GetPos();
		const Quaternion& body_rot = body->GetRot();
		int bodyId = body->GetIdentifier();

		for (int j = 0; j < body->GetAssets().size(); j++) {
			ChSharedPtr<ChAsset> asset = body->GetAssets().at(j);
			ChVisualization* visual_asset = dynamic_cast<ChVisualization*>(asset.get_ptr());
			if (!visual_asset)
				continue;

			const Vector& asset_pos = visual_asset->Pos;
			Quaternion    asset_rot = visual_asset->Rot.Get_A_quaternion();

			Vector     pos = body_pos + body_rot.Rotate(asset_pos);
			Quaternion rot = body_rot % asset_rot;

			std::string group;
			std::stringstream geometry;

			if (asset.IsType<ChSphereShape>()) {
				ChSphereShape* sphere = (ChSphereShape*) asset.get_ptr();
				group = bodyId < 0 ? "individual" : "g_sphere";
				geometry << "sphere" << delim << sphere->GetSphereGeometry().rad;
			} else if (asset.IsType<ChEllipsoidShape>()) {
				ChEllipsoidShape* ellipsoid = (ChEllipsoidShape*) asset.get_ptr();
				const Vector& size = ellipsoid->GetEllipsoidGeometry().rad;
				group = bodyId < 0 ? "individual" : "g_ellipsoid";
				geometry << "ellipsoid" << delim << size.x << delim << size.y << delim << size.z;
			} else if (asset.IsType<ChBoxShape>()) {
				ChBoxShape* box = (ChBoxShape*) asset.get_ptr();
				const Vector& size = box->GetBoxGeometry().Size;
				group = bodyId < 0 ? "individual" : "g_box";
				geometry << "box" << delim << size.x << delim << size.y << delim << size.z;
			} else if (asset.IsType<ChCylinderShape>()) {
				ChCylinderShape* cylinder = (ChCylinderShape*) asset.get_ptr();
				double rad = cylinder->GetCylinderGeometry().rad;
				double height = cylinder->GetCylinderGeometry().p1.y - cylinder->GetCylinderGeometry().p2.y;
				group = bodyId < 0 ? "individual" : "g_cylinder";
				geometry << "cylinder" << delim << rad << delim << height;
			} else if (asset.IsType<ChConeShape>()) {
				ChConeShape* cone = (ChConeShape*) asset.get_ptr();
				const Vector& size = cone->GetConeGeometry().rad;
				group = bodyId < 0 ? "individual" : "g_cone";
				geometry << "cone" << delim << size.x << delim << size.y;
			}

			csv << group << index << pos << rot << geometry.str() << std::endl;

			index++;
		}
	}

	csv.write_to_file(filename);
}


////  TODO:  figure out the Chrono inconsistent mess with relative transforms
////                body -> asset -> shape
////         what velocity is expected here? body or shape? Currently using
////         body velocity (probably wrong)

// -------------------------------------------------------------------------------
// WriteShapesPovray
//
// Write CSV output file for PovRay.
// Each line contains information about one visualization asset shape, as follows:
//    index, x, y, z, e0, e1, e2, e3, type, geometry
// where 'geometry' depends on 'type' (an enum).
// -------------------------------------------------------------------------------
void WriteShapesPovray(ChSystem*          system,
                       const std::string& filename,
                       const std::string& delim = ",")
{
	CSV_writer csv(delim);

	int index = 0;

	for (int i = 0; i < system->Get_bodylist()->size(); i++) {
		ChBody* body = system->Get_bodylist()->at(i);
		const Vector&     body_pos = body->GetPos();
		const Quaternion& body_rot = body->GetRot();
		const Vector&     body_vel = body->GetPos_dt();

		for (int j = 0; j < body->GetAssets().size(); j++) {
			ChSharedPtr<ChAsset> asset = body->GetAssets().at(j);
			ChVisualization* visual_asset = dynamic_cast<ChVisualization*>(asset.get_ptr());
			if (!visual_asset)
				continue;

			const Vector& asset_pos = visual_asset->Pos;
			Quaternion    asset_rot = visual_asset->Rot.Get_A_quaternion();

			Vector     pos = body_pos + body_rot.Rotate(asset_pos);
			Quaternion rot = body_rot % asset_rot;

			std::stringstream geometry;

			if (asset.IsType<ChSphereShape>()) {
				ChSphereShape* sphere = (ChSphereShape*) asset.get_ptr();
				geometry << SPHERE << delim << sphere->GetSphereGeometry().rad;
			} else if (asset.IsType<ChEllipsoidShape>()) {
				ChEllipsoidShape* ellipsoid = (ChEllipsoidShape*) asset.get_ptr();
				const Vector& size = ellipsoid->GetEllipsoidGeometry().rad;
				geometry << ELLIPSOID << delim << size.x << delim << size.y << delim << size.z;
			} else if (asset.IsType<ChBoxShape>()) {
				ChBoxShape* box = (ChBoxShape*) asset.get_ptr();
				const Vector& size = box->GetBoxGeometry().Size;
				geometry << BOX << delim << size.x << delim << size.y << delim << size.z;
			} else if (asset.IsType<ChCylinderShape>()) {
				ChCylinderShape* cylinder = (ChCylinderShape*) asset.get_ptr();
				double rad = cylinder->GetCylinderGeometry().rad;
				double height = cylinder->GetCylinderGeometry().p1.y - cylinder->GetCylinderGeometry().p2.y;
				geometry << CYLINDER << delim << rad << delim << height;
			} else if (asset.IsType<ChConeShape>()) {
				ChConeShape* cone = (ChConeShape*) asset.get_ptr();
				const Vector& size = cone->GetConeGeometry().rad;
				geometry << CONE << delim << size.x << delim << size.y;
			}

			csv << index << pos << rot << geometry.str() << std::endl;

			index++;
		}
	}

	csv.write_to_file(filename);
}


} // end namespace utils
} // end namespace chrono


#endif
