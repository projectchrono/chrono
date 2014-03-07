#ifndef UTILS_INOUT_H
#define UTILS_INOUT_H

#include <string>
#include <iostream>
#include <sstream> 
#include <fstream>

#include "assets/ChSphereShape.h"
#include "assets/ChBoxShape.h"
#include "assets/ChEllipsoidShape.h"
#include "assets/ChCylinderShape.h"
#include "assets/ChConeShape.h"

#include "ChParallelDefines.h"


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
template <typename T>
void WriteBodies(T*                 mSys,
                 const std::string& filename,
                 bool               active_only = false,
                 bool               dump_vel = false,
                 const std::string& delim = ",")
{
	CSV_writer csv(delim);

	for (int i = 0; i < mSys->Get_bodylist()->size(); i++) {
		ChBody* abody = mSys->Get_bodylist()->at(i);
		if (active_only && !abody->IsActive())
			continue;
		csv << abody->GetPos() << abody->GetRot();
		if (dump_vel)
			csv << abody->GetPos_dt() << abody->GetWvel_loc();
		csv << std::endl;
	}

	csv.write_to_file(filename);
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
template <typename T>
void WriteShapesRender(T*                 mSys,
                       const std::string& filename,
                       const std::string& delim = ",")
{
	CSV_writer csv(delim);

	int index = 0;

	for (int i = 0; i < mSys->Get_bodylist()->size(); i++) {
		ChBody* abody = mSys->Get_bodylist()->at(i);
		const Vector& body_pos = abody->GetPos();
		const Quaternion& body_rot = abody->GetRot();
		int bodyId = abody->GetIdentifier();

		for (int j = 0; j < abody->GetAssets().size(); j++) {
			ChSharedPtr<ChAsset> asset = abody->GetAssets().at(j);
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
//    p.x,p.y,p.z,q.e0,q.e1,q.e2,q.e3,v.x,v.y,v.z,type,geometry
// where 'geometry' depends on 'type' (an enum).
// -------------------------------------------------------------------------------
template <typename T>
void WriteShapesPovray(T*                 mSys,
                       const std::string& filename,
                       const std::string& delim = ",")
{
	CSV_writer csv(delim);

	for (int i = 0; i < mSys->Get_bodylist()->size(); i++) {
		ChBody* abody = mSys->Get_bodylist()->at(i);
		const Vector&     body_pos = abody->GetPos();
		const Quaternion& body_rot = abody->GetRot();
		const Vector&     body_vel = abody->GetPos_dt();

		for (int j = 0; j < abody->GetAssets().size(); j++) {
			ChSharedPtr<ChAsset> asset = abody->GetAssets().at(j);
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

			csv << pos << rot << body_vel << geometry.str() << std::endl;
		}
	}

	csv.write_to_file(filename);
}


} // end namespace utils
} // end namespace chrono


#endif
