//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2012 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

//////////////////////////////////////////////////
//
//   ChPovRay.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
//create geometry asset

#include "ChMitsubaRender.h"

using namespace chrono;
using namespace chrono::postprocess;
using namespace geometry;
using namespace std;
using namespace tinyxml2;

void ChMitsubaRender::AddIntegrator() {

	XMLElement* xml = doc.NewElement("integrator");
	xml->SetAttribute("type", integrator.c_str());

	for (int i = 0; i < integrator_options.size(); i++) {
		XMLElement* option = doc.NewElement(integrator_options[i].type.c_str());
		option->SetAttribute("name", integrator_options[i].parameter.c_str());
		option->SetAttribute("value", integrator_options[i].value.c_str());
		xml->LinkEndChild(option);
	}
	root->LinkEndChild(xml);
}

void ChMitsubaRender::AddSensor() {
	XMLElement* xml_sensor = doc.NewElement("sensor");
	xml_sensor->SetAttribute("type", sensor.c_str());
	for (int i = 0; i < sensor_options.size(); i++) {
		XMLElement* xml_sensor_option = doc.NewElement(sensor_options[i].type.c_str());
		xml_sensor_option->SetAttribute(sensor_options[i].parameter.c_str(), sensor_options[i].value.c_str());
		xml_sensor->LinkEndChild(xml_sensor_option);
	}
	{
		XMLElement* xml_transform = doc.NewElement("transform");
		xml_transform->SetAttribute("name", "toWorld");
		XMLElement* xml_camera = doc.NewElement("lookat");

		char t[50], o[50], u[50];

		sprintf(t, "%f , %f, %f", camera_target.x, camera_target.y, camera_target.z);
		sprintf(o, "%f , %f, %f", camera_origin.x, camera_origin.y, camera_origin.z);
		sprintf(u, "%f , %f, %f", camera_up.x, camera_up.y, camera_up.z);
		xml_camera->SetAttribute("target", t);
		xml_camera->SetAttribute("origin", o);
		xml_camera->SetAttribute("up", u);
		xml_transform->LinkEndChild(xml_camera);
		xml_sensor->LinkEndChild(xml_transform);
	}
	XMLElement* xml_sampler = doc.NewElement("sampler");
	xml_sampler->SetAttribute("type", sampler.c_str());
	xml_sensor->LinkEndChild(xml_sampler);

	XMLElement* xml_film = doc.NewElement("film");
	xml_film->SetAttribute("type", film.c_str());
	for (int i = 0; i < film_options.size(); i++) {
		XMLElement* xml_film_option = doc.NewElement(film_options[i].type.c_str());
		xml_film_option->SetAttribute("name", film_options[i].parameter.c_str());
		xml_film_option->SetAttribute("value", film_options[i].value.c_str());
		xml_film->LinkEndChild(xml_film_option);
	}

	XMLElement* xml_filter = doc.NewElement("rfilter");
	xml_filter->SetAttribute("type", filter.c_str());
	xml_film->LinkEndChild(xml_filter);

	xml_sensor->LinkEndChild(xml_film);

	root->LinkEndChild(xml_sensor);
}

void ChMitsubaRender::AddObject(ChSharedPtr<ChAsset> & asset, ChBody * abody) {

	if (asset.IsType<ChSphereShape>()) {
		XMLElement* xml = data.NewElement("shape");
		xml->SetAttribute("type", "sphere");

		ChSphereShape * sphere_shape = ((ChSphereShape *) (asset.get_ptr()));

		XMLElement* xml_pos = data.NewElement("float");
		xml_pos->SetAttribute("name", "radius");
		xml_pos->SetAttribute("value", sphere_shape->GetSphereGeometry().rad);
		xml->LinkEndChild(xml_pos);

		Vector center = sphere_shape->GetSphereGeometry().center;
		center = abody->GetRot().Rotate(center);

		XMLElement* xml_transform = data.NewElement("transform");
		xml_transform->SetAttribute("name", "toWorld");

		xml_pos = data.NewElement("translate");
		xml_pos->SetAttribute("x", double(abody->GetPos().x + center.x));
		xml_pos->SetAttribute("y", double(abody->GetPos().y + center.x));
		xml_pos->SetAttribute("z", double(abody->GetPos().z + center.x));
		xml_transform->LinkEndChild(xml_pos);
		//xml_pos = data.NewElement("scale");
		//xml_pos->SetAttribute("value", sphere_shape->GetSphereGeometry().rad);
		//xml_transform->LinkEndChild(xml_pos);
		xml->LinkEndChild(xml_transform);
		root_data->LinkEndChild(xml);

	}
	//if (asset.IsType<ChGenericRender>()) {

	//ChGenericRender * obj_shape = ((ChGenericRender *) (asset.get_ptr()));

	//xml->SetAttribute("type", "serialized");

	//xml_name = data.NewElement("string");
	//xml_name->SetAttribute("name", "filename");
	//xml_name->SetAttribute("value", obj_shape->model_file);
	//xml->LinkEndChild(xml_name);

	//XMLElement* xml_pos = data.NewElement("point");
	//xml_pos->SetAttribute("name", "center");
	//xml_pos->SetAttribute("x", double(obj_shape->GetBoxGeometry().Pos.x));
	//xml_pos->SetAttribute("y", double(obj_shape->GetBoxGeometry().Pos.y));
	//xml_pos->SetAttribute("z", double(obj_shape->GetBoxGeometry().Pos.z));
	//xml->LinkEndChild(xml_pos);

	//xml_pos = data.NewElement("float");
	//xml_pos->SetAttribute("name", "radius");
	//xml_pos->SetAttribute("value", box_shape->GetSphereGeometry().rad);
	//xml->LinkEndChild(xml_pos);
	//}

}

ChMitsubaRender::ChMitsubaRender(ChSystem* system) :
		ChPostProcessBase(system) {

	integrator = "photon";
	sensor = "perspective";
	sampler = "ldsampler";
	filter = "gaussian";
	film = "hdrfilm";
	camera_target = ChVector<>(0, 0, 0);
	camera_origin = ChVector<>(-1, 0, 0);
	camera_up = ChVector<>(0, 1, 0);

}

void ChMitsubaRender::ExportScript(const std::string &filename) {
	static const char* xml = "<?xml version=\"1.0\"?>";
	doc.Parse(xml);
	root = doc.NewElement("scene");
	root->SetAttribute("version", "0.4.0");

	AddIntegrator();
	AddSensor();
	//if (data_file.size() > 0) {
	XMLElement* xml_include = doc.NewElement("include");
	stringstream ss;
	ss << data_folder << "/$file.xml";

	xml_include->SetAttribute("filename", ss.str().c_str());
	root->LinkEndChild(xml_include);
	//}

	doc.InsertEndChild(root);
	doc.SaveFile(filename.c_str());

}
void ChMitsubaRender::ExportData(const std::string &filename) {
	static const char* xml = "<?xml version=\"1.0\"?>";
	data.Parse(xml);
	root_data = data.NewElement("scene");
	root_data->SetAttribute("version", "0.4.0");

	/*XMLElement* xml_shape = data.NewElement("shape");
	 xml_shape->SetAttribute("type", "shapegroup");
	 xml_shape->SetAttribute("id", "cube_shape");
	 {
	 XMLElement* new_shape = data.NewElement("shape");
	 new_shape->SetAttribute("type", "serialized");
	 XMLElement* xml_name = data.NewElement("string");
	 xml_name->SetAttribute("name", "filename");
	 xml_name->SetAttribute("value", "cube.serialized");
	 new_shape->LinkEndChild(xml_name);
	 xml_shape->LinkEndChild(new_shape);
	 }
	 root_data->LinkEndChild(xml_shape);
	 xml_shape = data.NewElement("shape");
	 xml_shape->SetAttribute("type", "shapegroup");
	 xml_shape->SetAttribute("id", "cylinder_shape");
	 {
	 XMLElement* new_shape = data.NewElement("shape");
	 new_shape->SetAttribute("type", "serialized");
	 XMLElement* xml_name = data.NewElement("string");
	 xml_name->SetAttribute("name", "filename");
	 xml_name->SetAttribute("value", "cylinder.serialized");
	 new_shape->LinkEndChild(xml_name);
	 xml_shape->LinkEndChild(new_shape);
	 }
	 root_data->LinkEndChild(xml_shape);

	 xml_shape = data.NewElement("shape");
	 xml_shape->SetAttribute("type", "shapegroup");
	 xml_shape->SetAttribute("id", "sphere_shape");
	 {
	 XMLElement* new_shape = data.NewElement("shape");
	 new_shape->SetAttribute("type", "sphere");
	 XMLElement* xml_name = data.NewElement("float");
	 xml_name->SetAttribute("name", "radius");
	 xml_name->SetAttribute("value", "1");
	 new_shape->LinkEndChild(xml_name);
	 xml_shape->LinkEndChild(new_shape);
	 }
	 root_data->LinkEndChild(xml_shape);*/
	for (int i = 0; i < mSystem->Get_bodylist()->size(); i++) {
		ChBody* abody = (ChBody*) mSystem->Get_bodylist()->at(i);
		if (abody->GetAssets().size() > 0) {
			for (int j = 0; j < abody->GetAssets().size(); j++) {
				ChSharedPtr<ChAsset> asset = abody->GetAssets().at(j);
				AddObject(asset, abody);
			}
		}
	}

	data.InsertEndChild(root_data);
	data.SaveFile(filename.c_str());
}

void ChMitsubaRender::ExportDriver(const std::string &filename) {
	ofstream of(filename.c_str());

	of << "#!/bin/sh" << endl;
	of << "for i in {0..999}" << endl;
	of << "do" << endl;
	of << "/Applications/Mitsuba.app/Contents/MacOS/mitsuba -D  file=$i -o " << render_folder << "/$i.png test.xml" << endl;
	of << "done" << endl;

}

