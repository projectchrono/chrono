//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHMITSUBARENDER_H
#define CHMITSUBARENDER_H

//////////////////////////////////////////////////
//
//   ChPovRay.h
//
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include <fstream>
#include <string>
#include <vector>
#include <utility>
#include <sstream>
#include "xml/tinyxml2.h"
#include "physics/ChSystem.h"
#include "ChPostProcessBase.h"
#include "assets/ChSphereShape.h"
#include "assets/ChBoxShape.h"
//#include "assets/ChGenericRender.h"

namespace chrono {
namespace postprocess {

/// Class for post processing implementation that generates
/// scripts for POVray. The script can be used in POVray to
/// render photo-realistic animations.

struct xml_option {
	std::string type, parameter, value;

};

class ChApiPostProcess ChMitsubaRender: public ChPostProcessBase {
public:
	ChMitsubaRender(ChSystem* system);
	~ChMitsubaRender() {
	}
	void ExportScript(const std::string &filename);
	void ExportData(const std::string &filename);
    void ExportDriver(const std::string &filename);
	void SetIntegrator(const std::string& type, bool irrcache = false, int irrcache_res = 6) {
		integrator = type;
		use_irrcache = irrcache;
		irr_resolution = irrcache_res;
	}
	void SetIntegratorOption(const std::string& type, const std::string& parameter, const std::string& value) {
		xml_option option;
		option.type = type;
		option.parameter = parameter;
		option.value = value;
		integrator_options.push_back(option);
	}
	void SetSensor(const std::string& type) {
		sensor = type;
	}
	void SetSensorOption(const std::string& type, const std::string& parameter, const std::string& value) {
		xml_option option;
		option.type = type;
		option.parameter = parameter;
		option.value = value;
		sensor_options.push_back(option);
	}
	void SetCamera(const ChVector<>& target, const ChVector<>& origin, const ChVector<>& up) {
		camera_target = target;
		camera_origin = origin;
		camera_up = up;
	}
	void SetSampler(const std::string& type) {
		sampler = type;
	}
	void SetSamplerOption(const std::string& type, const std::string& parameter, const std::string& value) {
		xml_option option;
		option.type = type;
		option.parameter = parameter;
		option.value = value;
		sampler_options.push_back(option);
	}
	void SetFilter(const std::string& type) {
		filter = type;
	}
	void SetFilm(const std::string& type) {
		film = type;
	}
	void SetFilmOption(const std::string& type, const std::string& parameter, const std::string& value) {
		xml_option option;
		option.type = type;
		option.parameter = parameter;
		option.value = value;
		film_options.push_back(option);
	}
	void SetDataFolder(const std::string& folder) {
		data_folder = folder;
	}
    void SetRenderFolder(const std::string& folder) {
		render_folder = folder;
	}

	void AddIntegrator();
	void AddSensor();
	void AddObject(ChSharedPtr<ChAsset> & asset, ChBody * abody);

	std::string integrator, sensor, sampler, filter, film;
	bool use_irrcache;
	int height, width, irr_resolution;
	std::vector<xml_option> integrator_options, sensor_options, sampler_options, film_options;
	ChVector<> camera_target, camera_origin, camera_up;
	std::string data_folder, render_folder;
	tinyxml2::XMLDocument doc, data;
	tinyxml2::XMLElement* root, *root_data;

};

}
}
#endif
