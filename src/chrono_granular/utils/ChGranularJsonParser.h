// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Nic Olsen
// =============================================================================

#include "chrono_granular/physics/ChGranular.h"
#include "chrono_granular/ChGranularDefines.h"
#include "chrono_thirdparty/rapidjson/filereadstream.h"
#include "chrono_thirdparty/rapidjson/document.h"

#include <string>
#include <iostream>

using std::string;
using std::cout;
using std::endl;

using namespace rapidjson;

namespace chrono {
namespace granular {
typedef struct sim_param_holder {
    float sphere_radius;
    float sphere_density;
    float box_X;
    float box_Y;
    float box_Z;
    GRAN_TIME_INTEGRATOR time_integrator;
    float step_size;
    float time_end;
    float grav_X;
    float grav_Y;
    float grav_Z;
    double normalStiffS2S;
    double normalStiffS2W;
    double normalStiffS2M;
    double normalDampS2S;
    double normalDampS2W;
    double normalDampS2M;
    double tangentDampS2S;
    double tangentDampS2W;
    double tangentDampS2M;
    double tangentStiffS2S;
    double tangentStiffS2W;
    double tangentStiffS2M;
    GRAN_FRICTION_MODE friction_mode;
    float static_friction_coeffS2S;
    float static_friction_coeffS2W;
    float static_friction_coeffS2M;
    GRAN_ROLLING_MODE rolling_mode;
    float rolling_friction_coeffS2S;
    float rolling_friction_coeffS2W;
    float rolling_friction_coeffS2M;
    float cohesion_ratio;
    float adhesion_ratio_s2w;
    float adhesion_ratio_s2m;
    GRAN_VERBOSITY verbose;
    int run_mode;
    unsigned int psi_T;
    unsigned int psi_L;
    string output_dir;
    string checkpoint_file;
    GRAN_OUTPUT_MODE write_mode;
} sim_param_holder;

void ShowJSONUsage() {
    cout << "JSON fields:" << endl;
    cout << "sphere_radius" << endl;
    cout << "sphere_density" << endl;
    cout << "box_X" << endl;
    cout << "box_Y" << endl;
    cout << "box_Z" << endl;
    cout << "time_integrator (forward_euler|chung|centered_difference|extended_taylor)" << endl;
    cout << "step_size" << endl;
    cout << "time_end" << endl;
    cout << "grav_X" << endl;
    cout << "grav_Y" << endl;
    cout << "grav_Z" << endl;
    cout << "normalStiffS2S" << endl;
    cout << "normalStiffS2W" << endl;
    cout << "normalStiffS2M" << endl;
    cout << "normalDampS2S" << endl;
    cout << "normalDampS2W" << endl;
    cout << "normalDampS2M" << endl;
    cout << "tangentStiffS2S" << endl;
    cout << "tangentStiffS2W" << endl;
    cout << "tangentStiffS2M" << endl;
    cout << "tangentDampS2S" << endl;
    cout << "tangentDampS2W" << endl;
    cout << "tangentDampS2M" << endl;
    cout << "friction_mode (frictionless|single_step|multi_step)" << endl;
    cout << "static_friction_coeffS2S" << endl;
    cout << "static_friction_coeffS2W" << endl;
    cout << "static_friction_coeffS2M" << endl;
    cout << "rolling_mode (no_resistance|constant_torque|viscous)" << endl;
    cout << "rolling_friction_coeffS2S" << endl;
    cout << "rolling_friction_coeffS2W" << endl;
    cout << "rolling_friction_coeffS2M" << endl;
    cout << "cohesion_ratio" << endl;
    cout << "adhesion_ratio_s2w" << endl;
    cout << "adhesion_ratio_s2m" << endl;
    cout << "verbose" << endl;
    cout << "psi_T" << endl;
    cout << "psi_L" << endl;
    cout << "output_dir" << endl;
    cout << "checkpoint_file" << endl;
    cout << "write_mode (csv|binary|hdf5|none)" << endl;
}

void InvalidArg(string arg) {
    cout << "Invalid arg: " << arg << endl;
    ShowJSONUsage();
}

// Returns true on successful parameter load.
// Returns false and prints error on invalid argument.
bool ParseJSON(const char* json_file, sim_param_holder& params, bool verbose = true) {
    CONDITIONAL_PRINTF(verbose, "Reading parameters: %s\n", json_file);
    FILE* fp = fopen(json_file, "r");
    if (!fp) {
        cout << "Invalid JSON file" << endl;
        ShowJSONUsage();
        return false;
    }

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));
    fclose(fp);

    Document doc;
    doc.ParseStream<ParseFlag::kParseCommentsFlag>(is);
    if (!doc.IsObject()) {
        cout << "Invalid JSON file" << endl;
        return false;
    }

    CONDITIONAL_PRINTF(verbose, "--- Parsing JSON ---\n");
    if (doc.HasMember("sphere_radius") && doc["sphere_radius"].IsNumber()) {
        params.sphere_radius = doc["sphere_radius"].GetDouble();
        CONDITIONAL_PRINTF(verbose, "params.sphere_radius %f\n", params.sphere_radius);
    }
    if (doc.HasMember("sphere_density") && doc["sphere_density"].IsNumber()) {
        params.sphere_density = doc["sphere_density"].GetDouble();
        CONDITIONAL_PRINTF(verbose, "params.sphere_density %f\n", params.sphere_density);
    }
    if (doc.HasMember("box_X") && doc["box_X"].IsNumber()) {
        params.box_X = doc["box_X"].GetDouble();
        CONDITIONAL_PRINTF(verbose, "params.box_X %f\n", params.box_X);
    }
    if (doc.HasMember("box_Y") && doc["box_Y"].IsNumber()) {
        params.box_Y = doc["box_Y"].GetDouble();
        CONDITIONAL_PRINTF(verbose, "params.box_Y %f\n", params.box_Y);
    }
    if (doc.HasMember("box_Z") && doc["box_Z"].IsNumber()) {
        params.box_Z = doc["box_Z"].GetDouble();
        CONDITIONAL_PRINTF(verbose, "params.box_Z %f\n", params.box_Z);
    }
    if (doc.HasMember("time_integrator") && doc["time_integrator"].IsString()) {
        if (doc["time_integrator"].GetString() == string("forward_euler")) {
            params.time_integrator = GRAN_TIME_INTEGRATOR::FORWARD_EULER;
            CONDITIONAL_PRINTF(verbose, "params.time_integrator forward_euler\n");
        } else if (doc["time_integrator"].GetString() == string("chung")) {
            params.time_integrator = GRAN_TIME_INTEGRATOR::CHUNG;
            CONDITIONAL_PRINTF(verbose, "params.time_integrator chung\n");
        } else if (doc["time_integrator"].GetString() == string("centered_difference")) {
            params.time_integrator = GRAN_TIME_INTEGRATOR::CENTERED_DIFFERENCE;
            CONDITIONAL_PRINTF(verbose, "params.time_integrator centered_difference\n");
        } else if (doc["time_integrator"].GetString() == string("extended_taylor")) {
            params.time_integrator = GRAN_TIME_INTEGRATOR::EXTENDED_TAYLOR;
            CONDITIONAL_PRINTF(verbose, "params.time_integrator extended_taylor\n");
        } else {
            InvalidArg("time_integrator");
            return false;
        }
    }
    if (doc.HasMember("time_end") && doc["time_end"].IsNumber()) {
        params.time_end = doc["time_end"].GetDouble();
        CONDITIONAL_PRINTF(verbose, "params.time_end %f\n", params.time_end);
    }
    if (doc.HasMember("grav_X") && doc["grav_X"].IsNumber()) {
        params.grav_X = doc["grav_X"].GetDouble();
        CONDITIONAL_PRINTF(verbose, "params.grav_X %f\n", params.grav_X);
    }
    if (doc.HasMember("grav_Y") && doc["grav_Y"].IsNumber()) {
        params.grav_Y = doc["grav_Y"].GetDouble();
        CONDITIONAL_PRINTF(verbose, "params.grav_Y %f\n", params.grav_Y);
    }
    if (doc.HasMember("grav_Z") && doc["grav_Z"].IsNumber()) {
        params.grav_Z = doc["grav_Z"].GetDouble();
        CONDITIONAL_PRINTF(verbose, "params.grav_Z %f\n", params.grav_Z);
    }
    if (doc.HasMember("normalStiffS2S") && doc["normalStiffS2S"].IsNumber()) {
        params.normalStiffS2S = doc["normalStiffS2S"].GetDouble();
        CONDITIONAL_PRINTF(verbose, "params.normalStiffS2S %f\n", params.normalStiffS2S);
    }
    if (doc.HasMember("normalStiffS2W") && doc["normalStiffS2W"].IsNumber()) {
        params.normalStiffS2W = doc["normalStiffS2W"].GetDouble();
        CONDITIONAL_PRINTF(verbose, "params.normalStiffS2W %f\n", params.normalStiffS2W);
    }
    if (doc.HasMember("normalStiffS2M") && doc["normalStiffS2M"].IsNumber()) {
        params.normalStiffS2M = doc["normalStiffS2M"].GetDouble();
        CONDITIONAL_PRINTF(verbose, "params.normalStiffS2M %f\n", params.normalStiffS2M);
    }
    if (doc.HasMember("normalDampS2S") && doc["normalDampS2S"].IsNumber()) {
        params.normalDampS2S = doc["normalDampS2S"].GetDouble();
        CONDITIONAL_PRINTF(verbose, "params.normalDampS2S %f\n", params.normalDampS2S);
    }
    if (doc.HasMember("normalDampS2W") && doc["normalDampS2W"].IsNumber()) {
        params.normalDampS2W = doc["normalDampS2W"].GetDouble();
        CONDITIONAL_PRINTF(verbose, "params.normalDampS2W %f\n", params.normalDampS2W);
    }
    if (doc.HasMember("normalDampS2M") && doc["normalDampS2M"].IsNumber()) {
        params.normalDampS2M = doc["normalDampS2M"].GetDouble();
        CONDITIONAL_PRINTF(verbose, "params.normalDampS2M %f\n", params.normalDampS2M);
    }
    if (doc.HasMember("tangentStiffS2S") && doc["tangentStiffS2S"].IsNumber()) {
        params.tangentStiffS2S = doc["tangentStiffS2S"].GetDouble();
        CONDITIONAL_PRINTF(verbose, "params.tangentStiffS2S %f\n", params.tangentStiffS2S);
    }
    if (doc.HasMember("tangentStiffS2W") && doc["tangentStiffS2W"].IsNumber()) {
        params.tangentStiffS2W = doc["tangentStiffS2W"].GetDouble();
        CONDITIONAL_PRINTF(verbose, "params.tangentStiffS2W %f\n", params.tangentStiffS2W);
    }
    if (doc.HasMember("tangentStiffS2M") && doc["tangentStiffS2M"].IsNumber()) {
        params.tangentStiffS2M = doc["tangentStiffS2M"].GetDouble();
        CONDITIONAL_PRINTF(verbose, "params.tangentStiffS2M %f\n", params.tangentStiffS2M);
    }
    if (doc.HasMember("tangentDampS2S") && doc["tangentDampS2S"].IsNumber()) {
        params.tangentDampS2S = doc["tangentDampS2S"].GetDouble();
        CONDITIONAL_PRINTF(verbose, "params.tangentDampS2S %f\n", params.tangentDampS2S);
    }
    if (doc.HasMember("tangentDampS2W") && doc["tangentDampS2W"].IsNumber()) {
        params.tangentDampS2W = doc["tangentDampS2W"].GetDouble();
        CONDITIONAL_PRINTF(verbose, "params.tangentDampS2W %f\n", params.tangentDampS2W);
    }
    if (doc.HasMember("tangentDampS2M") && doc["tangentDampS2M"].IsNumber()) {
        params.tangentDampS2M = doc["tangentDampS2M"].GetDouble();
        CONDITIONAL_PRINTF(verbose, "params.tangentDampS2M %f\n", params.tangentDampS2M);
    }
    if (doc.HasMember("friction_mode") && doc["friction_mode"].IsString()) {
        if (doc["friction_mode"].GetString() == string("frictionless")) {
            params.friction_mode = GRAN_FRICTION_MODE::FRICTIONLESS;
            CONDITIONAL_PRINTF(verbose, "params.friction_mode frictionless\n");
        } else if (doc["friction_mode"].GetString() == string("single_step")) {
            params.friction_mode = GRAN_FRICTION_MODE::SINGLE_STEP;
            CONDITIONAL_PRINTF(verbose, "params.friction_mode single_step\n");
        } else if (doc["friction_mode"].GetString() == string("multi_step")) {
            params.friction_mode = GRAN_FRICTION_MODE::MULTI_STEP;
            CONDITIONAL_PRINTF(verbose, "params.friction_mode multi_step\n");
        } else {
            InvalidArg("friction_mode");
            return false;
        }
    }
    if (doc.HasMember("static_friction_coeffS2S") && doc["static_friction_coeffS2S"].IsNumber()) {
        params.static_friction_coeffS2S = doc["static_friction_coeffS2S"].GetDouble();
        CONDITIONAL_PRINTF(verbose, "params.static_friction_coeffS2S %f\n", params.static_friction_coeffS2S);
    }
    if (doc.HasMember("static_friction_coeffS2W") && doc["static_friction_coeffS2W"].IsNumber()) {
        params.static_friction_coeffS2W = doc["static_friction_coeffS2W"].GetDouble();
        CONDITIONAL_PRINTF(verbose, "params.static_friction_coeffS2W %f\n", params.static_friction_coeffS2W);
    }
    if (doc.HasMember("static_friction_coeffS2M") && doc["static_friction_coeffS2M"].IsNumber()) {
        params.static_friction_coeffS2M = doc["static_friction_coeffS2M"].GetDouble();
        CONDITIONAL_PRINTF(verbose, "params.static_friction_coeffS2M %f\n", params.static_friction_coeffS2M);
    }
    if (doc.HasMember("rolling_mode") && doc["rolling_mode"].IsString()) {
        if (doc["rolling_mode"].GetString() == string("no_resistance")) {
            params.rolling_mode = GRAN_ROLLING_MODE::NO_RESISTANCE;
            CONDITIONAL_PRINTF(verbose, "params.rolling_mode no_resistance\n");
        } else if (doc["rolling_mode"].GetString() == string("constant_torque")) {
            params.rolling_mode = GRAN_ROLLING_MODE::CONSTANT_TORQUE;
            CONDITIONAL_PRINTF(verbose, "params.rolling_mode constant_torque\n");
        } else if (doc["rolling_mode"].GetString() == string("viscous")) {
            params.rolling_mode = GRAN_ROLLING_MODE::VISCOUS;
            CONDITIONAL_PRINTF(verbose, "params.rolling_mode viscous\n");
        } else {
            InvalidArg("rolling_mode");
            return false;
        }
    }
    if (doc.HasMember("rolling_friction_coeffS2S") && doc["rolling_friction_coeffS2S"].IsNumber()) {
        params.rolling_friction_coeffS2S = doc["rolling_friction_coeffS2S"].GetDouble();
        CONDITIONAL_PRINTF(verbose, "params.rolling_friction_coeffS2S %f\n", params.rolling_friction_coeffS2S);
    }
    if (doc.HasMember("rolling_friction_coeffS2W") && doc["rolling_friction_coeffS2W"].IsNumber()) {
        params.rolling_friction_coeffS2W = doc["rolling_friction_coeffS2W"].GetDouble();
        CONDITIONAL_PRINTF(verbose, "params.rolling_friction_coeffS2W %f\n", params.rolling_friction_coeffS2W);
    }
    if (doc.HasMember("rolling_friction_coeffS2M") && doc["rolling_friction_coeffS2M"].IsNumber()) {
        params.rolling_friction_coeffS2M = doc["rolling_friction_coeffS2M"].GetDouble();
        CONDITIONAL_PRINTF(verbose, "params.rolling_friction_coeffS2M %f\n", params.rolling_friction_coeffS2M);
    }
    if (doc.HasMember("cohesion_ratio") && doc["cohesion_ratio"].IsNumber()) {
        params.cohesion_ratio = doc["cohesion_ratio"].GetDouble();
        CONDITIONAL_PRINTF(verbose, "params.cohesion_ratio %f\n", params.cohesion_ratio);
    }
    if (doc.HasMember("adhesion_ratio_s2w") && doc["adhesion_ratio_s2w"].IsNumber()) {
        params.adhesion_ratio_s2w = doc["adhesion_ratio_s2w"].GetDouble();
        CONDITIONAL_PRINTF(verbose, "params.adhesion_ratio_s2w %f\n", params.adhesion_ratio_s2w);
    }
    if (doc.HasMember("adhesion_ratio_s2m") && doc["adhesion_ratio_s2m"].IsNumber()) {
        params.adhesion_ratio_s2m = doc["adhesion_ratio_s2m"].GetDouble();
        CONDITIONAL_PRINTF(verbose, "params.adhesion_ratio_s2m %f\n", params.adhesion_ratio_s2m);
    }
    if (doc.HasMember("verbose") && doc["verbose"].IsInt()) {
        params.verbose = (GRAN_VERBOSITY)doc["verbose"].GetInt();
        CONDITIONAL_PRINTF(verbose, "params.verbose %d\n", params.verbose);
    }
    if (doc.HasMember("psi_T") && doc["psi_T"].IsInt()) {
        params.psi_T = doc["psi_T"].GetInt();
        CONDITIONAL_PRINTF(verbose, "params.psi_T %d\n", params.psi_T);
    }
    if (doc.HasMember("psi_L") && doc["psi_L"].IsInt()) {
        params.psi_L = doc["psi_L"].GetInt();
        CONDITIONAL_PRINTF(verbose, "params.psi_L %d\n", params.psi_L);
    }
    if (doc.HasMember("run_mode") && doc["run_mode"].IsInt()) {
        params.run_mode = doc["run_mode"].GetInt();
        CONDITIONAL_PRINTF(verbose, "params.run_mode %d\n", params.run_mode);
    }
    if (doc.HasMember("output_dir") && doc["output_dir"].IsString()) {
        params.output_dir = doc["output_dir"].GetString();
        CONDITIONAL_PRINTF(verbose, "params.output_dir %s\n", params.output_dir.c_str());
    }
    if (doc.HasMember("checkpoint_file") && doc["checkpoint_file"].IsString()) {
        params.checkpoint_file = doc["checkpoint_file"].GetString();
        CONDITIONAL_PRINTF(verbose, "params.checkpoint_file %s\n", params.checkpoint_file.c_str());
    }
    if (doc.HasMember("write_mode") && doc["write_mode"].IsString()) {
        if (doc["write_mode"].GetString() == string("binary")) {
            params.write_mode = GRAN_OUTPUT_MODE::BINARY;
            CONDITIONAL_PRINTF(verbose, "params.write_mode binary\n");
        } else if (doc["write_mode"].GetString() == string("csv")) {
            params.write_mode = GRAN_OUTPUT_MODE::CSV;
            CONDITIONAL_PRINTF(verbose, "params.write_mode csv\n");
        } else if (doc["write_mode"].GetString() == string("hdf5")) {
            params.write_mode = GRAN_OUTPUT_MODE::HDF5;
            CONDITIONAL_PRINTF(verbose, "params.write_mode hdf5\n")
        } else if (doc["write_mode"].GetString() == string("none")) {
            params.write_mode = GRAN_OUTPUT_MODE::NONE;
            CONDITIONAL_PRINTF(verbose, "params.write_mode none\n");
        } else {
            InvalidArg("write_mode");
            return false;
        }
    }
    if (doc.HasMember("step_size") && doc["step_size"].IsNumber()) {
        params.step_size = doc["step_size"].GetDouble();
        CONDITIONAL_PRINTF(verbose, "params.step_size %f\n", params.step_size);
    }

    CONDITIONAL_PRINTF(verbose, "--------------------\n");

    // TODO sanity checks
    // necessary parameters
    // step_mode
    return true;
}
}  // namespace granular
}  // namespace chrono