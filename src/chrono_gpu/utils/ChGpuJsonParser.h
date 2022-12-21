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

#pragma once

#include <string>
#include <iostream>
#include <string>

#include "chrono_gpu/ChGpuDefines.h"

#include "chrono_thirdparty/rapidjson/filereadstream.h"
#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace gpu {

/// @addtogroup gpu_utils
/// @{

/// Structure with Chrono::Gpu simulation parameters.
struct ChGpuSimulationParameters {
    float sphere_radius;
    float sphere_density;
    float box_X;
    float box_Y;
    float box_Z;
    CHGPU_TIME_INTEGRATOR time_integrator;
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
    CHGPU_FRICTION_MODE friction_mode;
    float static_friction_coeffS2S;
    float static_friction_coeffS2W;
    float static_friction_coeffS2M;
    CHGPU_ROLLING_MODE rolling_mode;
    float rolling_friction_coeffS2S;
    float rolling_friction_coeffS2W;
    float rolling_friction_coeffS2M;
    float cohesion_ratio;
    float adhesion_ratio_s2w;
    float adhesion_ratio_s2m;
    CHGPU_VERBOSITY verbose;
    CHGPU_RUN_MODE run_mode;
    unsigned int psi_T;
    unsigned int psi_L;
    float psi_R;
    std::string output_dir;
    std::string checkpoint_file;
    CHGPU_OUTPUT_MODE write_mode;
};

/// Print scheme for JSON file with simulation settings.
void ShowJSONUsage() {
    std::cout << "JSON fields:" << std::endl;
    std::cout << "sphere_radius" << std::endl;
    std::cout << "sphere_density" << std::endl;
    std::cout << "box_X" << std::endl;
    std::cout << "box_Y" << std::endl;
    std::cout << "box_Z" << std::endl;
    std::cout << "time_integrator (forward_euler|chung|centered_difference|extended_taylor)" << std::endl;
    std::cout << "step_size" << std::endl;
    std::cout << "time_end" << std::endl;
    std::cout << "grav_X" << std::endl;
    std::cout << "grav_Y" << std::endl;
    std::cout << "grav_Z" << std::endl;
    std::cout << "normalStiffS2S" << std::endl;
    std::cout << "normalStiffS2W" << std::endl;
    std::cout << "normalStiffS2M" << std::endl;
    std::cout << "normalDampS2S" << std::endl;
    std::cout << "normalDampS2W" << std::endl;
    std::cout << "normalDampS2M" << std::endl;
    std::cout << "tangentStiffS2S" << std::endl;
    std::cout << "tangentStiffS2W" << std::endl;
    std::cout << "tangentStiffS2M" << std::endl;
    std::cout << "tangentDampS2S" << std::endl;
    std::cout << "tangentDampS2W" << std::endl;
    std::cout << "tangentDampS2M" << std::endl;
    std::cout << "friction_mode (frictionless|single_step|multi_step)" << std::endl;
    std::cout << "static_friction_coeffS2S" << std::endl;
    std::cout << "static_friction_coeffS2W" << std::endl;
    std::cout << "static_friction_coeffS2M" << std::endl;
    std::cout << "rolling_mode (no_resistance|schwartz)" << std::endl;
    std::cout << "rolling_friction_coeffS2S" << std::endl;
    std::cout << "rolling_friction_coeffS2W" << std::endl;
    std::cout << "rolling_friction_coeffS2M" << std::endl;
    std::cout << "cohesion_ratio" << std::endl;
    std::cout << "adhesion_ratio_s2w" << std::endl;
    std::cout << "adhesion_ratio_s2m" << std::endl;
    std::cout << "verbose" << std::endl;
    std::cout << "psi_T" << std::endl;
    std::cout << "psi_L" << std::endl;
    std::cout << "output_dir" << std::endl;
    std::cout << "checkpoint_file" << std::endl;
    std::cout << "write_mode (csv|binary|hdf5|none)" << std::endl;
}

/// Flag an invalid simulation parameter.
void InvalidArg(const std::string& arg) {
    std::cout << "Invalid arg: " << arg << std::endl;
    ShowJSONUsage();
}

/// Parse the specified JSON file with simulation settings.
/// Returns true on successful parameter load. Returns false and prints error on invalid argument.
bool ParseJSON(const std::string& json_file, ChGpuSimulationParameters& params, bool verbose = true) {
    CONDITIONAL_PRINTF(verbose, "Reading parameters: %s\n", json_file.c_str());
    FILE* fp = fopen(json_file.c_str(), "r");
    if (!fp) {
        std::cout << "Invalid JSON file" << std::endl;
        ShowJSONUsage();
        return false;
    }

    char readBuffer[16000];
    rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));
    fclose(fp);

    rapidjson::Document doc;
    doc.ParseStream<rapidjson::ParseFlag::kParseCommentsFlag>(is);
    if (!doc.IsObject()) {
        std::cout << "Invalid JSON file" << std::endl;
        return false;
    }

    CONDITIONAL_PRINTF(verbose, "--- Parsing JSON ---\n");
    if (doc.HasMember("sphere_radius") && doc["sphere_radius"].IsNumber()) {
        params.sphere_radius = doc["sphere_radius"].GetFloat();
        CONDITIONAL_PRINTF(verbose, "params.sphere_radius %f\n", params.sphere_radius);
    }
    if (doc.HasMember("sphere_density") && doc["sphere_density"].IsNumber()) {
        params.sphere_density = doc["sphere_density"].GetFloat();
        CONDITIONAL_PRINTF(verbose, "params.sphere_density %f\n", params.sphere_density);
    }
    if (doc.HasMember("box_X") && doc["box_X"].IsNumber()) {
        params.box_X = doc["box_X"].GetFloat();
        CONDITIONAL_PRINTF(verbose, "params.box_X %f\n", params.box_X);
    }
    if (doc.HasMember("box_Y") && doc["box_Y"].IsNumber()) {
        params.box_Y = doc["box_Y"].GetFloat();
        CONDITIONAL_PRINTF(verbose, "params.box_Y %f\n", params.box_Y);
    }
    if (doc.HasMember("box_Z") && doc["box_Z"].IsNumber()) {
        params.box_Z = doc["box_Z"].GetFloat();
        CONDITIONAL_PRINTF(verbose, "params.box_Z %f\n", params.box_Z);
    }
    if (doc.HasMember("time_integrator") && doc["time_integrator"].IsString()) {
        std::string time_integrator = doc["time_integrator"].GetString();
        if (time_integrator == std::string("forward_euler"))
            params.time_integrator = CHGPU_TIME_INTEGRATOR::FORWARD_EULER;
        else if (time_integrator == std::string("chung"))
            params.time_integrator = CHGPU_TIME_INTEGRATOR::CHUNG;
        else if (time_integrator == std::string("centered_difference"))
            params.time_integrator = CHGPU_TIME_INTEGRATOR::CENTERED_DIFFERENCE;
        else if (time_integrator == std::string("extended_taylor"))
            params.time_integrator = CHGPU_TIME_INTEGRATOR::EXTENDED_TAYLOR;
        else {
            InvalidArg("time_integrator");
            return false;
        }
        CONDITIONAL_PRINTF(verbose, "params.time_integrator %s\n", time_integrator.c_str());
    }
    if (doc.HasMember("time_end") && doc["time_end"].IsNumber()) {
        params.time_end = doc["time_end"].GetFloat();
        CONDITIONAL_PRINTF(verbose, "params.time_end %f\n", params.time_end);
    }
    if (doc.HasMember("grav_X") && doc["grav_X"].IsNumber()) {
        params.grav_X = doc["grav_X"].GetFloat();
        CONDITIONAL_PRINTF(verbose, "params.grav_X %f\n", params.grav_X);
    }
    if (doc.HasMember("grav_Y") && doc["grav_Y"].IsNumber()) {
        params.grav_Y = doc["grav_Y"].GetFloat();
        CONDITIONAL_PRINTF(verbose, "params.grav_Y %f\n", params.grav_Y);
    }
    if (doc.HasMember("grav_Z") && doc["grav_Z"].IsNumber()) {
        params.grav_Z = doc["grav_Z"].GetFloat();
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
        std::string friction_mode = doc["friction_mode"].GetString();
        if (friction_mode == std::string("frictionless"))
            params.friction_mode = CHGPU_FRICTION_MODE::FRICTIONLESS;
        else if (friction_mode == std::string("single_step"))
            params.friction_mode = CHGPU_FRICTION_MODE::SINGLE_STEP;
        else if (friction_mode == std::string("multi_step"))
            params.friction_mode = CHGPU_FRICTION_MODE::MULTI_STEP;
        else {
            InvalidArg("friction_mode");
            return false;
        }
        CONDITIONAL_PRINTF(verbose, "params.friction_mode %s\n", friction_mode.c_str());
    }
    if (doc.HasMember("static_friction_coeffS2S") && doc["static_friction_coeffS2S"].IsNumber()) {
        params.static_friction_coeffS2S = doc["static_friction_coeffS2S"].GetFloat();
        CONDITIONAL_PRINTF(verbose, "params.static_friction_coeffS2S %f\n", params.static_friction_coeffS2S);
    }
    if (doc.HasMember("static_friction_coeffS2W") && doc["static_friction_coeffS2W"].IsNumber()) {
        params.static_friction_coeffS2W = doc["static_friction_coeffS2W"].GetFloat();
        CONDITIONAL_PRINTF(verbose, "params.static_friction_coeffS2W %f\n", params.static_friction_coeffS2W);
    }
    if (doc.HasMember("static_friction_coeffS2M") && doc["static_friction_coeffS2M"].IsNumber()) {
        params.static_friction_coeffS2M = doc["static_friction_coeffS2M"].GetFloat();
        CONDITIONAL_PRINTF(verbose, "params.static_friction_coeffS2M %f\n", params.static_friction_coeffS2M);
    }
    if (doc.HasMember("rolling_mode") && doc["rolling_mode"].IsString()) {
        std::string rolling_mode = doc["rolling_mode"].GetString();
        if (rolling_mode == std::string("no_resistance"))
            params.rolling_mode = CHGPU_ROLLING_MODE::NO_RESISTANCE;
        else if (rolling_mode == std::string("schwartz"))
            params.rolling_mode = CHGPU_ROLLING_MODE::SCHWARTZ;
        else {
            InvalidArg("rolling_mode");
            return false;
        }
        CONDITIONAL_PRINTF(verbose, "params.rolling_mode %s\n", rolling_mode.c_str());
    }
    if (doc.HasMember("rolling_friction_coeffS2S") && doc["rolling_friction_coeffS2S"].IsNumber()) {
        params.rolling_friction_coeffS2S = doc["rolling_friction_coeffS2S"].GetFloat();
        CONDITIONAL_PRINTF(verbose, "params.rolling_friction_coeffS2S %f\n", params.rolling_friction_coeffS2S);
    }
    if (doc.HasMember("rolling_friction_coeffS2W") && doc["rolling_friction_coeffS2W"].IsNumber()) {
        params.rolling_friction_coeffS2W = doc["rolling_friction_coeffS2W"].GetFloat();
        CONDITIONAL_PRINTF(verbose, "params.rolling_friction_coeffS2W %f\n", params.rolling_friction_coeffS2W);
    }
    if (doc.HasMember("rolling_friction_coeffS2M") && doc["rolling_friction_coeffS2M"].IsNumber()) {
        params.rolling_friction_coeffS2M = doc["rolling_friction_coeffS2M"].GetFloat();
        CONDITIONAL_PRINTF(verbose, "params.rolling_friction_coeffS2M %f\n", params.rolling_friction_coeffS2M);
    }
    if (doc.HasMember("cohesion_ratio") && doc["cohesion_ratio"].IsNumber()) {
        params.cohesion_ratio = doc["cohesion_ratio"].GetFloat();
        CONDITIONAL_PRINTF(verbose, "params.cohesion_ratio %f\n", params.cohesion_ratio);
    }
    if (doc.HasMember("adhesion_ratio_s2w") && doc["adhesion_ratio_s2w"].IsNumber()) {
        params.adhesion_ratio_s2w = doc["adhesion_ratio_s2w"].GetFloat();
        CONDITIONAL_PRINTF(verbose, "params.adhesion_ratio_s2w %f\n", params.adhesion_ratio_s2w);
    }
    if (doc.HasMember("adhesion_ratio_s2m") && doc["adhesion_ratio_s2m"].IsNumber()) {
        params.adhesion_ratio_s2m = doc["adhesion_ratio_s2m"].GetFloat();
        CONDITIONAL_PRINTF(verbose, "params.adhesion_ratio_s2m %f\n", params.adhesion_ratio_s2m);
    }
    if (doc.HasMember("verbose") && doc["verbose"].IsInt()) {
        params.verbose = (CHGPU_VERBOSITY)doc["verbose"].GetInt();
        CONDITIONAL_PRINTF(verbose, "params.verbose %d\n", (int)params.verbose);
    }
    if (doc.HasMember("psi_T") && doc["psi_T"].IsInt()) {
        params.psi_T = doc["psi_T"].GetInt();
        CONDITIONAL_PRINTF(verbose, "params.psi_T %d\n", params.psi_T);
    }
    if (doc.HasMember("psi_L") && doc["psi_L"].IsInt()) {
        params.psi_L = doc["psi_L"].GetInt();
        CONDITIONAL_PRINTF(verbose, "params.psi_L %d\n", params.psi_L);
    }
    if (doc.HasMember("psi_R") && doc["psi_R"].IsNumber()) {
        params.psi_R = doc["psi_R"].GetFloat();
        CONDITIONAL_PRINTF(verbose, "params.psi_R %f\n", params.psi_R);
    }
    if (doc.HasMember("run_mode") && doc["run_mode"].IsString()) {
        std::string run_mode = doc["run_mode"].GetString();
        if (run_mode == std::string("frictionless"))
            params.run_mode = CHGPU_RUN_MODE::FRICTIONLESS;
        else if (run_mode == std::string("one_step"))
            params.run_mode = CHGPU_RUN_MODE::ONE_STEP;
        else if (run_mode == std::string("multi_step"))
            params.run_mode = CHGPU_RUN_MODE::MULTI_STEP;
        else {
            InvalidArg("write_mode");
            return false;
        }
        CONDITIONAL_PRINTF(verbose, "params.run_mode %s\n", run_mode.c_str());
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
        std::string write_mode = doc["write_mode"].GetString();
        if (write_mode == std::string("binary"))
            params.write_mode = CHGPU_OUTPUT_MODE::BINARY;
        else if (write_mode == std::string("csv"))
            params.write_mode = CHGPU_OUTPUT_MODE::CSV;
        else if (write_mode == std::string("hdf5"))
            params.write_mode = CHGPU_OUTPUT_MODE::HDF5;
        else if (write_mode == std::string("chpf"))
            params.write_mode = CHGPU_OUTPUT_MODE::CHPF;
        else if (write_mode == std::string("none"))
            params.write_mode = CHGPU_OUTPUT_MODE::NONE;
        else {
            InvalidArg("write_mode");
            return false;
        }
        CONDITIONAL_PRINTF(verbose, "params.write_mode %s\n", write_mode.c_str());
    }
    if (doc.HasMember("step_size") && doc["step_size"].IsNumber()) {
        params.step_size = doc["step_size"].GetFloat();
        CONDITIONAL_PRINTF(verbose, "params.step_size %f\n", params.step_size);
    }

    CONDITIONAL_PRINTF(verbose, "--------------------\n");

    // TODO sanity checks
    // necessary parameters
    // step_mode
    return true;
}

/// @} gpu_utils

}  // namespace gpu
}  // namespace chrono