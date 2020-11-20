// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Aaron Young
// =============================================================================
//
// Utility functions for parsing JSON files.
//
// =============================================================================

#include <fstream>

#include "chrono_sensor/utils/ChUtilsJSON.h"
//
#include "chrono_sensor/filters/ChFilter.h"
#include "chrono_sensor/filters/ChFilterOptixRender.h"
#include "chrono_sensor/filters/ChFilterIMUUpdate.h"
#include "chrono_sensor/filters/ChFilterGPSUpdate.h"
#include "chrono_sensor/filters/ChFilterCameraNoise.h"
#include "chrono_sensor/filters/ChFilterLidarNoise.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/filters/ChFilterSavePtCloud.h"
#include "chrono_sensor/filters/ChFilterGrayscale.h"
#include "chrono_sensor/filters/ChFilterLidarReduce.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterPCfromDepth.h"
#include "chrono_sensor/filters/ChFilterVisualizePointCloud.h"
#include "chrono_sensor/filters/ChFilterImageOps.h"
//
#include "chrono_thirdparty/rapidjson/filereadstream.h"
#include "chrono_thirdparty/rapidjson/istreamwrapper.h"

using namespace rapidjson;

namespace chrono {
namespace sensor {

// -----------------------------------------------------------------------------

Document ReadFileJSON(const std::string& filename) {
    Document d;
    std::ifstream ifs(filename);
    if (!ifs.good()) {
        GetLog() << "ERROR: Could not open JSON file: " << filename << "\n";
    } else {
        IStreamWrapper isw(ifs);
        d.ParseStream<ParseFlag::kParseCommentsFlag>(isw);
        if (d.IsNull()) {
            GetLog() << "ERROR: Invalid JSON file: " << filename << "\n";
        }
    }
    return d;
}

// -----------------------------------------------------------------------------

ChVector<> ReadVectorJSON(const Value& a) {
    assert(a.IsArray());
    assert(a.Size() == 3);
    return ChVector<>(a[0u].GetDouble(), a[1u].GetDouble(), a[2u].GetDouble());
}

ChQuaternion<> ReadQuaternionJSON(const Value& a) {
    assert(a.IsArray());
    assert(a.Size() == 4);
    return ChQuaternion<>(a[0u].GetDouble(), a[1u].GetDouble(), a[2u].GetDouble(), a[3u].GetDouble());
}

ChFrame<> ReadFrameJSON(const Value& a) {
    assert(a.HasMember("Location"));
    assert(a.HasMember("Orientation"));
    return ChFrame<>(ReadVectorJSON(a["Location"]), ReadQuaternionJSON(a["Orientation"]));
}

// -----------------------------------------------------------------------------

std::shared_ptr<ChSensor> ReadSensorJSON(const std::string& filename,
                                         std::shared_ptr<chrono::ChBody> parent,
                                         chrono::ChFrame<double> offsetPose) {
    std::shared_ptr<ChSensor> sensor;

    Document d = ReadFileJSON(filename);
    if (d.IsNull())
        return nullptr;

    // Check that the given file is a sensor specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("Sensor") == 0);

    // Extract the sensor type.
    assert(d.HasMember("Template"));
    std::string sensor_type = d["Template"].GetString();

    // Create the sensor using the appropriate template.
    if (sensor_type.compare("Camera") == 0) {
        sensor = ReadCameraSensorJSON(filename, parent, offsetPose);
    } else if (sensor_type.compare("GPS") == 0) {
        sensor = ReadGPSSensorJSON(filename, parent, offsetPose);
    } else if (sensor_type.compare("IMU") == 0) {
        sensor = ReadIMUSensorJSON(filename, parent, offsetPose);
    } else if (sensor_type.compare("Lidar") == 0) {
        sensor = ReadLidarSensorJSON(filename, parent, offsetPose);
    } else {
        throw ChException("Sensor type of " + sensor_type + " not supported in ReadSensorJSON.");
    }

    return sensor;
}

std::shared_ptr<ChCameraSensor> ReadCameraSensorJSON(const std::string& filename,
                                                     std::shared_ptr<chrono::ChBody> parent,
                                                     chrono::ChFrame<double> offsetPose) {
    Document d = ReadFileJSON(filename);
    if (d.IsNull())
        return nullptr;

    // Check that the given file is a sensor specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("Sensor") == 0);

    // Extract the sensor type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();
    if (!subtype.compare("Camera") == 0) {
        throw ChException("ChUtilsJSON::ReadCameraSensorJSON: Sensor type of " + subtype + " must be Camera.");
    }

    // Read sensor properties
    assert(d.HasMember("Properties"));
    const Value& properties = d["Properties"];

    // Create the camera sensor.
    float updateRate = properties["Update Rate"].GetFloat();
    // ChFrame<> offsetPose = ReadFrameJSON(properties["Offset Pose"]);
    unsigned int w = properties["Width"].GetUint();
    unsigned int h = properties["Height"].GetUint();
    float hFOV = properties["Horizontal Field of View"].GetFloat();
    unsigned int supersample_factor = 1;
    CameraLensModelType lens_model = PINHOLE;

    if (properties.HasMember("Supersample Factor")) {
        supersample_factor = properties["Supersample Factor"].GetInt();
    }
    if (properties.HasMember("Lens Type")) {
        std::string l = properties["Lens Model"].GetString();
        if (l == "SPHERICAL") {
            lens_model = SPHERICAL;
        }
    }

    auto camera = chrono_types::make_shared<ChCameraSensor>(parent, updateRate, offsetPose, w, h, hFOV,
                                                            supersample_factor, lens_model);
    if (properties.HasMember("Lag")) {
        float lag = properties["Lag"].GetFloat();
        camera->SetLag(lag);
    }
    if (properties.HasMember("Exposure Time")) {
        float collection = properties["Exposure Time"].GetFloat();
        camera->SetCollectionWindow(collection);
    }
    return camera;
}

std::shared_ptr<ChGPSSensor> ReadGPSSensorJSON(const std::string& filename,
                                               std::shared_ptr<chrono::ChBody> parent,
                                               chrono::ChFrame<double> offsetPose) {
    Document d = ReadFileJSON(filename);
    if (d.IsNull())
        return nullptr;

    // Check that the given file is a sensor specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("Sensor") == 0);

    // Extract the sensor type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();
    if (!subtype.compare("GPS") == 0) {
        throw ChException("ChUtilsJSON::ReadGPSSensorJSON: Sensor type of " + subtype + " must be GPS.");
    }

    // Read sensor properties
    assert(d.HasMember("Properties"));
    const Value& properties = d["Properties"];

    // Create the gps sensor.
    float updateRate = properties["Update Rate"].GetFloat();
    // ChFrame<> offsetPose = ReadFrameJSON(properties["Offset Pose"]);
    ChVector<> gps_reference = ReadVectorJSON(properties["GPS Reference"]);
    std::shared_ptr<ChGPSNoiseModel> noise_model = CreateGPSNoiseJSON(properties["GPS Noise Model"]);

    auto gps = chrono_types::make_shared<ChGPSSensor>(parent, updateRate, offsetPose, gps_reference, noise_model);
    if (properties.HasMember("Lag")) {
        float lag = properties["Lag"].GetFloat();
        gps->SetLag(lag);
    }
    if (properties.HasMember("Collection Window")) {
        float collection = properties["Collection Window"].GetFloat();
        gps->SetCollectionWindow(collection);
    }

    return gps;
}

std::shared_ptr<ChIMUSensor> ReadIMUSensorJSON(const std::string& filename,
                                               std::shared_ptr<chrono::ChBody> parent,
                                               chrono::ChFrame<double> offsetPose) {
    Document d = ReadFileJSON(filename);
    if (d.IsNull())
        return nullptr;

    // Check that the given file is a sensor specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("Sensor") == 0);

    // Extract the sensor type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();
    if (!subtype.compare("IMU") == 0) {
        throw ChException("ChUtilsJSON::ReadIMUSensorJSON: Sensor type of " + subtype + " must be GPS.");
    }

    // Read sensor properties
    assert(d.HasMember("Properties"));
    const Value& properties = d["Properties"];

    // Create the gps sensor.
    float updateRate = properties["Update Rate"].GetFloat();
    // ChFrame<> offsetPose = ReadFrameJSON(properties["Offset Pose"]);
    std::shared_ptr<ChIMUNoiseModel> noise_model = CreateIMUNoiseJSON(properties["IMU Noise Model"]);
    auto imu = chrono_types::make_shared<ChIMUSensor>(parent, updateRate, offsetPose, noise_model);

    if (properties.HasMember("Lag")) {
        float lag = properties["Lag"].GetFloat();
        imu->SetLag(lag);
    }
    if (properties.HasMember("Collection Window")) {
        float collection = properties["Collection Window"].GetFloat();
        imu->SetCollectionWindow(collection);
    }
    return imu;
}

std::shared_ptr<ChLidarSensor> ReadLidarSensorJSON(const std::string& filename,
                                                   std::shared_ptr<chrono::ChBody> parent,
                                                   chrono::ChFrame<double> offsetPose) {
    Document d = ReadFileJSON(filename);
    if (d.IsNull())
        return nullptr;

    // Check that the given file is a sensor specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("Sensor") == 0);

    // Extract the sensor type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();
    if (!subtype.compare("Lidar") == 0) {
        throw ChException("ChUtilsJSON::ReadLidarSensorJSON: Sensor type of " + subtype + " must be Lidar.");
    }

    // Read sensor properties
    assert(d.HasMember("Properties"));
    const Value& properties = d["Properties"];

    // Create the gps sensor.
    float updateRate = properties["Update Rate"].GetFloat();
    // ChFrame<> offsetPose = ReadFrameJSON(properties["Offset Pose"]);
    unsigned int w = properties["Width"].GetUint();
    unsigned int h = properties["Height"].GetUint();
    float hfov = properties["Horizontal Field of View"].GetFloat();
    float max_v_angle = properties["Max Vertical Angle"].GetFloat();
    float min_v_angle = properties["Min Vertical Angle"].GetFloat();
    float max_distance = properties["Max Distance"].GetFloat();
    // float lag = properties["Lag"].GetFloat();
    // float exposure_time = properties["Collection Window"].GetFloat();

    unsigned int sample_radius = 1;
    float divergence_angle = .003;
    LidarReturnMode return_mode = STRONGEST_RETURN;
    LidarModelType lidar_model = RAYCAST;

    if (properties.HasMember("Sample Radius")) {
        sample_radius = properties["Sample Radius"].GetInt();
    }
    if (properties.HasMember("Divergence Angle")) {
        divergence_angle = properties["Divergence Angle"].GetFloat();
    }
    if (properties.HasMember("Return Mode")) {
        std::string s = properties["Return Mode"].GetString();
        if (s == "MEAN_RETURN") {
            return_mode = MEAN_RETURN;
        }
    }
    if (properties.HasMember("Lidar Model")) {
        std::string s = properties["Lidar Model"].GetString();
        // for when we add raytraced lidar model
    }

    auto lidar = chrono_types::make_shared<ChLidarSensor>(parent, updateRate, offsetPose, w, h, hfov, max_v_angle,
                                                          min_v_angle, max_distance, sample_radius, divergence_angle,
                                                          return_mode, lidar_model);

    if (properties.HasMember("Lag")) {
        float lag = properties["Lag"].GetFloat();
        lidar->SetLag(lag);
    }
    if (properties.HasMember("Collection Window")) {
        float exposure_time = properties["Collection Window"].GetFloat();
        lidar->SetCollectionWindow(exposure_time);
    }

    return lidar;
}

void ReadFilterListJSON(const std::string& filename, std::shared_ptr<ChSensor> sensor) {
    Document d = ReadFileJSON(filename);
    if (d.IsNull())
        return;

    // Read inertia properties for all sub-components
    // and calculate composite inertia properties
    assert(d.HasMember("Filters"));
    assert(d["Filters"].IsArray());
    int num_filters = d["Filters"].Size();
    // std::cout << "Num Filters :: " << num_filters << std::endl;

    for (int i = 0; i < num_filters; i++) {
        // Create the filter.
        auto filter = CreateFilterJSON(d["Filters"][i]);
        sensor->PushFilter(filter);
    }
    // std::cout << "Successfully Loaded FilterList" << std::endl;
}

// -----------------------------------------------------------------------------

std::shared_ptr<ChFilter> CreateFilterJSON(const Value& value) {
    std::string type = value["Filter"].GetString();
    // std::cout << "Filter Type :: " << type << "." << std::endl;

    // Create the filter
    std::shared_ptr<ChFilter> filter;
    if (type.compare("ChFilterOptixRender") == 0) {
        filter = chrono_types::make_shared<ChFilterOptixRender>();
    } else if (type.compare("ChFilterIMUUpdate") == 0) {
        std::shared_ptr<ChIMUNoiseModel> model = CreateIMUNoiseJSON(value["IMU Noise Model"]);
        filter = chrono_types::make_shared<ChFilterIMUUpdate>(model);
    } else if (type.compare("ChFilterGPSUpdate") == 0) {
        ChVector<> gps_reference = ReadVectorJSON(value["GPS Reference"]);
        std::shared_ptr<ChGPSNoiseModel> model = CreateGPSNoiseJSON(value["GPS Noise Model"]);
        filter = chrono_types::make_shared<ChFilterGPSUpdate>(gps_reference, model);
    } else if (type.compare("ChFilterCameraNoiseConstNormal") == 0) {
        float mean = value["Mean"].GetFloat();
        float stdev = value["Standard Deviation"].GetFloat();
        std::string name = GetStringMemberWithDefault(value, "Name");
        filter = chrono_types::make_shared<ChFilterCameraNoiseConstNormal>(mean, stdev, name);
    } else if (type.compare("ChFilterCameraNoisePixDep") == 0) {
        float gain = value["Gain"].GetFloat();
        float sigma_read = value["Sigma Read"].GetFloat();
        float sigma_adc = value["Sigma ADC"].GetFloat();
        std::string name = GetStringMemberWithDefault(value, "Name");
        filter = chrono_types::make_shared<ChFilterCameraNoisePixDep>(gain, sigma_read, sigma_adc, name);
    } else if (type.compare("ChFilterLidarNoiseXYZI") == 0) {
        float stdev_range = value["Standard Deviation Range"].GetFloat();
        float stdev_v_angle = value["Standard Deviation Vertical Angle"].GetFloat();
        float stdev_h_angle = value["Standard Deviation Horizontal Angle"].GetFloat();
        float stdev_intensity = value["Standard Deviation Intensity"].GetFloat();
        std::string name = GetStringMemberWithDefault(value, "Name");
        filter = chrono_types::make_shared<ChFilterLidarNoiseXYZI>(stdev_range, stdev_v_angle, stdev_h_angle,
                                                                   stdev_intensity, name);
    } else if (type.compare("ChFilterVisualize") == 0) {
        int w = value["Width"].GetInt();
        int h = value["Height"].GetInt();
        std::string name = GetStringMemberWithDefault(value, "Name");
        filter = chrono_types::make_shared<ChFilterVisualize>(w, h, name);
    } else if (type.compare("ChFilterSave") == 0) {
        std::string data_path = GetStringMemberWithDefault(value, "Data Path");
        filter = chrono_types::make_shared<ChFilterSave>(data_path);
    } else if (type.compare("ChFilterSavePtCloud") == 0) {
        std::string data_path = GetStringMemberWithDefault(value, "Data Path");
        filter = chrono_types::make_shared<ChFilterSavePtCloud>(data_path);
    } else if (type.compare("ChFilterGrayscale") == 0) {
        std::string name = GetStringMemberWithDefault(value, "Name");
        filter = chrono_types::make_shared<ChFilterGrayscale>(name);
    } else if (type.compare("ChFilterR8Access") == 0) {
        std::string name = GetStringMemberWithDefault(value, "Name");
        filter = chrono_types::make_shared<ChFilterR8Access>(name);
    } else if (type.compare("ChFilterRGBA8Access") == 0) {
        std::string name = GetStringMemberWithDefault(value, "Name");
        filter = chrono_types::make_shared<ChFilterRGBA8Access>(name);
    } else if (type.compare("ChFilterXYZIAccess") == 0) {
        std::string name = GetStringMemberWithDefault(value, "Name");
        filter = chrono_types::make_shared<ChFilterXYZIAccess>(name);
    } else if (type.compare("ChFilterDIAccess") == 0) {
        std::string name = GetStringMemberWithDefault(value, "Name");
        filter = chrono_types::make_shared<ChFilterDIAccess>(name);
    } else if (type.compare("ChFilterIMUAccess") == 0) {
        std::string name = GetStringMemberWithDefault(value, "Name");
        filter = chrono_types::make_shared<ChFilterIMUAccess>(name);
    } else if (type.compare("ChFilterGPSAccess") == 0) {
        std::string name = GetStringMemberWithDefault(value, "Name");
        filter = chrono_types::make_shared<ChFilterGPSAccess>(name);
    } else if (type.compare("ChFilterPCfromDepth") == 0) {
        std::string name = GetStringMemberWithDefault(value, "Name");
        filter = chrono_types::make_shared<ChFilterPCfromDepth>(name);
    } else if (type.compare("ChFilterVisualizePointCloud") == 0) {
        std::string name = GetStringMemberWithDefault(value, "Name");
        int w = value["Width"].GetInt();
        int h = value["Height"].GetInt();
        float zoom = value["Zoom"].GetFloat();
        filter = chrono_types::make_shared<ChFilterVisualizePointCloud>(w, h, zoom, name);
    } else if (type.compare("ChFilterImageResize") == 0) {
        int w = value["Width"].GetInt();
        int h = value["Height"].GetInt();
        std::string name = GetStringMemberWithDefault(value, "Name");
        filter = chrono_types::make_shared<ChFilterImageResize>(w, h, name);
    } else {
        throw ChException("Filter type of \"" + type + "\" not supported in ReadFilterJSON.");
    }

    return filter;
}

std::shared_ptr<ChIMUNoiseModel> CreateIMUNoiseJSON(const Value& value) {
    std::string type = value["Noise Model"].GetString();
    // std::cout << "Noise Model Type :: " << type << "." << std::endl;

    // Create the filter
    std::shared_ptr<ChIMUNoiseModel> model;
    if (type.compare("ChIMUNoiseNone") == 0) {
        model = chrono_types::make_shared<ChIMUNoiseNone>();
    } else if (type.compare("ChIMUNoiseNormalDrift") == 0) {
        float updateRate = value["Update Rate"].GetFloat();
        float g_mean = value["Gaussian Mean"].GetFloat();
        float g_stdev = value["Gaussian Standard Deviation"].GetFloat();
        float g_bias_drift = value["Gaussian Bias Drift"].GetFloat();
        float g_tau_drift = value["Gaussian Tau Drift"].GetFloat();
        float a_mean = value["A Mean"].GetFloat();
        float a_stdev = value["A Standard Deviation"].GetFloat();
        float a_bias_drift = value["A Bias Drift"].GetFloat();
        float a_tau_drift = value["A Tau Drift"].GetFloat();
        model = chrono_types::make_shared<ChIMUNoiseNormalDrift>(updateRate, g_mean, g_stdev, g_bias_drift, g_tau_drift,
                                                                 a_mean, a_stdev, a_bias_drift, a_tau_drift);
    } else {
        throw ChException("IMU noise model type of \"" + type + "\" not supported in ReadIMUNoiseJSON.");
    }

    return model;
}

std::shared_ptr<ChGPSNoiseModel> CreateGPSNoiseJSON(const Value& value) {
    std::string type = value["Noise Model"].GetString();
    // std::cout << "Noise Model Type :: " << type << "." << std::endl;

    // Create the filter
    std::shared_ptr<ChGPSNoiseModel> model;
    if (type.compare("ChGPSNoiseNone") == 0) {
        model = chrono_types::make_shared<ChGPSNoiseNone>();
    } else if (type.compare("ChGPSNoiseNormal") == 0) {
        ChVector<float> mean = ReadVectorJSON(value["Mean"]);
        ChVector<float> stdev = ReadVectorJSON(value["Standard Deviation"]);
        model = chrono_types::make_shared<ChGPSNoiseNormal>(mean, stdev);
    } else {
        throw ChException("GPS noise model type of \"" + type + "\" not supported in ReadGPSNoiseJSON.");
    }

    return model;
}

std::string GetStringMemberWithDefault(const Value& value, const char* member, const char* def) {
    if (value.HasMember(member))
        return value[member].GetString();
    return def;
}

}  // namespace sensor
}  // namespace chrono
