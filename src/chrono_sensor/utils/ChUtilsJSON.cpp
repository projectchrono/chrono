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
#include "chrono_sensor/optix/ChFilterOptixRender.h"
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
#include "chrono_sensor/sensors/ChNoiseModel.h"
//
#include "chrono_thirdparty/rapidjson/filereadstream.h"
#include "chrono_thirdparty/rapidjson/istreamwrapper.h"

using namespace rapidjson;

namespace chrono {
namespace sensor {

// -----------------------------------------------------------------------------

void ReadFileJSON(const std::string& filename, Document& d) {
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

    Document d;
    ReadFileJSON(filename, d);
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
    } else if (sensor_type.compare("Accelerometer") == 0) {
        sensor = ReadAccelerometerSensorJSON(filename, parent, offsetPose);
    } else if (sensor_type.compare("Gyroscope") == 0) {
        sensor = ReadGyroscopeSensorJSON(filename, parent, offsetPose);
    } else if (sensor_type.compare("Magnetometer") == 0) {
        sensor = ReadMagnetometerSensorJSON(filename, parent, offsetPose);
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
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return nullptr;

    // Check that the given file is a sensor specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("Sensor") == 0);

    // Extract the sensor type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();
    if (subtype.compare("Camera") != 0) {
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
    CameraLensModelType lens_model = CameraLensModelType::PINHOLE;

    if (properties.HasMember("Supersample Factor")) {
        supersample_factor = properties["Supersample Factor"].GetInt();
    }
    if (properties.HasMember("Lens Type")) {
        std::string l = properties["Lens Model"].GetString();
        if (l == "FOV_LENS") {
            lens_model = CameraLensModelType::FOV_LENS;
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
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return nullptr;

    // Check that the given file is a sensor specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("Sensor") == 0);

    // Extract the sensor type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();
    if (subtype.compare("GPS") != 0) {
        throw ChException("ChUtilsJSON::ReadGPSSensorJSON: Sensor type of " + subtype + " must be GPS.");
    }

    // Read sensor properties
    assert(d.HasMember("Properties"));
    const Value& properties = d["Properties"];

    // Create the gps sensor.
    float updateRate = properties["Update Rate"].GetFloat();
    // ChFrame<> offsetPose = ReadFrameJSON(properties["Offset Pose"]);
    ChVector<> gps_reference = ReadVectorJSON(properties["GPS Reference"]);
    std::shared_ptr<ChNoiseModel> noise_model = CreateNoiseJSON(properties["Noise Model"]);

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

std::shared_ptr<ChAccelerometerSensor> ReadAccelerometerSensorJSON(const std::string& filename,
                                                                   std::shared_ptr<chrono::ChBody> parent,
                                                                   chrono::ChFrame<double> offsetPose) {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return nullptr;

    // Check that the given file is a sensor specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("Sensor") == 0);

    // Extract the sensor type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();
    if (subtype.compare("Accelerometer") != 0) {
        throw ChException("ChUtilsJSON::ReadAccelerometerSensorJSON: Sensor type of " + subtype +
                          " must be Accelerometer.");
    }

    // Read sensor properties
    assert(d.HasMember("Properties"));
    const Value& properties = d["Properties"];

    // Create the gps sensor.
    float updateRate = properties["Update Rate"].GetFloat();
    // ChFrame<> offsetPose = ReadFrameJSON(properties["Offset Pose"]);
    std::shared_ptr<ChNoiseModel> noise_model = CreateNoiseJSON(properties["Noise Model"]);
    auto acc = chrono_types::make_shared<ChAccelerometerSensor>(parent, updateRate, offsetPose, noise_model);

    if (properties.HasMember("Lag")) {
        float lag = properties["Lag"].GetFloat();
        acc->SetLag(lag);
    }
    if (properties.HasMember("Collection Window")) {
        float collection = properties["Collection Window"].GetFloat();
        acc->SetCollectionWindow(collection);
    }
    return acc;
}

std::shared_ptr<ChGyroscopeSensor> ReadGyroscopeSensorJSON(const std::string& filename,
                                                           std::shared_ptr<chrono::ChBody> parent,
                                                           chrono::ChFrame<double> offsetPose) {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return nullptr;

    // Check that the given file is a sensor specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("Sensor") == 0);

    // Extract the sensor type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();
    if (subtype.compare("Gyroscope") != 0) {
        throw ChException("ChUtilsJSON::ReadGyroscopeSensorJSON: Sensor type of " + subtype + " must be Gyroscope.");
    }

    // Read sensor properties
    assert(d.HasMember("Properties"));
    const Value& properties = d["Properties"];

    // Create the gps sensor.
    float updateRate = properties["Update Rate"].GetFloat();
    // ChFrame<> offsetPose = ReadFrameJSON(properties["Offset Pose"]);
    std::shared_ptr<ChNoiseModel> noise_model = CreateNoiseJSON(properties["Noise Model"]);
    auto gyro = chrono_types::make_shared<ChGyroscopeSensor>(parent, updateRate, offsetPose, noise_model);

    if (properties.HasMember("Lag")) {
        float lag = properties["Lag"].GetFloat();
        gyro->SetLag(lag);
    }
    if (properties.HasMember("Collection Window")) {
        float collection = properties["Collection Window"].GetFloat();
        gyro->SetCollectionWindow(collection);
    }
    return gyro;
}

std::shared_ptr<ChMagnetometerSensor> ReadMagnetometerSensorJSON(const std::string& filename,
                                                                 std::shared_ptr<chrono::ChBody> parent,
                                                                 chrono::ChFrame<double> offsetPose) {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return nullptr;

    // Check that the given file is a sensor specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("Sensor") == 0);

    // Extract the sensor type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();
    if (subtype.compare("Magnetometer") != 0) {
        throw ChException("ChUtilsJSON::ReadMagnetometerSensorJSON: Sensor type of " + subtype +
                          " must be Magnetometer.");
    }

    // Read sensor properties
    assert(d.HasMember("Properties"));
    const Value& properties = d["Properties"];

    // Create the gps sensor.
    float updateRate = properties["Update Rate"].GetFloat();
    // ChFrame<> offsetPose = ReadFrameJSON(properties["Offset Pose"]);
    std::shared_ptr<ChNoiseModel> noise_model = CreateNoiseJSON(properties["Noise Model"]);
    ChVector<> gps_reference = ReadVectorJSON(properties["GPS Reference"]);
    auto mag =
        chrono_types::make_shared<ChMagnetometerSensor>(parent, updateRate, offsetPose, noise_model, gps_reference);

    if (properties.HasMember("Lag")) {
        float lag = properties["Lag"].GetFloat();
        mag->SetLag(lag);
    }
    if (properties.HasMember("Collection Window")) {
        float collection = properties["Collection Window"].GetFloat();
        mag->SetCollectionWindow(collection);
    }
    return mag;
}

std::shared_ptr<ChLidarSensor> ReadLidarSensorJSON(const std::string& filename,
                                                   std::shared_ptr<chrono::ChBody> parent,
                                                   chrono::ChFrame<double> offsetPose) {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return nullptr;

    // Check that the given file is a sensor specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("Sensor") == 0);

    // Extract the sensor type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();
    if (subtype.compare("Lidar") != 0) {
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
    LidarBeamShape beam_shape = LidarBeamShape::RECTANGULAR;
    float vert_divergence_angle = .003f;
    float hori_divergence_angle = .003f;
    LidarReturnMode return_mode = LidarReturnMode::STRONGEST_RETURN;
    float near_clip = 0.f;

    if (properties.HasMember("Sample Radius")) {
        sample_radius = properties["Sample Radius"].GetInt();
    }
    if (properties.HasMember("Vertical Divergence Angle")) {
        vert_divergence_angle = properties["Vertical Divergence Angle"].GetFloat();
    }
    if (properties.HasMember("Horizontal Divergence Angle")) {
        hori_divergence_angle = properties["Horizontal Divergence Angle"].GetFloat();
    }
    if (properties.HasMember("Return Mode")) {
        std::string s = properties["Return Mode"].GetString();
        if (s == "MEAN_RETURN") {
            return_mode = LidarReturnMode::MEAN_RETURN;
        } else if (s == "FIRST_RETURN") {
            return_mode = LidarReturnMode::FIRST_RETURN;
        } else if (s == "LAST_RETURN") {
            return_mode = LidarReturnMode::LAST_RETURN;
        }
    }
    if (properties.HasMember("Near Clip")) {
        near_clip = properties["Near Clip"].GetFloat();
    }

    auto lidar = chrono_types::make_shared<ChLidarSensor>(
        parent, updateRate, offsetPose, w, h, hfov, max_v_angle, min_v_angle, max_distance, beam_shape, sample_radius,
        vert_divergence_angle, hori_divergence_angle, return_mode, near_clip);

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

std::shared_ptr<ChRadarSensor> ReadRadarSensorJSON(const std::string& filename,
                                                   std::shared_ptr<chrono::ChBody> parent,
                                                   chrono::ChFrame<double> offsetPose){
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return nullptr;

    // Check that the given file is a sensor specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("Sensor") == 0);

    // Extract the sensor type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();
    if (subtype.compare("Radar") != 0){
        throw ChException("ChUtilsJSON::ReadRdarSensorJSON: Sensor type of " + subtype + " must be Radar");
    }

    // Read sensor properties
    assert(d.HasMember("Properties"));
    const Value& properties = d["Properties"];

    // Create the radar sensor.
    float updateRate = properties["Update Rate"].GetFloat();
    unsigned int w = properties["Width"].GetUint();
    unsigned int h = properties["Height"].GetUint();
    float hfov = properties["Horizontal Field of View"].GetFloat();
    float vfov = properties["Vertical Field of View"].GetFloat();
    float max_distance = properties["Max Distance"].GetFloat();

    float near_clip = 0.f;

    if (properties.HasMember("Near Clip")) {
        near_clip = properties["Near Clip"].GetFloat();
    }

    auto radar = chrono_types::make_shared<ChRadarSensor>(
        parent, updateRate, offsetPose, w, h, hfov, vfov, max_distance, near_clip
    );

    if (properties.HasMember("Collection Window")) {
        float exposure_time = properties["Collection Window"].GetFloat();
    radar->SetCollectionWindow(exposure_time);
    }

    return radar;
}

void ReadFilterListJSON(const std::string& filename, std::shared_ptr<ChSensor> sensor) {
    Document d;
    ReadFileJSON(filename, d);
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
    if (type.compare("ChFilterCameraNoiseConstNormal") == 0) {
        float mean = value["Mean"].GetFloat();
        float stdev = value["Standard Deviation"].GetFloat();
        std::string name = GetStringMemberWithDefault(value, "Name");
        filter = chrono_types::make_shared<ChFilterCameraNoiseConstNormal>(mean, stdev, name);
    } else if (type.compare("ChFilterCameraNoisePixDep") == 0) {
        float variance_slope = value["Variance Slope"].GetFloat();
        float variance_intercept = value["Variance Intercept"].GetFloat();
        std::string name = GetStringMemberWithDefault(value, "Name");
        filter = chrono_types::make_shared<ChFilterCameraNoisePixDep>(variance_slope, variance_intercept, name);
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
    } else if (type.compare("ChFilterAccelAccess") == 0) {
        std::string name = GetStringMemberWithDefault(value, "Name");
        filter = chrono_types::make_shared<ChFilterAccelAccess>(name);
    } else if (type.compare("ChFilterGyroAccess") == 0) {
        std::string name = GetStringMemberWithDefault(value, "Name");
        filter = chrono_types::make_shared<ChFilterGyroAccess>(name);
    } else if (type.compare("ChFilterMagnetAccess") == 0) {
        std::string name = GetStringMemberWithDefault(value, "Name");
        filter = chrono_types::make_shared<ChFilterMagnetAccess>(name);
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

std::shared_ptr<ChNoiseModel> CreateNoiseJSON(const Value& value) {
    std::string type = value["Noise Type"].GetString();
    // std::cout << "Noise Model Type :: " << type << "." << std::endl;

    // Create the filter
    std::shared_ptr<ChNoiseModel> model;
    if (type.compare("ChNoiseNone") == 0) {
        model = chrono_types::make_shared<ChNoiseNone>();
    } else if (type.compare("ChNoiseNormal") == 0) {
        ChVector<float> mean = ReadVectorJSON(value["Mean"]);
        ChVector<float> stdev = ReadVectorJSON(value["Mean"]);
        model = chrono_types::make_shared<ChNoiseNormal>(mean, stdev);
    } else if (type.compare("ChNoiseNormalDrift") == 0) {
        float updateRate = value["Update Rate"].GetFloat();
        ChVector<float> mean = ReadVectorJSON(value["Mean"]);
        ChVector<float> stdev = ReadVectorJSON(value["Mean"]);
        float drift_bias = value["Drift Bias"].GetFloat();
        float tau_drift = value["Tau Drift"].GetFloat();
        model = chrono_types::make_shared<ChNoiseNormalDrift>(updateRate, mean, stdev, drift_bias, tau_drift);
    } else {
        throw ChException("Noise model type of \"" + type + "\" not supported in ReadNoiseJSON.");
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
