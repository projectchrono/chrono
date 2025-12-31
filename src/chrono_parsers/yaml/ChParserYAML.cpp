// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

#include "chrono/physics/ChSystem.h"
#include "chrono/utils/ChUtils.h"

#include "chrono/input_output/ChOutputASCII.h"
#ifdef CHRONO_HAS_HDF5
    #include "chrono/input_output/ChOutputHDF5.h"
#endif

#include "chrono_parsers/yaml/ChParserYAML.h"

#include "chrono_thirdparty/filesystem/path.h"

using std::cout;
using std::cerr;
using std::endl;

namespace chrono {
namespace parsers {

ChParserYAML::ChParserYAML()
    : m_name("YAML model"),
      m_data_path(DataPathType::ABS),
      m_rel_path("."),
      m_verbose(false),
      m_use_degrees(true),
      m_output_dir("") {}

// -----------------------------------------------------------------------------

void ChParserYAML::CheckVersion(const YAML::Node& a) {
    std::string chrono_version = a.as<std::string>();

    auto first = chrono_version.find(".");
    ChAssertAlways(first != std::string::npos);
    std::string chrono_major = chrono_version.substr(0, first);

    ChAssertAlways(first < chrono_version.size() - 1);
    chrono_version = &chrono_version[first + 1];

    auto second = chrono_version.find(".");
    if (second == std::string::npos)
        second = chrono_version.size();
    std::string chrono_minor = chrono_version.substr(0, second);

    ChAssertAlways(chrono_major == CHRONO_VERSION_MAJOR);
    ChAssertAlways(chrono_minor == CHRONO_VERSION_MINOR);
}

// -----------------------------------------------------------------------------

std::string ChParserYAML::ToUpper(std::string in) {
    std::transform(in.begin(), in.end(), in.begin(), ::toupper);
    return in;
}

std::string ChParserYAML::GetDatafilePath(const std::string& filename) {
    std::string full_filename = "";
    switch (m_data_path) {
        case DataPathType::ABS:
            full_filename = filename;
            break;
        case DataPathType::REL:
            full_filename = m_script_directory + "/" + m_rel_path + "/" + filename;
            break;
    }

    cout << "File: " << full_filename << endl;
    auto filepath = filesystem::path(full_filename);

    ChAssertAlways(filepath.exists());
    ChAssertAlways(filepath.is_file());

    return full_filename;
}

ChParserYAML::DataPathType ChParserYAML::ReadDataPathType(const YAML::Node& a) {
    auto type = ToUpper(a.as<std::string>());
    if (type == "RELATIVE")
        return DataPathType::REL;
    else
        return DataPathType::ABS;
}

ChVector3d ChParserYAML::ReadVector(const YAML::Node& a) {
    ChAssertAlways(a.IsSequence());
    ChAssertAlways(a.size() == 3);
    return ChVector3d(a[0].as<double>(), a[1].as<double>(), a[2].as<double>());
}

ChQuaterniond ChParserYAML::ReadRotation(const YAML::Node& a, bool use_degrees) {
    ChAssertAlways(a.IsSequence());
    ChAssertAlways(a.size() == 3 || a.size() == 4);

    return (a.size() == 3) ? ReadCardanAngles(a, use_degrees) : ReadQuaternion(a);
}

ChQuaterniond ChParserYAML::ReadQuaternion(const YAML::Node& a) {
    ChAssertAlways(a.IsSequence());
    ChAssertAlways(a.size() == 4);
    return ChQuaterniond(a[0].as<double>(), a[1].as<double>(), a[2].as<double>(), a[3].as<double>());
}

ChQuaterniond ChParserYAML::ReadCardanAngles(const YAML::Node& a, bool use_degrees) {
    ChAssertAlways(a.IsSequence());
    ChAssertAlways(a.size() == 3);
    ChVector3d angles = ReadVector(a);

    if (use_degrees)
        angles *= CH_DEG_TO_RAD;

    ChQuaterniond q1 = QuatFromAngleZ(angles.x());  // roll
    ChQuaterniond q2 = QuatFromAngleY(angles.y());  // pitch
    ChQuaterniond q3 = QuatFromAngleX(angles.z());  // yaw

    ChQuaterniond q = q1 * q2 * q3;

    return q;
}

ChCoordsysd ChParserYAML::ReadCoordinateSystem(const YAML::Node& a, bool use_degrees) {
    ChAssertAlways(a["location"]);
    ChAssertAlways(a["orientation"]);
    return ChCoordsysd(ReadVector(a["location"]), ReadRotation(a["orientation"], use_degrees));
}

std::shared_ptr<ChFunction> ChParserYAML::ReadFunction(const YAML::Node& a, bool use_degrees) {
    ChAssertAlways(a["type"]);
    auto type = ToUpper(a["type"].as<std::string>());

    bool repeat = false;
    double repeat_start = 0;
    double repeat_width = 0;
    double repeat_shift = 0;
    if (a["repeat"]) {
        repeat = true;
        repeat_start = a["repeat"]["start"].as<double>();
        repeat_width = a["repeat"]["width"].as<double>();
        repeat_shift = a["repeat"]["shift"].as<double>();
    }

    std::shared_ptr<ChFunction> f_base = nullptr;
    bool is_controller = false;
    if (type == "CONSTANT") {
        ChAssertAlways(a["value"]);
        auto value = a["value"].as<double>();
        f_base = chrono_types::make_shared<ChFunctionConst>(value);
    } else if (type == "POLYNOMIAL") {
        ChAssertAlways(a["coefficients"]);
        ChAssertAlways(a["coefficients"].IsSequence());
        auto num_coeffs = a["coefficients"].size();
        std::vector<double> coeffs;
        for (size_t i = 0; i < num_coeffs; i++)
            coeffs.push_back(a["coefficients"][i].as<double>());
        auto f_poly = chrono_types::make_shared<ChFunctionPoly>();
        f_poly->SetCoefficients(coeffs);
        f_base = f_poly;
    } else if (type == "SINE") {
        ChAssertAlways(a["amplitude"]);
        ChAssertAlways(a["frequency"]);
        double amplitude = a["amplitude"].as<double>();
        double frequency = a["frequency"].as<double>();
        double phase = 0;
        if (a["phase"])
            phase = a["phase"].as<double>();
        double shift = 0;
        if (a["shift"])
            shift = a["shift"].as<double>();
        if (use_degrees)
            phase *= CH_DEG_TO_RAD;
        f_base = chrono_types::make_shared<ChFunctionSine>(amplitude, frequency, phase, shift);
    } else if (type == "RAMP") {
        ChAssertAlways(a["slope"]);
        auto slope = a["slope"].as<double>();
        double intercept = 0;
        if (a["intercept"])
            intercept = a["intercept"].as<double>();
        f_base = chrono_types::make_shared<ChFunctionRamp>(intercept, slope);
    } else if (type == "DATA") {
        ChAssertAlways(a["data"]);                 // key 'data' must exist
        ChAssertAlways(a["data"].IsSequence());    // 'data' must be an array
        ChAssertAlways(a["data"][0].size() == 2);  // each entry in 'data' must have size 2
        auto num_points = a["data"].size();
        auto f_interp = chrono_types::make_shared<ChFunctionInterp>();
        for (size_t i = 0; i < num_points; i++) {
            double t = a["data"][i][0].as<double>();
            double f = a["data"][i][1].as<double>();
            f_interp->AddPoint(t, f);
        }
        f_base = f_interp;
    } else if (type == "CONTROLLER") {
        is_controller = true;
        f_base = chrono_types::make_shared<ChFunctionSetpoint>();
    } else {
        cerr << "Incorrect function type: " << a["type"] << endl;
        throw std::runtime_error("Incorrect function type");
    }
    //// TODO - more function types

    // Check if base function must be repeated
    if (repeat && !is_controller) {
        // Yes: construct and return repeated function
        auto f_repeat = chrono_types::make_shared<ChFunctionRepeat>(f_base, repeat_start, repeat_width, repeat_shift);
        return f_repeat;
    } else {
        // No: return base underlying function
        return f_base;
    }
}

ChColor ChParserYAML::ReadColor(const YAML::Node& a) {
    ChAssertAlways(a.IsSequence());
    ChAssertAlways(a.size() == 3);
    return ChColor(a[0].as<float>(), a[1].as<float>(), a[2].as<float>());
}

ChOutput::Type ChParserYAML::ReadOutputType(const YAML::Node& a) {
    auto type = ToUpper(a.as<std::string>());
    if (type == "ASCII")
        return ChOutput::Type::ASCII;
    if (type == "HDF5")
        return ChOutput::Type::HDF5;
    return ChOutput::Type::NONE;
}

ChOutput::Mode ChParserYAML::ReadOutputMode(const YAML::Node& a) {
    auto mode = ToUpper(a.as<std::string>());
    if (mode == "SERIES")
        return ChOutput::Mode::SERIES;
    if (mode == "FRAMES")
        return ChOutput::Mode::FRAMES;
    return ChOutput::Mode::FRAMES;
}

// -----------------------------------------------------------------------------

void ChParserYAML::PrintNodeType(const YAML::Node& node) {
    switch (node.Type()) {
        case YAML::NodeType::Null:
            cout << " Null" << endl;
            break;
        case YAML::NodeType::Scalar:
            cout << " Scalar" << endl;
            break;
        case YAML::NodeType::Sequence:
            cout << " Sequence" << endl;
            break;
        case YAML::NodeType::Map:
            cout << " Map" << endl;
            break;
        case YAML::NodeType::Undefined:
            cout << " Undefined" << endl;
            break;
    }
}

// -----------------------------------------------------------------------------

ChParserYAML::OutputParameters::OutputParameters()
    : type(ChOutput::Type::NONE), mode(ChOutput::Mode::FRAMES), fps(100), dir(".") {}

void ChParserYAML::OutputParameters::PrintInfo() {
    if (type == ChOutput::Type::NONE) {
        cout << "no output" << endl;
        return;
    }

    cout << "output" << endl;
    cout << "  type:                 " << ChOutput::GetOutputTypeAsString(type) << endl;
    cout << "  mode:                 " << ChOutput::GetOutputModeAsString(mode) << endl;
    cout << "  output FPS:           " << fps << endl;
    cout << "  outut directory:      " << dir << endl;
}

void ChParserYAML::SaveOutput(int frame) {
    if (m_output.type == ChOutput::Type::NONE)
        return;

    // Create the output DB if needed
    if (!m_output_db) {
        std::string filename = m_output.dir + "/" + m_name;

        if (!m_output_dir.empty())
            filename = m_output_dir + "/" + filename;
        
        switch (m_output.type) {
            case ChOutput::Type::ASCII:
                filename += ".txt";
                m_output_db = chrono_types::make_shared<ChOutputASCII>(filename);
                break;
            case ChOutput::Type::HDF5:
#ifdef CHRONO_HAS_HDF5
                filename += ".h5";
                m_output_db = chrono_types::make_shared<ChOutputHDF5>(filename, m_output.mode);
                break;
#else
                return;
#endif
        }

        if (m_verbose) {
            cout << "\n-------------------------------------------------" << endl;
            cout << "\nOutput file: " << filename << endl;
        }

        m_output_db->Initialize();
    }
}

}  // end namespace parsers
}  // namespace chrono
