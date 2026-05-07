// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Utility functions for parsing YAML files.
//
// =============================================================================

#include "chrono/input_output/ChUtilsYAML.h"

using std::cout;
using std::cerr;
using std::endl;

namespace chrono {

// -----------------------------------------------------------------------------

ChYamlFileHandler::ChYamlFileHandler() : m_type(Type::ABS), m_reference_dir(""), m_relative_path(".") {}

void ChYamlFileHandler::Read(const YAML::Node& a) {
    if (a["data_path"]) {
        ChAssertAlways(a["data_path"]["type"]);
        m_type = ReadType(a["data_path"]["type"]);
        if (a["data_path"]["root"])
            m_relative_path = a["data_path"]["root"].as<std::string>();
        else
            m_relative_path = ".";
    } else {
        m_type = Type::ABS;
        m_relative_path = ".";
    }
}

ChYamlFileHandler::Type ChYamlFileHandler::ReadType(const YAML::Node& a) {
    auto type = ChToUpper(a.as<std::string>());
    if (type == "RELATIVE")
        return Type::REL;
    else
        return Type::ABS;
}

void ChYamlFileHandler::SetReferenceDirectory(const std::string& pathname) {
    auto path = filesystem::path(pathname);
    if (!path.exists()) {
        cerr << "Error: path '" << pathname << "' does not exist." << endl;
        throw std::runtime_error("Path does not exist");
    }
    m_reference_dir = path.is_file() ? path.parent_path().str() : pathname;
}

std::string ChYamlFileHandler::GetFilename(const std::string& filename) const {
    std::string full_filename = "";
    switch (m_type) {
        case Type::ABS:
            full_filename = filename;
            break;
        case Type::REL:
            full_filename = m_reference_dir + "/" + m_relative_path + "/" + filename;
            break;
    }

    auto filepath = filesystem::path(full_filename);

    ChAssertAlways(filepath.exists());
    ChAssertAlways(filepath.is_file());

    return full_filename;
}

void ChYamlFileHandler::PrintInfo() const {
    switch (m_type) {
        case Type::ABS:
            cout << "using absolute file paths" << endl;
            break;
        case Type::REL:
            cout << "using file paths relative to: '" << m_reference_dir + "/" + m_relative_path << "'" << endl;
            break;
    }
}

// -----------------------------------------------------------------------------

void CheckVersion(const YAML::Node& a) {
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

ChVector3d ReadVector(const YAML::Node& a) {
    ChAssertAlways(a.IsSequence());
    ChAssertAlways(a.size() == 3);
    return ChVector3d(a[0].as<double>(), a[1].as<double>(), a[2].as<double>());
}

ChQuaterniond ReadRotation(const YAML::Node& a, bool use_degrees) {
    ChAssertAlways(a.IsSequence());
    ChAssertAlways(a.size() == 3 || a.size() == 4);

    return (a.size() == 3) ? ReadCardanAngles(a, use_degrees) : ReadQuaternion(a);
}

ChQuaterniond ReadQuaternion(const YAML::Node& a) {
    ChAssertAlways(a.IsSequence());
    ChAssertAlways(a.size() == 4);
    return ChQuaterniond(a[0].as<double>(), a[1].as<double>(), a[2].as<double>(), a[3].as<double>());
}

ChQuaterniond ReadCardanAngles(const YAML::Node& a, bool use_degrees) {
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

ChCoordsysd ReadCoordinateSystem(const YAML::Node& a, bool use_degrees) {
    ChAssertAlways(a["location"]);
    ChAssertAlways(a["orientation"]);
    return ChCoordsysd(ReadVector(a["location"]), ReadRotation(a["orientation"], use_degrees));
}

std::shared_ptr<ChFunction> ReadFunction(const YAML::Node& a, bool use_degrees) {
    ChAssertAlways(a["type"]);
    auto type = ChToUpper(a["type"].as<std::string>());

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

ChColor ReadColor(const YAML::Node& a) {
    ChAssertAlways(a.IsSequence());
    ChAssertAlways(a.size() == 3);
    return ChColor(a[0].as<float>(), a[1].as<float>(), a[2].as<float>());
}

ChColormap::Type ReadColorMapType(const YAML::Node& a) {
    auto val = ChToUpper(a.as<std::string>());
    if (val == "BLACK_BODY")
        return ChColormap::Type::BLACK_BODY;
    if (val == "BLUE")
        return ChColormap::Type::BLUE;
    if (val == "BROWN")
        return ChColormap::Type::BROWN;
    if (val == "COPPER")
        return ChColormap::Type::COPPER;
    if (val == "FAST")
        return ChColormap::Type::FAST;
    if (val == "INFERNO")
        return ChColormap::Type::INFERNO;
    if (val == "JET")
        return ChColormap::Type::JET;
    if (val == "KINDLMANN")
        return ChColormap::Type::KINDLMANN;
    if (val == "BLACK_BODY")
        return ChColormap::Type::BLACK_BODY;
    if (val == "PLASMA")
        return ChColormap::Type::PLASMA;
    if (val == "RED_BLUE")
        return ChColormap::Type::RED_BLUE;
    return ChColormap::Type::JET;
}

VisualizationType ReadVisualizationType(const YAML::Node& a) {
    auto type = ChToUpper(a.as<std::string>());
    if (type == "NONE")
        return VisualizationType::NONE;
    if (type == "PRIMITIVES")
        return VisualizationType::PRIMITIVES;
    if (type == "MODEL_FILE")
        return VisualizationType::MESH;
    if (type == "COLLISION")
        return VisualizationType::COLLISION;
    return VisualizationType::NONE;
}

ChOutput::Type ReadOutputType(const YAML::Node& a) {
    auto type = ChToUpper(a.as<std::string>());
    if (type == "ASCII")
        return ChOutput::Type::ASCII;
    if (type == "HDF5")
        return ChOutput::Type::HDF5;
    return ChOutput::Type::NONE;
}

ChOutput::Mode ReadOutputMode(const YAML::Node& a) {
    auto mode = ChToUpper(a.as<std::string>());
    if (mode == "SERIES")
        return ChOutput::Mode::SERIES;
    if (mode == "FRAMES")
        return ChOutput::Mode::FRAMES;
    return ChOutput::Mode::FRAMES;
}

ChSolver::Type ReadSolverType(const YAML::Node& a) {
    auto type = ChToUpper(a.as<std::string>());
    if (type == "BARZILAI_BORWEIN")
        return ChSolver::Type::BARZILAIBORWEIN;
    if (type == "PSOR")
        return ChSolver::Type::PSOR;
    if (type == "APGD")
        return ChSolver::Type::APGD;
    if (type == "MINRES")
        return ChSolver::Type::MINRES;
    if (type == "GMRES")
        return ChSolver::Type::GMRES;
    if (type == "BICGSTAB")
        return ChSolver::Type::BICGSTAB;
    if (type == "PARDISO")
        return ChSolver::Type::PARDISO_MKL;
    if (type == "MUMPS")
        return ChSolver::Type::MUMPS;
    if (type == "SPARSE_LU")
        return ChSolver::Type::SPARSE_LU;
    if (type == "SPARSE_QR")
        return ChSolver::Type::SPARSE_QR;

    cerr << "Unknown solver type: " << a.as<std::string>() << endl;
    throw std::runtime_error("Invalid solver type");
}

ChTimestepper::Type ReadIntegratorType(const YAML::Node& a) {
    auto type = ChToUpper(a.as<std::string>());
    if (type == "EULER_IMPLICIT_LINEARIZED")
        return ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED;
    if (type == "EULER_IMPLICIT_PROJECTED")
        return ChTimestepper::Type::EULER_IMPLICIT_PROJECTED;
    if (type == "EULER_IMPLICIT")
        return ChTimestepper::Type::EULER_IMPLICIT;
    if (type == "HHT")
        return ChTimestepper::Type::HHT;

    cerr << "Unknown integrator type: " << a.as<std::string>() << endl;
    throw std::runtime_error("Invalid integrator type");
}

// -----------------------------------------------------------------------------

void PrintNodeType(const YAML::Node& node) {
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

}  // namespace chrono
