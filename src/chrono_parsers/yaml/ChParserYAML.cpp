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

#include "chrono/input_output/ChOutputASCII.h"
#ifdef CHRONO_HAS_HDF5
    #include "chrono/input_output/ChOutputHDF5.h"
#endif

#include "chrono_parsers/yaml/ChParserYAML.h"

using std::cout;
using std::cerr;
using std::endl;

namespace chrono {
namespace parsers {

ChParserYAML::ChParserYAML() : m_name("model"), m_verbose(false), m_use_degrees(true), m_output_dir(".") {
    m_vis_settings.image_dir = "./images";
}

// -----------------------------------------------------------------------------

ChParserYAML::YamlFileType ChParserYAML::ReadYamlFileType(const std::string& yaml_filename) {
    auto path = std::filesystem::path(yaml_filename);
    if (!exists(path) || !is_regular_file(path)) {
        cerr << "Error: file '" << yaml_filename << "' not found." << endl;
        throw std::runtime_error("File not found");
    }
    YAML::Node yaml = YAML::LoadFile(yaml_filename);
    ChAssertAlways(yaml["type"]);
    return ReadYamlFileType(yaml["type"]);
}

ChParserYAML::YamlFileType ChParserYAML::ReadYamlFileType(const YAML::Node& a) {
    auto type = ChToUpper(a.as<std::string>());
    if (type == "MBS")
        return YamlFileType::MBS;
    if (type == "SPH")
        return YamlFileType::SPH;
    if (type == "TDPF")
        return YamlFileType::TDPF;
    if (type == "FSI")
        return YamlFileType::FSI;
    if (type == "VEHICLE")
        return YamlFileType::VEHICLE;
    return YamlFileType::UNKNOWN;
}

// -----------------------------------------------------------------------------

void ChParserYAML::LoadSimData(const YAML::Node& yaml) {
    // Output (optional)
    if (yaml["output"])
        m_output_settings = ChOutput::Settings::Read(yaml["output"]);

    // Run-time visualization (optional)
    if (yaml["visualization"])
        m_vis_settings = ChVisualSystem::Settings::Read(yaml["visualization"]);
}

// -----------------------------------------------------------------------------

void ChParserYAML::SetOutputDir(const std::string& out_dir) {
    m_output_dir = out_dir;
    m_vis_settings.image_dir = out_dir + "/images";

    if (m_output_settings.format != ChOutput::Format::NONE) {
        auto p = std::filesystem::path(m_output_dir);
        if (!exists(p) || !is_directory(p)) {
            std::cerr << "The path " << m_output_dir << " is not a valid directory." << std::endl;
            throw std::runtime_error("Invalid directory");
        }
    }

    if (m_vis_settings.write_images) {
        bool success = CreateOutputDirectory(std::filesystem::path(m_vis_settings.image_dir));
        if (!success) {
            std::cerr << "Error creating image output directory " << m_vis_settings.image_dir << std::endl;
            throw std::runtime_error("Could not create image output directory");
        }
    }

    if (m_verbose) {
        if (m_output_settings.format != ChOutput::Format::NONE) {
            auto filename = m_output_dir + "/" + m_name + "." + ChOutput::GetModeAsString(m_output_settings.mode);
            switch (m_output_settings.format) {
                case ChOutput::Format::ASCII:
                    cout << "Output file: " << filename << ".txt" << endl;
                    break;
                case ChOutput::Format::HDF5:
#ifdef CHRONO_HAS_HDF5
                    cout << "Output file: " << filename << ".h5" << endl;
                    break;
#else
                    cerr << "HDF5 not available. No output file will be generated" << endl;
#endif
            }
        }

        if (m_vis_settings.write_images) {
            cout << "Image directory: " << m_vis_settings.image_dir << endl;
        }
    }
}

void ChParserYAML::WriteOutput(int frame, double time) {
    if (m_output_settings.format == ChOutput::Format::NONE)
        return;

    // Create the output DB if needed
    if (!m_output_db) {
        switch (m_output_settings.format) {
            case ChOutput::Format::ASCII:
                m_output_db = chrono_types::make_shared<ChOutputASCII>(m_output_dir, m_name, m_output_settings.mode);
                break;
            case ChOutput::Format::HDF5:
#ifdef CHRONO_HAS_HDF5
                m_output_db = chrono_types::make_shared<ChOutputHDF5>(m_output_dir, m_name, m_output_settings.mode);
                break;
#else
                break;
#endif
        }
    }
}

// -----------------------------------------------------------------------------

void ChParserYAML::Output(double time) {
    static int output_frame = 0;
    if (time >= output_frame / m_output_settings.fps) {
        WriteOutput(output_frame, time);
        output_frame++;
    }
}

bool ChParserYAML::Render(ChVisualSystem& vis_sys, double time) {
    if (!vis_sys.Run())
        return false;

    static int render_frame = 0;
    if (time >= render_frame / m_vis_settings.render_fps) {
        vis_sys.Render();
        if (m_vis_settings.write_images) {
            std::ostringstream filename;
            filename << m_vis_settings.image_dir << "/img_" << std::setw(5) << std::setfill('0') << render_frame + 1 << "." << m_vis_settings.image_type;
            vis_sys.WriteImageToFile(filename.str());
        }
        render_frame++;
    }

    return true;
}

}  // end namespace parsers
}  // namespace chrono
