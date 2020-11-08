// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Aaron Young, Jay Taves
// =============================================================================
//
// Wrapper for cxxopts that provides several common command-line options to the
// user.
//
// =============================================================================

#ifndef CH_CLI_H
#define CH_CLI_H

#include "cxxopts.hpp"

namespace chrono {

const double STEP_SIZE = 3e-3;
const double END_TIME = 20.0;
const double HEARTBEAT = 1e-2;
const bool VERBOSE = false;

class ChCLI {
  public:
    /// Constructor
    ChCLI(const std::string& program, const std::string& help_string= " - command line options") : m_options(program, help_string) {
        m_options.add_option("", cxxopts::Option("h,help", "Print usage"));
    }

    /// Destructor
    ~ChCLI() {}

    /// Parse messages and display help message, if needed
    bool Parse(int argc, char* argv[], bool show_help = false) {
        try {
            m_result = std::make_shared<cxxopts::ParseResult>(m_options.parse(argc, argv));
        } catch (cxxopts::OptionException& e) {
            if (show_help) {
                std::cout << "Error when parsing command line inputs." << std::endl;
                std::cout << "what(): " << e.what() << std::endl << std::endl;
                Help();
            }
            return false;
        }

        if (m_result->count("help")) {
            if (show_help)
                Help();
            return false;
        }

        return true;
    }

    /// Print our the help menu
    void Help() { std::cout << m_options.help() << std::endl; }

    /// Add default options for synchrono demos
    void AddDefaultDemoOptions() {
        // General simulation options
        AddOption<double>("Simulation", "step_size", "Step size", std::to_string(STEP_SIZE));
        AddOption<double>("Simulation", "end_time", "End time", std::to_string(END_TIME));
        AddOption<double>("Simulation", "heartbeat", "Heartbeat", std::to_string(HEARTBEAT));
        AddOption<bool>("Simulation", "verbose", "Verbosity", std::to_string(VERBOSE));
    }

    /// Check for value in vector
    template <typename T>
    bool HasValueInVector(const std::string& option, T value) {
        const auto vector = Get(option).as<std::vector<T>>();
        return std::find(vector.begin(), vector.end(), value) != vector.end();
    }

    /// Add an option
    template <typename T = bool>
    void AddOption(const std::string& group,
                   const std::string& opts,
                   const std::string& desc,
                   const std::string& def) {
        m_options.add_option(group, cxxopts::Option(opts, desc, cxxopts::value<T>()->default_value(def), ""));
    }

    template <typename T = bool>
    void AddOption(const std::string& group,
                   const std::string& opts,
                   const std::string& desc) {
        m_options.add_option(group, cxxopts::Option(opts, desc, cxxopts::value<T>()));
    }

    void AddOption(const std::string& group, cxxopts::Option option) {
        m_options.add_option(group, option);
    }

    const cxxopts::OptionValue& Get(const std::string& option) {
        return (*m_result)[option];
    }

    /// Get option as type
    /// Recommanded way of accessing
    template <typename T>
    const T GetAsType(const std::string& option) {
        try {
            return (*m_result)[option].as<T>();
        } catch (std::domain_error e) {
            if (m_result->count(option) != 0) {
                std::cerr << "ChCLI::GetAsType: Could not cast \"" << option << "\" as " << typeid(T).name() << std::endl;
            } else {
                std::cerr << "Option \"" << option << "\" requested by ChCLI::GetAsType, but has no default value and not present on command line" << std::endl;
            }
            exit(-1);
        }
    }

  private:
    cxxopts::Options m_options;                      ///< Command line options
    std::shared_ptr<cxxopts::ParseResult> m_result;  ///< Parsing results
};

}  // namespace chrono

#endif
