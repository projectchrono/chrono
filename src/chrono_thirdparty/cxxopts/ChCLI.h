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

#include "chrono_thirdparty/cxxopts/cxxopts.hpp"

namespace chrono {

/// Wrapper for cxxopts.
class ChCLI {
  public:
    /// Constructor
    ChCLI(const std::string& program, const std::string& help_string= " - command line options") : m_options(program, help_string) {
        m_options.add_option("", cxxopts::Option("h,help", "Print usage"));
    }

    /// Destructor
    ~ChCLI() {}

    /// Parse messages and display help message, if needed
    bool Parse(int argc, char** argv, bool show_help = false, bool count_help = true) {
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

        if (count_help && m_result->count("help")) {
            if (show_help)
                Help();
            return false;
        }

        return true;
    }

    /// Print the help menu
    void Help() { std::cout << m_options.help() << std::endl; }

    /// Check if help was passed
    bool CheckHelp() { return m_result->count("help") > 0; }

    /// Check if the specified option was passed
    bool CheckOption(const std::string& option) {return m_result->count(option) > 0; }

    /// Check for value in vector
    template <typename T>
    bool HasValueInVector(const std::string& option, T value) {
        const auto vector = Get(option).as<std::vector<T>>();
        return std::find(vector.begin(), vector.end(), value) != vector.end();
    }

    template <typename T = bool>
    void AddOption(const std::string& group,
                   const std::string& opts,
                   const std::string& desc,
                   const std::string& def,
                   const std::string& arg_help) {
        m_options.add_option(group, cxxopts::Option(opts, desc, cxxopts::value<T>()->default_value(def), arg_help));
    }

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
        } catch (std::domain_error&) {
            if (m_result->count(option) != 0) {
                std::cerr << "ChCLI::GetAsType: Could not cast \"" << option << "\" as " << typeid(T).name() << std::endl;
            } else {
                std::cerr << "Option \"" << option << "\" requested by ChCLI::GetAsType, but has no default value and not present on command line" << std::endl;
            }
            exit(-1);
        }
    }

    template <typename T>
    const bool Matches(const T& option, const T& value) {
        return GetAsType<T>(option) == value;
    }

  private:
    cxxopts::Options m_options;                      ///< Command line options
    std::shared_ptr<cxxopts::ParseResult> m_result;  ///< Parsing results
};

}  // namespace chrono

#endif
