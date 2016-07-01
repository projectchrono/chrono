// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Felipe Gutierrez, Conlain Kelly, Radu Serban
// =============================================================================

#ifndef BASE_TEST_H
#define BASE_TEST_H

#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

class Object {
  public:
    Object() { m_content << "{"; }
    ~Object() {}

    void AddMember(const std::string& key, double value, bool verbose) {
        if (verbose)
            std::cout << "Adding entry " << key << " (type double)" << std::endl;
        m_content << "\n    \"" << key << "\": " << value << ",";
    }

    void AddMember(const std::string& key, Object& value, bool verbose) {
        if (verbose)
            std::cout << "Adding entry " << key << " (type Object)" << std::endl;
        m_content << "\n    \"" << key << "\": " << value.GetContent() << ",";
    }

    void AddMember(const std::string& key, uint64_t value, bool verbose) {
        if (verbose)
            std::cout << "Adding entry " << key << " (type uint64)" << std::endl;
        m_content << "\n    \"" << key << "\": " << value << ",";
    }

    void AddMember(const std::string& key, int value, bool verbose) {
        if (verbose)
            std::cout << "Adding entry " << key << " (type int)" << std::endl;
        m_content << "\n    \"" << key << "\": " << value << ",";
    }

    void AddMember(const std::string& key, const std::string& value, bool verbose) {
        if (verbose)
            std::cout << "Adding entry " << key << " (type string)" << std::endl;
        m_content << "\n    \"" << key << "\": \"" << value << "\",";
    }

    std::string GetContent() const {
        // Remove last comma and close the JSON object.
        return m_content.str().substr(0, m_content.str().length() - 1) + "\n}\n";
    }

  private:
    std::stringstream m_content;  // object content (JSON format)
};

class BaseTest {
  public:
    /// Constructor: Every test has to have a name and an associated project to it.
    BaseTest(const std::string& testName, const std::string& testProjectName)
        : m_name(testName), m_projectName(testProjectName), m_outDir("."), m_verbose(false) {}

    virtual ~BaseTest() {}

    /// Get the test name.
    const std::string& getTestName() const { return m_name; }

    /// Get the project name.
    const std::string& getProjectName() const { return m_projectName; }

    /// Get the output directory.
    const std::string& getOutDir() const { return m_outDir; }

    /// Enable/disable verbose output.
    void setVerbose(bool val) { m_verbose = val; }

    /// Set output directory (default: current directory).
    void setOutDir(const std::string& outDir) { m_outDir = outDir; }

    /// Main function for running the test.
    bool run() {
        // Execute the actual test and collect metrics
        bool passed = execute();

        // Populate output JSON string
        m_jsonTest.AddMember("name", m_name, false);
        m_jsonTest.AddMember("project_name", m_projectName, false);
        m_jsonTest.AddMember("passed", passed, false);
        m_jsonTest.AddMember("execution_time", getExecutionTime(), false);
        m_jsonTest.AddMember("metrics", m_jsonMetrics, false);

        // Write output file
        std::string fname = m_outDir + "/" + m_name + ".json";
        if (m_verbose)
            std::cout << "Write output file: " << fname << std::endl;
        m_jsonfile.open(fname);
        if (m_jsonfile.is_open()) {
            m_jsonfile << m_jsonTest.GetContent();
            m_jsonfile.close();
        } else {
            std::cerr << "UNABLE to open file." << std::endl;
        }

        return passed;
    }

    /// Add a test-specific metric (a key-value pair)
    void addMetric(const std::string& metricName, double metricValue) {
        m_jsonMetrics.AddMember(metricName, metricValue, m_verbose);
    }

    void addMetric(const std::string& metricName, int metricValue) {
        m_jsonMetrics.AddMember(metricName, metricValue, m_verbose);
    }

    void addMetric(const std::string& metricName, uint64_t metricValue) {
        m_jsonMetrics.AddMember(metricName, metricValue, m_verbose);
    }

    void addMetric(const std::string& metricName, const std::string& metricValue) {
        m_jsonMetrics.AddMember(metricName, metricValue, m_verbose);
    }

    // void addMetric(const std::string&   metricName,
    //                std::vector<double>& metricValue) {
    //   m_jsonMetrics.AddMember(metricName, metricValue, m_verbose);
    // }

    /// Execute the actual test.
    /// A derived class must implement this function to return true if the test passes
    /// and false otherwise. During execution of the test, various performance metrics
    /// can be cached by calling the addMetric() methods.
    virtual bool execute() = 0;

    /// Return total execution time for this test.
    virtual double getExecutionTime() const = 0;

    /// Print the content of the JSON string.
    void print() {
        std::cout << "Test Information: " << std::endl;
        std::cout << m_jsonTest.GetContent();
    }

  private:
    std::ofstream m_jsonfile;   ///< Output JSON file
    std::string m_name;         ///< Name of test
    std::string m_projectName;  ///< Name of the project
    std::string m_outDir;       ///< Name of output directory
    bool m_verbose;             ///< Verbose output
    Object m_jsonMetrics;       ///< collected metrics
    Object m_jsonTest;          ///< JSON output of the test
};

#endif
