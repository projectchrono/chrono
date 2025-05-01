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
// Authors: Jason Zhou
// =============================================================================
//
// Chrono::Gpu unit testing common functions
// =============================================================================

#pragma once
#include <string>
#include <iostream>
#include <iomanip>
#include <fstream>

#include "chrono/core/ChVector3.h"

void tokenizeCSVLine(std::ifstream& istream, std::vector<float>& data) {
    std::string line;
    std::getline(istream, line);  // load in current line
    std::stringstream lineStream(line);
    std::string cell;

    // iterate over cells
    while (std::getline(lineStream, cell, ',')) {
        data.push_back(std::stof(cell));
    }
}

// load sphere positions from a checkpoint file
template <typename T>
std::vector<chrono::ChVector3<T>> loadPositionCheckpoint(std::string infile) {
    // file stream to load in
    std::ifstream ptFile(infile);
    std::vector<chrono::ChVector3<T>> sphere_positions;
    std::string tmp_line;
    std::getline(ptFile, tmp_line);  // skip first header line
    // TODO look ahead and reserve space to avoid push_backs
    while (ptFile.good()) {
        std::vector<float> line_data;
        tokenizeCSVLine(ptFile, line_data);

        if (line_data.size() != 0) {
            chrono::ChVector3d curr_pos(line_data.at(0), line_data.at(1), line_data.at(2));
            sphere_positions.push_back(curr_pos);
        }
    }

    return sphere_positions;
}

// load a custom column from a checkpoint file
std::vector<float> loadColumnCheckpoint(std::string infile, int i) {
    // file stream to load in
    std::ifstream ptFile(infile);
    std::vector<float> sphere_positions;
    std::string tmp_line;
    std::getline(ptFile, tmp_line);  // skip first header line
    // TODO look ahead and reserve space to avoid push_backs
    while (ptFile.good()) {
        std::vector<float> line_data;
        tokenizeCSVLine(ptFile, line_data);

        if (line_data.size() != 0 && line_data.size() > i) {
            float curr_pos(line_data.at(i));
            sphere_positions.push_back(curr_pos);
        }
    }

    return sphere_positions;
}
// load a custom column from a checkpoint file

// load sphere velocities from a checkpoint file
template <typename T>
std::vector<chrono::ChVector3<T>> loadVelocityCheckpoint(std::string infile) {
    // file stream to load in
    std::ifstream ptFile(infile);
    std::vector<chrono::ChVector3<T>> sphere_velocities;
    std::string tmp_line;
    std::getline(ptFile, tmp_line);  // skip first header line
    // TODO look ahead and reserve space to avoid push_backs
    while (ptFile.good()) {
        std::vector<float> line_data;
        tokenizeCSVLine(ptFile, line_data);

        if (line_data.size() != 0) {
            chrono::ChVector3d curr_vel(line_data.at(3), line_data.at(4), line_data.at(5));
            sphere_velocities.push_back(curr_vel);
        }
    }

    return sphere_velocities;
}
// load sphere velocities from a checkpoint file