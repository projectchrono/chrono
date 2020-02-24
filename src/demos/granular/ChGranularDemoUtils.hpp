#pragma once
#include <string>
#include <iostream>
#include <fstream>

#include "chrono/core/ChVector.h"

void tokenizeCSVLine(std::ifstream& istream, std::vector<float> data) {
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
std::vector<chrono::ChVector<T>> loadPositionCheckpoint(std::string infile) {
    // file stream to load in
    std::ifstream ptFile(infile);

    std::vector<chrono::ChVector<T>> sphere_positions;
    std::string tmp_line;
    std::getline(ptFile, tmp_line);  // skip first header line
    // TODO look ahead and reserve space to avoid push_backs
    while (ptFile.good()) {
        std::vector<float> line_data;
        tokenizeCSVLine(ptFile, line_data);
        chrono::ChVector<> curr_pos(line_data.at(0), line_data.at(1), line_data.at(2));
        sphere_positions.push_back(curr_pos);
    }

    return sphere_positions;
}
