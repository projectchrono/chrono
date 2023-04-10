// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
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
// Parser utility class for URDF input files.
//
// =============================================================================

#include <string>
#include <fstream>
#include <iostream>

#include "chrono_parsers/ChParserURDF.h"

namespace chrono {
namespace parsers {

using namespace urdf;
using std::cout;
using std::cerr;
using std::endl;

void ChParserURDF::Parse(ChSystem& sys, const std::string& filename) {
    // Read input file into XML string
    std::string xml_string;
    std::fstream xml_file(filename, std::fstream::in);
    while (xml_file.good()) {
        std::string line;
        std::getline(xml_file, line);
        xml_string += (line + "\n");
    }
    xml_file.close();

    // Parse XML string
    m_model = parseURDF(xml_string);
    if (!m_model) {
        cerr << "ERROR: parsing the URDF file " << filename << " failed." << endl;
        return;
    }
}

// -----------------------------------------------------------------------------

// From check_urdf.cpp in the urdfdom distribution
void printTree(LinkConstSharedPtr link, int level = 0) {
    level += 2;
    int count = 0;
    for (auto child = link->child_links.begin(); child != link->child_links.end(); ++child) {
        if (*child) {
            for (int j = 0; j < level; j++)
                cout << "  ";  // indent
            cout << "child(" << (count++) + 1 << "):  " << (*child)->name << endl;
            // first grandchild
            printTree(*child, level);
        } else {
            for (int j = 0; j < level; j++)
                cout << " ";  // indent
            cout << "root link: " << link->name << " has a null child!" << *child << endl;
        }
    }
}

void ChParserURDF::PrintModelTree() {
    if (!m_model)
        return;

    auto root_link = m_model->getRoot();
    cout << "Name: " << m_model->getName() << endl;
    cout << "Root: " << root_link->name << " has " << root_link->child_links.size() << " child(ren)" << endl;
    printTree(root_link);
}

// -----------------------------------------------------------------------------

}  // end namespace parsers
}  // end namespace chrono
