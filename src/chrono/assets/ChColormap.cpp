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

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>

#include "chrono/assets/ChColormap.h"
#include "chrono/core/ChClassFactory.h"
#include "chrono/core/ChGlobal.h"
#include "chrono/utils/ChUtils.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChColormap)

// Initialize static members
std::unordered_map<ChColormap::Type, ChColormap::Files> ChColormap::m_colormap_files{
    {ChColormap::Type::BLACK_BODY,
     {GetChronoDataFile("colormaps/black-body-table-float-0512.csv"), GetChronoDataFile("colormaps/black-body.png")}},
    {ChColormap::Type::BROWN,
     {GetChronoDataFile("colormaps/brown-table-float-0512.csv"), GetChronoDataFile("colormaps/brown.png")}},
    {ChColormap::Type::COPPER,
     {GetChronoDataFile("colormaps/copper-table-float-0512.csv"), GetChronoDataFile("colormaps/copper.png")}},
    {ChColormap::Type::FAST,
     {GetChronoDataFile("colormaps/fast-table-float-0512.csv"), GetChronoDataFile("colormaps/fast.png")}},
    {ChColormap::Type::INFERNO,
     {GetChronoDataFile("colormaps/inferno-table-float-0512.csv"), GetChronoDataFile("colormaps/inferno.png")}},
    {ChColormap::Type::JET,
     {GetChronoDataFile("colormaps/jet-table-float-0512.csv"), GetChronoDataFile("colormaps/jet.png")}},
    {ChColormap::Type::KINDLMANN,
     {GetChronoDataFile("colormaps/kindlmann-table-float-0512.csv"), GetChronoDataFile("colormaps/kindlmann.png")}},
    {ChColormap::Type::PLASMA,
     {GetChronoDataFile("colormaps/plasma-table-float-0512.csv"), GetChronoDataFile("colormaps/plasma.png")}},
    {ChColormap::Type::RED_BLUE,
     {GetChronoDataFile("colormaps/red-blue-table-float-0512.csv"), GetChronoDataFile("colormaps/red-blue.png")}},
    {ChColormap::Type::VIRIDIS,
     {GetChronoDataFile("colormaps/viridis-table-float-0512.csv"), GetChronoDataFile("colormaps/viridis.png")}}};

ChColormap::ChColormap(Type type) {
    Load(type);
}

void ChColormap::Load(Type type) {
    const std::string& filename = m_colormap_files.at(type).data_file;
    std::ifstream ifs(filename);
    if (ifs.is_open()) {
        m_map.clear();

        std::string line;
        std::getline(ifs, line);
        while (std::getline(ifs, line)) {
            std::istringstream iss(line);
            char c;
            float v, R, G, B;
            iss >> v >> c >> R >> c >> G >> c >> B;
            m_map.push_back(Entry(v, ChColor(R, G, B)));
        }
        ifs.close();
    } else {
        std::cout << "Cannot open colormap data file " << filename << std::endl;
    }
}

ChColor ChColormap::Get(double value) const {
    ChClampValue(value, 0.0, 1.0);

    if (value <= m_map.front().first)
        return m_map.front().second;

    if (value >= m_map.back().first)
        return m_map.back().second;

    const auto it2 = std::find_if(m_map.begin(), m_map.end(), [value](Entry e) { return e.first >= value; });
    assert(it2 != m_map.begin());
    auto it1 = it2 - 1;

    const double v1 = it1->first;
    const ChColor& c1 = it1->second;
    const double v2 = it2->first;
    const ChColor& c2 = it2->second;

    float v = (float)((value - v1) / (v2 - v1));

    ChColor c(c1.R * (1 - v) + c2.R * v,  //
              c1.G * (1 - v) + c2.G * v,  //
              c1.B * (1 - v) + c2.B * v);
    return c;
}

ChColor ChColormap::Get(double value, double vmin, double vmax) const {
    return Get((value - vmin) / (vmax - vmin));
}

void ChColormap::ArchiveOut(ChArchiveOut& archive_out) {
    archive_out.VersionWrite<ChColormap>();
}

void ChColormap::ArchiveIn(ChArchiveIn& archive_in) {
    /*int version =*/archive_in.VersionRead<ChColormap>();
}

}  // end namespace chrono
