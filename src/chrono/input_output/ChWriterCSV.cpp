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
// Authors: Radu Serban
// =============================================================================
//
// =============================================================================

#include "chrono/input_output/ChWriterCSV.h"

namespace chrono {

ChWriterCSV::ChWriterCSV(const std::string& delim) : m_delim(delim) {}

ChWriterCSV::ChWriterCSV(const ChWriterCSV& source) : m_delim(source.m_delim) {
    // Note that we do not copy the stream buffer (as then it would be shared!)
    m_ss.copyfmt(source.m_ss);          // copy all data
    m_ss.clear(source.m_ss.rdstate());  // copy the error state
}

void ChWriterCSV::WriteToFile(const std::string& filename, const std::string& header) const {
    std::ofstream ofile(filename);
    if (!header.empty())
        ofile << header << std::endl;
    ofile << m_ss.str();
    ofile.close();
}

ChWriterCSV& ChWriterCSV::operator<<(std::ostream& (*t)(std::ostream&)) {
    m_ss << t;
    return *this;
}

ChWriterCSV& ChWriterCSV::operator<<(std::ios& (*t)(std::ios&)) {
    m_ss << t;
    return *this;
}

ChWriterCSV& ChWriterCSV::operator<<(std::ios_base& (*t)(std::ios_base&)) {
    m_ss << t;
    return *this;
}

}  // namespace chrono
