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
// General output to a Comma - Separated Values ASCII file.
//
// =============================================================================

#ifndef CH_WRITER_CSV_H
#define CH_WRITER_CSV_H

#include <string>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <vector>

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChVector3.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono/assets/ChColor.h"

namespace chrono {

/// @addtogroup chrono_io
/// @{

/// General output to a Comma-Separated Values ASCII file.
class ChApi ChWriterCSV {
  public:
    explicit ChWriterCSV(const std::string& delim = ",");
    ChWriterCSV(const ChWriterCSV& source);
    ~ChWriterCSV() {}

    void WriteToFile(const std::string& filename, const std::string& header = "") const;

    void SetDelimiter(const std::string& delim) { m_delim = delim; }
    const std::string& GetDelimiter() const { return m_delim; }
    std::ostringstream& Stream() { return m_ss; }

    template <typename T>
    ChWriterCSV& operator<<(const T& t) {
        m_ss << t << m_delim;
        return *this;
    }

    ChWriterCSV& operator<<(std::ostream& (*t)(std::ostream&));
    ChWriterCSV& operator<<(std::ios& (*t)(std::ios&));
    ChWriterCSV& operator<<(std::ios_base& (*t)(std::ios_base&));

  private:
    std::string m_delim;
    std::ostringstream m_ss;
};

template <typename T>
inline ChWriterCSV& operator<<(ChWriterCSV& out, const ChVector3<T>& v) {
    out << v.x() << v.y() << v.z();
    return out;
}

template <typename T>
inline ChWriterCSV& operator<<(ChWriterCSV& out, const ChQuaternion<T>& q) {
    out << q.e0() << q.e1() << q.e2() << q.e3();
    return out;
}

inline ChWriterCSV& operator<<(ChWriterCSV& out, const ChColor& c) {
    out << c.R << c.G << c.B;
    return out;
}

template <typename T>
inline ChWriterCSV& operator<<(ChWriterCSV& out, const std::vector<T>& vec) {
    for (const auto& v : vec)
        out << v;
    return out;
}

/// @} chrono_io

}  // namespace chrono

#endif
