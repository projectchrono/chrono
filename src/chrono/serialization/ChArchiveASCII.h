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

#ifndef CH_ARCHIVE_ASCII_H
#define CH_ARCHIVE_ASCII_H

#include "chrono/serialization/ChArchive.h"
#include <cstring>

namespace chrono {

/// Output objects structure to a human-readable ASCII format.
/// This class only allows to output/serialize the object, not to deserialize it.
class ChApi ChArchiveOutASCII : public ChArchiveOut {
  public:
    ChArchiveOutASCII(std::ostream& stream_out);

    virtual ~ChArchiveOutASCII();

    /// Suppress export of variable names
    void SetSuppressNames(bool msu);

    /// Access the stream used by the archive.
    std::ostream& GetStream();

    void indent();

    virtual void out(ChNameValue<bool> bVal);
    virtual void out(ChNameValue<int> bVal);
    virtual void out(ChNameValue<double> bVal);
    virtual void out(ChNameValue<float> bVal);
    virtual void out(ChNameValue<unsigned int> bVal);
    virtual void out(ChNameValue<unsigned long> bVal);
    virtual void out(ChNameValue<unsigned long long> bVal);
    virtual void out(ChNameValue<ChEnumMapperBase> bVal);

    virtual void out(ChNameValue<char> bVal);
    virtual void out(ChNameValue<std::string> bVal);

    virtual void out_array_pre(ChValue& bVal, size_t msize);
    virtual void out_array_between(ChValue& bVal, size_t msize);
    virtual void out_array_end(ChValue& bVal, size_t msize);

    // for custom c++ objects:
    virtual void out(ChValue& bVal, bool tracked, size_t obj_ID);

    virtual void out_ref(ChValue& bVal, bool already_inserted, size_t obj_ID, size_t ext_ID);

  protected:
    int tablevel;
    std::ostream& m_ostream;
    bool suppress_names;
};

}  // end namespace chrono

#endif
