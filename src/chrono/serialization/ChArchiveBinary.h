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

#ifndef CHARCHIVEBINARY_H
#define CHARCHIVEBINARY_H

#include <cstring>

#include "chrono/serialization/ChArchive.h"

namespace chrono {

/// Templated function for swapping bytes of objects of type 'T', for arbitrary T type.
/// This is used for cross-platform compatibility when sharing objects between big-endian and little-endian memory
/// models, depending on the microprocessor type.
template <class T>
inline void StreamSwapBytes(T* ptData) {
    char* acBytes = (char*)ptData;
    size_t iSize = sizeof(T);
    size_t iSizeM1 = iSize - 1;
    size_t iHSize = iSize / 2;
    for (size_t i0 = 0, i1 = iSizeM1; i0 < iHSize; i0++, i1--) {
        char cSave = acBytes[i0];
        acBytes[i0] = acBytes[i1];
        acBytes[i1] = cSave;
    }
}

/// Serialization to binary stream.
///
/// Typical usage:
/// \code{.cpp}
/// std::ofstream fileo("/file.dat", std::ios::binary);
/// ChArchiveOutBinary archiveout(fileo);
/// archiveout << CHNVP(myobj);
/// \endcode
/// Remember to set stream mode to `std::ios::binary`.
/// Data will always be written with little-endianness even in big-endian machines.
class ChApi ChArchiveOutBinary : public ChArchiveOut {
  public:
    ChArchiveOutBinary(std::ostream& stream_out);

    virtual ~ChArchiveOutBinary();

    virtual void out(ChNameValue<bool> bVal);
    virtual void out(ChNameValue<int> bVal);
    virtual void out(ChNameValue<double> bVal);
    virtual void out(ChNameValue<float> bVal);
    virtual void out(ChNameValue<char> bVal);
    virtual void out(ChNameValue<unsigned int> bVal);
    virtual void out(ChNameValue<unsigned long> bVal);
    virtual void out(ChNameValue<unsigned long long> bVal);
    virtual void out(ChNameValue<ChEnumMapperBase> bVal);

    virtual void out(ChNameValue<const char*> bVal);
    virtual void out(ChNameValue<std::string> bVal);

    virtual void out_array_pre(ChValue& bVal, size_t size);
    virtual void out_array_between(ChValue& bVal, size_t size);
    virtual void out_array_end(ChValue& bVal, size_t size);

    // for custom c++ objects:
    virtual void out(ChValue& bVal, bool tracked, size_t obj_ID);

    virtual void out_ref(ChValue& bVal, bool already_inserted, size_t obj_ID, size_t ext_ID);

  protected:
    std::ostream& m_ostream;

    template <typename T>
    std::ostream& write(T val) {
        return m_ostream.write(reinterpret_cast<char*>(&val), sizeof(T));
    }
};

template <>
std::ostream& ChArchiveOutBinary::write(std::string val);

template <>
std::ostream& ChArchiveOutBinary::write(bool val);

template <>
std::ostream& ChArchiveOutBinary::write(const char* val);

/// Deserialization from binary stream.
///
/// Typical usage:
/// \code{.cpp}
/// std::ifstream filei("/file.dat", std::ios::binary);
/// ChArchiveInBinary archivein(filei);
/// archivein >> CHNVP(myobj);
/// \endcode
/// Remember to set stream mode to `std::ios::binary`.
/// Data is expected to always be printed with little-endianness even in big-endian machines.
/// The class will convert to big-endian if necessary.
class ChApi ChArchiveInBinary : public ChArchiveIn {
  public:
    ChArchiveInBinary(std::istream& stream_in);

    virtual ~ChArchiveInBinary();

    virtual bool in(ChNameValue<bool> bVal) override;

    ////Can they merged in just one instance?
    virtual bool in(ChNameValue<int> bVal) override;
    virtual bool in(ChNameValue<double> bVal) override;
    virtual bool in(ChNameValue<float> bVal) override;
    virtual bool in(ChNameValue<unsigned int> bVal) override;
    virtual bool in(ChNameValue<unsigned long> bVal) override;
    virtual bool in(ChNameValue<unsigned long long> bVal) override;

    virtual bool in(ChNameValue<ChEnumMapperBase> bVal) override;

    virtual bool in(ChNameValue<char> bVal) override;
    virtual bool in(ChNameValue<std::string> bVal) override;

    // for wrapping arrays and lists
    virtual bool in_array_pre(const std::string& name, size_t& size) override;
    virtual void in_array_between(const std::string& name) override {}
    virtual void in_array_end(const std::string& name) override {}

    // for custom c++ objects
    virtual bool in(ChNameValue<ChFunctorArchiveIn> bVal) override;

    virtual bool in_ref(ChNameValue<ChFunctorArchiveIn> bVal, void** ptr, std::string& true_classname) override;

  protected:
    std::istream& m_istream;
    bool m_big_endian_machine;

    template <typename T>
    std::istream& read(T& val) {
        m_istream.read(reinterpret_cast<char*>(&val), sizeof(T));
        if (m_big_endian_machine) {
            StreamSwapBytes<T>(&val);
        }

        return m_istream;
    }
};

template <>
std::istream& ChArchiveInBinary::read(std::string& val);

template <>
std::istream& ChArchiveInBinary::read(bool& val);

template <>
std::istream& ChArchiveInBinary::read(char*& val);

}  // end namespace chrono

#endif
