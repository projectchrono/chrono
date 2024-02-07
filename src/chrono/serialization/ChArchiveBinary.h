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

///
/// This is a class for serializing to binary archives
///

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

    ////TODO: shall I specify that T is a reference or should I let the compiler optimize it?
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

///
/// This is a class for serializing from binary archives
///

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
    virtual bool in(ChNameValue<ChFunctorArchiveIn> bVal);

    virtual bool in_ref(ChNameValue<ChFunctorArchiveIn> bVal, void** ptr, std::string& true_classname);

  protected:
    std::istream& m_istream;

    template <typename T>
    std::istream& read(T& val) {
        return m_istream.read(reinterpret_cast<char*>(&val), sizeof(T));
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
