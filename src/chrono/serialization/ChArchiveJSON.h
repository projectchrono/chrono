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
#ifndef CHARCHIVEJSON_H
#define CHARCHIVEJSON_H

#include "chrono/serialization/ChArchive.h"

#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_thirdparty/rapidjson/prettywriter.h"
#include "chrono_thirdparty/rapidjson/filereadstream.h"
#include "chrono_thirdparty/rapidjson/filewritestream.h"

#include <stack>
#include <fstream>
#include <iostream>
#include <sstream>

namespace chrono {

/// Serialize objects using JSON format.
/// Input stream should be kept valid for the entire lifespan of the archive class.
class ChApi ChArchiveOutJSON : public ChArchiveOut {
  public:
    ChArchiveOutJSON(std::ostream& stream_out);

    virtual ~ChArchiveOutJSON();

    void indent();

    void comma_cr();

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

    virtual void out_array_pre(ChValue& bVal, size_t msize);
    virtual void out_array_between(ChValue& bVal, size_t msize);
    virtual void out_array_end(ChValue& bVal, size_t msize);

    // for custom c++ objects:
    virtual void out(ChValue& bVal, bool tracked, size_t obj_ID);

    virtual void out_ref(ChValue& bVal, bool already_inserted, size_t obj_ID, size_t ext_ID);

  protected:
    int tablevel;
    std::ostream& m_ostream;
    std::stack<int> nitems;
    std::stack<bool> is_array;
};

///
/// This is a class for deserializing from JSON archives
///

class ChApi ChArchiveInJSON : public ChArchiveIn {
  public:
    ChArchiveInJSON(std::ifstream& stream_in);

    virtual ~ChArchiveInJSON();

    rapidjson::Value* GetValueFromNameOrArray(const std::string& mname);

    virtual bool in(ChNameValue<bool> bVal) override;
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
    virtual bool in_array_pre(const std::string& name, size_t& msize) override;
    virtual void in_array_between(const std::string& name) override;

    virtual void in_array_end(const std::string& name) override;

    //  for custom c++ objects:
    virtual bool in(ChNameValue<ChFunctorArchiveIn> bVal) override;

    // for objects to construct, return non-null ptr if new object, return null ptr if just reused obj
    virtual bool in_ref(ChNameValue<ChFunctorArchiveIn> bVal, void** ptr, std::string& true_classname) override;

    virtual bool TryTolerateMissingTokens(bool try_tolerate) override;

  protected:
    void token_notfound(const std::string& mname);

    std::ifstream& m_istream;
    rapidjson::Document document;
    rapidjson::Value* level;
    std::stack<rapidjson::Value*> levels;
    std::stack<bool> is_array;
    std::stack<int> array_index;
};

}  // end namespace chrono

#endif
