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

#ifndef CHOBJECTEXPLORER_H
#define CHOBJECTEXPLORER_H

#include "chrono/serialization/ChArchive.h"
#include <typeindex>

namespace chrono {

/// A helper class to provide some basic mechanism of C++ reflection (introspection).
/// Properties of objects can be accessed/listed as name-value pairs. Names are those
/// provided in the CHNVP pairs in ArchiveOut/ArchiveIn.
/// As this is a quite simple reflection system, the value is always passed as void*,
/// it is up to you to do the proper static_cast<> to the class of the object. There
/// are some shortcuts though, to access double, int and other fundamental types with
/// a single statement.
class ChApi ChObjectExplorer : public ChArchiveOut {
  public:
    ChObjectExplorer();

    virtual ~ChObjectExplorer() { ClearSearch(); };

    /// Search a property in "root" and directly assign it to "vl".
    /// The property is searched by the "property_name", returning true if it matches
    /// the names you used in the CHNVP macros of ArchiveOut of your classes, and if
    /// the type of "val" is the same of the property (exactly the same type: no superclass/subclass).
    /// Note: the "root" object should be a class with sub properties and/or sub objects, and
    /// the root is not part of the search.
    /// The search string "property_name" can explore subobjects with "/", as in "employee/force/z"
    /// The search string "property_name" can contain wildcards * and ?, as in "e??loyee/*/z"
    /// In case of multiple matching properties it returns at the first found property.
    template <class T, class P>
    bool FetchValue(P& val, const T& root, const std::string& property_name) {
        // TODO: update this method to use getVoidPointer so to avoid PointerUpCast
        this->PrepareSearch(property_name);

        this->operator<<(CHNVP(root, ""));  // SCAN!

        if (found) {
            if (this->results[0]->GetTypeid() == std::type_index(typeid(P))) {
                val = *(static_cast<P*>(this->results[0]->GetRawPtr()));
                return true;
            } else if (P* valptr = this->results[0]->PointerUpCast<P>()) {
                val = *valptr;
                return true;
            } else
                return false;
        } else
            return false;
    }

    /// Search one or more values in "root" and return reference to a vector with results.
    /// The properties are searched by the "value_name", returning true if it matches
    /// the names you used in the CHNVP macros of ArchiveOut of your classes
    /// Note: the "root" object should be a class with sub properties and/or sub objects, and
    /// the root is not part of the search.
    /// Wildcards * and ? can be used. For example use property_name = "*" to ge the list
    /// of all subproperties of root object.
    template <class T>
    std::vector<ChValue*>& FetchValues(T& root, const std::string& value_name) {
        this->PrepareSearch(value_name);
        this->find_all = true;
        this->operator<<(CHNVP(root, ""));  // SCAN!
        return this->results;
    }

    /// Same as generic FetchValues(), but specialized for the case of finding
    /// sub ChValue properties of an already found ChValue
    std::vector<ChValue*>& FetchValues(ChValue& root, const std::string& value_name);

    /// Access the results of the last search, ex. after FetchValues()
    std::vector<ChValue*>& GetFetchResults() { return this->results; }

    /// Tell if "root" has sub values (i.e. "root" is a class with ArchiveOut
    /// implemented or a std container such as std::vector).
    template <class T>
    bool IsObject(const T& root) {
        this->PrepareSearch("_");
        this->operator<<(CHNVP(root, ""));  // SCAN!
        if (this->found_obj)
            return true;
        else
            return false;
    }

    /// Tell if "root" has sub values (i.e. "root" is a class with ArchiveOut
    /// implemented or a std container such as std::vector).
    bool IsObject(ChValue& root) {
        this->PrepareSearch("_");
        root.CallOut(*this);
        if (this->found_obj)
            return true;
        else
            return false;
    }

    void SetUseWildcards(const bool mw) { use_wildcards = mw; }
    bool GetUseWildcards() { return this->use_wildcards; }

    void SetUseUserNames(const bool mw) { use_user_names = mw; }
    bool GetUseUserNames() { return this->use_user_names; }

    // override base class :

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
    // There should be no need to call this by hand, as it is invoked
    // automatically at the beginning of each search.
    void ClearSearch();

    void PrepareSearch(const std::string& msearched_property);

    int wildcard_compare(const char* wildcard, const char* string);

    bool MatchName(const std::string& token, const std::string& string);

    std::string searched_property;
    std::vector<std::string> search_tokens;
    std::vector<ChValue*> results;

    bool found;
    bool find_all;
    bool found_obj;
    int tablevel;
    bool use_wildcards;
    bool use_user_names;
    std::vector<bool> in_array;
};

}  // end namespace chrono

#endif
