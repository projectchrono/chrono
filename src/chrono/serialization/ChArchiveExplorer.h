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

#ifndef CHARCHIVEEXPLORER_H
#define CHARCHIVEEXPLORER_H

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
class ChArchiveExplorer : public ChArchiveOut {
  public:
    ChArchiveExplorer() {
        this->found = false;
        this->searched_property = "";
        this->tablevel = 0;
        this->use_wildcards = true;
        this->use_user_names = true;
        this->find_all = false;
        this->found_obj = false;
        this->SetUseVersions(false);
    };

    virtual ~ChArchiveExplorer() { ClearSearch(); };

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
    /// Same as generic FetchValues() , but specialized for the case of finding
    /// sub ChValue properties of an already found ChValue
    std::vector<ChValue*>& FetchValues(ChValue& root, const std::string& value_name) {
        this->PrepareSearch(value_name);
        this->find_all = true;
        root.CallOut(*this);
        return this->results;
    }

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

    virtual void out(ChNameValue<bool> bVal) {
        if (this->found && !this->find_all)
            return;
        if (this->tablevel != this->search_tokens.size())
            return;
        if (this->MatchName(search_tokens[this->tablevel], bVal.name())) {
            this->results.push_back(new ChValueSpecific<bool>(bVal.value(), bVal.name(), bVal.flags(), bVal.GetCausality(), bVal.GetVariability()));
            this->found = true;
        }
    }
    virtual void out(ChNameValue<int> bVal) {
        if (this->found && !this->find_all)
            return;
        if (this->tablevel != this->search_tokens.size() - 1)
            return;
        if (this->MatchName(search_tokens[this->tablevel], bVal.name())) {
            this->results.push_back(new ChValueSpecific<int>(bVal.value(), bVal.name(), bVal.flags(), bVal.GetCausality(), bVal.GetVariability()));
            this->found = true;
        }
    }
    virtual void out(ChNameValue<double> bVal) {
        if (this->found && !this->find_all)
            return;
        if (this->tablevel != this->search_tokens.size() - 1)
            return;
        if (this->MatchName(search_tokens[this->tablevel], bVal.name())) {
            this->results.push_back(new ChValueSpecific<double>(bVal.value(), bVal.name(), bVal.flags(), bVal.GetCausality(), bVal.GetVariability()));
            this->found = true;
        }
    }
    virtual void out(ChNameValue<float> bVal) {
        if (this->found && !this->find_all)
            return;
        if (this->tablevel != this->search_tokens.size() - 1)
            return;
        if (this->MatchName(search_tokens[this->tablevel], bVal.name())) {
            this->results.push_back(new ChValueSpecific<float>(bVal.value(), bVal.name(), bVal.flags(), bVal.GetCausality(), bVal.GetVariability()));
            this->found = true;
        }
    }
    virtual void out(ChNameValue<char> bVal) {
        if (this->found && !this->find_all)
            return;
        if (this->tablevel != this->search_tokens.size() - 1)
            return;
        if (this->MatchName(search_tokens[this->tablevel], bVal.name())) {
            this->results.push_back(new ChValueSpecific<char>(bVal.value(), bVal.name(), bVal.flags(), bVal.GetCausality(), bVal.GetVariability()));
            this->found = true;
        }
    }
    virtual void out(ChNameValue<unsigned int> bVal) {
        if (this->found && !this->find_all)
            return;
        if (this->tablevel != this->search_tokens.size() - 1)
            return;
        if (this->MatchName(search_tokens[this->tablevel], bVal.name())) {
            this->results.push_back(new ChValueSpecific<unsigned int>(bVal.value(), bVal.name(), bVal.flags(), bVal.GetCausality(), bVal.GetVariability()));
            this->found = true;
        }
    }
    virtual void out(ChNameValue<std::string> bVal) {
        if (this->found)
            return;
        if (this->tablevel != this->search_tokens.size() - 1)
            return;
        if (this->MatchName(search_tokens[this->tablevel], bVal.name())) {
            this->results.push_back(new ChValueSpecific<std::string>(bVal.value(), bVal.name(), bVal.flags(), bVal.GetCausality(), bVal.GetVariability()));
            this->found = true;
        }
    }
    virtual void out(ChNameValue<unsigned long> bVal) {
        if (this->found && !this->find_all)
            return;
        if (this->tablevel != this->search_tokens.size() - 1)
            return;
        if (this->MatchName(search_tokens[this->tablevel], bVal.name())) {
            this->results.push_back(new ChValueSpecific<unsigned long>(bVal.value(), bVal.name(), bVal.flags(), bVal.GetCausality(), bVal.GetVariability()));
            this->found = true;
        }
    }
    virtual void out(ChNameValue<unsigned long long> bVal) {
        if (this->found)
            return;
        if (this->tablevel != this->search_tokens.size() - 1)
            return;
        if (this->MatchName(search_tokens[this->tablevel], bVal.name())) {
            this->results.push_back(new ChValueSpecific<unsigned long long>(bVal.value(), bVal.name(), bVal.flags(), bVal.GetCausality(), bVal.GetVariability()));
            this->found = true;
        }
    }
    virtual void out(ChNameValue<ChEnumMapperBase> bVal) {
        std::string mstr = bVal.value().GetValueAsString();
        if (this->found && !this->find_all)
            return;
        if (this->tablevel != this->search_tokens.size() - 1)
            return;
        if (this->MatchName(search_tokens[this->tablevel], bVal.name())) {
            this->results.push_back(new ChValueSpecific<std::string>(mstr, bVal.name(), bVal.flags(), bVal.GetCausality(), bVal.GetVariability()));
            this->found = true;
        }
    }

    virtual void out_array_pre(ChValue& bVal, size_t msize) {
        if (this->found && !this->find_all)
            return;
        if (this->tablevel >= this->search_tokens.size())
            return;

        if (this->MatchName(search_tokens[this->tablevel], bVal.name()) || tablevel == 0) {
            this->found_obj = true;

            if (this->tablevel + 1 == this->search_tokens.size()) {
                this->results.push_back(bVal.new_clone());
                this->found = true;
            }
            ++tablevel;
            in_array.push_back(true);
        }
    }
    virtual void out_array_between(ChValue& bVal, size_t msize) {}
    virtual void out_array_end(ChValue& bVal, size_t msize) {
        if (in_array[tablevel] == true) {
            --tablevel;
            in_array.pop_back();
        }
    }

    // for custom c++ objects:
    virtual void out(ChValue& bVal, bool tracked, size_t obj_ID) {
        if (this->found && !this->find_all)
            return;
        if (this->tablevel >= this->search_tokens.size())
            return;

        bool matched = this->MatchName(search_tokens[this->tablevel], bVal.name()) || (tablevel == 0);
        if (!matched && tablevel && this->in_array[tablevel] && this->use_user_names &&
            !search_tokens[this->tablevel].empty() &&
            (search_tokens[this->tablevel].front() == *"'" && search_tokens[this->tablevel].back() == *"'") &&
            bVal.HasArchiveContainerName()) {
            std::string username = search_tokens[this->tablevel].substr(1, search_tokens[this->tablevel].length() - 2);
            matched = this->MatchName(username, bVal.CallArchiveContainerName().c_str());
        }

        if (matched) {
            this->found_obj = true;
            if (this->tablevel + 1 == this->search_tokens.size()) {
                this->results.push_back(bVal.new_clone());
                this->found = true;
            } else {
                ++tablevel;
                in_array.push_back(false);
                bVal.CallArchiveOut(*this);
                --tablevel;
                in_array.pop_back();
            }
        }
    }

    virtual void out_ref(ChValue& bVal, bool already_inserted, size_t obj_ID, size_t ext_ID) {
        if (this->found && !this->find_all)
            return;
        if (this->tablevel >= this->search_tokens.size())
            return;

        bool matched = this->MatchName(search_tokens[this->tablevel], bVal.name()) || (tablevel == 0);
        if (!matched && tablevel && this->in_array[tablevel] && this->use_user_names &&
            !search_tokens[this->tablevel].empty() &&
            (search_tokens[this->tablevel].front() == *"'" && search_tokens[this->tablevel].back() == *"'") &&
            bVal.HasArchiveContainerName()) {
            std::string username = search_tokens[this->tablevel].substr(1, search_tokens[this->tablevel].length() - 2);
            matched = this->MatchName(username, bVal.CallArchiveContainerName().c_str());
        }

        if (matched) {
            this->found_obj = true;
            if (this->tablevel + 1 == this->search_tokens.size()) {
                this->results.push_back(bVal.new_clone());
                this->found = true;
            } else {
                ++tablevel;
                in_array.push_back(false);
                bVal.CallArchiveOut(*this);
                --tablevel;
                in_array.pop_back();
            }
        }
    }

  protected:
    // There should be no need to call this by hand, as it is invoked
    // automatically at the beginning of each search.
    void ClearSearch() {
        this->found = false;
        this->find_all = false;
        this->found_obj = false;
        this->tablevel = 0;
        this->results.clear();
        this->search_tokens.clear();
        this->search_tokens.push_back("");  // for root lavel
        this->in_array.clear();
        this->in_array.push_back(false);  // for root level
        for (int i = 0; i < this->results.size(); ++i) {
            delete (this->results[i]);
        }
    }

    void PrepareSearch(const std::string& msearched_property) {
        ClearSearch();
        this->searched_property = msearched_property;
        std::stringstream test(msearched_property);
        std::string segment;

        while (std::getline(test, segment, '/')) {
            search_tokens.push_back(segment);
        }
    }

    int wildcard_compare(const char* wildcard, const char* string) {
        const char *cp = 0, *mp = 0;

        while ((*string) && (*wildcard != '*')) {
            if ((*wildcard != *string) && (*wildcard != '?')) {
                return 0;
            }
            wildcard++;
            string++;
        }

        while (*string) {
            if (*wildcard == '*') {
                if (!*++wildcard) {
                    return 1;
                }
                mp = wildcard;
                cp = string + 1;
            } else {
                if ((*wildcard == *string) || (*wildcard == '?')) {
                    wildcard++;
                    string++;
                } else {
                    wildcard = mp;
                    string = cp++;
                }
            }
        }

        while (*wildcard == '*') {
            wildcard++;
        }
        return !*wildcard;
    }

    bool MatchName(const std::string& token, const std::string& string) {
        if (use_wildcards) {
            if (wildcard_compare(token.c_str(), string.c_str()))
                return true;
        } else {
            if (token == string)
                return true;
        }

        return false;
    }

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
