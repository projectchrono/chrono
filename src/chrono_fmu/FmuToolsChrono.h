#pragma once

// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2018 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Dario Mangoni
// =============================================================================

#ifndef CHARCHIVEFMU_H
#define CHARCHIVEFMU_H

#include "chrono/serialization/ChArchive.h"
#include "chrono/core/ChLog.h"
#include "chrono/core/ChMathematics.h"
#include <iostream>
#include <fstream>

// fmu_tools includes
//#include "rapidxml_ext.hpp"
#include "FmuToolsExport.h"
#include <stack>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>


// TODO expand serialization to have description
namespace chrono {

///
/// This is a class for serializing to FmuComponentBase
///


#define ADD_BVAL_AS_FMU_GETSET(returnType, codeGet, codeSet) \
    _fmucomp->AddFmuVariable(std::make_pair(std::function<fmi2##returnType(void)>([&bVal]() -> fmi2##returnType  \
        codeGet \
    ), \
    std::function<void(fmi2##returnType)>([&bVal](fmi2##returnType val) \
        codeSet \
    )), \
    current_parent_fullname + bVal.name(), \
    FmuVariable::Type::##returnType, \
    "", \
    "", \
    CausalityType_conv.at(bVal.GetCausality()), \
    VariabilityType_conv.at(bVal.GetVariability()) \
    );

#define ADD_BVAL_AS_FMU_POINTER(returnType) \
    _fmucomp->AddFmuVariable(&(bVal.value()), \
    current_parent_fullname + bVal.name(), \
    FmuVariable::Type::##returnType, \
    "", \
    "", \
    CausalityType_conv.at(bVal.GetCausality()), \
    VariabilityType_conv.at(bVal.GetVariability()) \
    );

const std::unordered_map<chrono::ChVariabilityType, FmuVariable::VariabilityType> VariabilityType_conv = {
    {chrono::ChVariabilityType::constant, FmuVariable::VariabilityType::constant},
    {chrono::ChVariabilityType::fixed, FmuVariable::VariabilityType::fixed},
    {chrono::ChVariabilityType::tunable, FmuVariable::VariabilityType::tunable},
    {chrono::ChVariabilityType::discrete, FmuVariable::VariabilityType::discrete},
    {chrono::ChVariabilityType::continuous, FmuVariable::VariabilityType::continuous}
};

const std::unordered_map<chrono::ChCausalityType, FmuVariable::CausalityType> CausalityType_conv = {
    {chrono::ChCausalityType::parameter, FmuVariable::CausalityType::parameter},
    {chrono::ChCausalityType::calculatedParameter, FmuVariable::CausalityType::calculatedParameter},
    {chrono::ChCausalityType::input, FmuVariable::CausalityType::input},
    {chrono::ChCausalityType::output, FmuVariable::CausalityType::output},
    {chrono::ChCausalityType::local, FmuVariable::CausalityType::local},
    {chrono::ChCausalityType::independent, FmuVariable::CausalityType::independent}
};



class ChArchiveFmu : public ChArchiveOut {
  public:
    ChArchiveFmu(FmuComponentBase& fmucomp) {
        _fmucomp = &fmucomp;

        tablevel = 0;
        nitems.push(0);
        is_array.push_back(false);
    };

    virtual ~ChArchiveFmu() {
        nitems.pop();
        is_array.pop_back();
    };


    virtual void out(ChNameValue<bool> bVal) {
        ADD_BVAL_AS_FMU_GETSET(Boolean, { return static_cast<int>(bVal.value()); }, {bVal.value() = val;})

        ++nitems.top();
    }
    virtual void out(ChNameValue<int> bVal) {
        ADD_BVAL_AS_FMU_POINTER(Integer)
        
        ++nitems.top();
    }
    virtual void out(ChNameValue<double> bVal) {
        ADD_BVAL_AS_FMU_POINTER(Real)

        ++nitems.top();
    }
    virtual void out(ChNameValue<float> bVal) {
        ADD_BVAL_AS_FMU_GETSET(Real, { return static_cast<double>(bVal.value()); }, {bVal.value() = static_cast<float>(val);})

        ++nitems.top();
    }
    virtual void out(ChNameValue<char> bVal) {
        ADD_BVAL_AS_FMU_GETSET(Integer, { return static_cast<fmi2Integer>(bVal.value()); }, {bVal.value() = val;})

        ++nitems.top();
    }
    virtual void out(ChNameValue<unsigned int> bVal) {
        ADD_BVAL_AS_FMU_GETSET(Integer, { return static_cast<fmi2Integer>(bVal.value()); }, {bVal.value() = val;})

        ++nitems.top();
    }
    virtual void out(ChNameValue<const char*> bVal) {
        // TODO

        ++nitems.top();
    }
    virtual void out(ChNameValue<std::string> bVal) {
        // TODO

        ++nitems.top();
    }
    virtual void out(ChNameValue<unsigned long> bVal) {
        // TODO

        ++nitems.top();
    }
    virtual void out(ChNameValue<unsigned long long> bVal) {
        // TODO

        ++nitems.top();
    }
    virtual void out(ChNameValue<ChEnumMapperBase> bVal) {

        ++nitems.top();
    }

    virtual void out_array_pre(ChValue& bVal, size_t msize) {
        pushLevelName(bVal.name());

        ++tablevel;
        nitems.push(0);
        is_array.push_back(true);

    }
    virtual void out_array_between(ChValue& bVal, size_t msize) {}

    virtual void out_array_end(ChValue& bVal, size_t msize) {
        --tablevel;
        nitems.pop();
        is_array.pop_back();
        ++nitems.top();
        popLevelName();

    }

    // for custom c++ objects:
    virtual void out(ChValue& bVal, bool tracked, size_t obj_ID) {
        pushLevelName(bVal.name());

        ++tablevel;
        nitems.push(0);
        is_array.push_back(false);

        bVal.CallArchiveOut(*this);

        --tablevel;
        nitems.pop();
        is_array.pop_back();

        ++nitems.top();
        popLevelName();
    }

    virtual void out_ref(ChValue& bVal, bool already_inserted, size_t obj_ID, size_t ext_ID) {
        
        pushLevelName(bVal.name());

        ++tablevel;
        nitems.push(0);
        is_array.push_back(false);

        if (!already_inserted) {
            // New Object, we have to full serialize it
            bVal.CallArchiveOutConstructor(*this);
            bVal.CallArchiveOut(*this);
        }

        --tablevel;
        nitems.pop();

        is_array.pop_back();

        ++nitems.top();
        
        popLevelName();
    }

  protected:

    void pushLevelName(const std::string& newLevelName){
        parent_names.push_back(is_array.back() ? ("[" + newLevelName + "]") : newLevelName);
        updateCurrentParentFullname();
    }

    void popLevelName(){
        parent_names.pop_back();
        updateCurrentParentFullname();
    }

    void updateCurrentParentFullname(){
        current_parent_fullname = "";
        for (size_t i = 0; i < parent_names.size(); ++i) {
            current_parent_fullname += parent_names[i];

            // add a dot separator to separate from the following, except if:
            // - there is no following name
            // - the current 
            if (i<is_array.size()-1 && !is_array.at(i+1)) {
                current_parent_fullname += ".";
            }
        }
    }


    int tablevel;
    FmuComponentBase* _fmucomp;
    std::stack<int> nitems;
    std::deque<bool> is_array;
    std::deque<std::string> parent_names;
    std::string current_parent_fullname;
};

}


#endif
