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
// Authors: Alessandro Tasora
// =============================================================================

#ifndef CHARCHIVEXML_H
#define CHARCHIVEXML_H

#include "chrono/serialization/ChArchive.h"
#include "chrono/core/ChLog.h"
#include "chrono/core/ChMathematics.h"
#include <iostream>
#include <fstream>

#include "chrono_thirdparty/rapidxml/rapidxml.hpp"

#include <stack>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace chrono {

///
/// This is a class for serializing to XML file format
///

class ChArchiveOutXML : public ChArchiveOut {
  public:
    ChArchiveOutXML(ChStreamOutAsciiFile& mostream) {
        ostream = &mostream;

        tablevel = 0;
        nitems.push(0);
        is_array.push(false);
    };

    virtual ~ChArchiveOutXML() {
        nitems.pop();
        is_array.pop();

        (*ostream) << "\n";
    };

    void indent() {
        for (int i = 0; i < tablevel; ++i)
            (*ostream) << "\t";
    }

    virtual void out(ChNameValue<bool> bVal) {
        indent();
        // if (is_array.top()==false)
        (*ostream) << "<" << bVal.name() << ">";
        if (bVal.value())
            (*ostream) << "true";
        else
            (*ostream) << "false";
        // if (is_array.top() == false)
        (*ostream) << "</" << bVal.name() << ">\n";
        ++nitems.top();
    }
    virtual void out(ChNameValue<int> bVal) {
        indent();
        // if (is_array.top()==false)
        (*ostream) << "<" << bVal.name() << ">";
        (*ostream) << bVal.value();
        // if (is_array.top() == false)
        (*ostream) << "</" << bVal.name() << ">\n";
        ++nitems.top();
    }
    virtual void out(ChNameValue<double> bVal) {
        indent();
        // if (is_array.top() == false)
        (*ostream) << "<" << bVal.name() << ">";
        (*ostream) << bVal.value();
        // if (is_array.top() == false)
        (*ostream) << "</" << bVal.name() << ">\n";
        ++nitems.top();
    }
    virtual void out(ChNameValue<float> bVal) {
        indent();
        // if (is_array.top() == false)
        (*ostream) << "<" << bVal.name() << ">";
        (*ostream) << bVal.value();
        // if (is_array.top() == false)
        (*ostream) << "</" << bVal.name() << ">\n";
        ++nitems.top();
    }
    virtual void out(ChNameValue<char> bVal) {
        indent();
        // if (is_array.top() == false)
        (*ostream) << "<" << bVal.name() << ">";
        (*ostream) << (int)bVal.value();
        // if (is_array.top() == false)
        (*ostream) << "</" << bVal.name() << ">\n";
        ++nitems.top();
    }
    virtual void out(ChNameValue<unsigned int> bVal) {
        indent();
        // if (is_array.top() == false)
        (*ostream) << "<" << bVal.name() << ">";
        (*ostream) << bVal.value();
        // if (is_array.top() == false)
        (*ostream) << "</" << bVal.name() << ">\n";
        ++nitems.top();
    }
    virtual void out(ChNameValue<const char*> bVal) {
        indent();
        // if (is_array.top() == false)
        (*ostream) << "<" << bVal.name() << ">";
        (*ostream) << "\"" << bVal.value() << "\"";
        // if (is_array.top() == false)
        (*ostream) << "</" << bVal.name() << ">\n";
        ++nitems.top();
    }
    virtual void out(ChNameValue<std::string> bVal) {
        indent();
        // if (is_array.top() == false)
        (*ostream) << "<" << bVal.name() << ">";
        (*ostream) << bVal.value();
        // if (is_array.top() == false)
        (*ostream) << "</" << bVal.name() << ">\n";
        ++nitems.top();
    }
    virtual void out(ChNameValue<unsigned long> bVal) {
        indent();
        // if (is_array.top() == false)
        (*ostream) << "<" << bVal.name() << ">";
        (*ostream) << bVal.value();
        // if (is_array.top() == false)
        (*ostream) << "</" << bVal.name() << ">\n";
        ++nitems.top();
    }
    virtual void out(ChNameValue<unsigned long long> bVal) {
        indent();
        // if (is_array.top() == false)
        (*ostream) << "<" << bVal.name() << ">";
        (*ostream) << bVal.value();
        // if (is_array.top() == false)
        (*ostream) << "</" << bVal.name() << ">\n";
        ++nitems.top();
    }
    virtual void out(ChNameValue<ChEnumMapperBase> bVal) {
        indent();
        // if (is_array.top() == false)
        (*ostream) << "<" << bVal.name() << ">";
        std::string mstr = bVal.value().GetValueAsString();
        (*ostream) << mstr;
        // if (is_array.top() == false)
        (*ostream) << "</" << bVal.name() << ">\n";
        ++nitems.top();
    }

    virtual void out_array_pre(ChValue& bVal, size_t msize) {
        // if (is_array.top()==false)
        //{
        indent();
        (*ostream) << "<" << bVal.name() << ">";
        //}
        (*ostream) << "\n";

        ++tablevel;
        nitems.push(0);
        is_array.push(true);
    }
    virtual void out_array_between(ChValue& bVal, size_t msize) {}
    virtual void out_array_end(ChValue& bVal, size_t msize) {
        --tablevel;
        nitems.pop();
        is_array.pop();

        // if (is_array.top() == false)
        //{
        indent();
        (*ostream) << "</" << bVal.name() << ">\n";
        //}
        ++nitems.top();
    }

    // for custom c++ objects:
    virtual void out(ChValue& bVal, bool tracked, size_t obj_ID) {
        // if (is_array.top()==false)
        //{
        indent();
        (*ostream) << "<" << bVal.name();

        if (tracked) {
            (*ostream) << " _object_ID=\"" << obj_ID << "\"";
        }

        (*ostream) << ">\n";
        //}
        // indent();

        ++tablevel;
        nitems.push(0);
        is_array.push(false);

        bVal.CallArchiveOut(*this);

        --tablevel;
        nitems.pop();
        is_array.pop();

        ++nitems.top();

        // if (is_array.top() == false)
        //{
        indent();
        (*ostream) << "</" << bVal.name() << ">\n";
        //}
    }

    virtual void out_ref(ChValue& bVal, bool already_inserted, size_t obj_ID, size_t ext_ID) {
        const char* classname = bVal.GetClassRegisteredName().c_str();

        // if (is_array.top() == false) {
        indent();
        (*ostream) << "<" << bVal.name();

        if (strlen(classname) > 0) {
            (*ostream) << " _type=\"" << classname << "\"";
        }

        if (!already_inserted) {
            (*ostream) << " _object_ID=\"" << obj_ID << "\"";
        }
        if (already_inserted) {
            if (obj_ID || bVal.IsNull()) {
                (*ostream) << " _reference_ID=\"" << obj_ID << "\"";
            }
            if (ext_ID) {
                (*ostream) << " _external_ID=\"" << ext_ID << "\"";
            }
        }

        (*ostream) << ">\n";
        //}

        ++tablevel;
        nitems.push(0);
        is_array.push(false);

        if (!already_inserted) {
            // New Object, we have to full serialize it
            bVal.CallArchiveOutConstructor(*this);
            bVal.CallArchiveOut(*this);
        }

        --tablevel;
        nitems.pop();
        is_array.pop();

        ++nitems.top();

        // if (is_array.top() == false){
        indent();
        (*ostream) << "</" << bVal.name() << ">\n";
        //}
    }

  protected:
    int tablevel;
    ChStreamOutAsciiFile* ostream;
    std::stack<int> nitems;
    std::stack<bool> is_array;
};

///
/// This is a class for deserializing from XML archives
///

class ChArchiveInXML : public ChArchiveIn {
  public:
    ChArchiveInXML(ChStreamInAsciiFile& mistream) {
        istream = &mistream;

        buffer.assign((std::istreambuf_iterator<char>(istream->GetFstream())), std::istreambuf_iterator<char>());
        buffer.push_back('\0');

        try {
            document.parse<0>(&buffer[0]);
        } catch (const rapidxml::parse_error& merror) {
            std::string line(merror.where<char>());
            line.erase(std::find_if(line.begin(), line.end(), [](int c) { return (c == *"\n"); }), line.end());
            throw ChExceptionArchive(std::string("XML parsing error: ") + merror.what() + " at: \n" + line + "\n");
        }

        if (!document.first_node())
            throw(ChExceptionArchive("The file is not a valid XML document"));

        level = &document;  //.first_node();
        levels.push(level);
        is_array.push(false);

        can_tolerate_missing_tokens = true;
        try_tolerate_missing_tokens = false;
    }

    virtual ~ChArchiveInXML(){};

    rapidxml::xml_node<>* GetValueFromNameOrArray(const std::string& mname) {
        rapidxml::xml_node<>* mnode = level->first_node(mname.c_str());
        if (!mnode)
            token_notfound(mname);
        return mnode;
    }

    virtual bool in(ChNameValue<bool> bVal) override {
        rapidxml::xml_node<>* mval = GetValueFromNameOrArray(bVal.name());
        if (!mval)
            return false;
        if (!strncmp(mval->value(), "true", mval->value_size())) {
            bVal.value() = true;
            return true;
        }
        if (!strncmp(mval->value(), "false", mval->value_size())) {
            bVal.value() = false;
            return true;
        }
        throw(ChExceptionArchive("Invalid true/false flag after '" + std::string(bVal.name()) + "'"));
    }
    virtual bool in(ChNameValue<int> bVal) override {
        rapidxml::xml_node<>* mval = GetValueFromNameOrArray(bVal.name());
        if (!mval)
            return false;
        try {
            bVal.value() = std::stoi(mval->value());
        } catch (...) {
            throw(ChExceptionArchive("Invalid integer number after '" + std::string(bVal.name()) + "'"));
        }
        return true;
    }
    virtual bool in(ChNameValue<double> bVal) override {
        rapidxml::xml_node<>* mval = GetValueFromNameOrArray(bVal.name());
        if (!mval)
            return false;
        try {
            bVal.value() = std::stod(mval->value());
        } catch (...) {
            throw(ChExceptionArchive("Invalid number after '" + std::string(bVal.name()) + "'"));
        }
        return true;
    }
    virtual bool in(ChNameValue<float> bVal) override {
        rapidxml::xml_node<>* mval = GetValueFromNameOrArray(bVal.name());
        if (!mval)
            return false;
        try {
            bVal.value() = std::stof(mval->value());
        } catch (...) {
            throw(ChExceptionArchive("Invalid number after '" + std::string(bVal.name()) + "'"));
        }
        return true;
    }
    virtual bool in(ChNameValue<char> bVal) override {
        rapidxml::xml_node<>* mval = GetValueFromNameOrArray(bVal.name());
        if (!mval)
            return false;
        try {
            bVal.value() = (char)std::stoi(mval->value());
        } catch (...) {
            throw(ChExceptionArchive("Invalid char code after '" + std::string(bVal.name()) + "'"));
        }
        return true;
    }
    virtual bool in(ChNameValue<unsigned int> bVal) override {
        rapidxml::xml_node<>* mval = GetValueFromNameOrArray(bVal.name());
        if (!mval)
            return false;
        try {
            bVal.value() = std::stoul(mval->value());
        } catch (...) {
            throw(ChExceptionArchive("Invalid unsigned integer number after '" + std::string(bVal.name()) + "'"));
        }
        return true;
    }
    virtual bool in(ChNameValue<std::string> bVal) override {
        rapidxml::xml_node<>* mval = GetValueFromNameOrArray(bVal.name());
        if (!mval)
            return false;
        bVal.value() = mval->value();
        return true;
    }
    virtual bool in(ChNameValue<unsigned long> bVal) override {
        rapidxml::xml_node<>* mval = GetValueFromNameOrArray(bVal.name());
        if (!mval)
            return false;
        try {
            bVal.value() = std::stoul(mval->value());
        } catch (...) {
            throw(ChExceptionArchive("Invalid unsigned long integer number after '" + std::string(bVal.name()) + "'"));
        }
        return true;
    }
    virtual bool in(ChNameValue<unsigned long long> bVal) override {
        rapidxml::xml_node<>* mval = GetValueFromNameOrArray(bVal.name());     
        if (!mval)
            return false;
        try {
            bVal.value() = std::stoull(mval->value());
        } catch (...) {
            throw(ChExceptionArchive("Invalid unsigned long long integer number after '" + std::string(bVal.name()) +
                                     "'"));
        }
        return true;
    }
    virtual bool in(ChNameValue<ChEnumMapperBase> bVal) override {
        rapidxml::xml_node<>* mval = GetValueFromNameOrArray(bVal.name());
        if (!mval)
            return false;
        std::string mstr = mval->value();
        if (!bVal.value().SetValueAsString(mstr)) {
            throw(ChExceptionArchive("Not recognized enum type '" + mstr + "'"));
        }
        return true;
    }

    // for wrapping arrays and lists
    virtual bool in_array_pre(const std::string& name, size_t& msize) override {
        rapidxml::xml_node<>* mval = GetValueFromNameOrArray(name);
        if (!mval)
            return false;
        msize = 0;
        for (rapidxml::xml_node<>* countnode = mval->first_node(); countnode; countnode = countnode->next_sibling()) {
            ++msize;
        }

        this->levels.push(mval);
        this->level = this->levels.top();
        this->is_array.push(true);
        this->array_index.push(0);
        return true;
    }
    virtual void in_array_between(const std::string& name) override { ++this->array_index.top(); }

    virtual void in_array_end(const std::string& name) override {
        this->levels.pop();
        this->level = this->levels.top();
        this->is_array.pop();
        this->array_index.pop();
    }

    //  for custom c++ objects:
    virtual bool in(ChNameValue<ChFunctorArchiveIn> bVal) override {
        rapidxml::xml_node<>* mval = GetValueFromNameOrArray(bVal.name());
        if (!mval)
            return false;

        
        this->levels.push(mval);
        this->level = this->levels.top();
        this->is_array.push(false);

        size_t obj_ID = 0;
        if (rapidxml::xml_attribute<>* midval = level->first_attribute("_object_ID")) {
            try {
                obj_ID = std::stoull(midval->value());
            } catch (...) {
                throw(ChExceptionArchive("Invalid _object_ID in '" + std::string(bVal.name()) + "'"));
            }
        }

        if (bVal.flags() & NVP_TRACK_OBJECT) {
            PutNewPointer(bVal.value().GetRawPtr(), obj_ID);
        }


        bVal.value().CallArchiveIn(*this);

        this->levels.pop();
        this->level = this->levels.top();
        this->is_array.pop();
        return true;
    }

    // for objects to construct, return non-null ptr if new object, return null ptr if just reused obj
    virtual bool in_ref(ChNameValue<ChFunctorArchiveIn> bVal, void** ptr, std::string& true_classname) override {
        void* new_ptr = nullptr;

        rapidxml::xml_node<>* mval = GetValueFromNameOrArray(bVal.name());
        if (!mval)
            return false;

        if (mval) {
            this->levels.push(mval);
            this->level = this->levels.top();
            this->is_array.push(false);

            if (bVal.value().IsPolymorphic()) {
                if (rapidxml::xml_attribute<>* mtypeval = level->first_attribute("_type")) {
                    true_classname = mtypeval->value();
                }
            }
            bool is_reference = false;
            size_t ref_ID = 0;
            if (rapidxml::xml_attribute<>* midval = level->first_attribute("_reference_ID")) {
                try {
                    ref_ID = std::stoull(midval->value());
                } catch (...) {
                    throw(ChExceptionArchive("Invalid _reference_ID in '" + std::string(bVal.name()) + "'"));
                }
                is_reference = true;
            }
            size_t ext_ID = 0;
            if (rapidxml::xml_attribute<>* midval = level->first_attribute("_external_ID")) {
                try {
                    ext_ID = std::stoull(midval->value());
                } catch (...) {
                    throw(ChExceptionArchive("Invalid _external_ID in '" + std::string(bVal.name()) + "'"));
                }
                is_reference = true;
            }

            if (!is_reference) {
                // See ChArchiveJSON for detailed explanation
                bVal.value().CallConstructor(*this, true_classname);

                void* new_ptr_void = bVal.value().GetRawPtr();

                size_t obj_ID = 0;
                if (rapidxml::xml_attribute<>* midval = level->first_attribute("_object_ID")) {
                    try {
                        obj_ID = std::stoull(midval->value());
                    } catch (...) {
                        throw(ChExceptionArchive("Invalid _object_ID in '" + std::string(bVal.name()) + "'"));
                    }
                }

                if (new_ptr_void) {
                    PutNewPointer(new_ptr_void, obj_ID);
                    // 3) Deserialize
                    bVal.value().CallArchiveIn(*this, true_classname);
                } else {
                    throw(ChExceptionArchive("Archive cannot create object " + std::string(bVal.name()) + "\n"));
                }
                new_ptr = bVal.value().GetRawPtr();
            } else {
                if (ref_ID) {
                    if (this->internal_id_ptr.find(ref_ID) == this->internal_id_ptr.end()) {
                        throw(ChExceptionArchive("In object '" + std::string(bVal.name()) + "' the _reference_ID " +
                                                 std::to_string((int)ref_ID) + " is not a valid number."));
                    }

                    bVal.value().SetRawPtr(
                        ChCastingMap::Convert(true_classname, bVal.value().GetObjectPtrTypeindex(), internal_id_ptr[ref_ID]));
                } else if (ext_ID) {
                    if (this->external_id_ptr.find(ext_ID) == this->external_id_ptr.end()) {
                        throw(ChExceptionArchive("In object '" + std::string(bVal.name()) + "' the _external_ID " +
                                                 std::to_string((int)ext_ID) + " is not valid."));
                    }
                    bVal.value().SetRawPtr(
                        ChCastingMap::Convert(true_classname, bVal.value().GetObjectPtrTypeindex(), external_id_ptr[ext_ID]));
                } else
                    bVal.value().SetRawPtr(nullptr);
            }
            this->levels.pop();
            this->level = this->levels.top();
            this->is_array.pop();
        }

        *ptr = new_ptr;
        return true;
    }

    virtual bool TryTolerateMissingTokens(bool try_tolerate) override {
        try_tolerate_missing_tokens = try_tolerate;
        return try_tolerate_missing_tokens;
    }

  protected:
    void token_notfound(const std::string& mname) {
        if (!try_tolerate_missing_tokens)
            throw(ChExceptionArchive("Cannot find '" + mname + "'"));
    }

    ChStreamInAsciiFile* istream;
    rapidxml::xml_document<> document;
    rapidxml::xml_node<>* level;
    std::stack<rapidxml::xml_node<>*> levels;
    std::stack<bool> is_array;
    std::stack<int> array_index;

    std::vector<char> buffer;  // needed: rapidxml is a in-place parser
};

}  // end namespace chrono

#endif
