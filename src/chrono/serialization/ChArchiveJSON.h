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
#include "chrono/core/ChLog.h"
#include "chrono/core/ChMathematics.h"

#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_thirdparty/rapidjson/prettywriter.h"
#include "chrono_thirdparty/rapidjson/filereadstream.h"
#include "chrono_thirdparty/rapidjson/filewritestream.h"

#include <stack>
#include <fstream>
#include <iostream>
#include <sstream>

namespace chrono {

///
/// This is a class for serializing to JSON
///

class ChArchiveOutJSON : public ChArchiveOut {
  public:
    ChArchiveOutJSON(ChStreamOutAsciiFile& mostream) {
        ostream = &mostream;

        (*ostream) << "{ ";
        ++tablevel;

        tablevel = 1;
        nitems.push(0);
        is_array.push(false);
    };

    virtual ~ChArchiveOutJSON() {
        --tablevel;
        nitems.pop();
        is_array.pop();

        (*ostream) << "\n}\n";
    };

    void indent() {
        for (int i = 0; i < tablevel; ++i)
            (*ostream) << "\t";
    }

    void comma_cr() {
        if (this->nitems.top() > 0) {
            (*ostream) << ",";
        }
        (*ostream) << "\n";
    }

    virtual void out(ChNameValue<bool> bVal) {
        comma_cr();
        indent();
        if (is_array.top() == false)
            (*ostream) << "\"" << bVal.name() << "\""
                       << "\t: ";
        if (bVal.value())
            (*ostream) << "true";
        else
            (*ostream) << "false";
        ++nitems.top();
    }
    virtual void out(ChNameValue<int> bVal) {
        comma_cr();
        indent();
        if (is_array.top() == false)
            (*ostream) << "\"" << bVal.name() << "\"";
        (*ostream) << "\t: ";
        (*ostream) << bVal.value();
        ++nitems.top();
    }
    virtual void out(ChNameValue<double> bVal) {
        comma_cr();
        indent();
        if (is_array.top() == false)
            (*ostream) << "\"" << bVal.name() << "\""
                       << "\t: ";
        (*ostream) << bVal.value();
        ++nitems.top();
    }
    virtual void out(ChNameValue<float> bVal) {
        comma_cr();
        indent();
        if (is_array.top() == false)
            (*ostream) << "\"" << bVal.name() << "\""
                       << "\t: ";
        (*ostream) << bVal.value();
        ++nitems.top();
    }
    virtual void out(ChNameValue<char> bVal) {
        comma_cr();
        indent();
        if (is_array.top() == false)
            (*ostream) << "\"" << bVal.name() << "\""
                       << "\t: ";
        (*ostream) << (int)bVal.value();
        ++nitems.top();
    }
    virtual void out(ChNameValue<unsigned int> bVal) {
        comma_cr();
        indent();
        if (is_array.top() == false)
            (*ostream) << "\"" << bVal.name() << "\""
                       << "\t: ";
        (*ostream) << bVal.value();
        ++nitems.top();
    }
    virtual void out(ChNameValue<const char*> bVal) {
        comma_cr();
        indent();
        if (is_array.top() == false)
            (*ostream) << "\"" << bVal.name() << "\""
                       << "\t: ";
        (*ostream) << "\"" << bVal.value() << "\"";
        ++nitems.top();
    }
    virtual void out(ChNameValue<std::string> bVal) {
        comma_cr();
        indent();
        if (is_array.top() == false)
            (*ostream) << "\"" << bVal.name() << "\""
                       << "\t: ";
        (*ostream) << "\"" << bVal.value() << "\"";
        ++nitems.top();
    }
    virtual void out(ChNameValue<unsigned long> bVal) {
        comma_cr();
        indent();
        if (is_array.top() == false)
            (*ostream) << "\"" << bVal.name() << "\""
                       << "\t: ";
        (*ostream) << bVal.value();
        ++nitems.top();
    }
    virtual void out(ChNameValue<unsigned long long> bVal) {
        comma_cr();
        indent();
        if (is_array.top() == false)
            (*ostream) << "\"" << bVal.name() << "\""
                       << "\t: ";
        (*ostream) << bVal.value();
        ++nitems.top();
    }
    virtual void out(ChNameValue<ChEnumMapperBase> bVal) {
        comma_cr();
        indent();
        if (is_array.top() == false)
            (*ostream) << "\"" << bVal.name() << "\""
                       << "\t: ";
        std::string mstr = bVal.value().GetValueAsString();
        (*ostream) << "\"" << mstr << "\"";
        ++nitems.top();
    }

    virtual void out_array_pre(ChValue& bVal, size_t msize) {
        comma_cr();
        if (is_array.top() == false) {
            indent();
            (*ostream) << "\"" << bVal.name() << "\""
                       << "\t: ";
        }
        (*ostream) << "\n";
        indent();
        (*ostream) << "[ ";

        ++tablevel;
        nitems.push(0);
        is_array.push(true);
    }
    virtual void out_array_between(ChValue& bVal, size_t msize) {}
    virtual void out_array_end(ChValue& bVal, size_t msize) {
        --tablevel;
        nitems.pop();
        is_array.pop();

        (*ostream) << "\n";
        indent();
        (*ostream) << "]";
        ++nitems.top();
    }

    // for custom c++ objects:
    virtual void out(ChValue& bVal, bool tracked, size_t obj_ID) {
        comma_cr();
        if (is_array.top() == false) {
            indent();
            (*ostream) << "\"" << bVal.name() << "\""
                       << "\t: \n";
        }
        indent();
        (*ostream) << "{";

        ++tablevel;
        nitems.push(0);
        is_array.push(false);

        if (tracked) {
            comma_cr();
            indent();
            (*ostream) << "\"_object_ID\"\t: " << obj_ID;
            ++nitems.top();
        }

        bVal.CallArchiveOut(*this);

        --tablevel;
        nitems.pop();
        is_array.pop();

        (*ostream) << "\n";
        indent();
        (*ostream) << "}";
        ++nitems.top();
    }

    virtual void out_ref(ChValue& bVal, bool already_inserted, size_t obj_ID, size_t ext_ID) {
        // the returned classname refers not to the type of the pointer itself, but to the *true* type of the object
        // i.e. the most derived type for inherited classes
        std::string classname = bVal.GetClassRegisteredName();
        comma_cr();
        if (is_array.top() == false) {
            indent();
            (*ostream) << "\"" << bVal.name() << "\""
                       << "\t: \n";
        }
        indent();
        (*ostream) << "{ ";

        ++tablevel;
        nitems.push(0);
        is_array.push(false);

        if (classname.length() > 0) {
            comma_cr();
            indent();
            (*ostream) << "\"_type\"\t: "
                       << "\"" << classname.c_str() << "\"";
            ++nitems.top();
        }

        if (!already_inserted) {
            comma_cr();
            indent();
            (*ostream) << "\"_object_ID\"\t: " << obj_ID;
            ++nitems.top();

            // New Object, we have to full serialize it
            bVal.CallArchiveOutConstructor(*this);
            bVal.CallArchiveOut(*this);
        } else {
            if (obj_ID || bVal.IsNull()) {
                comma_cr();
                indent();
                (*ostream) << "\"_reference_ID\"\t: " << obj_ID;
                ++nitems.top();
            }
            if (ext_ID) {
                comma_cr();
                indent();
                (*ostream) << "\"_external_ID\"\t: " << ext_ID;
                ++nitems.top();
            }
        }
        --tablevel;
        nitems.pop();
        is_array.pop();

        (*ostream) << "\n";
        indent();
        (*ostream) << "}";
        ++nitems.top();
    }

  protected:
    int tablevel;
    ChStreamOutAsciiFile* ostream;
    std::stack<int> nitems;
    std::stack<bool> is_array;
};

///
/// This is a class for deserializing from JSON archives
///

class ChArchiveInJSON : public ChArchiveIn {
  public:
    ChArchiveInJSON(ChStreamInAsciiFile& mistream) {
        istream = &mistream;

        std::stringstream buffer;
        buffer << istream->GetFstream().rdbuf();
        std::string mstr = buffer.str();
        const char* stringbuffer = mstr.c_str();

        document.Parse<0>(stringbuffer);
        if (document.HasParseError()) {
            std::string errstrA((const char*)(&stringbuffer[ChMax((int)document.GetErrorOffset() - 10, 0)]));
            errstrA.resize(10);
            std::string errstrB((const char*)(&stringbuffer[document.GetErrorOffset()]));
            errstrB.resize(20);
            throw(ChExceptionArchive("the file has bad JSON syntax," + std::to_string(document.GetParseError()) +
                                     " \n\n[...]" + errstrA + " <--- " + errstrB + "[...]\n"));
        }
        if (!document.IsObject())
            throw(ChExceptionArchive("the file is not a valid JSON document"));

        level = &document;
        levels.push(level);
        is_array.push(false);

        can_tolerate_missing_tokens = true;
        try_tolerate_missing_tokens = false;
    }

    virtual ~ChArchiveInJSON() {}

    rapidjson::Value* GetValueFromNameOrArray(const char* mname) {
        rapidjson::Value* mval = nullptr;
        if (this->is_array.top() == true) {
            if (!level->IsArray()) {
                throw(ChExceptionArchive("Cannot retrieve from ID num in non-array object."));
            }
            mval = &(*level)[this->array_index.top()];
        } else {
            if (level->HasMember(mname))
                mval = &(*level)[mname];
            else {
                token_notfound(mname);
            }
        }
        return mval;
    }

    virtual bool in(ChNameValue<bool> bVal) override {
        rapidjson::Value* mval = GetValueFromNameOrArray(bVal.name());
        if (!mval)
            return false;  // if not try_tolerate_missing_tokens, an exception would have been already thrown
        if (!mval->IsBool()) {
            throw(ChExceptionArchive("Invalid true/false flag after '" + std::string(bVal.name()) + "'"));
        }
        bVal.value() = mval->GetBool();
        return true;
    }
    virtual bool in(ChNameValue<int> bVal) override {
        rapidjson::Value* mval = GetValueFromNameOrArray(bVal.name());
        if (!mval)
            return false;  // if not try_tolerate_missing_tokens, an exception would have been already thrown
        if (!mval->IsInt()) {
            throw(ChExceptionArchive("Invalid integer number after '" + std::string(bVal.name()) + "'"));
        }
        bVal.value() = mval->GetInt();
        return true;
    }
    virtual bool in(ChNameValue<double> bVal) override {
        rapidjson::Value* mval = GetValueFromNameOrArray(bVal.name());
        if (!mval)
            return false;  // if not try_tolerate_missing_tokens, an exception would have been already thrown
        if (!mval->IsNumber()) {
            throw(ChExceptionArchive("Invalid number after '" + std::string(bVal.name()) + "'"));
        }
        bVal.value() = mval->GetDouble();
        return true;
    }
    virtual bool in(ChNameValue<float> bVal) override {
        rapidjson::Value* mval = GetValueFromNameOrArray(bVal.name());
        if (!mval)
            return false;  // if not try_tolerate_missing_tokens, an exception would have been already thrown
        if (!mval->IsNumber()) {
            throw(ChExceptionArchive("Invalid number after '" + std::string(bVal.name()) + "'"));
        }
        bVal.value() = (float)mval->GetDouble();
        return true;
    }
    virtual bool in(ChNameValue<char> bVal) override {
        rapidjson::Value* mval = GetValueFromNameOrArray(bVal.name());
        if (!mval)
            return false;  // if not try_tolerate_missing_tokens, an exception would have been already thrown
        if (!mval->IsInt()) {
            throw(ChExceptionArchive("Invalid char code after '" + std::string(bVal.name()) + "'"));
        }
        bVal.value() = (char)mval->GetInt();
        return true;
    }
    virtual bool in(ChNameValue<unsigned int> bVal) override {
        rapidjson::Value* mval = GetValueFromNameOrArray(bVal.name());
        if (!mval)
            return false;  // if not try_tolerate_missing_tokens, an exception would have been already thrown
        if (!mval->IsUint()) {
            throw(ChExceptionArchive("Invalid unsigned integer number after '" + std::string(bVal.name()) + "'"));
        }
        bVal.value() = mval->GetUint();
        return true;
    }
    virtual bool in(ChNameValue<std::string> bVal) override {
        rapidjson::Value* mval = GetValueFromNameOrArray(bVal.name());
        if (!mval)
            return false;  // if not try_tolerate_missing_tokens, an exception would have been already thrown
        if (!mval->IsString()) {
            throw(ChExceptionArchive("Invalid string after '" + std::string(bVal.name()) + "'"));
        }
        bVal.value() = mval->GetString();
        return true;
    }
    virtual bool in(ChNameValue<unsigned long> bVal) override {
        rapidjson::Value* mval = GetValueFromNameOrArray(bVal.name());
        if (!mval)
            return false;  // if not try_tolerate_missing_tokens, an exception would have been already thrown
        if (!mval->IsUint64()) {
            throw(ChExceptionArchive("Invalid unsigned long number after '" + std::string(bVal.name()) + "'"));
        }
        bVal.value() = (unsigned long)mval->GetUint64();
        return true;
    }
    virtual bool in(ChNameValue<unsigned long long> bVal) override {
        rapidjson::Value* mval = GetValueFromNameOrArray(bVal.name());
        if (!mval)
            return false;  // if not try_tolerate_missing_tokens, an exception would have been already thrown
        if (!mval->IsUint64()) {
            throw(ChExceptionArchive("Invalid unsigned long long number after '" + std::string(bVal.name()) + "'"));
        }
        bVal.value() = mval->GetUint64();
        return true;
    }
    virtual bool in(ChNameValue<ChEnumMapperBase> bVal) override {
        rapidjson::Value* mval = GetValueFromNameOrArray(bVal.name());
        if (!mval)
            return false;  // if not try_tolerate_missing_tokens, an exception would have been already thrown
        if (!mval->IsString()) {
            throw(ChExceptionArchive("Invalid string after '" + std::string(bVal.name()) + "'"));
        }
        std::string mstr = mval->GetString();
        if (!bVal.value().SetValueAsString(mstr)) {
            throw(ChExceptionArchive("Not recognized enum type '" + mstr + "'"));
        }
        return true;
    }

    // for wrapping arrays and lists
    virtual bool in_array_pre(const char* name, size_t& msize) override {
        rapidjson::Value* mval = GetValueFromNameOrArray(name);
        if (!mval) {
            msize = 0;
            return false;
        } else {
            if (!mval->IsArray()) {
                throw(ChExceptionArchive("Invalid array [...] after '" + std::string(name) + "'"));
            }
            msize = mval->Size();
            this->levels.push(mval);
            this->level = this->levels.top();
            this->is_array.push(true);
            this->array_index.push(0);
            return true;
        }
    }
    virtual void in_array_between(const char* name) override { ++this->array_index.top(); }

    virtual void in_array_end(const char* name) override {
        this->levels.pop();
        this->level = this->levels.top();
        this->is_array.pop();
        this->array_index.pop();
    }

    //  for custom c++ objects:
    virtual bool in(ChNameValue<ChFunctorArchiveIn> bVal) override {
        rapidjson::Value* mval = GetValueFromNameOrArray(bVal.name());
        if (!mval)
            return false;  // if not try_tolerate_missing_tokens, an exception would have been already thrown
        if (!mval->IsObject()) {
            throw(ChExceptionArchive("Invalid object {...} after '" + std::string(bVal.name()) + "'"));
        }

        this->levels.push(mval);
        this->level = this->levels.top();
        this->is_array.push(false);

        if (bVal.flags() & NVP_TRACK_OBJECT) {
            size_t obj_ID = 0;
            if (level->HasMember("_object_ID")) {
                if (!(*level)["_object_ID"].IsUint64()) {
                    throw(ChExceptionArchive("Wrong _object_ID for entry: '" + std::string(bVal.name()) + "'"));
                }
                obj_ID = (*level)["_object_ID"].GetUint64();
            }
            else
                throw(ChExceptionArchive("Missing _object_ID for entry: '" + std::string(bVal.name()) + "'"));

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

        rapidjson::Value* mval = GetValueFromNameOrArray(bVal.name());
        if (!mval)
            return false;
        if (!mval->IsObject()) {
            throw(ChExceptionArchive("Invalid object {...} after '" + std::string(bVal.name()) + "'"));
        }
        this->levels.push(mval);
        this->level = this->levels.top();
        this->is_array.push(false);

        if (bVal.value().IsPolymorphic()) {
            if (level->HasMember("_type")) {
                if (!(*level)["_type"].IsString()) {
                    throw(ChExceptionArchive("Invalid string after '" + std::string(bVal.name()) + "'"));
                }
                true_classname = (*level)["_type"].GetString();
            }
        }
        bool is_reference = false;
        size_t ref_ID = 0;
        if (level->HasMember("_reference_ID")) {
            if (!(*level)["_reference_ID"].IsUint64()) {
                throw(ChExceptionArchive("Wrong _reference_ID for entry: '" + std::string(bVal.name()) + "'"));
            }
            ref_ID = (*level)["_reference_ID"].GetUint64();
            is_reference = true;
        }
        size_t ext_ID = 0;
        if (level->HasMember("_external_ID")) {
            if (!(*level)["_external_ID"].IsUint64()) {
                throw(ChExceptionArchive("Wrong _external_ID for entry: '" + std::string(bVal.name()) + "'"));
            }
            ext_ID = (*level)["_external_ID"].GetUint64();
            is_reference = true;
        }

        if (!is_reference) {
            // 2) Dynamically create: call new(), or deserialize constructor params+call new()
            // The constructor to be called will be the one specified by true_classname (i.e. the true type of the object),
            // not by the type of bVal.value() which is just the type of the pointer
            bVal.value().CallConstructor(*this, true_classname.c_str());

            size_t obj_ID = 0;
            if (level->HasMember("_object_ID")) {
                if (!(*level)["_object_ID"].IsUint64()) {
                    throw(ChExceptionArchive("Wrong _object_ID for entry: '" + std::string(bVal.name()) + "'"));
                }
                obj_ID = (*level)["_object_ID"].GetUint64();
            }
            else
                throw(ChExceptionArchive("Missing _object_ID for entry: '" + std::string(bVal.name()) + "'"));

            // Calling bVal.value().GetRawPtr() will retrieve the true address of the object
            void* true_ptr = bVal.value().GetRawPtr();

            if (true_ptr) {
                PutNewPointer(true_ptr, obj_ID);
                // 3) Deserialize
                // It is required to specify the "true_classname" since the bValue.value() might be of a different type
                // compared to the true object, while we need to call the ArchiveIn of the proper derived class.
                bVal.value().CallArchiveIn(*this, true_classname.c_str());
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

        *ptr = new_ptr;
        return true;
    }


    virtual bool TryTolerateMissingTokens(bool try_tolerate) override {
        try_tolerate_missing_tokens = try_tolerate;
        return try_tolerate_missing_tokens;
    }

  protected:
    void token_notfound(const char* mname) {
        if (!try_tolerate_missing_tokens)
            throw(ChExceptionArchive("Cannot find '" + std::string(mname) + "'"));
    }

    ChStreamInAsciiFile* istream;
    rapidjson::Document document;
    rapidjson::Value* level;
    std::stack<rapidjson::Value*> levels;
    std::stack<bool> is_array;
    std::stack<int> array_index;
};

}  // end namespace chrono

#endif
