#include "chrono/serialization/ChArchiveJSON.h"

namespace chrono {
ChArchiveOutJSON::ChArchiveOutJSON(std::ostream& stream_out) : m_ostream(stream_out) {
    m_ostream << "{ ";
    ++tablevel;

    tablevel = 1;
    nitems.push(0);
    is_array.push(false);
}
ChArchiveOutJSON::~ChArchiveOutJSON() {
    --tablevel;
    nitems.pop();
    is_array.pop();

    m_ostream << "\n}" << std::endl;
}
void ChArchiveOutJSON::indent() {
    for (int i = 0; i < tablevel; ++i)
        m_ostream << "\t";
}
void ChArchiveOutJSON::comma_cr() {
    if (this->nitems.top() > 0) {
        m_ostream << ",";
    }
    m_ostream << "\n";
}
void ChArchiveOutJSON::out(ChNameValue<unsigned int> bVal) {
    comma_cr();
    indent();
    if (is_array.top() == false)
        m_ostream << "\"" << bVal.name() << "\""
                  << "\t: ";
    m_ostream << bVal.value();
    ++nitems.top();
}
void ChArchiveOutJSON::out(ChNameValue<char> bVal) {
    comma_cr();
    indent();
    if (is_array.top() == false)
        m_ostream << "\"" << bVal.name() << "\""
                  << "\t: ";
    m_ostream << (int)bVal.value();
    ++nitems.top();
}
void ChArchiveOutJSON::out(ChNameValue<float> bVal) {
    comma_cr();
    indent();
    if (is_array.top() == false)
        m_ostream << "\"" << bVal.name() << "\""
                  << "\t: ";
    m_ostream << bVal.value();
    ++nitems.top();
}
void ChArchiveOutJSON::out(ChNameValue<double> bVal) {
    comma_cr();
    indent();
    if (is_array.top() == false)
        m_ostream << "\"" << bVal.name() << "\""
                  << "\t: ";
    m_ostream << bVal.value();
    ++nitems.top();
}
void ChArchiveOutJSON::out(ChNameValue<int> bVal) {
    comma_cr();
    indent();
    if (is_array.top() == false)
        m_ostream << "\"" << bVal.name() << "\"";
    m_ostream << "\t: ";
    m_ostream << bVal.value();
    ++nitems.top();
}
void ChArchiveOutJSON::out(ChNameValue<bool> bVal) {
    comma_cr();
    indent();
    if (is_array.top() == false)
        m_ostream << "\"" << bVal.name() << "\""
                  << "\t: ";
    if (bVal.value())
        m_ostream << "true";
    else
        m_ostream << "false";
    ++nitems.top();
}
void ChArchiveOutJSON::out(ChNameValue<const char*> bVal) {
    comma_cr();
    indent();
    if (is_array.top() == false)
        m_ostream << "\"" << bVal.name() << "\""
                  << "\t: ";
    m_ostream << "\"" << bVal.value() << "\"";
    ++nitems.top();
}
void ChArchiveOutJSON::out(ChNameValue<std::string> bVal) {
    comma_cr();
    indent();
    if (is_array.top() == false)
        m_ostream << "\"" << bVal.name() << "\""
                  << "\t: ";
    m_ostream << "\"" << bVal.value() << "\"";
    ++nitems.top();
}
void ChArchiveOutJSON::out(ChNameValue<unsigned long> bVal) {
    comma_cr();
    indent();
    if (is_array.top() == false)
        m_ostream << "\"" << bVal.name() << "\""
                  << "\t: ";
    m_ostream << bVal.value();
    ++nitems.top();
}
void ChArchiveOutJSON::out(ChNameValue<unsigned long long> bVal) {
    comma_cr();
    indent();
    if (is_array.top() == false)
        m_ostream << "\"" << bVal.name() << "\""
                  << "\t: ";
    m_ostream << bVal.value();
    ++nitems.top();
}
void ChArchiveOutJSON::out(ChNameValue<ChEnumMapperBase> bVal) {
    comma_cr();
    indent();
    if (is_array.top() == false)
        m_ostream << "\"" << bVal.name() << "\""
                  << "\t: ";
    std::string mstr = bVal.value().GetValueAsString();
    m_ostream << "\"" << mstr << "\"";
    ++nitems.top();
}
void ChArchiveOutJSON::out_array_pre(ChValue& bVal, size_t msize) {
    comma_cr();
    if (is_array.top() == false) {
        indent();
        m_ostream << "\"" << bVal.name() << "\""
                  << "\t: ";
    }
    m_ostream << "\n";
    indent();
    m_ostream << "[ ";

    ++tablevel;
    nitems.push(0);
    is_array.push(true);
}
void ChArchiveOutJSON::out_array_between(ChValue& bVal, size_t msize) {}
void ChArchiveOutJSON::out_array_end(ChValue& bVal, size_t msize) {
    --tablevel;
    nitems.pop();
    is_array.pop();

    m_ostream << "\n";
    indent();
    m_ostream << "]";
    ++nitems.top();
}

// for custom c++ objects:

void ChArchiveOutJSON::out(ChValue& bVal, bool tracked, size_t obj_ID) {
    comma_cr();
    if (is_array.top() == false) {
        indent();
        m_ostream << "\"" << bVal.name() << "\""
                  << "\t: \n";
    }
    indent();
    m_ostream << "{";

    ++tablevel;
    nitems.push(0);
    is_array.push(false);

    if (tracked) {
        comma_cr();
        indent();
        m_ostream << "\"_object_ID\"\t: " << obj_ID;
        ++nitems.top();
    }

    bVal.CallArchiveOut(*this);

    --tablevel;
    nitems.pop();
    is_array.pop();

    m_ostream << "\n";
    indent();
    m_ostream << "}";
    ++nitems.top();
}
void ChArchiveOutJSON::out_ref(ChValue& bVal, bool already_inserted, size_t obj_ID, size_t ext_ID) {
    // the returned classname refers not to the type of the pointer itself, but to the *true* type of the object
    // i.e. the most derived type for inherited classes
    std::string classname = bVal.GetClassRegisteredName();
    comma_cr();
    if (is_array.top() == false) {
        indent();
        m_ostream << "\"" << bVal.name() << "\""
                  << "\t: \n";
    }
    indent();
    m_ostream << "{ ";

    ++tablevel;
    nitems.push(0);
    is_array.push(false);

    if (classname.length() > 0) {
        comma_cr();
        indent();
        m_ostream << "\"_type\"\t: "
                  << "\"" << classname.c_str() << "\"";
        ++nitems.top();
    }

    if (!already_inserted) {
        comma_cr();
        indent();
        m_ostream << "\"_object_ID\"\t: " << obj_ID;
        ++nitems.top();

        // New Object, we have to full serialize it
        bVal.CallArchiveOutConstructor(*this);
        bVal.CallArchiveOut(*this);
    } else {
        if (obj_ID || bVal.IsNull()) {
            comma_cr();
            indent();
            m_ostream << "\"_reference_ID\"\t: " << obj_ID;
            ++nitems.top();
        }
        if (ext_ID) {
            comma_cr();
            indent();
            m_ostream << "\"_external_ID\"\t: " << ext_ID;
            ++nitems.top();
        }
    }
    --tablevel;
    nitems.pop();
    is_array.pop();

    m_ostream << "\n";
    indent();
    m_ostream << "}";
    ++nitems.top();
}
ChArchiveInJSON::ChArchiveInJSON(std::ifstream& stream_in) : m_istream(stream_in) {
    std::stringstream buffer;
    buffer << m_istream.rdbuf();
    std::string mstr = buffer.str();
    const char* stringbuffer = mstr.c_str();

    document.Parse<0>(stringbuffer);
    if (document.HasParseError()) {
        std::string errstrA((const char*)(&stringbuffer[std::max((int)document.GetErrorOffset() - 10, 0)]));
        errstrA.resize(10);
        std::string errstrB((const char*)(&stringbuffer[document.GetErrorOffset()]));
        errstrB.resize(20);
        throw std::invalid_argument("ERROR: the file has bad JSON syntax," + std::to_string(document.GetParseError()) +
                                    " \n\n[...]" + errstrA + " <--- " + errstrB + "[...]");
    }
    if (!document.IsObject())
        throw std::invalid_argument("ERROR: the file is not a valid JSON document");

    level = &document;
    levels.push(level);
    is_array.push(false);

    can_tolerate_missing_tokens = true;
    try_tolerate_missing_tokens = false;
}
ChArchiveInJSON::~ChArchiveInJSON() {}
rapidjson::Value* ChArchiveInJSON::GetValueFromNameOrArray(const std::string& mname) {
    rapidjson::Value* mval = nullptr;
    if (this->is_array.top() == true) {
        if (!level->IsArray()) {
            throw(std::runtime_error("Cannot retrieve from ID num in non-array object."));
        }
        mval = &(*level)[this->array_index.top()];
    } else {
        if (level->HasMember(mname.c_str()))
            mval = &(*level)[mname.c_str()];
        else {
            token_notfound(mname);
        }
    }
    return mval;
}
bool ChArchiveInJSON::in(ChNameValue<unsigned int> bVal) {
    rapidjson::Value* mval = GetValueFromNameOrArray(bVal.name());
    if (!mval)
        return false;  // if not try_tolerate_missing_tokens, an exception would have been already thrown
    if (!mval->IsUint()) {
        throw std::runtime_error("Invalid unsigned integer number after '" + std::string(bVal.name()) + "'");
    }
    bVal.value() = mval->GetUint();
    return true;
}
bool ChArchiveInJSON::in(ChNameValue<char> bVal) {
    rapidjson::Value* mval = GetValueFromNameOrArray(bVal.name());
    if (!mval)
        return false;  // if not try_tolerate_missing_tokens, an exception would have been already thrown
    if (!mval->IsInt()) {
        throw std::runtime_error("Invalid char code after '" + std::string(bVal.name()) + "'");
    }
    bVal.value() = (char)mval->GetInt();
    return true;
}
bool ChArchiveInJSON::in(ChNameValue<float> bVal) {
    rapidjson::Value* mval = GetValueFromNameOrArray(bVal.name());
    if (!mval)
        return false;  // if not try_tolerate_missing_tokens, an exception would have been already thrown
    if (!mval->IsNumber()) {
        throw std::runtime_error("Invalid number after '" + std::string(bVal.name()) + "'");
    }
    bVal.value() = (float)mval->GetDouble();
    return true;
}
bool ChArchiveInJSON::in(ChNameValue<double> bVal) {
    rapidjson::Value* mval = GetValueFromNameOrArray(bVal.name());
    if (!mval)
        return false;  // if not try_tolerate_missing_tokens, an exception would have been already thrown
    if (!mval->IsNumber()) {
        throw std::runtime_error("Invalid number after '" + std::string(bVal.name()) + "'");
    }
    bVal.value() = mval->GetDouble();
    return true;
}
bool ChArchiveInJSON::in(ChNameValue<int> bVal) {
    rapidjson::Value* mval = GetValueFromNameOrArray(bVal.name());
    if (!mval)
        return false;  // if not try_tolerate_missing_tokens, an exception would have been already thrown
    if (!mval->IsInt()) {
        throw std::runtime_error("Invalid integer number after '" + std::string(bVal.name()) + "'");
    }
    bVal.value() = mval->GetInt();
    return true;
}
bool ChArchiveInJSON::in(ChNameValue<bool> bVal) {
    rapidjson::Value* mval = GetValueFromNameOrArray(bVal.name());
    if (!mval)
        return false;  // if not try_tolerate_missing_tokens, an exception would have been already thrown
    if (!mval->IsBool()) {
        throw std::runtime_error("Invalid true/false flag after '" + std::string(bVal.name()) + "'");
    }
    bVal.value() = mval->GetBool();
    return true;
}
bool ChArchiveInJSON::in(ChNameValue<std::string> bVal) {
    rapidjson::Value* mval = GetValueFromNameOrArray(bVal.name());
    if (!mval)
        return false;  // if not try_tolerate_missing_tokens, an exception would have been already thrown
    if (!mval->IsString()) {
        throw std::runtime_error("Invalid string after '" + std::string(bVal.name()) + "'");
    }
    bVal.value() = mval->GetString();
    return true;
}
bool ChArchiveInJSON::in(ChNameValue<unsigned long> bVal) {
    rapidjson::Value* mval = GetValueFromNameOrArray(bVal.name());
    if (!mval)
        return false;  // if not try_tolerate_missing_tokens, an exception would have been already thrown
    if (!mval->IsUint64()) {
        throw std::runtime_error("Invalid unsigned long number after '" + std::string(bVal.name()) + "'");
    }
    bVal.value() = (unsigned long)mval->GetUint64();
    return true;
}
bool ChArchiveInJSON::in(ChNameValue<unsigned long long> bVal) {
    rapidjson::Value* mval = GetValueFromNameOrArray(bVal.name());
    if (!mval)
        return false;  // if not try_tolerate_missing_tokens, an exception would have been already thrown
    if (!mval->IsUint64()) {
        throw std::runtime_error("Invalid unsigned long long number after '" + std::string(bVal.name()) + "'");
    }
    bVal.value() = mval->GetUint64();
    return true;
}
bool ChArchiveInJSON::in(ChNameValue<ChEnumMapperBase> bVal) {
    rapidjson::Value* mval = GetValueFromNameOrArray(bVal.name());
    if (!mval)
        return false;  // if not try_tolerate_missing_tokens, an exception would have been already thrown
    if (!mval->IsString()) {
        throw std::runtime_error("Invalid string after '" + std::string(bVal.name()) + "'");
    }
    std::string mstr = mval->GetString();
    if (!bVal.value().SetValueAsString(mstr)) {
        throw std::runtime_error("Not recognized enum type '" + mstr + "'");
    }
    return true;
}

// for wrapping arrays and lists

bool ChArchiveInJSON::in_array_pre(const std::string& name, size_t& msize) {
    rapidjson::Value* mval = GetValueFromNameOrArray(name);
    if (!mval) {
        msize = 0;
        return false;
    } else {
        if (!mval->IsArray()) {
            throw std::runtime_error("Invalid array [...] after '" + std::string(name) + "'");
        }
        msize = mval->Size();
        this->levels.push(mval);
        this->level = this->levels.top();
        this->is_array.push(true);
        this->array_index.push(0);
        return true;
    }
}
void ChArchiveInJSON::in_array_between(const std::string& name) {
    ++this->array_index.top();
}
void ChArchiveInJSON::in_array_end(const std::string& name) {
    this->levels.pop();
    this->level = this->levels.top();
    this->is_array.pop();
    this->array_index.pop();
}

//  for custom c++ objects:

bool ChArchiveInJSON::in(ChNameValue<ChFunctorArchiveIn> bVal) {
    rapidjson::Value* mval = GetValueFromNameOrArray(bVal.name());
    if (!mval)
        return false;  // if not try_tolerate_missing_tokens, an exception would have been already thrown
    if (!mval->IsObject()) {
        throw std::runtime_error("Invalid object {...} after '" + std::string(bVal.name()) + "'");
    }

    this->levels.push(mval);
    this->level = this->levels.top();
    this->is_array.push(false);

    if (bVal.flags() & NVP_TRACK_OBJECT) {
        size_t obj_ID = 0;
        if (level->HasMember("_object_ID")) {
            if (!(*level)["_object_ID"].IsUint64()) {
                throw std::runtime_error("Wrong _object_ID for entry: '" + std::string(bVal.name()) + "'");
            }
            obj_ID = (*level)["_object_ID"].GetUint64();
        } else
            throw std::runtime_error("Missing _object_ID for entry: '" + std::string(bVal.name()) + "'");

        PutNewPointer(bVal.value().GetRawPtr(), obj_ID);
    }

    bVal.value().CallArchiveIn(*this);

    this->levels.pop();
    this->level = this->levels.top();
    this->is_array.pop();
    return true;
}

// for objects to construct, return non-null ptr if new object, return null ptr if just reused obj

bool ChArchiveInJSON::in_ref(ChNameValue<ChFunctorArchiveIn> bVal, void** ptr, std::string& true_classname) {
    void* new_ptr = nullptr;

    rapidjson::Value* mval = GetValueFromNameOrArray(bVal.name());
    if (!mval)
        return false;
    if (!mval->IsObject()) {
        throw std::runtime_error("Invalid object {...} after '" + std::string(bVal.name()) + "'");
    }
    this->levels.push(mval);
    this->level = this->levels.top();
    this->is_array.push(false);

    if (bVal.value().IsPolymorphic()) {
        if (level->HasMember("_type")) {
            if (!(*level)["_type"].IsString()) {
                throw std::runtime_error("Invalid string after '" + std::string(bVal.name()) + "'");
            }
            true_classname = (*level)["_type"].GetString();
        }
    }
    bool is_reference = false;
    size_t ref_ID = 0;
    if (level->HasMember("_reference_ID")) {
        if (!(*level)["_reference_ID"].IsUint64()) {
            throw std::runtime_error("Wrong _reference_ID for entry: '" + std::string(bVal.name()) + "'");
        }
        ref_ID = (*level)["_reference_ID"].GetUint64();
        is_reference = true;
    }
    size_t ext_ID = 0;
    if (level->HasMember("_external_ID")) {
        if (!(*level)["_external_ID"].IsUint64()) {
            throw std::runtime_error("Wrong _external_ID for entry: '" + std::string(bVal.name()) + "'");
        }
        ext_ID = (*level)["_external_ID"].GetUint64();
        is_reference = true;
    }

    if (!is_reference) {
        // 2) Dynamically create: call new(), or deserialize constructor params+call new()
        // The constructor to be called will be the one specified by true_classname (i.e. the true type of the
        // object), not by the type of bVal.value() which is just the type of the pointer
        bVal.value().CallConstructor(*this, true_classname);

        size_t obj_ID = 0;
        if (level->HasMember("_object_ID")) {
            if (!(*level)["_object_ID"].IsUint64()) {
                throw std::runtime_error("Wrong _object_ID for entry: '" + std::string(bVal.name()) + "'");
            }
            obj_ID = (*level)["_object_ID"].GetUint64();
        } else
            throw std::runtime_error("Missing _object_ID for entry: '" + std::string(bVal.name()) + "'");

        // Calling bVal.value().GetRawPtr() will retrieve the true address of the object
        void* true_ptr = bVal.value().GetRawPtr();

        if (true_ptr) {
            PutNewPointer(true_ptr, obj_ID);
            // 3) Deserialize
            // It is required to specify the "true_classname" since the bValue.value() might be of a different type
            // compared to the true object, while we need to call the ArchiveIn of the proper derived class.
            bVal.value().CallArchiveIn(*this, true_classname);
        } else {
            throw std::runtime_error("Archive cannot create object " + std::string(bVal.name()) + "\n");
        }

        new_ptr = bVal.value().GetRawPtr();
    } else {
        if (ref_ID) {
            if (this->internal_id_ptr.find(ref_ID) == this->internal_id_ptr.end()) {
                throw std::runtime_error("In object '" + std::string(bVal.name()) + "' the _reference_ID " +
                                         std::to_string((int)ref_ID) + " is not a valid number.");
            }
            bVal.value().SetRawPtr(
                ChCastingMap::Convert(true_classname, bVal.value().GetObjectPtrTypeindex(), internal_id_ptr[ref_ID]));

        } else if (ext_ID) {
            if (this->external_id_ptr.find(ext_ID) == this->external_id_ptr.end()) {
                throw std::runtime_error("In object '" + std::string(bVal.name()) + "' the _external_ID " +
                                         std::to_string((int)ext_ID) + " is not valid.");
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
bool ChArchiveInJSON::TryTolerateMissingTokens(bool try_tolerate) {
    try_tolerate_missing_tokens = try_tolerate;
    return try_tolerate_missing_tokens;
}
void ChArchiveInJSON::token_notfound(const std::string& mname) {
    if (!try_tolerate_missing_tokens) {
        std::cerr << "Cannot find '" + mname + "'" << std::endl;
        throw std::runtime_error("Cannot find '" + mname + "'");
    }
}
}  // end namespace chrono
