#include "chrono/serialization/ChArchiveXML.h"

#include <fstream>
#include <sstream>
#include <string>
#include <cstring>

namespace chrono {

///////////////////////// ChArchiveOutXML /////////////////////////

ChArchiveOutXML::ChArchiveOutXML(std::ostream& stream_out) : m_ostream(stream_out) {
    tablevel = 0;
    nitems.push(0);
    is_array.push(false);
}
ChArchiveOutXML::~ChArchiveOutXML() {
    nitems.pop();
    is_array.pop();
}
void ChArchiveOutXML::indent() {
    for (int i = 0; i < tablevel; ++i)
        m_ostream << "\t";
}
void ChArchiveOutXML::out(ChNameValue<unsigned int> bVal) {
    indent();
    // if (is_array.top() == false)
    m_ostream << "<" << bVal.name() << ">";
    m_ostream << bVal.value();
    // if (is_array.top() == false)
    m_ostream << "</" << bVal.name() << ">\n";
    ++nitems.top();
}
void ChArchiveOutXML::out(ChNameValue<char> bVal) {
    indent();
    // if (is_array.top() == false)
    m_ostream << "<" << bVal.name() << ">";
    m_ostream << (int)bVal.value();
    // if (is_array.top() == false)
    m_ostream << "</" << bVal.name() << ">\n";
    ++nitems.top();
}
void ChArchiveOutXML::out(ChNameValue<float> bVal) {
    indent();
    // if (is_array.top() == false)
    m_ostream << "<" << bVal.name() << ">";
    m_ostream << bVal.value();
    // if (is_array.top() == false)
    m_ostream << "</" << bVal.name() << ">\n";
    ++nitems.top();
}
void ChArchiveOutXML::out(ChNameValue<double> bVal) {
    indent();
    // if (is_array.top() == false)
    m_ostream << "<" << bVal.name() << ">";
    m_ostream << bVal.value();
    // if (is_array.top() == false)
    m_ostream << "</" << bVal.name() << ">\n";
    ++nitems.top();
}
void ChArchiveOutXML::out(ChNameValue<int> bVal) {
    indent();
    // if (is_array.top()==false)
    m_ostream << "<" << bVal.name() << ">";
    m_ostream << bVal.value();
    // if (is_array.top() == false)
    m_ostream << "</" << bVal.name() << ">\n";
    ++nitems.top();
}
void ChArchiveOutXML::out(ChNameValue<bool> bVal) {
    indent();
    // if (is_array.top()==false)
    m_ostream << "<" << bVal.name() << ">";
    if (bVal.value())
        m_ostream << "true";
    else
        m_ostream << "false";
    // if (is_array.top() == false)
    m_ostream << "</" << bVal.name() << ">\n";
    ++nitems.top();
}
void ChArchiveOutXML::out(ChNameValue<const char*> bVal) {
    indent();
    // if (is_array.top() == false)
    m_ostream << "<" << bVal.name() << ">";
    m_ostream << "\"" << bVal.value() << "\"";
    // if (is_array.top() == false)
    m_ostream << "</" << bVal.name() << ">\n";
    ++nitems.top();
}
void ChArchiveOutXML::out(ChNameValue<std::string> bVal) {
    indent();
    // if (is_array.top() == false)
    m_ostream << "<" << bVal.name() << ">";
    m_ostream << bVal.value();
    // if (is_array.top() == false)
    m_ostream << "</" << bVal.name() << ">\n";
    ++nitems.top();
}
void ChArchiveOutXML::out(ChNameValue<unsigned long> bVal) {
    indent();
    // if (is_array.top() == false)
    m_ostream << "<" << bVal.name() << ">";
    m_ostream << bVal.value();
    // if (is_array.top() == false)
    m_ostream << "</" << bVal.name() << ">\n";
    ++nitems.top();
}
void ChArchiveOutXML::out(ChNameValue<unsigned long long> bVal) {
    indent();
    // if (is_array.top() == false)
    m_ostream << "<" << bVal.name() << ">";
    m_ostream << bVal.value();
    // if (is_array.top() == false)
    m_ostream << "</" << bVal.name() << ">\n";
    ++nitems.top();
}
void ChArchiveOutXML::out(ChNameValue<ChEnumMapperBase> bVal) {
    indent();
    // if (is_array.top() == false)
    m_ostream << "<" << bVal.name() << ">";
    std::string mstr = bVal.value().GetValueAsString();
    m_ostream << mstr;
    // if (is_array.top() == false)
    m_ostream << "</" << bVal.name() << ">\n";
    ++nitems.top();
}
void ChArchiveOutXML::out_array_pre(ChValue& bVal, size_t msize) {
    // if (is_array.top()==false)
    //{
    indent();
    m_ostream << "<" << bVal.name() << ">";
    //}
    m_ostream << "\n";

    ++tablevel;
    nitems.push(0);
    is_array.push(true);
}
void ChArchiveOutXML::out_array_between(ChValue& bVal, size_t msize) {}
void ChArchiveOutXML::out_array_end(ChValue& bVal, size_t msize) {
    --tablevel;
    nitems.pop();
    is_array.pop();

    // if (is_array.top() == false)
    //{
    indent();
    m_ostream << "</" << bVal.name() << ">\n";
    //}
    ++nitems.top();
}

// for custom c++ objects:

void ChArchiveOutXML::out(ChValue& bVal, bool tracked, size_t obj_ID) {
    // if (is_array.top()==false)
    //{
    indent();
    m_ostream << "<" << bVal.name();

    if (tracked) {
        m_ostream << " _object_ID=\"" << obj_ID << "\"";
    }

    m_ostream << ">\n";
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
    m_ostream << "</" << bVal.name() << ">\n";
    //}
}
void ChArchiveOutXML::out_ref(ChValue& bVal, bool already_inserted, size_t obj_ID, size_t ext_ID) {
    const char* classname = bVal.GetClassRegisteredName().c_str();

    // if (is_array.top() == false) {
    indent();
    m_ostream << "<" << bVal.name();

    if (strlen(classname) > 0) {
        m_ostream << " _type=\"" << classname << "\"";
    }

    if (!already_inserted) {
        m_ostream << " _object_ID=\"" << obj_ID << "\"";
    }
    if (already_inserted) {
        if (obj_ID || bVal.IsNull()) {
            m_ostream << " _reference_ID=\"" << obj_ID << "\"";
        }
        if (ext_ID) {
            m_ostream << " _external_ID=\"" << ext_ID << "\"";
        }
    }

    m_ostream << ">\n";
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
    m_ostream << "</" << bVal.name() << ">\n";
    //}
}
ChArchiveInXML::ChArchiveInXML(std::istream& stream_in) : m_istream(stream_in) {
    buffer.assign((std::istreambuf_iterator<char>(m_istream)), std::istreambuf_iterator<char>());
    buffer.push_back('\0');

    try {
        document.parse<0>(&buffer[0]);
    } catch (const rapidxml::parse_error& merror) {
        std::string line(merror.where<char>());
        line.erase(std::find_if(line.begin(), line.end(), [](int c) { return (c == *"\n"); }), line.end());
        throw std::invalid_argument(std::string("XML parsing error: ") + merror.what() + " at: \n" + line);
    }

    if (!document.first_node())
        throw std::runtime_error("The file is not a valid XML document");

    level = &document;  //.first_node();
    levels.push(level);
    is_array.push(false);

    can_tolerate_missing_tokens = true;
    try_tolerate_missing_tokens = false;
}

///////////////////////// ChArchiveInXML /////////////////////////

ChArchiveInXML::~ChArchiveInXML() {}
rapidxml::xml_node<>* ChArchiveInXML::GetValueFromNameOrArray(const std::string& mname) {
    rapidxml::xml_node<>* mnode = level->first_node(mname.c_str());
    if (!mnode)
        token_notfound(mname);
    return mnode;
}
bool ChArchiveInXML::in(ChNameValue<unsigned int> bVal) {
    rapidxml::xml_node<>* mval = GetValueFromNameOrArray(bVal.name());
    if (!mval)
        return false;
    try {
        bVal.value() = std::stoul(mval->value());
    } catch (...) {
        throw std::runtime_error("Invalid unsigned integer number after '" + std::string(bVal.name()) + "'");
    }
    return true;
}
bool ChArchiveInXML::in(ChNameValue<char> bVal) {
    rapidxml::xml_node<>* mval = GetValueFromNameOrArray(bVal.name());
    if (!mval)
        return false;
    try {
        bVal.value() = (char)std::stoi(mval->value());
    } catch (...) {
        throw std::runtime_error("Invalid char code after '" + std::string(bVal.name()) + "'");
    }
    return true;
}
bool ChArchiveInXML::in(ChNameValue<float> bVal) {
    rapidxml::xml_node<>* mval = GetValueFromNameOrArray(bVal.name());
    if (!mval)
        return false;
    try {
        bVal.value() = std::stof(mval->value());
    } catch (...) {
        throw std::runtime_error("Invalid number after '" + std::string(bVal.name()) + "'");
    }
    return true;
}
bool ChArchiveInXML::in(ChNameValue<double> bVal) {
    rapidxml::xml_node<>* mval = GetValueFromNameOrArray(bVal.name());
    if (!mval)
        return false;
    try {
        bVal.value() = std::stod(mval->value());
    } catch (...) {
        throw std::runtime_error("Invalid number after '" + std::string(bVal.name()) + "'");
    }
    return true;
}
bool ChArchiveInXML::in(ChNameValue<int> bVal) {
    rapidxml::xml_node<>* mval = GetValueFromNameOrArray(bVal.name());
    if (!mval)
        return false;
    try {
        bVal.value() = std::stoi(mval->value());
    } catch (...) {
        throw std::runtime_error("Invalid integer number after '" + std::string(bVal.name()) + "'");
    }
    return true;
}
bool ChArchiveInXML::in(ChNameValue<bool> bVal) {
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
    throw std::runtime_error("Invalid true/false flag after '" + std::string(bVal.name()) + "'");
}
bool ChArchiveInXML::in(ChNameValue<std::string> bVal) {
    rapidxml::xml_node<>* mval = GetValueFromNameOrArray(bVal.name());
    if (!mval)
        return false;
    bVal.value() = mval->value();
    return true;
}
bool ChArchiveInXML::in(ChNameValue<unsigned long> bVal) {
    rapidxml::xml_node<>* mval = GetValueFromNameOrArray(bVal.name());
    if (!mval)
        return false;
    try {
        bVal.value() = std::stoul(mval->value());
    } catch (...) {
        throw std::runtime_error("Invalid unsigned long integer number after '" + std::string(bVal.name()) + "'");
    }
    return true;
}
bool ChArchiveInXML::in(ChNameValue<unsigned long long> bVal) {
    rapidxml::xml_node<>* mval = GetValueFromNameOrArray(bVal.name());
    if (!mval)
        return false;
    try {
        bVal.value() = std::stoull(mval->value());
    } catch (...) {
        throw std::runtime_error("Invalid unsigned long long integer number after '" + std::string(bVal.name()) + "'");
    }
    return true;
}
bool ChArchiveInXML::in(ChNameValue<ChEnumMapperBase> bVal) {
    rapidxml::xml_node<>* mval = GetValueFromNameOrArray(bVal.name());
    if (!mval)
        return false;
    std::string mstr = mval->value();
    if (!bVal.value().SetValueAsString(mstr)) {
        throw std::runtime_error("Not recognized enum type '" + mstr + "'");
    }
    return true;
}

// for wrapping arrays and lists

bool ChArchiveInXML::in_array_pre(const std::string& name, size_t& msize) {
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
void ChArchiveInXML::in_array_between(const std::string& name) {
    ++this->array_index.top();
}
void ChArchiveInXML::in_array_end(const std::string& name) {
    this->levels.pop();
    this->level = this->levels.top();
    this->is_array.pop();
    this->array_index.pop();
}

//  for custom c++ objects:

bool ChArchiveInXML::in(ChNameValue<ChFunctorArchiveIn> bVal) {
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
            throw std::runtime_error("Invalid _object_ID in '" + std::string(bVal.name()) + "'");
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

bool ChArchiveInXML::in_ref(ChNameValue<ChFunctorArchiveIn> bVal, void** ptr, std::string& true_classname) {
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
                throw std::runtime_error("Invalid _reference_ID in '" + std::string(bVal.name()) + "'");
            }
            is_reference = true;
        }
        size_t ext_ID = 0;
        if (rapidxml::xml_attribute<>* midval = level->first_attribute("_external_ID")) {
            try {
                ext_ID = std::stoull(midval->value());
            } catch (...) {
                throw std::runtime_error("Invalid _external_ID in '" + std::string(bVal.name()) + "'");
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
                    throw std::runtime_error("Invalid _object_ID in '" + std::string(bVal.name()) + "'");
                }
            }

            if (new_ptr_void) {
                PutNewPointer(new_ptr_void, obj_ID);
                // 3) Deserialize
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

                bVal.value().SetRawPtr(ChCastingMap::Convert(true_classname, bVal.value().GetObjectPtrTypeindex(),
                                                             internal_id_ptr[ref_ID]));
            } else if (ext_ID) {
                if (this->external_id_ptr.find(ext_ID) == this->external_id_ptr.end()) {
                    throw std::runtime_error("In object '" + std::string(bVal.name()) + "' the _external_ID " +
                                             std::to_string((int)ext_ID) + " is not valid.");
                }
                bVal.value().SetRawPtr(ChCastingMap::Convert(true_classname, bVal.value().GetObjectPtrTypeindex(),
                                                             external_id_ptr[ext_ID]));
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
bool ChArchiveInXML::TryTolerateMissingTokens(bool try_tolerate) {
    try_tolerate_missing_tokens = try_tolerate;
    return try_tolerate_missing_tokens;
}
void ChArchiveInXML::token_notfound(const std::string& mname) {
    if (!try_tolerate_missing_tokens)
        throw std::runtime_error("Cannot find '" + mname + "'");
}
}  // end namespace chrono
