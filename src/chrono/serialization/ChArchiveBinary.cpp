#include "chrono/serialization/ChArchiveBinary.h"

namespace chrono {

ChArchiveOutBinary::ChArchiveOutBinary(std::ostream& stream_out) : m_ostream(stream_out) {}

ChArchiveOutBinary::~ChArchiveOutBinary() {}

void ChArchiveOutBinary::out(ChNameValue<unsigned int> bVal) {
    write(bVal.value());
}

void ChArchiveOutBinary::out(ChNameValue<char> bVal) {
    write(bVal.value());
}

void ChArchiveOutBinary::out(ChNameValue<float> bVal) {
    write(bVal.value());
}

void ChArchiveOutBinary::out(ChNameValue<double> bVal) {
    write(bVal.value());
}

void ChArchiveOutBinary::out(ChNameValue<int> bVal) {
    write(bVal.value());
}

void ChArchiveOutBinary::out(ChNameValue<bool> bVal) {
    write(bVal.value());
}

void ChArchiveOutBinary::out(ChNameValue<unsigned long> bVal) {
    write(bVal.value());
}

void ChArchiveOutBinary::out(ChNameValue<unsigned long long> bVal) {
    write(bVal.value());
}

void ChArchiveOutBinary::out(ChNameValue<ChEnumMapperBase> bVal) {
    int int_val = bVal.value().GetValueAsInt();
    write(int_val);
}

void ChArchiveOutBinary::out(ChNameValue<const char*> bVal) {
    write(bVal.value());
}

void ChArchiveOutBinary::out(ChNameValue<std::string> bVal) {
    write(bVal.value());
}

void ChArchiveOutBinary::out_array_pre(ChValue& bVal, size_t size) {
    write(size);
}

void ChArchiveOutBinary::out_array_between(ChValue& bVal, size_t size) {}

void ChArchiveOutBinary::out_array_end(ChValue& bVal, size_t size) {}

// for custom c++ objects:

void ChArchiveOutBinary::out(ChValue& bVal, bool tracked, size_t obj_ID) {
    if (tracked) {
        ////TODO: DARIOM here the original code uses const char* instead of std::string as in any other place
        write("typ");
        write(bVal.GetClassRegisteredName());

        write(std::string("oID"));
        write(obj_ID);
    }
    bVal.CallArchiveOut(*this);
}

void ChArchiveOutBinary::out_ref(ChValue& bVal, bool already_inserted, size_t obj_ID, size_t ext_ID) {
    std::string classname = bVal.GetClassRegisteredName();
    if (classname.length() > 0) {
        write(std::string("typ"));
        write(classname);
    }

    if (!already_inserted) {
        // new object, we have to full serialize it
        write(std::string("oID"));  // serialize 'this was already saved' info as "oID" string
        write(obj_ID);              // serialize obj_ID in pointers vector as ID
        bVal.CallArchiveOutConstructor(*this);
        bVal.CallArchiveOut(*this);
    } else {
        if (obj_ID || bVal.IsNull()) {
            // Object already in list. Only store obj_ID as ID
            write(std::string("rID"));  // serialize 'this was already saved' info as "rID" string
            write(obj_ID);              // serialize obj_ID in pointers vector as ID
        }
        if (ext_ID) {
            // Object is external. Only store ref_ID as ID
            write(std::string("eID"));  // serialize info as "eID" string
            write(ext_ID);              // serialize ext_ID in pointers vector as ID
        }
    }
}

template <>
std::ostream& ChArchiveOutBinary::write(std::string val) {
    ////TODO: int might not have the same sizeof in every architecture
    ////TODO: behaviour differ from char* because of the length!!!
    //// should be 'return write(val.c_str());'

    int str_len = (int)strlen(val.c_str());
    write(str_len);
    return m_ostream.write(val.c_str(), str_len);
}

template <>
std::ostream& ChArchiveOutBinary::write(bool val) {
    ////TODO: could be return m_ostream.write(val ? "1" : "0", 1);
    char bool_val = static_cast<char>(val);
    return m_ostream.write(&bool_val, 1);
}

template <>
std::ostream& ChArchiveOutBinary::write(const char* val) {
    int str_len = (int)strlen(val) + 1;
    write(str_len);
    return m_ostream.write(val, str_len);
}

// for wrapping arrays and lists
ChArchiveInBinary::ChArchiveInBinary(std::istream& stream_in) : m_istream(stream_in) {
    can_tolerate_missing_tokens = false;
}

ChArchiveInBinary::~ChArchiveInBinary() {}

bool ChArchiveInBinary::in(ChNameValue<unsigned int> bVal) {
    read(bVal.value());
    return true;
}

bool ChArchiveInBinary::in(ChNameValue<char> bVal) {
    read(bVal.value());
    return true;
}

bool ChArchiveInBinary::in(ChNameValue<float> bVal) {
    read(bVal.value());
    return true;
}

bool ChArchiveInBinary::in(ChNameValue<double> bVal) {
    read(bVal.value());
    return true;
}

////Can they merged in just one instance?

bool ChArchiveInBinary::in(ChNameValue<int> bVal) {
    read(bVal.value());
    return true;
}

bool ChArchiveInBinary::in(ChNameValue<bool> bVal) {
    char bool_char;
    m_istream.read(&bool_char, sizeof(char));
    bVal.value() = static_cast<bool>(bool_char);
    return true;
}

bool ChArchiveInBinary::in(ChNameValue<unsigned long> bVal) {
    read(bVal.value());
    return true;
}

bool ChArchiveInBinary::in(ChNameValue<unsigned long long> bVal) {
    read(bVal.value());
    return true;
}

bool ChArchiveInBinary::in_array_pre(const std::string& name, size_t& size) {
    read(size);
    return true;
}

bool ChArchiveInBinary::in_ref(ChNameValue<ChFunctorArchiveIn> bVal, void** ptr, std::string& true_classname) {
    void* new_ptr = nullptr;

    std::string entry;
    read(entry);

    if (entry == "typ") {
        read(true_classname);
        read(entry);
    }

    // TODO: DARIOM check when things are not found like in JSON and XML

    if (entry == "rID") {
        size_t ref_ID = 0;
        //  Was a shared object: just get the pointer to already-retrieved
        read(ref_ID);

        if (this->internal_id_ptr.find(ref_ID) == this->internal_id_ptr.end())
            throw std::runtime_error("In object '" + std::string(bVal.name()) + "' the reference ID " +
                                     std::to_string((int)ref_ID) + " is not a valid number.");

        bVal.value().SetRawPtr(
            ChCastingMap::Convert(true_classname, bVal.value().GetObjectPtrTypeindex(), internal_id_ptr[ref_ID]));
    } else if (entry == "eID") {
        size_t ext_ID = 0;
        // Was an external object: just get the pointer to external
        read(ext_ID);

        if (this->external_id_ptr.find(ext_ID) == this->external_id_ptr.end())
            throw std::runtime_error("In object '" + std::string(bVal.name()) + "' the external reference ID " +
                                     std::to_string((int)ext_ID) + " cannot be rebuilt.");

        bVal.value().SetRawPtr(
            ChCastingMap::Convert(true_classname, bVal.value().GetObjectPtrTypeindex(), external_id_ptr[ext_ID]));
    } else if (entry == "oID") {
        size_t obj_ID = 0;
        read(obj_ID);

        // see ChArchiveJSON for further details
        bVal.value().CallConstructor(*this, true_classname);

        void* new_ptr_void = bVal.value().GetRawPtr();

        if (new_ptr_void) {
            PutNewPointer(new_ptr_void, obj_ID);
            // 3) Deserialize
            bVal.value().CallArchiveIn(*this, true_classname);
        } else {
            throw std::runtime_error("Archive cannot create object" + true_classname);
        }
        new_ptr = bVal.value().GetRawPtr();
    }

    ////TODO: DARIOM check if it is needed
    ////else
    ////    bVal.value().SetRawPtr(nullptr);

    *ptr = new_ptr;
    return true;
}

//  for custom c++ objects:
bool ChArchiveInBinary::in(ChNameValue<ChFunctorArchiveIn> bVal) {
    if (bVal.flags() & NVP_TRACK_OBJECT) {
        std::string entry;
        size_t obj_ID = 0;

        // TODO: DARIOM check if it works this way
        read(entry);
        if (entry == "typ") {
            read(entry);
            read(entry);
        }
        if (entry == "oID") {
            read(obj_ID);
        }

        PutNewPointer(bVal.value().GetRawPtr(), obj_ID);
    }
    bVal.value().CallArchiveIn(*this);
    return true;
}

bool ChArchiveInBinary::in(ChNameValue<ChEnumMapperBase> bVal) {
    int foo;
    read(foo);
    bVal.value().SetValueAsInt(foo);
    return true;
}

bool ChArchiveInBinary::in(ChNameValue<std::string> bVal) {
    read(bVal.value());
    return true;
}

template <>
std::istream& ChArchiveInBinary::read(std::string& val) {
    ////TODO: DARIOM int might not have the same sizeof in every architecture

    val = "";
    int str_len = 0;
    read(str_len);
    val.reserve(str_len);

    ////TODO: DARIOM awkward way to skip \0 at the end of the string while still parsing the stream
    char buf[2];
    buf[1] = '\0';
    for (int i = 0; i < str_len; i++) {
        m_istream.read(buf, 1);
        val.append(buf);
    }

    return m_istream;
    // std::remove(val.begin(), val.end(), '\0');
}

template <>
std::istream& ChArchiveInBinary::read(bool& val) {
    char bool_val;
    m_istream.read(&bool_val, 1);
    val = static_cast<bool>(bool_val);
    return m_istream;
}

template <>
std::istream& ChArchiveInBinary::read(char*& val) {
    int str_len = 0;
    read(str_len);
    return m_istream.read(val, str_len);
}

}  // end namespace chrono