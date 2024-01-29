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

#ifndef CHARCHIVE_H
#define CHARCHIVE_H

#include <string>
#include <sstream>
#include <vector>
#include <list>
#include <typeindex>
#include <type_traits>
#include <unordered_set>
#include <memory>
#include <algorithm>

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChStream.h"
#include "chrono/core/ChClassFactory.h"
#include "chrono/core/ChTemplateExpressions.h"

namespace chrono {

/// @addtogroup chrono_serialization
/// @{

// forward reference
class ChArchiveOut;
class ChArchiveIn;

/// Macro to create a ChDetect_ArchiveInConstructor that can be used in
/// templates, to select which specialized template to use
// CH_CREATE_MEMBER_DETECTOR(ArchiveInConstructor) already defined in ChClassFactory

/// Macro to create a ChDetect_ArchiveOutConstructor that can be used in
/// templates, to select which specialized template to use
// CH_CREATE_MEMBER_DETECTOR(ArchiveOutConstructor) already defined in ChClassFactory

/// Macro to create a ChDetect_ArchiveOut that can be used in
/// templates, to select which specialized template to use
// CH_CREATE_MEMBER_DETECTOR(ArchiveOut) already defined in ChClassFactory

/// Macro to create a ChDetect_ArchiveIn that can be used in
/// templates, to select which specialized template to use
// CH_CREATE_MEMBER_DETECTOR(ArchiveIn) already defined in ChClassFactory

/// Macro to create a ChDetect_ArchiveContainerName that can be used in
/// templates, to select which specialized template to use
// CH_CREATE_MEMBER_DETECTOR(ArchiveContainerName) already defined in ChClassFactory

#define MAKE_GETSET_FUNPAIR(returnType, codeGet, codeSet)                       \
    std::make_pair(std::function<returnType()>([this]() -> returnType codeGet), \
                   std::function<void(returnType)>([this](returnType val) codeSet))

/// Exceptions for archives should inherit from this
class ChExceptionArchive : public ChException {
  public:
    ChExceptionArchive(std::string swhat) : ChException(swhat){};
};

/// Functor to call the ArchiveIn function for unrelated classes that
/// implemented them. This helps stripping out the templating, to make ChArchiveIn
/// easier and equippable with virtual functions.

class ChFunctorArchiveIn {
  public:
    virtual ~ChFunctorArchiveIn(){};

    /// Use this to call member function ArchiveIn.
    virtual void CallArchiveIn(ChArchiveIn& marchive, const std::string& classname) = 0;

    virtual void CallArchiveIn(ChArchiveIn& marchive) = 0;

    /// Use this to call
    /// - the default constructor, or,
    /// - the constructor embedded in an (optional) static member function ArchiveInConstructor, if implemented
    /// The latter is expected to a) deserialize constructor parameters, b) create a new obj as pt2Object = new
    /// myclass(params..). If classname not registered, call default constructor via new(T) or call construction via
    /// T::ArchiveInConstructor()
    virtual void CallConstructor(ChArchiveIn& marchive, const std::string& classname) = 0;

    /// Use this to create a new object as pt2Object = new myclass(), but using the class factory for polymorphic
    /// classes. If classname not registered, creates simply via new(T).
    virtual void CallConstructorDefault(const std::string& classname) = 0;

    /// Set the pointer (use static_cast)
    virtual void SetRawPtr(void* mptr) = 0;

    /// Get the pointer (use static_cast)
    virtual void* GetRawPtr() = 0;

    /// Tell if the pointed object is polymorphic
    virtual bool IsPolymorphic() = 0;

    /// Get the classname of the object holding the ArchiveIn function
    virtual std::type_index GetObjectPtrTypeindex() = 0;
};

template <class TClass>
class ChFunctorArchiveInSpecific : public ChFunctorArchiveIn {
  private:
    TClass* pt2Object;  // pointer to object

  public:
    // constructor - takes pointer to an object and pointer to a member
    ChFunctorArchiveInSpecific(TClass* _pt2Object) { pt2Object = _pt2Object; };

    virtual void CallArchiveIn(ChArchiveIn& marchive) override { this->_archive_in(marchive); }

    virtual void CallArchiveIn(ChArchiveIn& marchive, const std::string& classname) override {
        this->_archive_in(marchive, classname);
    }

    virtual void CallConstructor(ChArchiveIn& marchive, const std::string& classname) override {
        throw(ChExceptionArchive("Cannot call CallConstructor() for a constructed object."));
    }

    virtual void CallConstructorDefault(const std::string& classname) override {
        throw(ChExceptionArchive("Cannot call CallConstructorDefault() for a constructed object."));
    }

    virtual void SetRawPtr(void* mptr) override {
        throw(ChExceptionArchive("Cannot call SetRawPtr() for a constructed object."));
    }

    virtual void* GetRawPtr() override { return getVoidPointer<TClass>(pt2Object); };

    virtual bool IsPolymorphic() override {
        throw(ChExceptionArchive("Cannot call IsPolymorphic() for a constructed object."));
    }

    virtual std::type_index GetObjectPtrTypeindex() override { return std::type_index(typeid(pt2Object)); }

  private:
    template <class Tc = TClass>
    typename enable_if<ChDetect_ArchiveIn<Tc>::value, void>::type _archive_in(ChArchiveIn& marchive) {
        this->pt2Object->ArchiveIn(marchive);
    }

    template <class Tc = TClass>
    typename enable_if<!ChDetect_ArchiveIn<Tc>::value, void>::type _archive_in(ChArchiveIn& marchive) {}

    template <class Tc = TClass>
    typename enable_if<ChDetect_ArchiveIn<Tc>::value, void>::type _archive_in(ChArchiveIn& marchive,
                                                                              const std::string& classname) {
        if (ChClassFactory::IsClassRegistered(classname))
            ChClassFactory::archive_in(classname, marchive, pt2Object);
        else
            this->pt2Object->ArchiveIn(marchive);
    }

    template <class Tc = TClass>
    typename enable_if<!ChDetect_ArchiveIn<Tc>::value, void>::type _archive_in(ChArchiveIn& marchive,
                                                                               const std::string& classname) {
        if (ChClassFactory::IsClassRegistered(classname))
            ChClassFactory::archive_in(classname, marchive, pt2Object);
    }
};

template <class TClass>
class ChFunctorArchiveInSpecificPtr : public ChFunctorArchiveIn {
  private:
    TClass** pt2Object;  // pointer to pointer to object

  public:
    // constructor - takes pointer to an object and pointer to a member
    ChFunctorArchiveInSpecificPtr(TClass** _pt2Object) { pt2Object = _pt2Object; }

    virtual void CallArchiveIn(ChArchiveIn& marchive) override { this->_archive_in(marchive); }

    virtual void CallArchiveIn(ChArchiveIn& marchive, const std::string& classname) override {
        this->_archive_in(marchive, classname);
    }

    virtual void CallConstructor(ChArchiveIn& marchive, const std::string& classname) override {
        this->_constructor(marchive, classname);
    }

    virtual void CallConstructorDefault(const std::string& classname) override {
        this->_constructor_default(classname);
    }

    virtual void SetRawPtr(void* mptr) override { *pt2Object = static_cast<TClass*>(mptr); };

    virtual void* GetRawPtr() override { return getVoidPointer<TClass>(*pt2Object); };

    virtual bool IsPolymorphic() override { return this->_is_polymorphic(); };

    virtual std::type_index GetObjectPtrTypeindex() override { return std::type_index(typeid(*pt2Object)); };

  private:
    template <class Tc = TClass>
    typename enable_if<ChDetect_ArchiveInConstructor<Tc>::value, void>::type _constructor(
        ChArchiveIn& marchive,
        const std::string& classname) {
        // WARNING: as for the !ChDetect_ArchiveInConstructor, checking if the class of the pointe has
        // ArchiveInConstructor, does not guarantee that a derived class might have a normal constructor (even if more
        // corner case)
        if (ChClassFactory::IsClassRegistered(classname))
            ChClassFactory::create(classname, marchive, pt2Object);
        else
            *pt2Object = static_cast<Tc*>(Tc::ArchiveInConstructor(marchive));
    }

    template <class Tc = TClass>
    typename enable_if<!ChDetect_ArchiveInConstructor<Tc>::value, void>::type _constructor(
        ChArchiveIn& marchive,
        const std::string& classname) {
        // WARNING: DARIOM checking if Tc has an ArchiveInConstructor has little meaning;
        // in fact, a ChBody* will land to this function since it does not have ArchiveInConstructor,
        // however, if the ChBody* actually holds a ChBodyEasyBox, then the ArchiveInConstructor should be called
        // So, first check if the underlying object has it before rolling back to default constructor
        if (ChClassFactory::IsClassRegistered(classname))
            ChClassFactory::create(classname, marchive, pt2Object);
        else {
            this->CallConstructorDefault(classname);
        }
    }

    template <class Tc = TClass>
    typename enable_if<std::is_polymorphic<Tc>::value, bool>::type _is_polymorphic() {
        return true;
    }

    template <class Tc = TClass>
    typename enable_if<!std::is_polymorphic<Tc>::value, bool>::type _is_polymorphic() {
        return false;
    }

    template <class Tc = TClass>
    typename enable_if<std::is_default_constructible<Tc>::value && !std::is_abstract<Tc>::value, void>::type
    _constructor_default(const std::string& classname) {
        if (ChClassFactory::IsClassRegistered(classname))
            ChClassFactory::create(classname, pt2Object);
        else
            *pt2Object = new (TClass);
    }

    template <class Tc = TClass>
    typename enable_if<std::is_default_constructible<Tc>::value && std::is_abstract<Tc>::value, void>::type
    _constructor_default(const std::string& classname) {
        if (ChClassFactory::IsClassRegistered(classname))
            ChClassFactory::create(classname, pt2Object);
        else
            throw(ChExceptionArchive("Cannot call CallConstructorDefault(). Class  " + classname +
                                     " not registered, and is an abstract class."));
    }

    template <class Tc = TClass>
    typename enable_if<!std::is_default_constructible<Tc>::value, void>::type _constructor_default(
        const std::string& classname) {
        if (ChClassFactory::IsClassRegistered(classname))
            ChClassFactory::create(classname, pt2Object);
        else
            throw(ChExceptionArchive("Cannot call CallConstructorDefault(). Class " + classname +
                                     " not registered, and is without default constructor."));
    }

    template <class Tc = TClass>
    typename enable_if<ChDetect_ArchiveIn<Tc>::value, void>::type _archive_in(ChArchiveIn& marchive) {
        (*pt2Object)->ArchiveIn(marchive);
    }

    template <class Tc = TClass>
    typename enable_if<ChDetect_ArchiveIn<Tc>::value, void>::type _archive_in(ChArchiveIn& marchive,
                                                                              const std::string& classname) {
        if (!classname.empty() && ChClassFactory::IsClassRegistered(classname))
            ChClassFactory::archive_in(classname, marchive, *pt2Object);
        else
            _archive_in(marchive);
    }

    template <class Tc = TClass>
    typename enable_if<!ChDetect_ArchiveIn<Tc>::value, void>::type _archive_in(ChArchiveIn& marchive,
                                                                               const std::string& classname) {
        if (!classname.empty() && ChClassFactory::IsClassRegistered(classname))
            ChClassFactory::archive_in(classname, marchive, *pt2Object);
        else {
            // do nothing
        }
    }

    // template <class Tc=TClass>
    // typename enable_if< !ChDetect_ArchiveIn<Tc>::value, void >::type
    //_archive_in(ChArchiveIn& marchive) {
    //     //std::static_assert(true, "ArchiveIn() not provided in class to be deserialized.");
    // }
};

enum class ChCausalityType { parameter, calculatedParameter, input, output, local, independent };

enum class ChVariabilityType { constant, fixed, tunable, discrete, continuous };

/// Class to manage name-value pairs for serialization/deserialization.
template <class T>
class ChNameValue {
  public:
    ChNameValue(const std::string& mname,
                const T& mvalue,
                char mflags = 0,
                ChCausalityType causality = ChCausalityType::local,
                ChVariabilityType variability = ChVariabilityType::continuous)
        : _name(mname), _value((T*)(&mvalue)), _flags((char)mflags), _causality(causality), _variability(variability) {}

    ChNameValue(const ChNameValue<T>& other) {
        _name = other._name;
        _value = other._value;
        _flags = other._flags;
        _causality = other._causality;
        _variability = other._variability;
    }

    ~ChNameValue(){};

    const std::string& name() const { return this->_name; }

    char& flags() { return this->_flags; }
    const ChCausalityType& GetCausality() const { return this->_causality; }
    const ChVariabilityType& GetVariability() const { return this->_variability; }

    T& value() const { return *(this->_value); }

    const T& const_value() const { return *(this->_value); }

  protected:
    std::string _name;
    T* _value;
    char _flags;
    ChCausalityType _causality;
    ChVariabilityType _variability;
};

// Flag to mark a ChNameValue for a C++ object serialized by value but that
// that can be later referenced by pointer too.
static const char NVP_TRACK_OBJECT = (1 << 0);

template <class T>
ChNameValue<T> make_ChNameValue(const std::string& auto_name,
                                const T& t,
                                const std::string& custom_name,
                                char flags = 0,
                                ChCausalityType causality = ChCausalityType::local,
                                ChVariabilityType variability = ChVariabilityType::continuous) {
    auto name = custom_name.empty() ? auto_name : custom_name;
    return ChNameValue<T>(name, t, flags, causality, variability);
}

template <class T>
ChNameValue<T> make_ChNameValue(const std::string& auto_name,
                                const T& t,
                                char flags = 0,
                                ChCausalityType causality = ChCausalityType::local,
                                ChVariabilityType variability = ChVariabilityType::continuous) {
    return ChNameValue<T>(auto_name, t, flags, causality, variability);
}

// Macros to create ChNameValue objects easier

// utilities to avoid the ##__VA_ARGS__ that is not yet portable:
#define FIRST(...) FIRST_HELPER(__VA_ARGS__, throwaway)
#define FIRST_HELPER(first, ...) first
#define REST(...) REST_HELPER(NUM(__VA_ARGS__), __VA_ARGS__)
#define REST_HELPER(qty, ...) REST_HELPER2(qty, __VA_ARGS__)
#define REST_HELPER2(qty, ...) REST_HELPER_##qty(__VA_ARGS__)
#define REST_HELPER_ONE(first)
#define REST_HELPER_TWOORMORE(first, ...) , __VA_ARGS__
#define NUM(...)                                                                                                     \
    SELECT_10TH(__VA_ARGS__, TWOORMORE, TWOORMORE, TWOORMORE, TWOORMORE, TWOORMORE, TWOORMORE, TWOORMORE, TWOORMORE, \
                ONE, throwaway)
#define SELECT_10TH(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, ...) a10

#define STRINGIFY0(v) #v
#define STRINGIFY(v) STRINGIFY0(v)

/// Use this macro to mark a value, ex
///    myarchive << CHNVP (myvalue, "mnemonic name")
/// or, not providing the mnemonic name, the name will be get from the C name of the variable:
///    myarchive << CHNVP (myvalue)
#define CHNVP(...) make_ChNameValue("" STRINGIFY(FIRST(__VA_ARGS__)) "", FIRST(__VA_ARGS__) REST(__VA_ARGS__))

///////////////////////////////

/// Class to handle C++ values of generic type using type erasure and functors.
/// For example used to call the ArchiveOut function for unrelated classes that implemented them. This helps stripping
/// out the templating, to make ChArchiveOut easier and equippable with virtual functions.
class ChValue {
  public:
    virtual ~ChValue() {}

    virtual ChValue* new_clone() = 0;

    // Type helpers

    /// Get registered name in class factory.
    /// If type is not previously registered, returns a "" string.
    /// This is platform-independent.
    virtual std::string& GetClassRegisteredName() = 0;

    /// Get class version, if class version is registered, otherwise defaults to 0.
    virtual int GetClassRegisteredVersion() = 0;

    /// Get platform-dependent typeid name of referenced data.
    virtual const char* GetTypeidName() = 0;

    /// Get platform-dependent typeid of referenced data.
    virtual const std::type_index GetTypeid() = 0;

    /// Tell if it is a null pointer.
    virtual bool IsNull() = 0;

    /// Tell if the underlying original type is polymorphic.
    virtual bool IsPolymorphic() = 0;
    /// Tell if the underlying original type is an array.
    virtual bool IsArray() = 0;
    /// Tell if the underlying original type is a class.
    virtual bool IsClass() = 0;
    /// Tell if the underlying original type is a pointer.
    virtual bool IsPointer() = 0;

    /// Access the data by a raw pointer, given as static_cast
    virtual void* GetRawPtr() = 0;

    /// Get name of property
    const char* name() const { return this->_name.c_str(); }

    /// Get flags of property
    char& flags() { return this->_flags; }
    /// Get causality attribute
    const ChCausalityType& GetCausality() const { return this->_causality; }

    /// Get variability attribute
    const ChVariabilityType& GetVariability() const { return this->_variability; }

    //
    // Casting:
    //

    /// Use this to do safe dynamic cast.
    /// This uses the cast/throw trick for dynamic cast after type-erasure; note that this
    /// has performance penalty respect to usual dynamic_cast<>, which is not possible here.
    /// Note: can only upcast, but no downcast (i.e. U must be higher the TClass used
    /// when instantiating ChValueSpecific); otherwise use a later dynamic_cast<>.
    template <typename U>
    U* PointerUpCast() {
        try {
            this->thrower();
        } catch (U* ptr) {
            return static_cast<U*>(ptr);
        } catch (...) {
        }
        return 0;
    }

    //
    // Call members from names (without needing to have them in base classes)
    //

    /// Use this to call ArchiveOut member function.
    virtual void CallArchiveOut(ChArchiveOut& marchive) = 0;

    /// Use this to call (optional) member function ArchiveOutConstructor. This is
    /// expected to serialize constructor parameters if any.
    /// If ArchiveOutConstructor is not provided, simply does nothing.
    virtual void CallArchiveOutConstructor(ChArchiveOut& marchive) = 0;

    /// Tell if the object has the ArchiveContainerName() function; if so you might call CallArchiveContainerName
    virtual bool HasArchiveContainerName() = 0;

    /// Use this to call ArchiveContainerName member function, if present
    virtual std::string& CallArchiveContainerName() = 0;

    virtual void CallOut(ChArchiveOut& marchive) = 0;

  protected:
    virtual void thrower() const = 0;

    std::string _name;
    char _flags;
    ChCausalityType _causality;
    ChVariabilityType _variability;
};

template <class TClass>
class ChValueSpecific : public ChValue {
  private:
    TClass* _ptr_to_val;  // pointer to value
  public:
    // constructor
    ChValueSpecific(TClass& mvalp,
                    const std::string& mname,
                    char flags,
                    ChCausalityType causality = ChCausalityType::local,
                    ChVariabilityType variability = ChVariabilityType::continuous) {
        _ptr_to_val = (TClass*)(&mvalp);
        _name = mname;
        _flags = flags;
        _causality = causality;
        _variability = variability;
    };

    // copy constructor
    ChValueSpecific(ChNameValue<TClass> mval) {
        _ptr_to_val = mval.value();
        _name = mval.name();
        _flags = mval.flags();
        _causality = mval.GetCausality();
        _variability = mval.GetVariability();
    };

    virtual ChValue* new_clone() {
        return new ChValueSpecific<TClass>(*_ptr_to_val, _name, _flags, _causality, _variability);
    }

    virtual std::string& GetClassRegisteredName() {
        static std::string nostring("");
        if (!_ptr_to_val) {
            return nostring;
        }
        return ChClassFactory::IsClassRegistered(std::type_index(typeid(*_ptr_to_val)))
                   ? ChClassFactory::GetClassTagName(std::type_index(typeid(*_ptr_to_val)))
                   : nostring;
    }

    virtual int GetClassRegisteredVersion() { return chrono::class_factory::ChClassVersion<TClass>::version; }

    virtual const std::type_index GetTypeid() { return std::type_index(typeid(TClass)); }

    virtual const char* GetTypeidName() { return GetTypeid().name(); }

    virtual bool IsNull() { return (_ptr_to_val == 0); };

    virtual void* GetRawPtr() { return getVoidPointer<TClass>(_ptr_to_val); };

    virtual bool IsPolymorphic() { return std::is_polymorphic<TClass>::value; };

    virtual bool IsArray() { return std::is_array<TClass>::value; };

    virtual bool IsClass() { return std::is_class<TClass>::value; };

    virtual bool IsPointer() { return std::is_pointer<TClass>::value; };

    virtual bool HasArchiveContainerName() { return this->_has_get_name_string(); };

    virtual std::string& CallArchiveContainerName() { return this->_get_name_string(); }

    virtual void CallArchiveOut(ChArchiveOut& marchive) { this->_archive_out(marchive); }

    virtual void CallArchiveOutConstructor(ChArchiveOut& marchive) { this->_archive_out_constructor(marchive); }

    virtual void CallOut(ChArchiveOut& marchive);

  private:
    virtual void thrower() const { throw static_cast<TClass*>(_ptr_to_val); }

    template <class Tc = TClass>
    typename enable_if<ChDetect_ArchiveOutConstructor<Tc>::value, void>::type _archive_out_constructor(
        ChArchiveOut& marchive) {
        // while TClass refers to the type of the pointer
        // the underlying object might be of a more derived type
        // GetClassRegisteredName() will retrieve the most derived type
        std::string classname = GetClassRegisteredName();
        if (ChClassFactory::IsClassRegistered(classname))
            ChClassFactory::archive_out_constructor(classname, marchive, _ptr_to_val);
        else {
            this->_ptr_to_val->ArchiveOutConstructor(marchive);
        }
    }

    template <class Tc = TClass>
    typename enable_if<!ChDetect_ArchiveOutConstructor<Tc>::value, void>::type _archive_out_constructor(
        ChArchiveOut& marchive) {
        std::string classname = GetClassRegisteredName();
        if (ChClassFactory::IsClassRegistered(classname))
            ChClassFactory::archive_out_constructor(classname, marchive, _ptr_to_val);
        else {
            // do nothing
        }
    }

    template <class Tc = TClass>
    typename enable_if<ChDetect_ArchiveOut<Tc>::value, void>::type _archive_out(ChArchiveOut& marchive) {
        std::string classname = GetClassRegisteredName();
        if (ChClassFactory::IsClassRegistered(classname))
            ChClassFactory::archive_out(classname, marchive, _ptr_to_val);
        else {
            this->_ptr_to_val->ArchiveOut(marchive);
        }
        // also here we might want to check if some derived class had implemented ArchiveOut
    }

    template <class Tc = TClass>
    typename enable_if<!ChDetect_ArchiveOut<Tc>::value, void>::type _archive_out(ChArchiveOut& marchive) {
        std::string classname = GetClassRegisteredName();
        if (ChClassFactory::IsClassRegistered(classname))
            ChClassFactory::archive_out(classname, marchive, _ptr_to_val);
        else {
            // do nothing
        }
    }

    template <class Tc = TClass>
    typename enable_if<ChDetect_ArchiveContainerName<Tc>::value, bool>::type _has_get_name_string() {
        return true;
    }

    template <class Tc = TClass>
    typename enable_if<!ChDetect_ArchiveContainerName<Tc>::value, bool>::type _has_get_name_string() {
        return false;  // nothing to do if not provided
    }

    template <class Tc = TClass>
    typename enable_if<ChDetect_ArchiveContainerName<Tc>::value, std::string&>::type _get_name_string() {
        return this->_ptr_to_val->ArchiveContainerName();
    }

    template <class Tc = TClass>
    typename enable_if<!ChDetect_ArchiveContainerName<Tc>::value, std::string&>::type _get_name_string() {
        static std::string nostring("");
        return nostring;  // nothing to do if not provided
    }
};

/// Class for mapping enums to ChNameValue pairs that contain a 'readable' ascii string of the selected enum.
/// This could be used when streaming from/to human readable formats such as JSON or XML or ascii dumps.
class ChEnumMapperBase {
  public:
    ChEnumMapperBase() {}

    virtual int GetValueAsInt() = 0;
    virtual void SetValueAsInt(const int mval) = 0;

    virtual std::string GetValueAsString() = 0;
    virtual bool SetValueAsString(const std::string& mname) = 0;
};

template <class Te>
class ChEnumNamePair {
  public:
    ChEnumNamePair(const std::string& mname, Te menumid) : name(mname), enumid(menumid) {}

    std::string name;
    Te enumid;
};

template <class Te>
class ChEnumMapper : public ChEnumMapperBase {
  public:
    ChEnumMapper() : value_ptr(0) {
        enummap = std::shared_ptr<std::vector<ChEnumNamePair<Te>>>(new std::vector<ChEnumNamePair<Te>>);
    }

    ChEnumMapper(std::shared_ptr<std::vector<ChEnumNamePair<Te>>> mmap) : value_ptr(0), enummap(mmap) {}

    virtual ~ChEnumMapper() {}

    void AddMapping(const std::string& name, Te enumid) {
        ChEnumNamePair<Te> mpair(name, enumid);
        enummap->push_back(mpair);
    }
    void AddMapping(const std::string& auto_name, Te enumid, const std::string& custom_name) {
        auto name = custom_name.empty() ? auto_name : custom_name;
        ChEnumNamePair<Te> mpair(name, enumid);
        enummap->push_back(mpair);
    }

    Te& Value() { return *value_ptr; }

    virtual int GetValueAsInt() override { return static_cast<int>(*value_ptr); }
    virtual void SetValueAsInt(const int mval) override { *value_ptr = static_cast<Te>(mval); }

    virtual std::string GetValueAsString() override {
        for (size_t i = 0; i < enummap->size(); ++i) {
            if (enummap->at(i).enumid == *value_ptr)
                return enummap->at(i).name;
        }
        // not found, just string as number:
        return std::to_string(GetValueAsInt());
    }

    virtual bool SetValueAsString(const std::string& mname) override {
        for (size_t i = 0; i < enummap->size(); ++i) {
            if (enummap->at(i).name == mname) {
                *value_ptr = enummap->at(i).enumid;
                return true;
            }
        }
        // try to find from integer:
        int numb;
        std::istringstream mstream(mname);
        mstream >> numb;
        if (mstream.fail())
            return false;

        // Set value from number
        SetValueAsInt(numb);
        return true;
    }

    Te* value_ptr;

  protected:
    std::shared_ptr<std::vector<ChEnumNamePair<Te>>> enummap;
};

/// Three macros to simplify the use of enum mapper.
/// Use them always in sequence, with nothing else in between.
/// Possibly, use them just after you defined an enum.
/// After this, you have a class called "MyEnum_mapper", inherited
/// from ChEnumMapper, and you can use it for converting enums from/to strings.
/// Example:
///
/// enum eChMyenum {
///           ATHLETIC = 0,
///           SKINNY = 3,
///           FAT
///   };
///
///  CH_ENUM_MAPPER_BEGIN(MyEnum);
///   CH_ENUM_VAL(ATHLETIC);
///   CH_ENUM_VAL(SKINNY);
///   CH_ENUM_VAL(FAT, "fatty");  // overrides the "FAT" mapped string with "fatty"
///  CH_ENUM_MAPPER_END(MyEnum);
///
#define CH_ENUM_MAPPER_BEGIN(__enum_type)                           \
    class __enum_type##_mapper : public ChEnumMapper<__enum_type> { \
      public:                                                       \
        __enum_type##_mapper() {
#define CH_ENUM_VAL(...) this->AddMapping("" STRINGIFY(FIRST(__VA_ARGS__)) "", FIRST(__VA_ARGS__) REST(__VA_ARGS__));

#define CH_ENUM_MAPPER_END(__enum_type)                       \
    }                                                         \
    ;                                                         \
    ChEnumMapper<__enum_type> operator()(__enum_type& mval) { \
        ChEnumMapper<__enum_type> res(this->enummap);         \
        res.value_ptr = &mval;                                \
        return res;                                           \
    }                                                         \
    }                                                         \
    ;

//
// Wrapper class to ease the archival of std::pair
//

template <class T, class Tv>
class _wrap_pair {
  public:
    _wrap_pair(std::pair<T, Tv>& apair) { _wpair = &apair; }
    void ArchiveOut(ChArchiveOut& marchive) {
        marchive << CHNVP(_wpair->first, "1");
        marchive << CHNVP(_wpair->second, "2");
    }
    void ArchiveIn(ChArchiveIn& marchive) {
        marchive >> CHNVP(_wpair->first, "1");
        marchive >> CHNVP(_wpair->second, "2");
    }

  private:
    std::pair<T, Tv>* _wpair;
};

/// Base class for archives with pointers to shared objects.
class ChArchive {
  protected:
    bool cluster_class_versions;

    std::unordered_map<std::type_index, int> class_versions;

    bool use_versions;

  public:
    ChArchive() {
        use_versions = true;
        cluster_class_versions = true;
    }

    virtual ~ChArchive(){};

    /// Turn off version info in archives.
    /// By default, version numbers are saved in archives
    /// Either save/load both with version info, or not, do not mix because it could give problems in binary archives.
    void SetUseVersions(bool muse) { this->use_versions = muse; }

    /// If true, the version number is not saved in each class.
    /// Rather, it is saved only the first time that class is encountered.
    /// The same setting must be used for both serialization and deserialization.
    void SetClusterClassVersions(bool mcl) { this->cluster_class_versions = mcl; }
};

/// Base class for serializing into archives.
class ChArchiveOut : public ChArchive {
  protected:
    std::unordered_map<void*, size_t> internal_ptr_id;
    size_t currentID;

    std::unordered_map<void*, size_t> external_ptr_id;

    std::unordered_set<void*> cut_pointers;

    bool cut_all_pointers;

  public:
    ChArchiveOut() {
        cut_all_pointers = false;

        internal_ptr_id.clear();
        internal_ptr_id[nullptr] = (0);  // ID=0 -> null pointer.
        currentID = 0;
    };

    virtual ~ChArchiveOut(){};

    /// If you enable  SetCutAllPointers(true), no serialization happens for
    /// objects referenced via pointers. This can be useful to save a single object,
    /// regardless of the fact that it contains pointers to other 'children' objects.
    /// Cut pointers are turned into null pointers.
    void SetCutAllPointers(bool mcut) { this->cut_all_pointers = mcut; }

    /// Access the container of pointers that must not be serialized.
    /// This is in case SetCutAllPointers(true) is too extreme. So you can
    /// selectively 'cut' the network of pointers when serializing an object that
    /// has a network of sub objects. Works also for shared pointers, but remember to store
    /// the embedded pointer, not the shared pointer itself. For instance:
    ///    myarchive.CutPointers().insert(my_raw_pointer); // normal pointers
    ///    myarchive.CutPointers().insert(my_shared_pointer.get());  // shared pointers
    /// The cut pointers are serialized as null pointers.
    std::unordered_set<void*>& CutPointers() { return cut_pointers; }

    /// Use the following to declare pointer(s) that must not be de-serialized
    /// but rather be 'unbind' and be saved just as unique IDs.
    /// Note, the IDs can be whatever integer > 0. Use unique IDs per each pointer.
    /// Note, the same IDs must be used when de-serializing pointers in ArchiveIn.
    void UnbindExternalPointer(void* mptr, size_t ID) { external_ptr_id[mptr] = ID; }

    /// Use the following to declare pointer(s) that must not be de-serialized
    /// but rather be 'unbind' and be saved just as unique IDs.
    /// Note, the IDs can be whatever integer > 0. Use unique IDs per each pointer.
    /// Note, the same IDs must be used when de-serializing pointers in ArchiveIn.
    void UnbindExternalPointer(std::shared_ptr<void> mptr, size_t ID) { external_ptr_id[mptr.get()] = ID; }

  protected:
    /// Find a pointer in pointer map: eventually add it to map if it
    /// was not previously inserted. Returns already_stored=false if was
    /// already inserted. Return 'obj_ID' offset in vector in any case.
    /// For null pointers, always return 'already_stored'=true, and 'obj_ID'=0.
    void PutPointer(void* object, bool& already_stored, size_t& obj_ID) {
        if (this->internal_ptr_id.find(static_cast<void*>(object)) != this->internal_ptr_id.end()) {
            already_stored = true;
            obj_ID = internal_ptr_id[static_cast<void*>(object)];
            return;
        }

        // wasn't in list.. add to it
        ++currentID;
        obj_ID = currentID;
        internal_ptr_id[static_cast<void*>(object)] = obj_ID;
        already_stored = false;
        return;
    }

  public:
    //---------------------------------------------------
    // INTERFACES - to be implemented by children classes
    //

    // for integral types:
    virtual void out(ChNameValue<bool> bVal) = 0;
    virtual void out(ChNameValue<int> bVal) = 0;
    virtual void out(ChNameValue<double> bVal) = 0;
    virtual void out(ChNameValue<float> bVal) = 0;
    virtual void out(ChNameValue<char> bVal) = 0;
    virtual void out(ChNameValue<unsigned int> bVal) = 0;
    virtual void out(ChNameValue<std::string> bVal) = 0;
    virtual void out(ChNameValue<unsigned long> bVal) = 0;
    virtual void out(ChNameValue<unsigned long long> bVal) = 0;
    virtual void out(ChNameValue<ChEnumMapperBase> bVal) = 0;

    // handling methods (only for in-memory dump)
    template <class T>
    using FunGetSet = std::pair<std::function<T(void)>, std::function<void(T)>>;

    virtual void out(ChNameValue<FunGetSet<bool>> bVal) const {}
    virtual void out(ChNameValue<FunGetSet<int>> bVal) const {}
    virtual void out(ChNameValue<FunGetSet<double>> bVal) const {}
    virtual void out(ChNameValue<FunGetSet<float>> bVal) const {}
    virtual void out(ChNameValue<FunGetSet<char>> bVal) const {}
    virtual void out(ChNameValue<FunGetSet<unsigned int>> bVal) const {}
    virtual void out(ChNameValue<FunGetSet<const char*>> bVal) const {}
    virtual void out(ChNameValue<FunGetSet<unsigned long>> bVal) const {}
    virtual void out(ChNameValue<FunGetSet<unsigned long long>> bVal) const {}

    // for custom C++ objects - see 'wrapping' trick below
    virtual void out(ChValue& bVal, bool tracked, size_t obj_ID) = 0;

    // for pointed objects
    virtual void out_ref(ChValue& bVal, bool already_inserted, size_t obj_ID, size_t ext_ID) = 0;

    // for wrapping arrays and lists
    virtual void out_array_pre(ChValue& bVal, size_t msize) = 0;
    virtual void out_array_between(ChValue& bVal, size_t msize) = 0;
    virtual void out_array_end(ChValue& bVal, size_t msize) = 0;

    //---------------------------------------------------

    // trick to wrap enum mappers:
    template <class T>
    void out(ChNameValue<ChEnumMapper<T>> bVal) {
        ChNameValue<ChEnumMapperBase> tmpnv(bVal.name(), bVal.value(), bVal.flags(), bVal.GetCausality(),
                                            bVal.GetVariability());
        this->out(tmpnv);
    }

    // trick to wrap C++ fixed-size arrays:
    template <class T, size_t N>
    void out(ChNameValue<T[N]> bVal) {
        size_t arraysize = sizeof(bVal.value()) / sizeof(T);
        ChValueSpecific<T[N]> specVal(bVal.value(), bVal.name(), bVal.flags(), bVal.GetCausality(),
                                      bVal.GetVariability());
        this->out_array_pre(specVal, arraysize);
        for (size_t i = 0; i < arraysize; ++i) {
            ChNameValue<T> array_val(std::to_string(i), bVal.value()[i]);
            this->out(array_val);
            this->out_array_between(specVal, arraysize);
        }
        this->out_array_end(specVal, arraysize);
    }

    // trick to wrap std::vector container
    template <class T>
    void out(ChNameValue<std::vector<T>> bVal) {
        ChValueSpecific<std::vector<T>> specVal(bVal.value(), bVal.name(), bVal.flags(), bVal.GetCausality(),
                                                bVal.GetVariability());
        this->out_array_pre(specVal, bVal.value().size());
        for (size_t i = 0; i < bVal.value().size(); ++i) {
            ChNameValue<T> array_val(std::to_string(i), bVal.value()[i]);
            this->out(array_val);
            this->out_array_between(specVal, bVal.value().size());
        }
        this->out_array_end(specVal, bVal.value().size());
    }

    // trick to wrap std::list container
    template <class T>
    void out(ChNameValue<std::list<T>> bVal) {
        ChValueSpecific<std::list<T>> specVal(bVal.value(), bVal.name(), bVal.flags(), bVal.GetCausality(),
                                              bVal.GetVariability());
        this->out_array_pre(specVal, bVal.value().size());
        typename std::list<T>::iterator iter;
        size_t i = 0;
        for (iter = bVal.value().begin(); iter != bVal.value().end(); ++iter, ++i) {
            ChNameValue<T> array_val(std::to_string(i), (*iter));
            this->out(array_val);
            this->out_array_between(specVal, bVal.value().size());
        }
        this->out_array_end(specVal, bVal.value().size());
    }

    // trick to wrap std::pair container
    template <class T, class Tv>
    void out(ChNameValue<std::pair<T, Tv>> bVal) {
        _wrap_pair<T, Tv> mpair(bVal.value());
        ChNameValue<_wrap_pair<T, Tv>> pair_val(bVal.name(), mpair);
        this->out(pair_val);
    }

    // trick to wrap std::unordered_map container
    template <class T, class Tv>
    void out(ChNameValue<std::unordered_map<T, Tv>> bVal) {
        ChValueSpecific<std::unordered_map<T, Tv>> specVal(bVal.value(), bVal.name(), bVal.flags(), bVal.GetCausality(),
                                                           bVal.GetVariability());
        this->out_array_pre(specVal, bVal.value().size());
        int i = 0;
        for (auto it = bVal.value().begin(); it != bVal.value().end(); ++it) {
            ChNameValue<std::pair<T, Tv>> array_key(std::to_string(i), (*it));
            this->out(array_key);
            this->out_array_between(specVal, bVal.value().size());
            ++i;
        }
        this->out_array_end(specVal, bVal.value().size());
    }

    // trick to call out_ref on std::shared_ptr
    template <class T>
    void out(ChNameValue<std::shared_ptr<T>> bVal) {
        T* mptr = bVal.value().get();
        void* idptr = getVoidPointer<T>(mptr);

        if (this->cut_all_pointers || this->cut_pointers.find(idptr) != this->cut_pointers.end())
            return;

        bool already_stored = false;
        size_t obj_ID = 0;
        size_t ext_ID = 0;
        if (this->external_ptr_id.find(idptr) != this->external_ptr_id.end()) {
            already_stored = true;
            ext_ID = external_ptr_id[idptr];
        } else {
            PutPointer(idptr, already_stored, obj_ID);
        }
        ChValueSpecific<T> specVal(
            *mptr, bVal.name(), bVal.flags(), bVal.GetCausality(),
            bVal.GetVariability());  // TODO: mptr might be null if cut_all_pointers; double check if it is ok
        this->out_ref(specVal, already_stored, obj_ID,
                      ext_ID);  // note, this class name is not platform independent
    }

    // trick to call out_ref on raw pointers:
    template <class T>
    void out(ChNameValue<T*> bVal) {
        T* mptr = bVal.value();
        void* idptr = getVoidPointer<T>(mptr);

        if (this->cut_all_pointers || this->cut_pointers.find(idptr) != this->cut_pointers.end())
            return;

        bool already_stored = false;
        size_t obj_ID = 0;
        size_t ext_ID = 0;

        if (this->external_ptr_id.find(idptr) != this->external_ptr_id.end()) {
            already_stored = true;
            ext_ID = external_ptr_id[idptr];
        } else {
            PutPointer(idptr, already_stored, obj_ID);
        }
        ChValueSpecific<T> specVal(*mptr, bVal.name(), bVal.flags(), bVal.GetCausality(), bVal.GetVariability());
        this->out_ref(specVal, already_stored, obj_ID,
                      ext_ID);  // note, this class name is not platform independent
    }

    // trick to apply 'virtual out..' on remaining C++ object, that has a function "ArchiveOut"
    template <class T>
    void out(ChNameValue<T> bVal) {
        bool tracked = false;
        size_t obj_ID = 0;
        if (bVal.flags() & NVP_TRACK_OBJECT) {
            bool already_stored;
            T* mptr = &bVal.value();
            void* idptr = getVoidPointer<T>(mptr);
            PutPointer(idptr, already_stored, obj_ID);
            if (already_stored) {
                throw(ChExceptionArchive("Cannot serialize tracked object '" + std::string(bVal.name()) +
                                         "' by value, AFTER already serialized by pointer."));
            }
            tracked = true;
        }
        ChValueSpecific<T> specVal(bVal.value(), bVal.name(), bVal.flags(), bVal.GetCausality(), bVal.GetVariability());
        this->out(specVal, tracked, obj_ID);
    }

    /// Operator to allow easy serialization as myarchive << mydata;
    template <class T>
    ChArchiveOut& operator<<(ChNameValue<T> bVal) {
        this->out(bVal);
        return (*this);
    }

    // for backward compatibility
    void VersionWrite(int iv) {
        if (use_versions) {
            this->out(ChNameValue<int>("_version", iv));
        }
    }

    template <class T>
    void VersionWrite() {
        if (!use_versions)
            return;
        if (this->cluster_class_versions) {
            if (this->class_versions.find(std::type_index(typeid(T))) == this->class_versions.end()) {
                this->out_version(chrono::class_factory::ChClassVersion<T>::version, std::type_index(typeid(T)));
                this->class_versions[std::type_index(typeid(T))] = chrono::class_factory::ChClassVersion<T>::version;
            }
        } else {
            this->out_version(chrono::class_factory::ChClassVersion<T>::version, std::type_index(typeid(T)));
        }
    }

  protected:
    virtual void out_version(int mver, const std::type_index mtypeid) {
        if (use_versions) {
            std::string class_name = ChClassFactory::IsClassRegistered(mtypeid)
                                         ? ChClassFactory::GetClassTagName(mtypeid)
                                         : std::string(mtypeid.name());
            // to avoid troubles in xml archives
            std::replace(class_name.begin(), class_name.end(), '<', '[');
            std::replace(class_name.begin(), class_name.end(), '>', ']');
            std::replace(class_name.begin(), class_name.end(), ' ', '_');
            this->out(ChNameValue<int>(("_version_" + class_name).c_str(), mver));
        }
    }
};

/// Base class for deserializing from archives.
class ChArchiveIn : public ChArchive {
  protected:
    std::unordered_map<void*, size_t> internal_ptr_id;
    std::unordered_map<size_t, void*> internal_id_ptr;
    using shared_pair_type = std::pair<std::shared_ptr<void>, std::string>;
    std::unordered_map<void*, shared_pair_type> shared_ptr_map;

    /// container of pointers marker with external IDs to re-bind instead of de-serializing
    std::unordered_map<size_t, void*> external_id_ptr;

    bool can_tolerate_missing_tokens;
    bool try_tolerate_missing_tokens;

  public:
    ChArchiveIn() : can_tolerate_missing_tokens(false) {
        internal_ptr_id.clear();
        internal_ptr_id[nullptr] = 0;  // null pointer -> ID=0.
        internal_id_ptr.clear();
        internal_id_ptr[0] = nullptr;  // ID=0 -> null pointer.
    }

    virtual ~ChArchiveIn() {}

    /// Use the following to declare object IDs that must not be de-serialized
    /// but rather be 'rebind' to already-existing external pointers, given unique IDs.
    /// Note, the IDs can be whatever integer > 0. Use unique IDs per each pointer.
    /// Note, the same IDs must be used when serializing pointers in ArchiveOut.
    /// Note, there is no check on pointer types when rebinding!
    void RebindExternalPointer(void* mptr, size_t ID) { external_id_ptr[ID] = mptr; }

    /// Use the following to declare object IDs that must not be de-serialized
    /// but rather be 'rebind' to already-existing external shared pointers, given unique IDs.
    /// Note, the IDs can be whatever integer > 0. Use unique IDs per each pointer.
    /// Note, the same IDs must be used when serializing pointers in ArchiveOut.
    /// Note, there is no check on pointer types when rebinding!
    void RebindExternalPointer(std::shared_ptr<void> mptr, size_t ID) {
        external_id_ptr[ID] = mptr.get();
        // shared_ptr_map[mptr.get()] = shared_pair_type(mptr, std::type_index(typeid(void)));
        shared_ptr_map.emplace(std::make_pair(mptr.get(), shared_pair_type(mptr, "")));
    }

    /// Returns true if the class can tolerate missing tokes;
    /// on the contrary the archive must contain a complete set of info about the object that is going to be loaded
    bool CanTolerateMissingTokens() const { return can_tolerate_missing_tokens; }

    /// Suggest the archive to tolerate missing tokens/properties in the archive file;
    /// The function returns the new settings: for those classes for which CanTolerateMissingTokens returns false
    /// this function does not have any effect and will trivially return false.
    virtual bool TryTolerateMissingTokens(bool try_tolerate) { return try_tolerate_missing_tokens; }

    /// Returns true if the archive is currently tolerating missing tokens.
    bool GetTolerateMissingTokens() const { return try_tolerate_missing_tokens; }

  protected:
    /// Find a pointer in pointer map: eventually add it to map i f it
    /// was not previously inserted. Returns already_stored=false if was
    /// already inserted. Return 'obj_ID' offset in vector in any case.
    /// For null pointers, always return 'already_stored'=true, and 'obj_ID'=0.
    void PutNewPointer(void* object, size_t obj_ID) {
        if (internal_ptr_id.find(object) != internal_ptr_id.end() ||
            internal_id_ptr.find(obj_ID) != internal_id_ptr.end())
            throw(ChExceptionArchive("Expected to create a new object from pointer, but was already created."));

        internal_ptr_id[object] = obj_ID;
        internal_id_ptr[obj_ID] = object;
    }

  public:
    //---------------------------------------------------
    // INTERFACES - to be implemented by children classes
    //

    // for integral types:
    virtual bool in(ChNameValue<bool> bVal) = 0;
    virtual bool in(ChNameValue<int> bVal) = 0;
    virtual bool in(ChNameValue<double> bVal) = 0;
    virtual bool in(ChNameValue<float> bVal) = 0;
    virtual bool in(ChNameValue<char> bVal) = 0;
    virtual bool in(ChNameValue<unsigned int> bVal) = 0;
    virtual bool in(ChNameValue<std::string> bVal) = 0;
    virtual bool in(ChNameValue<unsigned long> bVal) = 0;
    virtual bool in(ChNameValue<unsigned long long> bVal) = 0;
    virtual bool in(ChNameValue<ChEnumMapperBase> bVal) = 0;

    // for custom C++ objects - see 'wrapping' trick below
    virtual bool in(ChNameValue<ChFunctorArchiveIn> bVal) = 0;

    // for pointed objects (return pointer of created object if new allocation, if reused just return null)
    virtual bool in_ref(ChNameValue<ChFunctorArchiveIn> bVal, void** ptr, std::string& true_classname) = 0;

    // for wrapping arrays and lists
    virtual bool in_array_pre(const std::string& name, size_t& msize) = 0;
    virtual void in_array_between(const std::string& name) = 0;
    virtual void in_array_end(const std::string& name) = 0;

    //---------------------------------------------------

    // trick to wrap enum mappers:
    template <class T>
    bool in(ChNameValue<ChEnumMapper<T>> bVal) {
        ChNameValue<ChEnumMapperBase> tmpnv(bVal.name(), bVal.value(), bVal.flags(), bVal.GetCausality(),
                                            bVal.GetVariability());
        return this->in(tmpnv);
    }

    // trick to wrap C++ fixed-size arrays:
    template <class T, size_t N>
    bool in(ChNameValue<T[N]> bVal) {
        size_t arraysize;
        if (!this->in_array_pre(bVal.name(), arraysize))
            return false;
        if (arraysize != sizeof(bVal.value()) / sizeof(T)) {
            throw(ChExceptionArchive("Size of [] saved array does not match size of receiver array " +
                                     std::string(bVal.name()) + "."));
        }
        for (size_t i = 0; i < arraysize; ++i) {
            T element;
            ChNameValue<T> array_val(std::to_string(i), element);
            this->in(array_val);
            bVal.value()[i] = element;
            this->in_array_between(bVal.name());
        }
        this->in_array_end(bVal.name());
        return true;
    }

    // trick to wrap std::vector container
    template <class T>
    bool in(ChNameValue<std::vector<T>> bVal) {
        bVal.value().clear();
        size_t arraysize;
        if (!this->in_array_pre(bVal.name(), arraysize))  // TODO: DARIOM check why it was commented out
            return false;
        bVal.value().resize(arraysize);
        for (size_t i = 0; i < arraysize; ++i) {
            T element;
            ChNameValue<T> array_val(std::to_string(i), element);
            this->in(array_val);
            bVal.value()[i] = element;
            this->in_array_between(bVal.name());
        }
        this->in_array_end(bVal.name());
        return true;
    }

    // trick to wrap std::list container
    template <class T>
    bool in(ChNameValue<std::list<T>> bVal) {
        bVal.value().clear();
        size_t arraysize;
        if (!this->in_array_pre(bVal.name(), arraysize))
            return false;
        for (size_t i = 0; i < arraysize; ++i) {
            T element;
            ChNameValue<T> array_val(std::to_string(i), element);
            this->in(array_val);
            bVal.value().push_back(element);
            this->in_array_between(bVal.name());
        }
        this->in_array_end(bVal.name());
        return true;
    }

    // trick to wrap std::pair container
    template <class T, class Tv>
    bool in(ChNameValue<std::pair<T, Tv>> bVal) {
        _wrap_pair<T, Tv> mpair(bVal.value());
        ChNameValue<_wrap_pair<T, Tv>> pair_val(bVal.name(), mpair);
        return this->in(pair_val);
    }

    // trick to wrap std::unordered_map container
    template <class T, class Tv>
    bool in(ChNameValue<std::unordered_map<T, Tv>> bVal) {
        bVal.value().clear();
        size_t arraysize;
        if (!this->in_array_pre(bVal.name(), arraysize))
            return false;
        for (size_t i = 0; i < arraysize; ++i) {
            std::pair<T, Tv> mpair;
            ChNameValue<std::pair<T, Tv>> array_val(std::to_string(i), mpair);
            this->in(array_val);
            // store in map; constant time:
            bVal.value()[mpair.first] = mpair.second;
            this->in_array_between(bVal.name());
        }
        this->in_array_end(bVal.name());
        return true;
    }

    // trick to call in_ref on ChSharedPointer:
    template <class T>
    bool in(ChNameValue<std::shared_ptr<T>> bVal) {
        T* mptr;
        ChFunctorArchiveInSpecificPtr<T> specFuncA(&mptr);
        ChNameValue<ChFunctorArchiveIn> mtmp(bVal.name(), specFuncA, bVal.flags(), bVal.GetCausality(),
                                             bVal.GetVariability());
        void* newptr;
        std::string true_classname;
        bool success = this->in_ref(mtmp, &newptr, true_classname);

        if (!success)
            return false;

        // 'mtmp' might contain a pointer to:
        // I) an object of type T
        // II) an object of type derived from T
        // III) null
        // (see ChArchiveJSON::in_ref for further details)

        // III) Get rid of nullptr immediately; no need to update shared_ptr_map
        if (!mptr) {
            bVal.value() = std::shared_ptr<T>(nullptr);
            return true;
        }

        // bVal must now be updated (in_ref just modified mtmp!)
        // Some additional complication respect to raw pointers:
        // it must properly increment shared count of shared pointers.
        void* true_ptr = getVoidPointer<T>(mptr);
        std::unordered_map<void*, shared_pair_type>::iterator existing_sh_ptr = shared_ptr_map.find(true_ptr);
        if (newptr || existing_sh_ptr == shared_ptr_map.end()) {
            // CASE A: newly created object or object created through raw pointers.
            //   mptr (== *mtmp._value.pt2Object) has already been properly converted to type T
            // no matter if case I) or case II)
            bVal.value() = std::shared_ptr<T>(mptr);
            // shared_ptr_map, on the contrary, needs to know the *true* address of the referred object
            // i.e. the address of the most derived type
            // and it holds a copy of a shared_prt so to be able to add to it if anyone else is going to refer to it
            shared_ptr_map.emplace(
                std::make_pair(true_ptr, std::make_pair(std::static_pointer_cast<void>(bVal.value()), true_classname)));
        } else {
            // case B: existing object already referenced by a shared_ptr, so increment ref count
            std::shared_ptr<void> converted_shptr =
                ChCastingMap::Convert(existing_sh_ptr->second.second, std::type_index(typeid(bVal.value().get())),
                                      existing_sh_ptr->second.first);  // TODO: DARIOM compact
            bVal.value() = std::static_pointer_cast<T>(converted_shptr);
        }

        return true;
    }

    // trick to call in_ref on raw pointers:
    template <class T>
    bool in(ChNameValue<T*> bVal) {
        ChFunctorArchiveInSpecificPtr<T> specFuncA(&bVal.value());
        void* newptr;
        std::string true_classname;
        return this->in_ref(ChNameValue<ChFunctorArchiveIn>(bVal.name(), specFuncA, bVal.flags(), bVal.GetCausality(),
                                                            bVal.GetVariability()),
                            &newptr, true_classname);
    }

    // trick to apply 'virtual in..' on C++ objects that has a function "ArchiveIn":
    template <class T>
    bool in(ChNameValue<T> bVal) {
        ChFunctorArchiveInSpecific<T> specFuncA(&bVal.value());
        return this->in(ChNameValue<ChFunctorArchiveIn>(bVal.name(), specFuncA, bVal.flags(), bVal.GetCausality(),
                                                        bVal.GetVariability()));
    }

    /// Operator to allow easy serialization as   myarchive << mydata;

    template <class T>
    ChArchiveIn& operator>>(ChNameValue<T> bVal) {
        this->in(bVal);
        return (*this);
    }

    // For backward compatibility
    int VersionRead() {
        if (use_versions) {
            int mver;
            this->in(ChNameValue<int>("_version", mver));
            return mver;
        }
        return 99999;
    }

    template <class T>
    int VersionRead() {
        int iv = 99999;
        if (!use_versions)
            return iv;
        if (this->cluster_class_versions) {
            if (this->class_versions.find(std::type_index(typeid(T))) == this->class_versions.end()) {
                int jv = this->in_version(typeid(T));
                this->class_versions[std::type_index(typeid(T))] = jv;
                return jv;
            }
        } else {
            return this->in_version(typeid(T));
        }
        return iv;
    }

  protected:
    virtual int in_version(const std::type_index mtypeid) {
        int mver;
        std::string class_name = ChClassFactory::IsClassRegistered(mtypeid) ? ChClassFactory::GetClassTagName(mtypeid)
                                                                            : std::string(mtypeid.name());
        // to avoid troubles in xml archives
        std::replace(class_name.begin(), class_name.end(), '<', '[');
        std::replace(class_name.begin(), class_name.end(), '>', ']');
        std::replace(class_name.begin(), class_name.end(), ' ', '_');
        this->in(ChNameValue<int>(("_version_" + class_name).c_str(), mver));
        return mver;
    }
};

template <class TClass>
void ChValueSpecific<TClass>::CallOut(ChArchiveOut& marchive) {
    marchive.out(CHNVP(*this->_ptr_to_val, this->_name.c_str()));
}

/// @} chrono_serialization

}  // end namespace chrono

#endif
