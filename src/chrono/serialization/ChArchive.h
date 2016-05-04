//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2012 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHARCHIVE_H
#define CHARCHIVE_H

#include <iostream>
#include <list>
#include <sstream>
#include <string>
#include <type_traits>
#include <typeinfo>
#include <unordered_set>
#include <utility>
#include <memory>
#include <vector>

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChStream.h"
#include "chrono/core/ChTemplateExpressions.h"

namespace chrono {

/// @addtogroup chrono_serialization
/// @{

// forward reference
class ChArchiveOut;
class ChArchiveIn;

/// Type after removing constness from a type.
template <class T>
using ChMutableT = typename std::remove_const<T>::type;


/// Macro to create a  ChDetect_GetRTTI  detector that can be used in 
/// templates, to select which specialized template to use depending if the
/// GetRTTI function is present in the class of type T.
CH_CREATE_MEMBER_DETECTOR(GetRTTI)



/// Exceptions for archives should inherit from this
class ChExceptionArchive : public ChException 
{
public:
    ChExceptionArchive(std::string swhat) : ChException(swhat){};
};



template<typename T>
struct ChArchiveOutMemberFunction {
    using type = void (T::*)(ChArchiveOut&) const;
    static constexpr type pointer = static_cast<type>(&T::ArchiveOUT);
};

template<typename T>
typename ChArchiveOutMemberFunction<T>::type
getArchiveOUT()
{
    return ChArchiveOutMemberFunction<T>::pointer;
}



/// Functor to call the ArchiveOUT function for unrelated classes that
/// implemented them. This helps stripping out the templating, to make ChArchiveOut
/// easier and equippable with virtual functions.

class ChFunctorArchiveOut {
public:
    virtual void CallArchiveOut(ChArchiveOut& marchive) const = 0;
    virtual bool IsNull() const = 0;
};

template <class TClass> 
class ChFunctorArchiveOutSpecific : public ChFunctorArchiveOut
{
private:
      using ArchiveOutMemberPtr =
          typename ChArchiveOutMemberFunction<TClass>::type;
      ArchiveOutMemberPtr fpt;         // pointer to member function
      TClass* pt2Object;               // pointer to object

public:

      // constructor - takes pointer to an object and pointer to a member
      ChFunctorArchiveOutSpecific(TClass* _pt2Object, ArchiveOutMemberPtr _fpt)
         { pt2Object = _pt2Object;  fpt=_fpt; };

      virtual void CallArchiveOut(ChArchiveOut& marchive) const
        { (*pt2Object.*fpt)(marchive);};             // execute member function

      virtual bool IsNull() const
        { return (pt2Object==0);};            

};


/// Functor to call the ArchiveIN function for unrelated classes that
/// implemented them. This helps stripping out the templating, to make ChArchiveOut
/// easier and equippable with virtual functions.

class ChFunctorArchiveIn {
public:
    virtual void CallArchiveIn(ChArchiveIn& marchive)=0;
    virtual void CallNew(ChArchiveIn& marchive) {};
    virtual void CallNewAbstract(ChArchiveIn& marchive, const char* classname) {};
    virtual void  CallSetRawPtr(ChArchiveIn& marchive, void* mptr) {};
    virtual void* CallGetRawPtr(ChArchiveIn& marchive) { return 0;};
};

template <class TClass> 
class ChFunctorArchiveInSpecific : public ChFunctorArchiveIn
{
private:
      void (TClass::*fpt)(ChArchiveIn&);   // pointer to member function
      TClass* pt2Object;                    // pointer to object

public:

      // constructor - takes pointer to an object and pointer to a member 
      ChFunctorArchiveInSpecific(TClass* _pt2Object, void(TClass::*_fpt)(ChArchiveIn&))
         { pt2Object = _pt2Object;  fpt=_fpt; };

      virtual void CallArchiveIn(ChArchiveIn& marchive)
        { (*pt2Object.*fpt)(marchive);};             // execute member function
};

template <class TClass> 
class ChFunctorArchiveInSpecificPtr : public ChFunctorArchiveIn
{
private:
      void (TClass::*fpt)(ChArchiveIn&);   // pointer to member function
      TClass** pt2Object;                    // pointer to object

public:

      // constructor - takes pointer to an object and pointer to a member 
      ChFunctorArchiveInSpecificPtr(TClass** _pt2Object, void(TClass::*_fpt)(ChArchiveIn&))
         { pt2Object = _pt2Object;  fpt=_fpt; };

      virtual void CallArchiveIn(ChArchiveIn& marchive)
        { (**pt2Object.*fpt)(marchive);};             // execute member function

      virtual void CallNew(ChArchiveIn& marchive)
        { *pt2Object = new(TClass); }

      virtual void  CallSetRawPtr(ChArchiveIn& marchive, void* mptr) 
        { *pt2Object = static_cast<TClass*>(mptr); };

      virtual void* CallGetRawPtr(ChArchiveIn& marchive) 
        { return static_cast<void*>(*pt2Object); };
};


template <class TClass> 
class ChFunctorArchiveInSpecificPtrAbstract : public ChFunctorArchiveIn
{
private:
      void (TClass::*fpt)(ChArchiveIn&);   // pointer to member function
      TClass** pt2Object;                    // pointer to object

public:

      // constructor - takes pointer to an object and pointer to a member 
      ChFunctorArchiveInSpecificPtrAbstract(TClass** _pt2Object, void(TClass::*_fpt)(ChArchiveIn&))
        { pt2Object = _pt2Object;  fpt=_fpt; };

      virtual void CallArchiveIn(ChArchiveIn& marchive)
        { (**pt2Object.*fpt)(marchive);};             // execute member function

      virtual void CallNewAbstract(ChArchiveIn& marchive, const char* classname)
        { ::chrono::create(classname, pt2Object); }

      virtual void  CallSetRawPtr(ChArchiveIn& marchive, void* mptr) 
        { *pt2Object = static_cast<TClass*>(mptr); };

      virtual void* CallGetRawPtr(ChArchiveIn& marchive) 
        { return static_cast<void*>(*pt2Object); };
};




/// So we can test if a ChNameValue is valid.
template<class T>
struct ChIsAssignable {
    using BaseT = typename std::remove_all_extents<T>::type;
    using LValueBaseT =  typename std::add_lvalue_reference<BaseT>::type;
    static constexpr bool value =
        std::is_assignable<LValueBaseT, BaseT>::value;
};

/// Check if a non-pointer object involves constness.
template<class T>
struct ChIsNonConst {
    static constexpr bool value = ! std::is_const<T>::value;
};

/// Check if there's any constness involved in a pointer.
/// int const****, int****const, int**const** all fail.
template<class T>
struct ChIsNonConst<T*> {
    static constexpr bool value = ChIsNonConst<T>::value;
};


#ifdef __GNUC__
#define DEPRECATED(func) __attribute__ ((deprecated)) func
#elif defined(_MSC_VER)
#define DEPRECATED(func) __declspec(deprecated) func
#else
#pragma message("WARNING: You need to implement DEPRECATED for this compiler")
#define DEPRECATED(func) func
#endif

///
/// Holds a value as an lvalue reference. See ChNameValue.
///
template<class T, bool isAssignable>
class  ChValue {
  public:
        ChValue(T& mvalue) : _value(mvalue) {}
        T& value() { return this->_value; }
  private:
        T& _value;
};

/// Holds a value as a copy.
/// Reading is okay, but writing gives a runtime warning.
template<class T>
class  ChValue<T, false> {
  public:
        using MutableT = ChMutableT<T>;
        ChValue(T& mvalue) : _value(mvalue) {}
        DEPRECATED(MutableT& value()) { return this->_value; }
  private:
        MutableT _value;
};


///
/// Holds a value as an lvalue reference. See ChNameValue.
///
template<class T>
class  ChValueOut {
  public:
        ChValueOut(T& mvalue) : _value(mvalue) {}
        ChValueOut(const T& mvalue) : _value(mvalue) {}
        const T& value() const { return this->_value; }

  private:
        const T& _value;
};


///
/// Base class for outputting name-value pairs
///
template<class T>
class ChNameValueOut {
  public:
        ChNameValueOut() = delete;

        ChNameValueOut(const char* mname, const T& mvalue, char mflags = 0) :
            _value(mvalue),
            _name(mname),
            _flags((char)mflags) {}

        ChNameValueOut(const ChNameValueOut&) = default;
        ChNameValueOut(ChNameValueOut&&) = default;
        ChNameValueOut& operator=(const ChNameValueOut&) = default;
        ChNameValueOut& operator=(ChNameValueOut&&) = default;

        virtual ~ChNameValueOut() = default;

        const char * name() const { return this->_name; }

        char& flags() { return this->_flags; }

        const T& value() const { return this->_value; }

  protected:
        const T& _value;
        const char* _name;
        char _flags;
};

///
/// Base class for inputting name-value pairs
///
template<class T>
class ChNameValueIn {
  public:
        ChNameValueIn() = delete;

        ChNameValueIn(const char* mname, T& mvalue, char mflags = 0) :
            _value(mvalue),
            _name(mname),
            _flags((char)mflags) {}

        ChNameValueIn(const ChNameValueIn&) = default;
        ChNameValueIn(ChNameValueIn&&) = default;
        ChNameValueIn& operator=(const ChNameValueIn&) = default;
        ChNameValueIn& operator=(ChNameValueIn&&) = default;

        virtual ~ChNameValueIn() = default;

        const char * name() const { return this->_name; }

        char& flags() { return this->_flags; }

        T& value() { return this->_value.value(); }

  protected:
        ChValue<T, ChIsNonConst<T>::value> _value;
        const char* _name;
        char _flags;
};


// Flag to mark a ChNameValue for a C++ object serialized by value but that
// that can be later referenced by pointer too.
static const char NVP_TRACK_OBJECT = (1 << 0);




/// Make a ChNameValueOut that captures an lvalue reference to the specified
/// assignable value. The name for the value is the custom name if that is not
/// null, or the auto name otherwise. Also @see CHNVP.
template<class T>
ChNameValueOut<T> make_ChNameValueOut(
    const char * auto_name,
    const T & value,
    const char * custom_name,
    char flags = 0)
{
    return ChNameValueOut<ChMutableT<T>> (
        (custom_name ? custom_name : auto_name), value, flags);
}

/// Three-argument version of make_ChNameValue. The custom_name is omitted.
template<class T>
ChNameValueOut<T> make_ChNameValueOut(
    const char * auto_name,
    const T & value,
    char flags = 0)
{
    return make_ChNameValueOut(auto_name, value, nullptr, flags);
}


/// Make a ChNameValueIn that captures an lvalue reference to the specified
/// assignable value. The name for the value is the custom name if that is not
/// null, or the auto name otherwise. Also @see CHNVP.
template<class T>
ChNameValueIn<T> make_ChNameValueIn(
    const char * auto_name,
    T & value,
    const char * custom_name,
    char flags = 0)
{
    return ChNameValueIn<T> (
        (custom_name ? custom_name : auto_name), value, flags);
}

/// Three-argument version of make_ChNameValue. The custom_name is omitted.
template<class T>
ChNameValueIn<T> make_ChNameValueIn(
    const char * auto_name,
    T & value,
    char flags = 0)
{
    return make_ChNameValueIn(auto_name, value, nullptr, flags);
}


/// Special case of make_ChNameValueOut for a const pointer
/// as an rvalue reference.
template<class T>
ChNameValueOut<T*> make_ChNameValueOut(
    const char * auto_name,
    const T* && value,
    const char * custom_name,
    char flags = 0)
{
    // Note that the value is not forwarded here.
    return ChNameValueOut<T*> (
        (custom_name ? custom_name : auto_name), value, flags);
}

/// Three argument version of make_ChNameValueOut for a pointer rvalue
/// reference.
template<class T>
ChNameValueOut<ChMutableT<T> *> make_ChNameValueOut(
    const char * auto_name,
    const T* && value,
    char flags = 0)
{
    // Note that the value *is* perfectly-forwarded here.
    return make_ChNameValueOut(
        auto_name, std::forward<const T*>(value), nullptr, flags);
}


/// Special case of make_ChNameValueIn for a T pointer as an rvalue reference.
template<class T>
ChNameValueIn<T*> make_ChNameValueIn(
    const char * auto_name,
    T* && value,
    const char * custom_name,
    char flags = 0)
{
    // Note that the value is not forwarded here.
    return ChNameValueIn<T*> (
        (custom_name ? custom_name : auto_name), value, flags);
}

/// Three argument version of make_ChNameValue for a T* rvalue reference.
template<class T>
ChNameValueIn<T*> make_ChNameValueIn(
    const char * auto_name,
    const T* && value,
    char flags = 0)
{
    // Note that the value *is* perfectly-forwarded here.
    return make_ChNameValueIn(
        auto_name, std::forward<T*>(value), nullptr, flags);
}




/// Macros to create ChNameValue objects easier

// utilities to avoid the ##__VA_ARGS__ that is not yet portable: 
#define FIRST(...) FIRST_HELPER(__VA_ARGS__, throwaway)
#define FIRST_HELPER(first, ...) first
#define REST(...) REST_HELPER(NUM(__VA_ARGS__), __VA_ARGS__)
#define REST_HELPER(qty, ...) REST_HELPER2(qty, __VA_ARGS__)
#define REST_HELPER2(qty, ...) REST_HELPER_##qty(__VA_ARGS__)
#define REST_HELPER_ONE(first)
#define REST_HELPER_TWOORMORE(first, ...) , __VA_ARGS__
#define NUM(...)  SELECT_10TH(__VA_ARGS__, TWOORMORE, TWOORMORE, TWOORMORE, TWOORMORE, TWOORMORE, TWOORMORE, TWOORMORE, TWOORMORE, ONE, throwaway)
#define SELECT_10TH(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, ...) a10

#define STRINGIFY0(v) #v
#define STRINGIFY(v) STRINGIFY0(v)

/// Use this macro to mark a value, ex  
///    myarchive << CHNVP (myvalue, "mnemonic name")
/// or, not providing the mnemonic name, the name will be get from the C name of the variable:
///    myarchive << CHNVP (myvalue)


#define CHNVP_OUT(...) \
    make_ChNameValueOut( \
        "" STRINGIFY(FIRST(__VA_ARGS__)) "", \
        FIRST(__VA_ARGS__) REST(__VA_ARGS__) )

#if 1
#define CHNVP_IN(...) \
    make_ChNameValueIn( \
        "" STRINGIFY(FIRST(__VA_ARGS__)) "", \
        FIRST(__VA_ARGS__) REST(__VA_ARGS__) )
#else
#define CHNVP_IN(...) make_ChNameValueIn( "" STRINGIFY(FIRST(__VA_ARGS__)) "", FIRST(__VA_ARGS__), typeid(FIRST(__VA_ARGS__)).name(), 0)
#endif



/// Class for mapping enums to ChNameValue pairs that contain a 'readable' ascii string
/// of the selected enum. This could be used when streaming from/to human readable formats
/// such as JSON or XML or ascii dumps.

class ChEnumMapperOutBase {
public:
    ChEnumMapperOutBase ()  {};

    virtual int  GetValueAsInt() const = 0;

    virtual std::string GetValueAsString() const = 0;

};

class ChEnumMapperInBase {
public:
    ChEnumMapperInBase ()  {};

    virtual void SetValueAsInt(int mval) = 0;

    virtual bool SetValueAsString(const std::string& mname) = 0;

};


template <class Te>
class ChEnumNamePair {
public:
    ChEnumNamePair(const char* mname, Te menumid) : name(mname), enumid(menumid) {}

    std::string name;
    Te enumid;
};


template <class Te>
class ChEnumMapper {
public:
    using MapType = std::shared_ptr<std::vector<ChEnumNamePair<Te>>>;

    ChEnumMapper () : enummap() {}

    ChEnumMapper (MapType mmap) : enummap(mmap) {}

    ChEnumMapper (const ChEnumMapper<Te>&) = default;

    void AddMapping(const char* name, Te enumid) {
        ChEnumNamePair<Te> mpair (name, enumid);
        enummap->push_back(mpair);
    }
    void AddMapping(const char* autoname, Te enumid, const char* custom_name) {
        const char* name = autoname;
        if (custom_name)
            name = custom_name;
        ChEnumNamePair<Te> mpair (name, enumid);
        enummap->push_back(mpair);
    }

    int enumToInt(Te mid) const {
        return static_cast<int>(mid);
    }

    std::string enumToString(Te mid) const {
        for (auto& elem : *enummap) {
            if (elem.enumid == mid) {
                return elem.name;
            }
        }
        // not found, just string as number:
        char buffer [10];
        sprintf(buffer, "%d", enumToInt(mid));
        return std::string(buffer);
    }

    Te intToEnum(int mval) const {
        return static_cast<Te>(mval);
    }

    bool stringToEnum(Te& mid, const std::string& mname) const {
        for (auto& elem : *enummap) {
            if (elem.name == mname) {
                mid = elem.enumid;
                return true;
            }
        }
        // not found, try to convert string to a number:
        int numb;
        std::istringstream mstream(mname);
        mstream >> numb;
        if (mstream.fail())
            return false;
        else{
            mid = intToEnum(numb);
            return true;
        }
    }

 protected:

    MapType enummap;
};


template <class Te>
class ChEnumMapperOut : public ChEnumMapperOutBase {
public:
    ChEnumMapperOut(const ChEnumMapper<Te>& mmap, const Te& mval) :
        map(mmap), value(mval)
    {}

    virtual int GetValueAsInt() const override {
        return map.enumToInt(value);
    }

    virtual std::string GetValueAsString() const override {
        return map.enumToString(value);
    }

protected:
    const ChEnumMapper<Te>& map;
    Te value;
};

template <class Te>
class ChEnumMapperIn : public ChEnumMapperInBase {
public:
    ChEnumMapperIn(const ChEnumMapper<Te>& mmap, Te& mval) :
        map(mmap), value(mval)
    {}

    virtual void SetValueAsInt(int mval) override {
        value = map.intToEnum(mval);
    }

    virtual bool SetValueAsString(const std::string& mname) override {
        return map.stringToEnum(value, mname);
    }

protected:
    const ChEnumMapper<Te>& map;
    Te& value;
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

#define CH_ENUM_MAPPER_BEGIN(EnumType) \
  class EnumType##_mapper : public ChEnumMapper< EnumType > { \
    public: \
      EnumType##_mapper() {

#define CH_ENUM_VAL(...) \
        this->AddMapping( \
          "" STRINGIFY(FIRST(__VA_ARGS__)) "", \
          FIRST(__VA_ARGS__) REST(__VA_ARGS__) );

#define CH_ENUM_MAPPER_END(EnumType) \
      }; \
      ChEnumMapperOut< EnumType > out (EnumType mval) { \
        return ChEnumMapperOut< EnumType > (*this, mval); \
      } \
      ChEnumMapperIn< EnumType > in (EnumType& mval) { \
        return ChEnumMapperIn< EnumType > (*this, mval); \
      } \
    };



/// Check if a ChEnumMapper is wrapped around a const.
template<class T>
struct ChIsNonConst<ChEnumMapper<T>> {
    static constexpr bool value = ChIsNonConst<T>::value;
};


/// Special case of make_ChNameValueOut for a ChEnumMapperOut<T> as an rvalue
/// reference.

template<class T>
ChNameValueOut<ChEnumMapperOut<T>>
make_ChNameValueOut(
    const char * auto_name,
    ChEnumMapperOut<T> && value,
    const char * custom_name,
    char flags = 0)
{
    // Note that the value is not forwarded here.
    return ChNameValueOut<ChEnumMapperOut<T>> (
        (custom_name ? custom_name : auto_name), value, flags);
}

/// Three argument version of make_ChNameValue for a ChEnumMapper<T> as an
/// rvalue reference.
template<class T>
ChNameValueOut<ChEnumMapperOut<T>> make_ChNameValueOut(
    const char * auto_name,
    ChEnumMapper<T> && value,
    char flags = 0)
{
    // Note that the value *is* perfectly-forwarded here.
    return make_ChNameValueOut(
        auto_name, std::forward<ChEnumMapperOut<T>>(value), nullptr, flags);
}


/// Special case of make_ChNameValueIn for a ChEnumMapperIn<T> as an rvalue
/// reference.

template<class T>
ChNameValueIn<ChEnumMapperIn<T>>
make_ChNameValueIn(
    const char * auto_name,
    ChEnumMapperIn<T> && value,
    const char * custom_name,
    char flags = 0)
{
    // Note that the value is not forwarded here.
    return ChNameValueIn<ChEnumMapperIn<T>> (
        (custom_name ? custom_name : auto_name), value, flags);
}

/// Three argument version of make_ChNameValue for a ChEnumMapper<T> as an
/// rvalue reference.
template<class T>
ChNameValueIn<ChEnumMapperIn<T>> make_ChNameValueIn(
    const char * auto_name,
    ChEnumMapper<T> && value,
    char flags = 0)
{
    // Note that the value *is* perfectly-forwarded here.
    return make_ChNameValueIn(
        auto_name, std::forward<ChEnumMapperIn<T>>(value), nullptr, flags);
}



///
/// This is a base class for archives with pointers to shared objects 
///

class ChArchive {
  protected:

    /// vector of pointers to stored/retrieved objects,
    /// to avoid saving duplicates or deadlocks
    std::vector<void*> objects_pointers;

    /// container of pointers to not serialize if ever encountered
    std::unordered_set<void*>  cut_pointers;

    bool use_versions;
    bool cut_all_pointers;

  public:
    ChArchive() {
        use_versions = true;
        cut_all_pointers = false;
        Init();
    }

    virtual ~ChArchive() {};

    /// Reinitialize the vector of pointers to loaded/saved objects
    void Init() {
        objects_pointers.clear();
        objects_pointers.push_back(0); // objects_pointers[0] for null pointer.
    }
    /// Find a pointer in pointer vector: eventually add it to vecor if it
    /// was not previously inserted. Returns already_stored=false if was
    /// already inserted. Return 'pos' offset in vector in any case.
    /// For null pointers, always return 'already_stored'=true, and 'pos'=0.
    void PutPointer(void* object, bool& already_stored, size_t& pos) {
        for (size_t i = 0; i < objects_pointers.size(); ++i) {
            if (objects_pointers[i] == object)
            {
                already_stored = true;
                pos = i;
                return;
            }
        }
        // wasn't in list.. add to it
        objects_pointers.push_back(object);

        already_stored = false;
        pos = objects_pointers.size()-1;
        return;
    }

    /// By default, version numbers are saved in archives
    /// Use this to turn off version info in archives (either save/load both
    /// with version info, or not, do not mix because it could give problems in binary archives.).
    void SetUseVersions(bool muse) {this->use_versions = muse;}

    /// If you enable  SetCutAllPointers(true), no serialization happens for 
    /// objects referenced via pointers. This can be useful to save a single object, 
    /// regardless of the fact that it contains pointers to other 'children' objects.
    /// Cut pointers are turned into null pointers.
    void SetCutAllPointers(bool mcut) {this->cut_all_pointers = mcut;}

    /// Access the container of pointers that must not be serialized.
    /// This is in case SetCutAllPointers(true) is too extreme. So you can 
    /// selectively 'cut' the network of pointers when serializing an object that
    /// has a network of sub objects. Works also for shared pointers, but remember to store
    /// the embedded pointer, not the shared pointer itself. For instance:
    ///    myarchive.CutPointers().insert(my_raw_pointer); // normal pointers
    ///    myarchive.CutPointers().insert(my_shared_pointer.get());  // shared pointers
    /// The cut pointers are serialized as null pointers.
    std::unordered_set<void*>&  CutPointers() {return cut_pointers;}
};



///
/// This is a base class for serializing into archives
///

class  ChArchiveOut : public ChArchive {
  public:
      virtual ~ChArchiveOut() {};

      //---------------------------------------------------
      // INTERFACES - to be implemented by children classes
      //

        // for integral types:
      virtual void out     (ChNameValueOut<bool> bVal) = 0;
      virtual void out     (ChNameValueOut<int> bVal) = 0;
      virtual void out     (ChNameValueOut<double> bVal) = 0;
      virtual void out     (ChNameValueOut<float> bVal) = 0;
      virtual void out     (ChNameValueOut<char> bVal) = 0;
      virtual void out     (ChNameValueOut<unsigned int> bVal) =0;
      virtual void out     (ChNameValueOut<std::string> bVal) = 0;
      virtual void out     (ChNameValueOut<unsigned long> bVal) = 0;
      virtual void out     (ChNameValueOut<unsigned long long> bVal) =0;
      virtual void out     (ChNameValueOut<ChEnumMapperOutBase> bVal) =0;

        // for custom C++ objects - see 'wrapping' trick below
      virtual void out (
          ChNameValueOut<ChFunctorArchiveOut> bVal,
          const char* classname, bool tracked, size_t position) = 0;

        // for pointed objects with abstract class system (i.e. supporting class factory)
      virtual void out_ref_abstract (
          ChNameValueOut<ChFunctorArchiveOut> bVal,
          bool already_inserted, size_t position, const char* classname) = 0;
      
        // for pointed objects without class abstraction
      virtual void out_ref (
          ChNameValueOut<ChFunctorArchiveOut> bVal,
          bool already_inserted, size_t position, const char* classname) = 0;

        // for wrapping arrays and lists
      virtual void out_array_pre (const char* name, size_t msize, const char* classname) = 0;
      virtual void out_array_between (size_t msize, const char* classname) = 0;
      virtual void out_array_end (size_t msize,const char* classname) = 0;


      //---------------------------------------------------

           // trick to wrap enum mappers:
      template<class T>
      void out (ChNameValueOut< ChEnumMapperOut<T> > bVal)
      {
          ChNameValueOut< ChEnumMapperOutBase > tmpnv(bVal.name(),bVal.value(),bVal.flags());
          this->out(tmpnv);
      }

        // trick to wrap C++ fixed-size arrays:
      template<class T, size_t N>
      void out     (ChNameValueOut<T[N]> bVal) {
          size_t arraysize = sizeof(bVal.value())/sizeof(T);
          const auto& type_name =  typeid(bVal.value()).name();

          this->out_array_pre(bVal.name(), arraysize, typeid(T).name());

          unsigned long idx = 0;
          for (auto& elem : bVal.value())
          {
              char buffer[20];
              sprintf(buffer, "el_%lu", idx);
              ChNameValueOut< ChMutableT<T> > array_val(
                  buffer, const_cast<ChMutableT<T>&>(elem));
              this->out (array_val);
              this->out_array_between(arraysize, type_name);
              ++idx;
          }
          this->out_array_end(arraysize, typeid(bVal.value()).name());
      }

        // trick to wrap stl::vector container
      template<class T>
      void out     (ChNameValueOut< std::vector<T> > bVal) {
          size_t arraysize = bVal.value().size();
          const auto& type_name =  typeid(bVal.value()).name();

          this->out_array_pre(bVal.name(), arraysize, typeid(T).name());

          unsigned long idx = 0;
          for (auto& elem : bVal.value())
          {
              char buffer[20];
              sprintf(buffer, "el_%lu", idx);
              ChNameValueOut< ChMutableT<T> > array_val(
                  buffer, const_cast<ChMutableT<T>&>(elem));
              this->out (array_val);
              this->out_array_between(arraysize, type_name);
              ++idx;
          }
          this->out_array_end(arraysize, type_name);
      }
        // trick to wrap stl::list container
      template<class T>
      void out     (ChNameValueOut< std::list<T> > bVal) {
          size_t arraysize = bVal.value().size();
          const auto& type_name =  typeid(bVal.value()).name();

          this->out_array_pre(bVal.name(), arraysize, typeid(T).name());

          unsigned long idx = 0;
          for (auto& elem : bVal.value())
          {
              char buffer[20];
              sprintf(buffer, "el_%lu", idx);
              ChNameValueOut< ChMutableT<T> > array_val(
                  buffer, const_cast<ChMutableT<T>&>(elem));
              this->out (array_val);
              this->out_array_between(arraysize, type_name);
              ++idx;
          }
          this->out_array_end(arraysize, type_name);
      }
     


        // trick to call out_ref on ChSharedPointer, with class abstraction:
      template<class T>
      typename enable_if< ChDetect_GetRTTI<T>::value >::type
      out     (ChNameValueOut< std::shared_ptr<T> > bVal) {
          bool already_stored; size_t pos;
          T* mptr = bVal.value().get();
          if (this->cut_all_pointers)
              mptr = 0;
          if (this->cut_pointers.find((void*)mptr) != this->cut_pointers.end())
              mptr = 0;
          PutPointer((void*)mptr, already_stored, pos);
          ChFunctorArchiveOutSpecific<T> specFuncA(mptr, getArchiveOUT<T>());
          const char* class_name;
          if (bVal.value())
              class_name = bVal.value()->GetRTTI()->GetName();
          else // null ptr
              class_name = typeid(T).name(); // note, this class name is not platform independent but enough here since for null ptr
          this->out_ref_abstract(
              ChNameValueOut<ChFunctorArchiveOut>(bVal.name(), specFuncA, bVal.flags()), 
              already_stored, 
              pos, 
              class_name );
      }

        // trick to call out_ref on ChSharedPointer, without class abstraction:
      template<class T>
      typename enable_if< !ChDetect_GetRTTI<T>::value >::type 
      out     (ChNameValueOut< std::shared_ptr<T> > bVal) {
          bool already_stored; size_t pos;
          T* mptr = bVal.value().get();
          if (this->cut_all_pointers)
              mptr = 0;
          if (this->cut_pointers.find((void*)mptr) != this->cut_pointers.end())
              mptr = 0;
          PutPointer((void*)mptr, already_stored, pos);
          ChFunctorArchiveOutSpecific<T> specFuncA(mptr, getArchiveOUT<T>());
          this->out_ref(
              ChNameValueOut<ChFunctorArchiveOut>(bVal.name(), specFuncA, bVal.flags()), 
              already_stored, 
              pos, 
              typeid(T).name() ); // note, this class name is not platform independent
      }

        // trick to call out_ref on plain pointers, with class abstraction:
      template<class T>
      typename enable_if< ChDetect_GetRTTI<T>::value >::type
      out     (ChNameValueOut<T*> bVal) {
          bool already_stored; size_t pos;
          T* mptr = bVal.value();
          if (this->cut_all_pointers)
              mptr = 0;
          if (this->cut_pointers.find((void*)mptr) != this->cut_pointers.end())
              mptr = 0;
          PutPointer((void*)mptr, already_stored, pos);
          ChFunctorArchiveOutSpecific<T> specFuncA(mptr, getArchiveOUT<T>());
          const char* class_name;
          if (bVal.value())
              class_name = bVal.value()->GetRTTI()->GetName();
          else // null ptr
              class_name = typeid(T).name(); // note, this class name is not platform independent but enough here since for null ptr
          this->out_ref_abstract(
              ChNameValueOut<ChFunctorArchiveOut>(bVal.name(), specFuncA, bVal.flags()), 
              already_stored,
              pos, 
              class_name ); // this class name is platform independent
      }

         // trick to call out_ref on plain pointers (no class abstraction):
      template<class T>
      typename enable_if< !ChDetect_GetRTTI<T>::value >::type 
      out     (ChNameValueOut<T*> bVal) {
          bool already_stored; size_t pos;
          T* mptr = bVal.value();
          if (this->cut_all_pointers)
              mptr = 0;
          if (this->cut_pointers.find((void*)mptr) != this->cut_pointers.end())
              mptr = 0;
          PutPointer((void*)mptr, already_stored, pos);
          ChFunctorArchiveOutSpecific<T> specFuncA(mptr, getArchiveOUT<T>());
          this->out_ref(
              ChNameValueOut<ChFunctorArchiveOut>(bVal.name(), specFuncA, bVal.flags()), 
              already_stored,
              pos, 
              typeid(T).name() ); // note, this class name is not platform independent
      }

        // trick to apply 'virtual out..' on remaining C++ object, that has a function "ArchiveOUT":
      template<class T>
      void out     (ChNameValueOut<T> bVal) {
          bool tracked = false;
          size_t pos =0;
          T* mptr = const_cast<T*>(&bVal.value());
          if (bVal.flags() & NVP_TRACK_OBJECT)
          {
              bool already_stored; 
              PutPointer((void*) mptr, already_stored, pos);
              if (already_stored) 
                  {throw (ChExceptionArchive( "Cannot serialize tracked object '" + std::string(bVal.name()) + "' by value, AFTER already serialized by pointer."));}
              tracked = true;
          }
          ChFunctorArchiveOutSpecific<T> specFuncA(mptr, getArchiveOUT<T>());
          this->out(
              ChNameValueOut<ChFunctorArchiveOut>(bVal.name(), specFuncA, bVal.flags()), 
              typeid(T).name(),
              tracked, pos);
      }

        /// Operator to allow easy serialization as   myarchive >> mydata;
      
      template<class T>
      ChArchiveOut& operator<<(ChNameValueOut<T> bVal) {
          this->out(bVal);
          return (*this);
      }
      
      void VersionWrite(int mver) {
          if (use_versions)
            this->out(ChNameValueOut<int>("version",mver));
      }
      
  protected:
      
};



///
/// This is a base class for serializing from archives
///

class  ChArchiveIn : public ChArchive {
  public:
      virtual ~ChArchiveIn() {};

      //---------------------------------------------------
      // INTERFACES - to be implemented by children classes
      //

        // for integral types:
      virtual void in     (ChNameValueIn<bool> bVal) = 0;
      virtual void in     (ChNameValueIn<int> bVal) = 0;
      virtual void in     (ChNameValueIn<double> bVal) = 0;
      virtual void in     (ChNameValueIn<float> bVal) = 0;
      virtual void in     (ChNameValueIn<char> bVal) = 0;
      virtual void in     (ChNameValueIn<unsigned int> bVal) = 0;
      virtual void in     (ChNameValueIn<std::string> bVal) = 0;
      virtual void in     (ChNameValueIn<unsigned long> bVal) = 0;
      virtual void in     (ChNameValueIn<unsigned long long> bVal) = 0;
      virtual void in     (ChNameValueIn<ChEnumMapperInBase> bVal) =0;

        // for custom C++ objects - see 'wrapping' trick below
      virtual void in     (ChNameValueIn<ChFunctorArchiveIn> bVal) = 0;

        // for pointed objects 
      virtual void in_ref_abstract (ChNameValueIn<ChFunctorArchiveIn> bVal) = 0;
      virtual void in_ref          (ChNameValueIn<ChFunctorArchiveIn> bVal) = 0;

        // for wrapping arrays and lists
      virtual void in_array_pre (const char* name, size_t& msize) = 0;
      virtual void in_array_between (const char* name) = 0;
      virtual void in_array_end (const char* name) = 0;

      //---------------------------------------------------

           // trick to wrap enum mappers:
      template<class T>
      void in     (ChNameValueIn< ChEnumMapperIn<T> > bVal) {
          ChNameValueIn< ChEnumMapperInBase > tmpnv(bVal.name(),bVal.value(),bVal.flags());
          this->in(tmpnv);
      }

             // trick to wrap C++ fixed-size arrays:
      template<class T, size_t N>
      void in     (ChNameValueIn<T[N]> bVal) {
          size_t arraysize;
          this->in_array_pre(bVal.name(), arraysize);
          if (arraysize != sizeof(bVal.value())/sizeof(T) ) {throw (ChExceptionArchive( "Size of [] saved array does not match size of receiver array " + std::string(bVal.name()) + "."));}
          for (size_t i = 0; i<arraysize; ++i)
          {
              char idname[20];
              sprintf(idname, "el_%lu", (unsigned long)i);
              T element;
              ChNameValueIn< T > array_val(idname, element);
              this->in (array_val);
              bVal.value()[i]=element;
              this->in_array_between(bVal.name());
          }
          this->in_array_end(bVal.name());
      }

             // trick to wrap stl::vector container
      template<class T>
      void in     (ChNameValueIn< std::vector<T> > bVal) {
          bVal.value().clear();
          size_t arraysize;
          this->in_array_pre(bVal.name(), arraysize);
          bVal.value().resize(arraysize);
          for (size_t i = 0; i<arraysize; ++i)
          {
              char idname[20];
              sprintf(idname, "el_%lu", (unsigned long)i);
              T element;
              ChNameValueIn< T > array_val(idname, element);
              this->in (array_val);
              bVal.value()[i]=element;
              this->in_array_between(bVal.name());
          }
          this->in_array_end(bVal.name());
      }
             // trick to wrap stl::list container
      template<class T>
      void in     (ChNameValueIn< std::list<T> > bVal) {
          bVal.value().clear();
          size_t arraysize;
          this->in_array_pre(bVal.name(), arraysize);
          for (size_t i = 0; i<arraysize; ++i)
          {
              char idname[20];
              sprintf(idname, "el_%lu", (unsigned long)i);
              T element;
              ChNameValueIn< T > array_val(idname, element);
              this->in (array_val);
              bVal.value().push_back(element);
              this->in_array_between(bVal.name());
          }
          this->in_array_end(bVal.name());
      }

        // trick to call in_ref on ChSharedPointer, with class abstraction:
      template<class T>
      typename enable_if< ChDetect_GetRTTI<T>::value >::type 
      in     (ChNameValueIn< std::shared_ptr<T> > bVal) {
          T* mptr;
          ChFunctorArchiveInSpecificPtrAbstract<T> specFuncA(&mptr, &T::ArchiveIN);
          ChNameValueIn<ChFunctorArchiveIn> mtmp(bVal.name(), specFuncA, bVal.flags());
          this->in_ref_abstract(mtmp);
          bVal.value() = std::shared_ptr<T> ( mptr );
      }

         // trick to call in_ref on ChSharedPointer (no class abstraction):
      template<class T>
      typename enable_if< !ChDetect_GetRTTI<T>::value >::type
      in     (ChNameValueIn< std::shared_ptr<T> > bVal) {
          T* mptr;
          ChFunctorArchiveInSpecificPtr<T> specFuncA(&mptr, &T::ArchiveIN);
          ChNameValueIn<ChFunctorArchiveIn> mtmp(bVal.name(), specFuncA, bVal.flags());
          this->in_ref(mtmp);
          bVal.value() = std::shared_ptr<T> ( mptr );
      }

        // trick to call in_ref on plain pointers, with class abstraction:
      template<class T>
      typename enable_if< ChDetect_GetRTTI<T>::value >::type
      in     (ChNameValueIn<T*> bVal) {
          ChFunctorArchiveInSpecificPtrAbstract<T> specFuncA(&bVal.value(), &T::ArchiveIN);
          this->in_ref_abstract(ChNameValueIn<ChFunctorArchiveIn>(bVal.name(), specFuncA, bVal.flags()) );
      }

        // trick to call in_ref on plain pointers (no class abstraction):
      template<class T>
      typename enable_if< !ChDetect_GetRTTI<T>::value >::type
      in     (ChNameValueIn<T*> bVal) {
          ChFunctorArchiveInSpecificPtr<T> specFuncA(&bVal.value(), &T::ArchiveIN);
          this->in_ref(ChNameValueIn<ChFunctorArchiveIn>(bVal.name(), specFuncA, bVal.flags()) );
      }

        // trick to apply 'virtual in..' on C++ objects that has a function "ArchiveIN":
      template<class T>
      void in     (ChNameValueIn<T> bVal) {
          ChFunctorArchiveInSpecific<T> specFuncA(&bVal.value(), &T::ArchiveIN);
          this->in(ChNameValueIn<ChFunctorArchiveIn>(bVal.name(), specFuncA, bVal.flags()));
      }

        /// Operator to allow easy serialization as   myarchive << mydata;

      template<class T>
      ChArchiveIn& operator>>(ChNameValueIn<T> bVal) {
          this->in(bVal);
          return (*this);
      }
      
      int VersionRead() {
          if (use_versions) {
              int mver;
              this->in(ChNameValueIn<int>("version",mver));
              return mver;
          }
          return 99999;
      }

  protected:

};

/// @} chrono_serialization

}  // END_OF_NAMESPACE____

#endif
