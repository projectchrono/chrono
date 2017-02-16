//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHARCHIVE_H
#define CHARCHIVE_H

#include <string>
#include <sstream>
#include <vector>
#include <list>
#include <typeinfo>
#include <unordered_set>
#include <memory>

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


/// Macro to create a  ChDetect_FactoryNameTag  detector that can be used in 
/// templates, to select which specialized template to use depending if the
/// FactoryNameTag function is present in the class of type T.
CH_CREATE_MEMBER_DETECTOR(FactoryNameTag)

/// Macro to create a  ChDetect_ArchiveINconstructor  
//CH_CREATE_MEMBER_DETECTOR(ArchiveINconstructor) already defined in ChClassFactory

/// Macro to create a  ChDetect_ArchiveOUTconstructor 
CH_CREATE_MEMBER_DETECTOR(ArchiveOUTconstructor)

/// Macro to create a  ChDetect_ArchiveOUT 
CH_CREATE_MEMBER_DETECTOR(ArchiveOUT)


/// Exceptions for archives should inherit from this
class ChExceptionArchive : public ChException 
{
public:
    ChExceptionArchive(std::string swhat) : ChException(swhat){};
};


/// Functor to call the ArchiveOUT function for unrelated classes that
/// implemented them. This helps stripping out the templating, to make ChArchiveOut
/// easier and equippable with virtual functions.
/// Also use this to call ArchiveOUTconstructor. 

class ChFunctorArchiveOut {
public:
        /// Use this to call ArchiveOut member function.
    virtual void CallArchiveOut(ChArchiveOut& marchive)=0;

        /// Use this to call (optional) member function ArchiveOUTconstructor. This is
        /// expected to serialize constructor parameters if any. 
        /// If ArchiveOUTconstructor is not provided, simply does nothing.
    virtual void CallArchiveOutConstructor(ChArchiveOut& marchive)=0;
    
    virtual bool IsNull()=0;
};

template <class TClass> 
class ChFunctorArchiveOutSpecific : public ChFunctorArchiveOut
{
private:
      void (TClass::*fpt)(ChArchiveOut&);   // pointer to member function
      TClass* pt2Object;                    // pointer to object

public:

      // constructor - takes pointer to an object and pointer to a member
      ChFunctorArchiveOutSpecific(TClass* _pt2Object, void(TClass::*_fpt)(ChArchiveOut&))
         { pt2Object = _pt2Object;  fpt=_fpt; };

      virtual void CallArchiveOut(ChArchiveOut& marchive)
        { (*pt2Object.*fpt)(marchive);};             // execute member function

      virtual void CallArchiveOutConstructor(ChArchiveOut& marchive) {
          this->_archive_out_constructor(marchive);
        }

      virtual bool IsNull()
        { return (pt2Object==0);};   

     
private:

        template <class Tc=TClass>
        typename enable_if< ChDetect_ArchiveOUTconstructor<Tc>::value, void >::type
        _archive_out_constructor(ChArchiveOut& marchive) {
            this->pt2Object->ArchiveOUTconstructor(marchive);
        }
        template <class Tc=TClass>
        typename enable_if< !ChDetect_ArchiveOUTconstructor<Tc>::value, void >::type 
        _archive_out_constructor(ChArchiveOut& marchive) {
            // nothing to do if not provided
        }

};


/// Functor to call the ArchiveIN function for unrelated classes that
/// implemented them. This helps stripping out the templating, to make ChArchiveOut
/// easier and equippable with virtual functions.

class ChFunctorArchiveIn {
public:
        /// Use this to call member function ArchiveIn. 
    virtual void CallArchiveIn(ChArchiveIn& marchive)=0;

        /// Use this to call an (optional) static member function ArchiveINconstructor. This 
        /// is expected to a) deserialize constructor parameters, b) create a new obj as new obj(params..).
        /// If ArchiveINconstructor is not provided, simply creates a new object as new obj(), as in CallNew().
    virtual void CallArchiveInConstructor(ChArchiveIn& marchive, const char* classname)=0;

    virtual void CallNew(ChArchiveIn& marchive) {};
    virtual void CallNewPolimorphic(ChArchiveIn& marchive, const char* classname) {};
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

      virtual void CallArchiveInConstructor(ChArchiveIn& marchive, const char* classname)
        { throw (ChExceptionArchive( "Cannot call CallArchiveInConstructor() for an object on heap.")); };

      virtual void  CallSetRawPtr(ChArchiveIn& marchive, void* mptr) 
        { throw (ChExceptionArchive( "Cannot call CallSetRawPtr() for an object on heap.")); };

      virtual void* CallGetRawPtr(ChArchiveIn& marchive) 
        { return static_cast<void*>(pt2Object); };
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

      virtual void CallArchiveInConstructor(ChArchiveIn& marchive, const char* classname) 
        { this->_archive_in_constructor(marchive); }

      virtual void CallNew(ChArchiveIn& marchive)
        { *pt2Object = new(TClass); }

      virtual void  CallSetRawPtr(ChArchiveIn& marchive, void* mptr) 
        { *pt2Object = static_cast<TClass*>(mptr); };

      virtual void* CallGetRawPtr(ChArchiveIn& marchive) 
        { return static_cast<void*>(*pt2Object); };

      
private:
        template <class Tc=TClass>
        typename enable_if< ChDetect_ArchiveINconstructor<Tc>::value, void >::type
        _archive_in_constructor(ChArchiveIn& marchive) {
            *pt2Object = Tc::ArchiveINconstructor(marchive);
        }
        template <class Tc=TClass>
        typename enable_if< !ChDetect_ArchiveINconstructor<Tc>::value, void >::type 
        _archive_in_constructor(ChArchiveIn& marchive) {
            this->CallNew(marchive);
        }
};


template <class TClass> 
class ChFunctorArchiveInSpecificPtrPolimorphic : public ChFunctorArchiveIn
{
private:
      void (TClass::*fpt)(ChArchiveIn&);   // pointer to member function
      TClass** pt2Object;                    // pointer to object

public:

      // constructor - takes pointer to an object and pointer to a member 
      ChFunctorArchiveInSpecificPtrPolimorphic(TClass** _pt2Object, void(TClass::*_fpt)(ChArchiveIn&))
        { pt2Object = _pt2Object;  fpt=_fpt; };

      virtual void CallArchiveIn(ChArchiveIn& marchive)
        { (**pt2Object.*fpt)(marchive);};             // execute member function

      virtual void CallArchiveInConstructor(ChArchiveIn& marchive, const char* classname) 
        { this->_archive_in_constructor(marchive, classname); }

      virtual void CallNewPolimorphic(ChArchiveIn& marchive, const char* classname) { 
            std::string sclassname(classname);
            ChClassFactory::create(sclassname, pt2Object); 
        }

      virtual void  CallSetRawPtr(ChArchiveIn& marchive, void* mptr) 
        { *pt2Object = static_cast<TClass*>(mptr); };

      virtual void* CallGetRawPtr(ChArchiveIn& marchive) 
        { return static_cast<void*>(*pt2Object); };

private:
        //template <class Tc=TClass>
        //typename enable_if< ChDetect_ArchiveINconstructor<Tc>::value, void >::type
        void _archive_in_constructor(ChArchiveIn& marchive, const char* classname) {
            // Here class factory will either
            // - call new(), with default constructor 
            // - or call static ArchiveINconstructor(), if any (that deserializes parameters and then makes new())
            std::string sclassname(classname);
            ChClassFactory::archive_in_create(sclassname, marchive, pt2Object); 
        }

};




///
/// This is a base class for name-value pairs
///

template<class T>
class  ChNameValue {
  public:
        ChNameValue(const char* mname, const T& mvalue, char mflags = 0) : 
            _name(mname), 
            _value((T*)(& mvalue)),
            _flags((char)mflags) {}

        ChNameValue(const ChNameValue<T>& other){
            _name  = other._name;
            _value = other._value;
            _flags = other._flags;
        }
        virtual ~ChNameValue() {};

        const char * name() const {
            return this->_name;
        }

        char& flags() {
            return this->_flags;
        }

        T & value() const {
            return *(this->_value);
        }

        const T & const_value() const {
            return *(this->_value);
        }

  protected:
        T* _value;
        const char* _name;
        char _flags;
};

// Flag to mark a ChNameValue for a C++ object serialized by value but that
// that can be later referenced by pointer too.
static const char NVP_TRACK_OBJECT = (1 << 0);


template<class T>
ChNameValue< T > make_ChNameValue(const char * auto_name, const T & t, const char * custom_name, char flags = 0){
    const char* mname = auto_name;
    if (custom_name)
        mname = custom_name;
    return ChNameValue< T >(mname, t, flags);
}
template<class T>
ChNameValue< T > make_ChNameValue(const char * auto_name, const T & t, char flags = 0){
    const char* mname = auto_name;
    return ChNameValue< T >(mname, t, flags);
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

#define CHNVP(...) \
    make_ChNameValue("" STRINGIFY(FIRST(__VA_ARGS__)) "", FIRST(__VA_ARGS__) REST(__VA_ARGS__) )



/// Class for mapping enums to ChNameValue pairs that contain a 'readable' ascii string
/// of the selected enum. This could be used when streaming from/to human readable formats
/// such as JSON or XML or ascii dumps.

class ChEnumMapperBase {
public:
    ChEnumMapperBase ()  {}

    virtual int  GetValueAsInt() = 0;
    virtual void SetValueAsInt(const int mval) = 0;

    virtual std::string GetValueAsString() = 0;
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
class ChEnumMapper : public ChEnumMapperBase {
  public:
    ChEnumMapper() : value_ptr(0) {
        enummap = std::shared_ptr<std::vector<ChEnumNamePair<Te> > >(new std::vector<ChEnumNamePair<Te> >);
    }

    ChEnumMapper(std::shared_ptr<std::vector<ChEnumNamePair<Te> > > mmap) : value_ptr(0), enummap(mmap) {}

    virtual ~ChEnumMapper() {}

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

    Te& Value() {return *value_ptr;}

    virtual int  GetValueAsInt() override { 
        return static_cast<int>(*value_ptr);
    };
    virtual void SetValueAsInt(const int mval) override {
        *value_ptr = static_cast<Te>(mval);
    };

    virtual std::string GetValueAsString() override {
        for (int i = 0; i < enummap->size(); ++i)
        {
            if(enummap->at(i).enumid == *value_ptr)
                return enummap->at(i).name;
        }
        // not found, just string as number:
        char buffer [10];
        sprintf(buffer, "%d", GetValueAsInt());
        return std::string(buffer);
    };

    virtual bool SetValueAsString(const std::string& mname) override {
        for (int i = 0; i < enummap->size(); ++i)
        {
            if(enummap->at(i).name == mname) {
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
        else{
            SetValueAsInt(numb);
            return true;
        }
        // neither found enum from string, nor from number...
        return false; 
    };

    Te* value_ptr;

 protected:
        
    std::shared_ptr< std::vector< ChEnumNamePair<Te> > > enummap;
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

#define CH_ENUM_MAPPER_BEGIN(__enum_type) \
            class __enum_type##_mapper : public ChEnumMapper< __enum_type > { \
            public: \
                __enum_type##_mapper() { 

#define CH_ENUM_VAL(...) \
        this->AddMapping("" STRINGIFY(FIRST(__VA_ARGS__)) "", FIRST(__VA_ARGS__) REST(__VA_ARGS__) );

#define CH_ENUM_MAPPER_END(__enum_type) \
            }; \
        ChEnumMapper< __enum_type > operator() ( __enum_type & mval) { \
            ChEnumMapper< __enum_type > res(this->enummap); \
            res.value_ptr = &mval; \
            return res; \
        } }; 


//
// Wrapper class to ease the archival of std::pair
//

template<class T, class Tv>
class _wrap_pair {
public:
    _wrap_pair(std::pair<T, Tv>& apair) {
        _wpair = &apair;
    }
    void ArchiveOUT(ChArchiveOut& marchive)
    {
        marchive << CHNVP(_wpair->first,"1");
        marchive << CHNVP(_wpair->second,"2");
    }
    void ArchiveIN(ChArchiveIn& marchive)
    {
        marchive >> CHNVP(_wpair->first,"1");
        marchive >> CHNVP(_wpair->second,"2");
    }
private:
    std::pair<T, Tv>* _wpair;
};


///
/// This is a base class for archives with pointers to shared objects 
///

class ChArchive {
  protected:

    /// vector of pointers to stored/retrieved objects,
    /// to avoid saving duplicates or deadlocks
    std::unordered_map<size_t, void*> objects_pointers;

    /// container of pointers to not serialize if ever encountered
    std::unordered_set<void*>  cut_pointers;

    bool cluster_class_versions;
    std::unordered_map<std::string, int> class_versions;

    bool use_versions;
    bool cut_all_pointers;

    size_t currentID;

  public:
    ChArchive() {
        use_versions = true;
        cut_all_pointers = false;
        cluster_class_versions = false;
        Init();
    }

    virtual ~ChArchive() {};

    /// Reinitialize the vector of pointers to loaded/saved objects
    void Init() {
        objects_pointers.clear();
        objects_pointers[0]=(0); // ID=0 for null pointer.
        currentID = 0;
    }
    /// Find a pointer in pointer vector: eventually add it to vecor if it
    /// was not previously inserted. Returns already_stored=false if was
    /// already inserted. Return 'pos' offset in vector in any case.
    /// For null pointers, always return 'already_stored'=true, and 'pos'=0.
    void PutPointer(void* object, bool& already_stored, size_t& pos) {

        for (const auto& elem: objects_pointers) {
            if (elem.second == object)
            {
                already_stored = true;
                pos = elem.first;
                return;
            }
        }

        // wasn't in list.. add to it
        ++currentID;
        pos = currentID;
        objects_pointers[pos] = object;
        already_stored = false;

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

    /// If true, the version number is not saved in each class: rather, 
    /// when << serialization happens, a 'silent' sweep through the serialized object (and sub
    /// objects) is done, storing the versions in a map that is saved once at the beginning 
    /// of the archive, before sweeping again for the true serialization. And viceversa for deserialization.
    void SetClusterClassVersions(bool mcl) {this->cluster_class_versions = mcl;}

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
      virtual void out     (ChNameValue<bool> bVal) = 0;
      virtual void out     (ChNameValue<int> bVal) = 0;
      virtual void out     (ChNameValue<double> bVal) = 0;
      virtual void out     (ChNameValue<float> bVal) = 0;
      virtual void out     (ChNameValue<char> bVal) = 0;
      virtual void out     (ChNameValue<unsigned int> bVal) =0;
      virtual void out     (ChNameValue<std::string> bVal) = 0;
      virtual void out     (ChNameValue<unsigned long> bVal) = 0;
      virtual void out     (ChNameValue<unsigned long long> bVal) =0;
      virtual void out     (ChNameValue<ChEnumMapperBase> bVal) =0;

        // for custom C++ objects - see 'wrapping' trick below
      virtual void out     (ChNameValue<ChFunctorArchiveOut> bVal, const char* classname, bool tracked, size_t position) = 0;

        // for pointed objects with polimorphic class system (i.e. supporting class factory)
      virtual void out_ref_polimorphic (ChNameValue<ChFunctorArchiveOut> bVal, bool already_inserted, size_t position, const char* classname) = 0;
      
        // for pointed objects without class polimorphism
      virtual void out_ref          (ChNameValue<ChFunctorArchiveOut> bVal, bool already_inserted, size_t position, const char* classname) = 0;

        // for wrapping arrays and lists
      virtual void out_array_pre (const char* name, size_t msize, const char* classname) = 0;
      virtual void out_array_between (size_t msize, const char* classname) = 0;
      virtual void out_array_end (size_t msize,const char* classname) = 0;


      //---------------------------------------------------

           // trick to wrap enum mappers:
      template<class T>
      void out     (ChNameValue< ChEnumMapper<T> > bVal) {
          ChNameValue< ChEnumMapperBase > tmpnv(bVal.name(),bVal.value(),bVal.flags());
          this->out(tmpnv);
      }

        // trick to wrap C++ fixed-size arrays:
      template<class T, size_t N>
      void out     (ChNameValue<T[N]> bVal) {
          size_t arraysize = sizeof(bVal.value())/sizeof(T);
          this->out_array_pre(bVal.name(), arraysize, typeid(T).name());
          for (size_t i = 0; i<arraysize; ++i)
          {
              char buffer[20];
              sprintf(buffer, "el_%lu", (unsigned long)i);
              ChNameValue< T > array_val(buffer, bVal.value()[i]);
              this->out (array_val);
              this->out_array_between(arraysize, typeid(bVal.value()).name());
          }
          this->out_array_end(arraysize, typeid(bVal.value()).name());
      }

        // trick to wrap stl::vector container
      template<class T>
      void out     (ChNameValue< std::vector<T> > bVal) {
          this->out_array_pre(bVal.name(), bVal.value().size(), typeid(T).name());
          for (size_t i = 0; i<bVal.value().size(); ++i)
          {
              char buffer[20];
              sprintf(buffer, "el_%lu", (unsigned long)i);
              ChNameValue< T > array_val(buffer, bVal.value()[i]);
              this->out (array_val);
              this->out_array_between(bVal.value().size(), typeid(bVal.value()).name());
          }
          this->out_array_end(bVal.value().size(), typeid(bVal.value()).name());
      }
        // trick to wrap stl::list container
      template<class T>
      void out     (ChNameValue< std::list<T> > bVal) {
          this->out_array_pre(bVal.name(), bVal.value().size(), typeid(T).name());
          typename std::list<T>::iterator iter;
          size_t i = 0;
          for (iter = bVal.value().begin(); iter != bVal.value().end(); ++iter, ++i)
          {
              char buffer[20];
              sprintf(buffer, "el_%lu", (unsigned long)i);
              ChNameValue< T > array_val(buffer, (*iter));
              this->out (array_val);
              this->out_array_between(bVal.value().size(), typeid(bVal.value()).name());
          }
          this->out_array_end(bVal.value().size(), typeid(bVal.value()).name());
      }
        // trick to wrap stl::pair container
      template<class T, class Tv>
      void out     (ChNameValue< std::pair<T, Tv> > bVal) {
          _wrap_pair<T,Tv> mpair(bVal.value());
          ChNameValue< _wrap_pair<T,Tv> > pair_val(bVal.name(), mpair);
          this->out (pair_val);
      }
        // trick to wrap stl::unordered_map container
      template<class T, class Tv>
      void out     (ChNameValue< std::unordered_map<T, Tv> > bVal) {
          this->out_array_pre(bVal.name(), bVal.value().size(), typeid(std::pair<T, Tv>).name());
          int i=0;
          for ( auto it = bVal.value().begin(); it != bVal.value().end(); ++it )
          {
              char buffer[20];
              sprintf(buffer, "el_%lu", (unsigned long)i);
              ChNameValue< std::pair<T, Tv> > array_key(buffer, (*it));
              this->out (array_key);
              this->out_array_between(bVal.value().size(), typeid(bVal.value()).name());
              ++i;
          }
          this->out_array_end(bVal.value().size(), typeid(bVal.value()).name());
      }
     


        // trick to call out_ref on ChSharedPointer, with class polimorphism:
      template<class T>
      typename enable_if< ChDetect_FactoryNameTag<T>::value >::type
      out     (ChNameValue< std::shared_ptr<T> > bVal) {
          bool already_stored; size_t pos;
          T* mptr = bVal.value().get();
          if (this->cut_all_pointers)
              mptr = 0;
          if (this->cut_pointers.find((void*)mptr) != this->cut_pointers.end())
              mptr = 0;
          PutPointer(mptr, already_stored, pos);
          ChFunctorArchiveOutSpecific<T> specFuncA(mptr, &T::ArchiveOUT);
          const char* class_name;
          if (bVal.value())
              class_name = bVal.value()->FactoryNameTag().c_str();
          else // null ptr
              class_name = typeid(T).name(); // note, this class name is not platform independent but enough here since for null ptr
          this->out_ref_polimorphic(
              ChNameValue<ChFunctorArchiveOut>(bVal.name(), specFuncA, bVal.flags()), 
              already_stored, 
              pos, 
              class_name );
      }

        // trick to call out_ref on ChSharedPointer, without class polimorphism:
      template<class T>
      typename enable_if< !ChDetect_FactoryNameTag<T>::value >::type 
      out     (ChNameValue< std::shared_ptr<T> > bVal) {
          bool already_stored; size_t pos;
          T* mptr = bVal.value().get();
          if (this->cut_all_pointers)
              mptr = 0;
          if (this->cut_pointers.find((void*)mptr) != this->cut_pointers.end())
              mptr = 0;
          PutPointer(mptr, already_stored, pos);
          ChFunctorArchiveOutSpecific<T> specFuncA(mptr, &T::ArchiveOUT);
          this->out_ref(
              ChNameValue<ChFunctorArchiveOut>(bVal.name(), specFuncA, bVal.flags()), 
              already_stored, 
              pos, 
              typeid(T).name() ); // note, this class name is not platform independent
      }

        // trick to call out_ref on plain pointers, with class polimorphism:
      template<class T>
      typename enable_if< ChDetect_FactoryNameTag<T>::value >::type
      out     (ChNameValue<T*> bVal) {
          bool already_stored; size_t pos;
          T* mptr = bVal.value();
          if (this->cut_all_pointers)
              mptr = 0;
          if (this->cut_pointers.find((void*)mptr) != this->cut_pointers.end())
              mptr = 0;
          PutPointer(mptr, already_stored, pos);
          ChFunctorArchiveOutSpecific<T> specFuncA(mptr, &T::ArchiveOUT);
          const char* class_name;
          if (bVal.value())
              class_name = bVal.value()->FactoryNameTag().c_str();
          else // null ptr
              class_name = typeid(T).name(); // note, this class name is not platform independent but enough here since for null ptr
          this->out_ref_polimorphic(
              ChNameValue<ChFunctorArchiveOut>(bVal.name(), specFuncA, bVal.flags()), 
              already_stored,
              pos, 
              class_name ); // this class name is platform independent
      }

         // trick to call out_ref on plain pointers (no class polimorphism):
      template<class T>
      typename enable_if< !ChDetect_FactoryNameTag<T>::value >::type 
      out     (ChNameValue<T*> bVal) {
          bool already_stored; size_t pos;
          T* mptr = bVal.value();
          if (this->cut_all_pointers)
              mptr = 0;
          if (this->cut_pointers.find((void*)mptr) != this->cut_pointers.end())
              mptr = 0;
          PutPointer(mptr, already_stored, pos);
          ChFunctorArchiveOutSpecific<T> specFuncA(mptr, &T::ArchiveOUT);
          this->out_ref(
              ChNameValue<ChFunctorArchiveOut>(bVal.name(), specFuncA, bVal.flags()), 
              already_stored,
              pos, 
              typeid(T).name() ); // note, this class name is not platform independent
      }

       // trick to apply 'virtual out..' on remaining C++ object, that has a function "ArchiveOUT" 
      template<class T>
      void out (ChNameValue<T> bVal) {
          bool tracked = false;
          size_t pos =0;
          if (bVal.flags() & NVP_TRACK_OBJECT)
          {
              bool already_stored; 
              T* mptr = &bVal.value();
              PutPointer(mptr, already_stored, pos);
              if (already_stored) 
                  {throw (ChExceptionArchive( "Cannot serialize tracked object '" + std::string(bVal.name()) + "' by value, AFTER already serialized by pointer."));}
              tracked = true;
          }
          ChFunctorArchiveOutSpecific<T> specFuncA(&bVal.value(), &T::ArchiveOUT);
          this->out(
              ChNameValue<ChFunctorArchiveOut>(bVal.name(), specFuncA, bVal.flags()), 
              typeid(T).name(),  // not platform independent, but not needed in this case
              tracked, pos);
      }

        /// Operator to allow easy serialization as   myarchive << mydata;
      template<class T> 
      ChArchiveOut& operator<<(ChNameValue<T> bVal) {
          this->out(bVal);
          return (*this);
      }

      void VersionWrite(int mver) {
          if (use_versions) {
                this->out(ChNameValue<int>("version",mver));
          }
      }
      /*
      template<class T>
      typename enable_if< ChDetect_ArchiveOUT<T>::value,ChArchiveOut& >::type 
       operator<<(ChNameValue<T> bVal) {
          if (cluster_class_versions) {
            this->out(bVal);
          }
          else {
            this->out(bVal);
          }
          return (*this);
      }

      template<class T>
      typename enable_if< !ChDetect_ArchiveOUT<T>::value,ChArchiveOut& >::type 
       operator<<(ChNameValue<T> bVal) {
          if (cluster_class_versions) {
            // skip saving 
          }
          else {
            this->out(bVal);
          }
          return (*this);
      }
      

      void VersionWrite(int mver) {
          if (use_versions) {
            if (cluster_class_versions){
                this->class_versions["test"]=mver;
            } else
            {
                this->out(ChNameValue<int>("version",mver));
            }
          }
      }
      */
      
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
      virtual void in     (ChNameValue<bool> bVal) = 0;
      virtual void in     (ChNameValue<int> bVal) = 0;
      virtual void in     (ChNameValue<double> bVal) = 0;
      virtual void in     (ChNameValue<float> bVal) = 0;
      virtual void in     (ChNameValue<char> bVal) = 0;
      virtual void in     (ChNameValue<unsigned int> bVal) = 0;
      virtual void in     (ChNameValue<std::string> bVal) = 0;
      virtual void in     (ChNameValue<unsigned long> bVal) = 0;
      virtual void in     (ChNameValue<unsigned long long> bVal) = 0;
      virtual void in     (ChNameValue<ChEnumMapperBase> bVal) =0;

        // for custom C++ objects - see 'wrapping' trick below
      virtual void in     (ChNameValue<ChFunctorArchiveIn> bVal) = 0;

        // for pointed objects 
      virtual void in_ref_polimorphic (ChNameValue<ChFunctorArchiveIn> bVal) = 0;
      virtual void in_ref          (ChNameValue<ChFunctorArchiveIn> bVal) = 0;

        // for wrapping arrays and lists
      virtual void in_array_pre (const char* name, size_t& msize) = 0;
      virtual void in_array_between (const char* name) = 0;
      virtual void in_array_end (const char* name) = 0;

      //---------------------------------------------------

           // trick to wrap enum mappers:
      template<class T>
      void in     (ChNameValue< ChEnumMapper<T> > bVal) {
          ChNameValue< ChEnumMapperBase > tmpnv(bVal.name(),bVal.value(),bVal.flags());
          this->in(tmpnv);
      }

             // trick to wrap C++ fixed-size arrays:
      template<class T, size_t N>
      void in     (ChNameValue<T[N]> bVal) {
          size_t arraysize;
          this->in_array_pre(bVal.name(), arraysize);
          if (arraysize != sizeof(bVal.value())/sizeof(T) ) {throw (ChExceptionArchive( "Size of [] saved array does not match size of receiver array " + std::string(bVal.name()) + "."));}
          for (size_t i = 0; i<arraysize; ++i)
          {
              char idname[20];
              sprintf(idname, "el_%lu", (unsigned long)i);
              T element;
              ChNameValue< T > array_val(idname, element);
              this->in (array_val);
              bVal.value()[i]=element;
              this->in_array_between(bVal.name());
          }
          this->in_array_end(bVal.name());
      }

             // trick to wrap stl::vector container
      template<class T>
      void in     (ChNameValue< std::vector<T> > bVal) {
          bVal.value().clear();
          size_t arraysize;
          this->in_array_pre(bVal.name(), arraysize);
          bVal.value().resize(arraysize);
          for (size_t i = 0; i<arraysize; ++i)
          {
              char idname[20];
              sprintf(idname, "el_%lu", (unsigned long)i);
              T element;
              ChNameValue< T > array_val(idname, element);
              this->in (array_val);
              bVal.value()[i]=element;
              this->in_array_between(bVal.name());
          }
          this->in_array_end(bVal.name());
      }
             // trick to wrap stl::list container
      template<class T>
      void in     (ChNameValue< std::list<T> > bVal) {
          bVal.value().clear();
          size_t arraysize;
          this->in_array_pre(bVal.name(), arraysize);
          for (size_t i = 0; i<arraysize; ++i)
          {
              char idname[20];
              sprintf(idname, "el_%lu", (unsigned long)i);
              T element;
              ChNameValue< T > array_val(idname, element);
              this->in (array_val);
              bVal.value().push_back(element);
              this->in_array_between(bVal.name());
          }
          this->in_array_end(bVal.name());
      }
        // trick to wrap stl::pair container
      template<class T, class Tv>
      void in     (ChNameValue< std::pair<T, Tv> > bVal) {
          _wrap_pair<T,Tv> mpair(bVal.value());
          ChNameValue< _wrap_pair<T,Tv> > pair_val(bVal.name(), mpair);
          this->in (pair_val);
      }
        // trick to wrap stl::unordered_map container
      template<class T, class Tv>
      void in     (ChNameValue< std::unordered_map<T, Tv> > bVal) {
          bVal.value().clear();
          size_t arraysize;
          this->in_array_pre(bVal.name(), arraysize);
          for (size_t i = 0; i<arraysize; ++i)
          {
              char idname[20];
              sprintf(idname, "el_%lu", (unsigned long)i);
              std::pair<T,Tv> mpair;
              ChNameValue< std::pair<T,Tv> > array_val(idname, mpair);
              this->in (array_val);
              // store in map; constant time:
              bVal.value()[mpair.first]=mpair.second; 
              this->in_array_between(bVal.name());
          }
          this->in_array_end(bVal.name());
      }


        // trick to call in_ref on ChSharedPointer, with class polimorphism:
      template<class T>
      typename enable_if< ChDetect_FactoryNameTag<T>::value >::type 
      in     (ChNameValue< std::shared_ptr<T> > bVal) {
          T* mptr;
          ChFunctorArchiveInSpecificPtrPolimorphic<T> specFuncA(&mptr, &T::ArchiveIN);
          ChNameValue<ChFunctorArchiveIn> mtmp(bVal.name(), specFuncA, bVal.flags());
          this->in_ref_polimorphic(mtmp);
          bVal.value() = std::shared_ptr<T> ( mptr );
      }

         // trick to call in_ref on ChSharedPointer (no class polimorphism):
      template<class T>
      typename enable_if< !ChDetect_FactoryNameTag<T>::value >::type
      in     (ChNameValue< std::shared_ptr<T> > bVal) {
          T* mptr;
          ChFunctorArchiveInSpecificPtr<T> specFuncA(&mptr, &T::ArchiveIN);
          ChNameValue<ChFunctorArchiveIn> mtmp(bVal.name(), specFuncA, bVal.flags());
          this->in_ref(mtmp);
          bVal.value() = std::shared_ptr<T> ( mptr );
      }

        // trick to call in_ref on plain pointers, with class polimorphism:
      template<class T>
      typename enable_if< ChDetect_FactoryNameTag<T>::value >::type
      in     (ChNameValue<T*> bVal) {
          ChFunctorArchiveInSpecificPtrPolimorphic<T> specFuncA(&bVal.value(), &T::ArchiveIN);
          this->in_ref_polimorphic(ChNameValue<ChFunctorArchiveIn>(bVal.name(), specFuncA, bVal.flags()) );
      }

        // trick to call in_ref on plain pointers (no class polimorphism):
      template<class T>
      typename enable_if< !ChDetect_FactoryNameTag<T>::value >::type
      in     (ChNameValue<T*> bVal) {
          ChFunctorArchiveInSpecificPtr<T> specFuncA(&bVal.value(), &T::ArchiveIN);
          this->in_ref(ChNameValue<ChFunctorArchiveIn>(bVal.name(), specFuncA, bVal.flags()) );
      }

        // trick to apply 'virtual in..' on C++ objects that has a function "ArchiveIN":
      template<class T>
      void in     (ChNameValue<T> bVal) {
          ChFunctorArchiveInSpecific<T> specFuncA(&bVal.value(), &T::ArchiveIN);
          this->in(ChNameValue<ChFunctorArchiveIn>(bVal.name(), specFuncA, bVal.flags()));
      }

        /// Operator to allow easy serialization as   myarchive << mydata;

      template<class T>
      ChArchiveIn& operator>>(ChNameValue<T> bVal) {
          this->in(bVal);
          return (*this);
      }
      
      int VersionRead() {
          if (use_versions) {
              int mver;
              this->in(ChNameValue<int>("version",mver));
              return mver;
          }
          return 99999;
      }

  protected:

};

/// @} chrono_serialization

}  // end namespace chrono

#endif
