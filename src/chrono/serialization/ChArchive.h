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


#include "core/ChApiCE.h"
#include "core/ChStream.h"
#include "core/ChSmartpointers.h"
#include "core/ChTemplateExpressions.h"
#include <string>
#include <sstream>
#include <vector>
#include <list>
#include <typeinfo>

namespace chrono {

// forward reference
class ChArchiveOut;
class ChArchiveIn;


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


/// Functor to call the ArchiveOUT function for unrelated classes that
/// implemented them. This helps stripping out the templating, to make ChArchiveOut
/// easier and equippable with virtual functions.

class ChFunctorArchiveOut {
public:
    virtual void CallArchiveOut(ChArchiveOut& marchive)=0;   
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

      virtual bool IsNull()
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
        { chrono::create(classname, pt2Object); }

      virtual void  CallSetRawPtr(ChArchiveIn& marchive, void* mptr) 
        { *pt2Object = static_cast<TClass*>(mptr); };

      virtual void* CallGetRawPtr(ChArchiveIn& marchive) 
        { return static_cast<void*>(*pt2Object); };
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
    ChEnumMapperBase ()  {};

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
    ChEnumMapper () : 
        value_ptr(0) {
        enummap = ChSmartPtr< std::vector< ChEnumNamePair<Te> > >(new std::vector< ChEnumNamePair<Te> >);
    };
    ChEnumMapper (ChSmartPtr< std::vector< ChEnumNamePair<Te> > >  mmap) : 
        value_ptr(0), 
        enummap(mmap) {};

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

    virtual int  GetValueAsInt() { 
        return static_cast<int>(*value_ptr);
    };
    virtual void SetValueAsInt(const int mval) {
        *value_ptr = static_cast<Te>(mval);
    };

    virtual std::string GetValueAsString() {
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

    virtual bool SetValueAsString(const std::string& mname) {
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
        
    ChSmartPtr< std::vector< ChEnumNamePair<Te> > > enummap;
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


///
/// This is a base class for archives with pointers to shared objects 
///

class ChArchive {
  protected:

    /// vector of pointers to stored/retrieved objects,
    /// to avoid saving duplicates or deadlocks
    std::vector<void*> objects_pointers;

    bool use_versions;
    bool cut_pointers;

  public:
    ChArchive() {
        use_versions = true;
        cut_pointers = false;
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

    /// If you enable  SetCutPointers(true), no serialization happens for 
    /// objects referenced via pointers. This can be useful to save a single object, 
    /// regardless of the fact that it contains pointers to other 'children' objects.
    /// Cut pointers are turned into null pointers.
    void SetCutPointers(bool mcut) {this->cut_pointers = mcut;}
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

        // for pointed objects with abstract class system (i.e. supporting class factory)
      virtual void out_ref_abstract (ChNameValue<ChFunctorArchiveOut> bVal, bool already_inserted, size_t position, const char* classname) = 0;
      
        // for pointed objects without class abstraction
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
     


        // trick to call out_ref on ChSharedPointer, with class abstraction:
      template<class T>
      typename enable_if< ChDetect_GetRTTI<T>::value >::type
      out     (ChNameValue< ChSharedPtr<T> > bVal) {
          bool already_stored; size_t pos;
          T* mptr = bVal.value().get_ptr();
          if (this->cut_pointers)
              mptr = 0;
          PutPointer(mptr, already_stored, pos);
          ChFunctorArchiveOutSpecific<T> specFuncA(mptr, &T::ArchiveOUT);
          this->out_ref_abstract(
              ChNameValue<ChFunctorArchiveOut>(bVal.name(), specFuncA), 
              already_stored, 
              pos, 
              bVal.value()->GetRTTI()->GetName() );
      }

        // trick to call out_ref on ChSharedPointer, without class abstraction:
      template<class T>
      typename enable_if< !ChDetect_GetRTTI<T>::value >::type 
      out     (ChNameValue< ChSharedPtr<T> > bVal) {
          bool already_stored; size_t pos;
          T* mptr = bVal.value().get_ptr();
          if (this->cut_pointers)
              mptr = 0;
          PutPointer(mptr, already_stored, pos);
          ChFunctorArchiveOutSpecific<T> specFuncA(mptr, &T::ArchiveOUT);
          this->out_ref(
              ChNameValue<ChFunctorArchiveOut>(bVal.name(), specFuncA, bVal.flags()), 
              already_stored, 
              pos, 
              typeid(T).name() ); // note, this class name is not platform independent
      }

        // trick to call out_ref on plain pointers, with class abstraction:
      template<class T>
      typename enable_if< ChDetect_GetRTTI<T>::value >::type
      out     (ChNameValue<T*> bVal) {
          bool already_stored; size_t pos;
          T* mptr = bVal.value();
          if (this->cut_pointers)
              mptr = 0;
          PutPointer(mptr, already_stored, pos);
          ChFunctorArchiveOutSpecific<T> specFuncA(mptr, &T::ArchiveOUT);
          this->out_ref_abstract(
              ChNameValue<ChFunctorArchiveOut>(bVal.name(), specFuncA, bVal.flags()), 
              already_stored,
              pos, 
              bVal.value()->GetRTTI()->GetName() ); // this class name is platform independent
      }

         // trick to call out_ref on plain pointers (no class abstraction):
      template<class T>
      typename enable_if< !ChDetect_GetRTTI<T>::value >::type 
      out     (ChNameValue<T*> bVal) {
          bool already_stored; size_t pos;
          T* mptr = bVal.value();
          if (this->cut_pointers)
              mptr = 0;
          PutPointer(mptr, already_stored, pos);
          ChFunctorArchiveOutSpecific<T> specFuncA(mptr, &T::ArchiveOUT);
          this->out_ref(
              ChNameValue<ChFunctorArchiveOut>(bVal.name(), specFuncA, bVal.flags()), 
              already_stored,
              pos, 
              typeid(T).name() ); // note, this class name is not platform independent
      }

        // trick to apply 'virtual out..' on remaining C++ object, that has a function "ArchiveOUT":
      template<class T>
      void out     (ChNameValue<T> bVal) {
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
              typeid(T).name(),
              tracked, pos);
      }

        /// Operator to allow easy serialization as   myarchive >> mydata;
      
      template<class T>
      ChArchiveOut& operator<<(ChNameValue<T> bVal) {
          this->out(bVal);
          return (*this);
      }
      
      void VersionWrite(int mver) {
          if (use_versions)
            this->out(ChNameValue<int>("version",mver));
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
      virtual void in_ref_abstract (ChNameValue<ChFunctorArchiveIn> bVal) = 0;
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

        // trick to call in_ref on ChSharedPointer, with class abstraction:
      template<class T>
      typename enable_if< ChDetect_GetRTTI<T>::value >::type 
      in     (ChNameValue< ChSharedPtr<T> > bVal) {
          T* mptr;
          ChFunctorArchiveInSpecificPtrAbstract<T> specFuncA(&mptr, &T::ArchiveIN);
          ChNameValue<ChFunctorArchiveIn> mtmp(bVal.name(), specFuncA, bVal.flags());
          this->in_ref_abstract(mtmp);
          bVal.value() = ChSharedPtr<T> ( mptr );
      }

         // trick to call in_ref on ChSharedPointer (no class abstraction):
      template<class T>
      typename enable_if< !ChDetect_GetRTTI<T>::value >::type
      in     (ChNameValue< ChSharedPtr<T> > bVal) {
          T* mptr;
          ChFunctorArchiveInSpecificPtr<T> specFuncA(&mptr, &T::ArchiveIN);
          ChNameValue<ChFunctorArchiveIn> mtmp(bVal.name(), specFuncA, bVal.flags());
          this->in_ref(mtmp);
          bVal.value() = ChSharedPtr<T> ( mptr );
      }

        // trick to call in_ref on plain pointers, with class abstraction:
      template<class T>
      typename enable_if< ChDetect_GetRTTI<T>::value >::type
      in     (ChNameValue<T*> bVal) {
          ChFunctorArchiveInSpecificPtrAbstract<T> specFuncA(&bVal.value(), &T::ArchiveIN);
          this->in_ref_abstract(ChNameValue<ChFunctorArchiveIn>(bVal.name(), specFuncA, bVal.flags()) );
      }

        // trick to call in_ref on plain pointers (no class abstraction):
      template<class T>
      typename enable_if< !ChDetect_GetRTTI<T>::value >::type
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






 




}  // END_OF_NAMESPACE____

#endif
