//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHCLASSFACTORY_H
#define CHCLASSFACTORY_H


//
//
//  HOW TO REGISTER A CLASS IN THE CLASS FACTORY
//
//   Assuming you have such a class, say it is named 'myEmployee',
//  you just have to put the following line in your .cpp code
//  (for example in myEmployee.cpp, but not in myEmployee.h!):
//
//  CH_FACTORY_REGISTER(myEmployee)
//
//  Also, there is a problem: class names cannot be inferred from
//  standard C++ stuff like:  typeid(myclass).name() because the result
//  is compiler-dependent, with different name decorations on different 
//  platforms. To overcome this issue, it is strongly suggested to put 
//  the CH_FACTORY_TAG macro in the class declaration, as in:
//  
//  class myclass {
//      CH_FACTORY_TAG(myclass)
//   public:
//      ...
//   };
//
//

#include <cstdio>
#include <string>
#include <typeinfo>
#include <typeindex>
#include <unordered_map>

#include "chrono/core/ChLog.h"
#include "chrono/core/ChTemplateExpressions.h"

namespace chrono {

// forward decl.
class ChArchiveIn;


/// Base class for all registration data of classes 
/// whose objects can be created via a class factory.

class ChApi ChClassRegistrationBase {
public:
    /// The signature of create method for derived classes. Calls new().
    virtual void* create() = 0;

    /// Call the ArchiveINconstructor(ChArchiveIn&) function if available (deserializes constructor params and return new()),
    /// otherwise just call new().
    virtual void* create(ChArchiveIn& marchive) = 0;

    /// Get the type_info of the class
    virtual std::type_index  get_type_index() = 0;

    /// Get the name used for registering
    virtual std::string& get_tag_name() = 0;
};



/// A class factory. 
/// It can create C++ objects from their string name.
/// Just use the  ChClassFactory::create()  function, given a string.
/// NOTE: desired classes must be previously registered 
///  via the CH_FACTORY_REGISTER macro, or using ClassRegister, otherwise
///  the ChClassFactory::create()  throws an exception.
/// NOTE: You do not need to explicitly create it: a static ChClassFactory
///  class factory is automatically instanced once, at the first time
///  that someone registers a class. It is consistent also across different DLLs.


class ChApi ChClassFactory {
  public:

    ChClassFactory () {
    }
    ~ChClassFactory () {
    }

    //
    // METHODS
    //

    /// Register a class into the global class factory.
    /// Provide an unique name and a ChClassRegistration object.
    /// If multiple registrations with the same name are attempted, only one is done.
    static void ClassRegister(const std::string& keyName, ChClassRegistrationBase* mregistration) {
        ChClassFactory* global_factory = GetGlobalClassFactory();

        global_factory->_ClassRegister(keyName, mregistration);
    }
    
    /// Unregister a class from the global class factory.
    /// Provide an unique name.
    static void ClassUnregister(const std::string& keyName) {
        ChClassFactory* global_factory = GetGlobalClassFactory();

        global_factory->_ClassUnregister(keyName);

        if (global_factory->_GetNumberOfRegisteredClasses()==0)
            DisposeGlobalClassFactory();
    }

    /// Tell if a class is registered, from its name
    static bool IsClassRegistered(const std::string& keyName) {
        ChClassFactory* global_factory = GetGlobalClassFactory();
        return global_factory->_IsClassRegistered(keyName);
    }

    /// Tell the tag name of a class. 
    /// This is the mnemonic name, given at registration, not the typeid.name().
    static std::string& GetClassTagName(const std::type_info& mtype) {
        ChClassFactory* global_factory = GetGlobalClassFactory();
        return global_factory->_GetClassTagName(mtype);
    }

    /// Create from tag name, for registered classes.
    /// The created object is returned in "ptr"
    template <class T>
    static void create(const std::string& keyName, T** ptr) {
        ChClassFactory* global_factory = GetGlobalClassFactory();
        *ptr = reinterpret_cast<T*>(global_factory->_create(keyName));
    }

    /// Create from tag name, for registered classes.
    /// If a static T* ArchiveINconstructor(ChArchiveIn&) function is available, call it instead.
    /// The created object is returned in "ptr"
    template <class T>
    static void create(const std::string& keyName, ChArchiveIn& marchive, T** ptr) {
        ChClassFactory* global_factory = GetGlobalClassFactory();
        *ptr = reinterpret_cast<T*>(global_factory->_archive_in_create(keyName, marchive));
    }

private:
    /// Access the unique class factory here. It is unique even 
    /// between dll boundaries. It is allocated the 1st time it is called, if null.
    static ChClassFactory* GetGlobalClassFactory();

    /// Delete the global class factory
    static void DisposeGlobalClassFactory();

    void _ClassRegister(const std::string& keyName, ChClassRegistrationBase* mregistration)
    {
       class_map[keyName] = mregistration;
       class_map_typeids[mregistration->get_type_index()] = mregistration;
    }

    void _ClassUnregister(const std::string& keyName)
    {
       // GetLog() << " unregister class: " << keyName << "  map n." << class_map.size() << "  map_typeids n." << class_map_typeids.size() << "\n";
       class_map_typeids.erase(class_map[keyName]->get_type_index());
       class_map.erase(keyName);
    }

    bool _IsClassRegistered(const std::string& keyName) {
        const auto &it = class_map.find(keyName);
        if (it != class_map.end())
            return true;
        else 
            return false;
    }

    std::string& _GetClassTagName(const std::type_info& mtype) {
        const auto &it = class_map_typeids.find(std::type_index(mtype));
        if (it != class_map_typeids.end()) {
            return it->second->get_tag_name();
        }
        throw ( ChException("ChClassFactory::GetClassTagName() cannot find the class. Please register it.\n") );
    }

    size_t _GetNumberOfRegisteredClasses() {
        return class_map.size();
    }

    void* _create(const std::string& keyName) {
        const auto &it = class_map.find(keyName);
        if (it != class_map.end()) {
            return it->second->create();
        }
        throw ( ChException("ChClassFactory::create() cannot find the class with name " + keyName + ". Please register it.\n") );
    }
    void* _archive_in_create(const std::string& keyName, ChArchiveIn& marchive) {
        const auto &it = class_map.find(keyName);
        if (it != class_map.end()) {
            return it->second->create(marchive);
        }
        throw ( ChException("ChClassFactory::create() cannot find the class with name " + keyName + ". Please register it.\n") );
    }

private:
    std::unordered_map<std::string, ChClassRegistrationBase*> class_map;
    std::unordered_map<std::type_index, ChClassRegistrationBase*> class_map_typeids;
};


/// Macro to create a  ChDetect_ArchiveINconstructor  
CH_CREATE_MEMBER_DETECTOR(ArchiveINconstructor)




/// Class for registration data of classes 
/// whose objects can be created via a class factory.

template <class t>
class ChClassRegistration : public ChClassRegistrationBase {
  protected:
    //
    // DATA
    //

    /// Name of the class for dynamic creation
    std::string m_sTagName;

  public:
    //
    // CONSTRUCTORS
    //

    /// Creator (adds this to the global list of
    /// ChClassRegistration<t> objects).
    ChClassRegistration(const char* mtag_name) {
        // set name using the 'fake' RTTI system of Chrono
        this->m_sTagName = mtag_name; //t::FactoryClassNameTag();

        // register in global class factory
        ChClassFactory::ClassRegister(this->m_sTagName, this);
    }

    /// Destructor (removes this from the global list of
    /// ChClassRegistration<t> objects).
    virtual ~ChClassRegistration() {

        // register in global class factory
        ChClassFactory::ClassUnregister(this->m_sTagName);
    }

    //
    // METHODS
    //

    virtual void* create() {
        return _create();
    }

    virtual void* create(ChArchiveIn& marchive) {
        return _archive_in_create(marchive);
    }
 
    virtual std::type_index get_type_index() {
        return std::type_index(typeid(t));
    }

    virtual std::string& get_tag_name() {
        return _get_tag_name();
    }

protected:

    template <class Tc=t>
    typename enable_if< std::is_default_constructible<Tc>::value, void* >::type
    _create() {
        return reinterpret_cast<void*>(new Tc);
    }
    template <class Tc=t>
    typename enable_if< !std::is_default_constructible<Tc>::value, void* >::type
    _create() {
        throw ("ChClassFactory::create() failed for class " + std::string(typeid(Tc).name())  + ": it has no default constructor.\n" );
    }

    template <class Tc=t>
    typename enable_if< ChDetect_ArchiveINconstructor<Tc>::value, void* >::type
    _archive_in_create(ChArchiveIn& marchive) {
        return reinterpret_cast<void*>(Tc::ArchiveINconstructor(marchive));
    }
    template <class Tc=t>
    typename enable_if< !ChDetect_ArchiveINconstructor<Tc>::value, void* >::type 
    _archive_in_create(ChArchiveIn& marchive) {
        return reinterpret_cast<void*>(new Tc);
    }

    std::string& _get_tag_name() {
        return m_sTagName;
    }

};






/// MACRO TO MARK CLASSES FOR CLASS FACTORY
/// Different compilers use different name decorations, so typeid(ptr).name() is 
/// not guaranteed to be the same across different platforms/compilers. 
/// ***obsolete***?
#define CH_FACTORY_TAG(classname) \


/// MACRO TO REGISTER A CLASS INTO THE GLOBAL CLASS FACTORY 
/// - Put this macro into a .cpp, where you prefer, but not into a .h header!
/// - Use it as 
///      CH_FACTORY_REGISTER(my_class)


#define CH_FACTORY_REGISTER(classname)                                          \
namespace class_factory {                                                       \
    static ChClassRegistration< classname > classname ## _factory_registration(#classname); \
}  


// Class version registration 

// Default version=0 for class whose version is not registered.
namespace class_factory {
    template<class T>
    class ChClassVersion {
    public:
        static const int version = 0;
    };
}

}  // end namespace chrono



/// Call this macro to register a custom version for a class "classname". 
/// If you do not do this, the default version for all classes is 0.
/// The m_version parameter should be an integer greater than 0.

#define CH_CLASS_VERSION(classname, m_version)                  \
     namespace class_factory {                                  \
      template<>                                                \
      class ChClassVersion<classname> {                         \
      public:                                                   \
        static const int version = m_version;                   \
      };                                                        \
    };                                                          \



#endif
