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
// Authors: Alessandro Tasora
// =============================================================================
//
// Demo code about
// - archives for serialization,
// - serialization, with versioning and dynamic creation (class factory)
//
// =============================================================================

#include <typeinfo>

#include "chrono/serialization/ChArchive.h"
#include "chrono/serialization/ChArchiveBinary.h"
#include "chrono/serialization/ChArchiveAsciiDump.h"
#include "chrono/serialization/ChArchiveJSON.h"
#include "chrono/serialization/ChArchiveXML.h"
#include "chrono/serialization/ChArchiveExplorer.h"

#include "chrono/core/ChGlobal.h"

#include "chrono/core/ChLog.h"
#include "chrono/core/ChVector.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChMatrix.h"
#include "chrono/core/ChException.h"
#include "chrono/solver/ChConstraintTuple.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;

// A bit of forewords on 'enums'.
// Serializing enums to ascii 'human readable' strings is possible by means
// of the three macros CH_ENUM_MAPPER_BEGIN CH_ENUM_VAL CH_ENUM_MAPPER_END.
// Use them as in the following example - possibly implement it right after your enums.
// After this, you have a class called "MyEnum_mapper", inherited
// from ChEnumMapper, and you can use it for converting enums from/to strings.
// If you forget one of the CH_ENUM_VAL, the correspondent enum value will
// be simply converted to a string with a number, and all will work fine.
enum myEnum { ATHLETIC = 0, SKINNY = 3, FAT };

CH_ENUM_MAPPER_BEGIN(myEnum);
CH_ENUM_VAL(ATHLETIC);
CH_ENUM_VAL(SKINNY);
CH_ENUM_VAL(FAT, "fatty");  // overrides the "FAT" mapped string with "fatty"
CH_ENUM_MAPPER_END(myEnum);

// Define some example classes just for showing how CHRONO serialization
// system works, even with inheritance and versioning...
//
// The statements marked with //##### are mandatory if you want to
// take advantage of basic Chrono streaming and serialization mechanisms,
// that is if you want to save/load the object to&from a stream.
//
// The statements marked with //***** are needed only if you want to
// take advantage of Chrono advanced serialization mechanism, that is the
// polymorphic creation (class factory) which can load an
// object from stream even if the object class is not known in advance.
// This more advanced feature requires a 'class factory' registration
// of your object type by means of the CH_FACTORY_REGISTER macro
class myEmployee {
  public:
    int age;
    double wages;
    myEnum body;
    std::string name;

    myEmployee(int m_age = 18, double m_wages = 1020.3, myEnum m_body = myEnum::ATHLETIC, std::string m_name = "John")
        : age(m_age), wages(m_wages), body(m_body), name(m_name){};

    virtual ~myEmployee() {}

    // MEMBER FUNCTIONS FOR BINARY I/O
    // NOTE!!!In order to allow serialization with Chrono approach,
    // at least implement these two functions, with the exact names
    // ArchiveIn() and ArchiveOut():
    virtual void ArchiveOut(ChArchiveOut& marchive)  //##### for Chrono serialization
    {
        // suggested: use versioning
        marchive.VersionWrite<myEmployee>();
        // stream out all member data
        marchive << CHNVP(age);
        marchive << CHNVP(wages);
        myEnum_mapper enum_map;
        marchive << CHNVP(enum_map(body),
                          "body");  // note: CHNVP macro can override names used when streaming to ascii..
    }
    virtual void ArchiveIn(ChArchiveIn& marchive)  // for Chrono serialization
    {
        // suggested: use versioning
        /*int version =*/marchive.VersionRead<myEmployee>();
        // stream in all member data
        marchive >> CHNVP(age);
        marchive >> CHNVP(wages);
        myEnum_mapper enum_map;
        marchive >> CHNVP(enum_map(body), "body");
    }

    // Optional: implement ArchiveContainerName()  so that, when supported as in JSON,
    // serialization of containers (std::vector, arrays, etc.) show a mnemonic name
    // instead of "0", "1", "2", etc. :
    virtual std::string& ArchiveContainerName() { return name; }  //  optional, for advanced serialization
};

// Somewhere in your .cpp code (not in .h headers) you should put the
// 'class factory' registration of your class if you want to deserialize
// objects whose exact class is not known in advance:

CH_FACTORY_REGISTER(myEmployee)  //*****  for advanced serialization

// Ok, now let's do something even more difficult: an inherited class.
class myEmployeeBoss : public myEmployee {
  public:
    bool is_dumb;
    myEmployee slave;

    myEmployeeBoss(const myEmployeeBoss& other) { GetLog() << "------------------copy a boss \n"; }

    myEmployeeBoss(int m_age = 38, double m_wages = 9000.4, bool m_is_dumb = true)
        : myEmployee(m_age, m_wages), is_dumb(m_is_dumb), slave(21, 300){};

    // MEMBER FUNCTIONS FOR BINARY I/O

    virtual void ArchiveOut(ChArchiveOut& marchive)  //##### for Chrono serialization
    {
        // suggested: use versioning
        marchive.VersionWrite<myEmployeeBoss>();
        // remember to serialize the parent class data too!!!
        myEmployee::ArchiveOut(marchive);

        // stream out member data
        marchive << CHNVP(is_dumb);
        marchive << CHNVP(slave);  // this added only from version >1
    }
    virtual void ArchiveIn(ChArchiveIn& marchive)  // for Chrono serialization
    {
        // suggested: use versioning
        int version = marchive.VersionRead<myEmployeeBoss>();
        // remember to deserialize the parent class data too!!!
        myEmployee::ArchiveIn(marchive);

        // stream in member data
        marchive >> CHNVP(is_dumb);
        if (version > 1) {
            marchive >> CHNVP(slave);  // this added only from version >1
        }
    }
};

CH_FACTORY_REGISTER(myEmployeeBoss)  // for advanced serialization

// Use the following to mark a class version:   // optional, for advanced serialization
namespace chrono {
CH_CLASS_VERSION(myEmployeeBoss, 2)
}

// Finally, let's serialize a class that has no default constructor.
// How to manage the (de)serialization of the initialization parameters?
// The trick is adding two optional ArchiveOutConstructor() and ArchiveInConstructor():

class myEmployeeCustomConstructor : public myEmployee {
  public:
    double latitude;
    int kids;
    int legs;

    myEmployeeCustomConstructor(int m_kids, double m_latitude)
        : myEmployee(80, 4000), latitude(m_latitude), kids(m_kids), legs(2){};

    // MEMBER FUNCTIONS FOR BINARY I/O

    virtual void ArchiveOut(ChArchiveOut& marchive)  // for Chrono serialization
    {
        // suggested: use versioning
        marchive.VersionWrite<myEmployeeCustomConstructor>();
        // remember to serialize the parent class data too!!!
        myEmployee::ArchiveOut(marchive);
        // stream out member data (except data used in constructor, already saved in ArchiveOutConstructor)
        marchive << CHNVP(legs);
    }
    virtual void ArchiveIn(ChArchiveIn& marchive)  // for Chrono serialization
    {
        // suggested: use versioning
        /*int version =*/marchive.VersionRead<myEmployeeCustomConstructor>();
        // remember to deserialize the parent class data too!!!
        myEmployee::ArchiveIn(marchive);
        // stream in member data (except data used in constructor, already saved in ArchiveOutConstructor)
        marchive >> CHNVP(legs);
    }

    // Add a  ArchiveOutConstructor  function to deserialize the parameters
    // of the non-default constructor!!!
    virtual void ArchiveOutConstructor(ChArchiveOut& marchive) {
        // suggested: use versioning
        marchive.VersionWrite<myEmployeeCustomConstructor>();

        // serialize the parameters of the constructor:
        marchive << CHNVP(latitude);
        marchive << CHNVP(kids);
    }

    // Add a  ArchiveInConstructor  static function to deserialize the parameters
    // of the non-default constructor!!!
    static void* ArchiveInConstructor(ChArchiveIn& marchive) {
        // suggested: use versioning
        /*int version =*/marchive.VersionRead<myEmployeeCustomConstructor>();

        // 1) Deserialize the parameters of the constructor:
        // you need some auxiliary variables because this method is static
        // (the object will be created right after the >> parsing)
        // Note, be sure that the names of those auxiliary vars are the same of member
        // variables of your class, or use  CHNVP(..., "myname") tags.
        double latitude;
        int kids;
        marchive >> CHNVP(latitude);
        marchive >> CHNVP(kids);

        // 2) Important!!! Finally you MUST return an object of this class,
        // constructed with the parameters that you just deserialized:
        return new myEmployeeCustomConstructor(kids, latitude);
    }
};

CH_FACTORY_REGISTER(myEmployeeCustomConstructor)  //  for advanced serialization

// Example on how to serialize OUT some data:
void my_serialization_example(ChArchiveOut& marchive) {
    // All basic primitives (strings, int,etc.), plus and objects that has
    // an ArchiveOut() function defined can be serialized in archives.

    // Write from transient data into persistent binary file
    double m_double = 0.123456;
    int m_int = -123;
    double m_array[] = {13.1, 15.3, 16.5};
    char m_text[] = "test string";  // better use std::string
    std::string m_string = "hey! stl string";
    std::vector<double> m_stlvector;
    m_stlvector.push_back(2.3);
    m_stlvector.push_back(45.3);
    m_stlvector.push_back(66.44);
    std::list<ChVector<> > m_stllist;
    m_stllist.push_back(ChVector<>(1, 2, 3));
    m_stllist.push_back(ChVector<>(3, 4, 5));
    std::pair<int, double> m_stlpair(120, 0.99);
    std::unordered_map<int, double> m_stlunorderedmap;
    m_stlunorderedmap[12] = 11.2;
    m_stlunorderedmap[41] = 44.8;
    m_stlunorderedmap[34] = 33.6;
    ChMatrixDynamic<double> m_matr_dyn(3, 5);
    m_matr_dyn.fillRandom(0, 10);
    ChMatrixNM<double, 2, 3> m_matr_NM;
    m_matr_NM.fillRandom(-1, +1);
    ChVector<> m_vect(0.5, 0.6, 0.7);
    ChQuaternion<> m_quat(0.1, 0.2, 0.3, 0.4);

    marchive << CHNVP(m_double, "custom_double");  // store data n.1
    marchive << CHNVP(m_int);                      // store data n.2
    marchive << CHNVP(m_array);                    // store data n.3
    marchive << CHNVP(m_text);                     // store data n....
    marchive << CHNVP(m_string);
    marchive << CHNVP(m_stlvector);
    marchive << CHNVP(m_stllist);
    marchive << CHNVP(m_stlpair);
    marchive << CHNVP(m_stlunorderedmap);
    marchive << CHNVP(m_matr_dyn);
    marchive << CHNVP(m_matr_NM);
    marchive << CHNVP(m_vect);
    marchive << CHNVP(m_quat, "m_quaternion", NVP_TRACK_OBJECT);

    // Also store a c++ object
    // In order to use this feature, the classes must implement
    // ArchiveIn and ArchiveOut functions.
    myEmployeeBoss m_boss(53, 12000.34, true);
    m_boss.body = FAT;
    marchive << CHNVP(m_boss);

    // Also store a c++ objects referenced by pointer(s).
    // One could have multiple pointers to the same object:
    // the serialization of pointers takes care of redundancy.
    // In order to use this feature, the classes must implement
    // ArchiveIn and ArchiveOut functions.
    ChVector<>* a_vect = new ChVector<>(1, 2, 3);
    marchive << CHNVP(a_vect);
    delete a_vect;

    // Null pointers can be serialized. They will be deserialized as null.
    ChVector<>* a_null_ptr = 0;
    marchive << CHNVP(a_null_ptr);

    // Also store c++ objects referenced by pointer, using the
    // class abstraction (class factory) mechanism, so that it
    // can be loaded later even if we do not know if it was an object of
    // class 'myEmployee' or specialized class 'myEmployeeBoss'...
    // In order to use this feature, classes must use the CH_FACTORY_REGISTER macros,
    // and must implement ArchiveIn() and ArchiveOut().
    myEmployeeBoss* a_boss = new myEmployeeBoss(64, 22356, false);
    a_boss->slave.age = 24;
    marchive << CHNVP(a_boss);  //  object was referenced by pointer.

    // If another pointer shares the same object instance, you can serialize
    // it too without worrying, because the serialization system will save only
    // the first copy and following copies will just use references.

    myEmployeeBoss* a_boss2 = a_boss;
    marchive << CHNVP(a_boss2);  //  object was referenced by pointer.

    // Also store c++ objects referenced by shared pointers.
    // If classes of pointed objects used CH_FACTORY_REGISTER, class abstraction
    // will be automatically used.
    auto s_boss = chrono_types::make_shared<myEmployeeBoss>();
    marchive << CHNVP(s_boss);  //  object was referenced by shared pointer.

    // Serialize a shared pointer pointing to the same shared resource of s_boss.
    // Note, base class works fine too, as polymorphic object.
    std::shared_ptr<myEmployee> s_boss_b(s_boss);
    marchive << CHNVP(s_boss_b);

    // Serialize null shared pointer
    std::shared_ptr<myEmployeeBoss> null_boss;
    marchive << CHNVP(null_boss);

    // Serialize an object with non-default constructor:
    myEmployeeCustomConstructor* mcustomconstr = new myEmployeeCustomConstructor(3, 40);
    marchive << CHNVP(mcustomconstr);

    // Serialize an object where some pointers are un-linked as external, marking them with unique IDs
    std::vector<ChVector<>*> vect_of_pointers;
    ChVector<>* mvp1 = new ChVector<>(1, 2, 3);
    ChVector<>* mvp2 = new ChVector<>(7, 8, 7);
    vect_of_pointers.push_back(mvp1);
    vect_of_pointers.push_back(mvp2);
    // define that some object should not be serialized, but rather marked with ID for later rebinding
    marchive.UnbindExternalPointer(mvp1, 1001);  // use unique identifier > 0
    marchive << CHNVP(vect_of_pointers);

    delete a_boss;
}

// Example on how to deserialize IN some data:
void my_deserialization_example(ChArchiveIn& marchive) {
    // Read from persistent binary file to transient data
    double m_double;
    int m_int;
    double m_array[3];
    char m_text[12];  // better use std::string
    std::string m_string;
    std::vector<double> m_stlvector;
    std::list<ChVector<> > m_stllist;
    std::pair<int, double> m_stlpair;
    std::unordered_map<int, double> m_stlunorderedmap;
    ChMatrixDynamic<> m_matr_dyn;
    ChMatrixNM<double, 2, 3> m_matr_NM;
    ChVector<> m_vect;
    ChQuaternion<> m_quat;
    myEmployeeBoss m_boss;
    ChVector<>* a_vect;
    ChVector<>* a_null_ptr;

    marchive >> CHNVP(m_double, "custom_double");  // deserialize data n.1
    marchive >> CHNVP(m_int);                      // deserialize data n.2
    marchive >> CHNVP(m_array);                    // deserialize data n.3
    marchive >> CHNVP(m_text);                     // deserialize data n...
    marchive >> CHNVP(m_string);
    marchive >> CHNVP(m_stlvector);
    marchive >> CHNVP(m_stllist);
    marchive >> CHNVP(m_stlpair);
    marchive >> CHNVP(m_stlunorderedmap);
    marchive >> CHNVP(m_matr_dyn);
    marchive >> CHNVP(m_matr_NM);
    marchive >> CHNVP(m_vect);
    marchive >> CHNVP(m_quat, "m_quaternion", NVP_TRACK_OBJECT);

    // Also deserialize the C++ object
    marchive >> CHNVP(m_boss);

    // Also deserialize the C++ pointer: an object will be created!
    marchive >> CHNVP(a_vect);

    // Also deserialize the null C++ pointer: no object is created, and pointer set as null.
    marchive >> CHNVP(a_null_ptr);

    // Also retrieve c++ objects, referenced by a base class pointer, using the
    // class abstraction (class factory) mechanism, so that it
    // can be loaded even if we do not know if it was an object of
    // the base class 'myEmployee' or the specialized 'myEmployeeBoss' class.
    myEmployee* a_boss = 0;
    marchive >> CHNVP(a_boss);  // object will be created

    // Since the two pointers a_boss and a_boss2 were serialized when pointing to
    // the same object instance, now the following will Not create another copy but will
    // automatically point to the same object of a_boss.
    myEmployee* a_boss2 = 0;
    marchive >> CHNVP(a_boss2);

    // Deserialize c++ objects referenced by shared pointers.
    // If classes of pointed objects used CH_FACTORY_REGISTER, class abstraction
    // will be automatically used.
    std::shared_ptr<myEmployeeBoss> s_boss(0);
    marchive >> CHNVP(s_boss);

    // Deserialize a shared pointer pointing to the same resource of s_boss.
    // Since the two pointers s_boss and s_boss_b were serialized when pointing to
    // the same object instance, do not create new, but just point to the same of s_boss.
    // Also, the shared pointer reference count is increased automatically.
    std::shared_ptr<myEmployeeBoss> s_boss_b(0);
    marchive >> CHNVP(s_boss_b);

    // Deserialize a null shared pointer
    std::shared_ptr<myEmployeeBoss> null_boss(0);
    marchive >> CHNVP(null_boss);

    // Deserialize an object with non-default constructor:
    myEmployeeCustomConstructor* mcustomconstr = 0;
    marchive >> CHNVP(mcustomconstr);

    // Deserialize an object where some pointers are re-linked from external pre-existing objects,
    // marking them with unique IDs. Assume a ChVector is already here.
    std::vector<ChVector<>*> vect_of_pointers;
    ChVector<>* mvp1 = new ChVector<>(5, 6, 7);
    marchive.RebindExternalPointer(mvp1, 1001);  // use unique identifier > 0
    marchive >> CHNVP(vect_of_pointers);

    // Just for safety, log some of the restored data:

    GetLog() << "\n\nSome results of deserialization I/O: \n\n"
             << m_text << " \n"
             << m_int << " \n"
             << m_double << "\n";
    GetLog() << m_matr_NM;
    GetLog() << m_vect;
    GetLog() << m_quat;
    GetLog() << m_string.c_str() << "\n";
    GetLog() << m_stlvector;
    GetLog() << m_stlpair;
    GetLog() << m_stlunorderedmap;
    GetLog() << m_boss;
    GetLog() << a_vect;

    if (a_boss) {
        GetLog() << "\n\n We loaded an obj inherited from myEmployee class:\n";
        GetLog() << a_boss;
    }
    if (a_boss2) {
        GetLog() << "\n\n We loaded a 2nd obj inherited from myEmployee class (referencing the 1st):\n";
        GetLog() << a_boss2;
    }
    if (s_boss) {
        GetLog() << "\n\n We loaded a 3nd obj inherited from myEmployee class:\n";
        GetLog() << s_boss;
        GetLog() << "(This object is handled by shared pointers, with ref.count=" << (int)s_boss.use_count() << ")\n";
    }
    if (!null_boss) {
        GetLog() << "\n\n We tried to load a 4th obj with shared pointer, but was null.\n";
    }
    if (mcustomconstr) {
        GetLog() << "\n\n We loaded a 5th object with non-default constructor with 2 parameters.\n";
        GetLog() << mcustomconstr;
    }
    GetLog() << "\n\n We loaded a 6th object where sub-objects were unbind/rebind using IDs:\n";
    GetLog() << vect_of_pointers;
    GetLog() << *vect_of_pointers[0];
    GetLog() << *vect_of_pointers[1];

    GetLog() << "\n";
    GetLog() << "loaded object is a myEmployee?     :" << (dynamic_cast<myEmployee*>(a_boss) != nullptr) << "\n";
    GetLog() << "loaded object is a myEmployeeBoss? :" << (dynamic_cast<myEmployeeBoss*>(a_boss) != nullptr) << "\n";

    delete a_boss;
}

// Example on how to serialize OUT complex Chrono data: 
// we will serialize a ChSystem including its children objects (bodies, links etc.)
void my_system_serialization_example(ChArchiveOut& marchive) {

    // ..create a system:
    ChSystemNSC sys;

    // ..create a truss
    auto my_body_A = chrono_types::make_shared<ChBody>();
    sys.AddBody(my_body_A);
    my_body_A->SetBodyFixed(true);  // truss does not move!

    // ..create a flywheel
    auto my_body_B = chrono_types::make_shared<ChBody>();
    sys.AddBody(my_body_B);
    my_body_B->SetPos(ChVector<>(0, 0, 0)); 

    // ..create a constraint, i.e. a motor between flywheel and truss:
    auto my_link_AB = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    my_link_AB->Initialize(my_body_A, my_body_B, ChFrame<>(ChVector<>(0, 0, 0)));
    sys.AddLink(my_link_AB);
    auto my_speed_function = chrono_types::make_shared<ChFunction_Const>(CH_C_PI);  // speed w=3.145 rad/sec
    my_link_AB->SetSpeedFunction(my_speed_function);

    // Serialize all the physical system (including bodies, links etc.),
    // it takes just a single line:
    marchive << CHNVP(sys); 
}


// Example on how to serialize IN complex Chrono data:
// we will deserialize a ChSystem including its children objects (bodies, links etc.)
void my_system_deserialization_example(ChArchiveIn& marchive) {

    ChSystemNSC sys;

    // deserialize all the physical system (including bodies, links etc.),
    // it takes just a single line:
    marchive >> CHNVP(sys);  
}


// Example on how to use reflection (c++ introspection) to explore the properties exposed
// through the ArchiveIn() and ArchiveOut() functions.
void my_reflection_example() {
    ChArchiveExplorer mexplorer;
    mexplorer.SetUseWildcards(true);
    mexplorer.SetUseUserNames(true);

    myEmployeeBoss m_boss(71, 42000.4, true);
    m_boss.body = FAT;

    double my_wages;
    if (mexplorer.FetchValue(my_wages, m_boss, "wages"))
        GetLog() << "Property explorer : retrieved 'wages'=" << my_wages << "\n";
    else
        GetLog() << "Property explorer : cannot retrieve 'wages'! \n";

    myEmployee my_slave;
    if (mexplorer.FetchValue(my_slave, m_boss, "slave"))
        GetLog() << "Property explorer : retrieved 'slave'=\n" << my_slave << "\n";
    else
        GetLog() << "Property explorer : cannot retrieve 'slave'! \n";

    int my_age;
    if (mexplorer.FetchValue(my_age, m_boss, "s?*e/age"))
        GetLog() << "Property explorer : retrieved 'slave/age'=" << my_age << "\n";
    else
        GetLog() << "Property explorer : cannot retrieve 'slave/age'! \n";

    int my_foo = 123;
    if (mexplorer.FetchValue(my_foo, m_boss, "foo"))
        GetLog() << "Property explorer : retrieved 'int foo'=" << my_foo << "\n";
    else
        GetLog() << "Property explorer : cannot retrieve 'int foo'! \n";

    // Test access to containers (std::vector, arrays, etc.). Elements can
    // be fetched using two approaches: integer indexes or menmonic names.
    std::vector<myEmployee> mcontainer;
    mcontainer.push_back(myEmployee(19, 4000, ATHLETIC, "Josh"));
    mcontainer.push_back(myEmployeeBoss(29, 5000, "Jeff"));
    mcontainer.push_back(myEmployee(31, 6000, ATHLETIC, "Marie"));
    myEmployee a_employee;

    // Method A: just use the index in the search string,
    //   ex: "stuff/arrayofpositions/6/x" as in:
    if (mexplorer.FetchValue(a_employee, mcontainer, "1"))
        GetLog() << "Property explorer : retrieved from element number in container '1' =\n" << a_employee << "\n";
    else
        GetLog() << "Property explorer : cannot retrieve from element number! \n";

    // Method B: if you deal if working with objects that implement
    // ArchiveContainerName(), you can use the name between single quotation marks '...',
    //   ex: "stuff/arrayofbodies/'Crank'/mass" as in:
    if (mexplorer.FetchValue(a_employee, mcontainer, "'Marie'"))
        GetLog() << "Property explorer : retrieved from element container name 'Marie' =\n" << a_employee << "\n";
    else
        GetLog() << "Property explorer : cannot retrieve from element container name! \n";

    // Test if some object can be explored
    GetLog() << "This has sub properties? : " << mexplorer.IsObject(mcontainer) << "\n";
    GetLog() << "This has sub properties? : " << mexplorer.IsObject(my_age) << "\n\n";

    // Fetch all subproperties of an object using "*"
    GetLog() << "List of fetched properties in std::vector of employees: \n";
    auto props = mexplorer.FetchValues(mcontainer, "*");
    for (auto i : props) {
        GetLog() << " val: " << i->name() << ",  reg.class: " << i->GetClassRegisteredName()
                 << ",  typeid: " << i->GetTypeidName() << "\n";
        // if (auto pi = dynamic_cast<myEmployeeBoss*>(i->PointerUpCast<myEmployee>()))
        //    GetLog() <<"   castable to myEmployeeBoss \n";
        ChArchiveExplorer mexplorer2;
        auto props2 = mexplorer2.FetchValues(*i, "*");
        for (auto i2 : props2) {
            GetLog() << "    val: " << i2->name() << ",  reg.class: " << i2->GetClassRegisteredName()
                     << ",  typeid: " << i2->GetTypeidName() << "\n";
        }
    }
}





int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    GetLog() << "CHRONO foundation classes demo: archives (serialization)\n\n";

    // Create (if needed) output directory
    const std::string out_dir = GetChronoOutputPath() + "DEMO_ARCHIVE";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // Archives inherited from the base class ChArchiveOut can be
    // used to serialize objects, and streams inherited from ChArchiveIn
    // can be used to get them back. For example, file streams like
    // ChArchiveOutBinary and ChArchiveInBinary can be used for this
    // purpose.


    // Example: SERIALIZE TO ASCII DUMP (useful for debugging etc.):
    {
        std::string asciifile = out_dir + "/foo_archive.txt";
        ChStreamOutAsciiFile mfileo(asciifile.c_str());

        // Create an ASCII archive object, for dumping C++ objects into a readable file
        ChArchiveAsciiDump marchiveout(mfileo);

        my_serialization_example(marchiveout);
    }

    // Example: SERIALIZE TO/FROM BINARY:
    {
        std::string binfile = out_dir + "/foo_archive.dat";
        ChStreamOutBinaryFile mfileo(binfile.c_str());

        // Use a binary archive object to serialize C++ objects into the binary file
        ChArchiveOutBinary marchiveout(mfileo);

        my_serialization_example(marchiveout);
    }

    {
        std::string binfile = out_dir + "/foo_archive.dat";
        ChStreamInBinaryFile mfilei(binfile.c_str());

        // Use a binary archive object to deserialize C++ objects from the binary file
        ChArchiveInBinary marchivein(mfilei);

        my_deserialization_example(marchivein);
    }

    // Example: SERIALIZE TO/FROM JSON:
    {
        std::string jsonfile = out_dir + "/foo_archive.json";
        ChStreamOutAsciiFile mfileo(jsonfile.c_str());

        // Use a JSON archive object to serialize C++ objects into the file
        ChArchiveOutJSON marchiveout(mfileo);

        my_serialization_example(marchiveout);
    }

    {
        std::string jsonfile = out_dir + "/foo_archive.json";
        ChStreamInAsciiFile mfilei(jsonfile.c_str());

        // Use a JSON archive object to deserialize C++ objects from the file
        ChArchiveInJSON marchivein(mfilei);

        my_deserialization_example(marchivein);
    }

    // Example: SERIALIZE TO/FROM XML
    {
        std::string xmlfile = out_dir + "/foo_archive.xml";
        ChStreamOutAsciiFile mfileo(xmlfile.c_str());

        // Use a XML archive object to serialize C++ objects into the file
        ChArchiveOutXML marchiveout(mfileo);

        my_serialization_example(marchiveout);
    }

    {
        std::string xmlfile = out_dir + "/foo_archive.xml";
        ChStreamInAsciiFile mfilei(xmlfile.c_str());

        // Use a XML archive object to deserialize C++ objects from the file
        ChArchiveInXML marchivein(mfilei);

        my_deserialization_example(marchivein);
    }

    GetLog() << "Serialization test ended with success.\n\n";


    // Example: SERIALIZE A FULL CHRONO SYSTEM TO/FROM JSON
    {
        std::string jsonfile = out_dir + "/chsystem_archive.json";
        ChStreamOutAsciiFile mfileo(jsonfile.c_str());

        // Use a JSON archive object to serialize C++ objects into the file
        ChArchiveOutJSON marchiveout(mfileo);

        my_system_serialization_example(marchiveout);
    }

    {
        std::string jsonfile = out_dir + "/chsystem_archive.json";
        ChStreamInAsciiFile mfilei(jsonfile.c_str());

        // Use a JSON archive object to deserialize C++ objects from the file
        ChArchiveInJSON marchivein(mfilei);

        my_system_deserialization_example(marchivein);
    }

    GetLog() << "Serialization of ChSystem ended with success.\n\n";
        

    my_reflection_example();

    GetLog() << "Reflection test ended with success.\n";


    return 0;
}
