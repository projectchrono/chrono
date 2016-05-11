//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//


//////////////////////////////////////////////////
//
//   Demos code about
//
//     - archives for serialization,
//     - serialization, with versioning and dynamic
//       creation (class factory)
//     - class runtime type identification, without
//       enabling the RTTI compiler feature
//
///////////////////////////////////////////////////

#include <typeinfo>

#include "chrono/serialization/ChArchive.h"
#include "chrono/serialization/ChArchiveBinary.h"
#include "chrono/serialization/ChArchiveAsciiDump.h"
#include "chrono/serialization/ChArchiveJSON.h"
#include "chrono/core/ChLog.h"
#include "chrono/core/ChVector.h"
#include "chrono/core/ChMatrixDynamic.h"
#include "chrono/core/ChMatrix.h"
#include "chrono/core/ChException.h"
#include "chrono/lcp/ChLcpConstraintTuple.h"


using namespace chrono;


/// A bit of forewords on 'enums'. 
/// Serializing enums to ascii 'human readable' strings is possible by means
/// of the three macros CH_ENUM_MAPPER_BEGIN CH_ENUM_VAL CH_ENUM_MAPPER_END.
/// Use them as in the following example - possibly implement it right after your enums.
/// After this, you have a class called "MyEnum_mapper", inherited
/// from ChEnumMapper, and you can use it for converting enums from/to strings.
/// If you forget one of the CH_ENUM_VAL, the correspondent enum value will
/// be simply converted to a string with a number, and all will work fine.

enum myEnum {
     ATHLETIC = 0,
     SKINNY = 3,
     FAT
};

CH_ENUM_MAPPER_BEGIN(myEnum);
  CH_ENUM_VAL(ATHLETIC);
  CH_ENUM_VAL(SKINNY);
  CH_ENUM_VAL(FAT, "fatty");  // overrides the "FAT" mapped string with "fatty"
CH_ENUM_MAPPER_END(myEnum);


//
// Define some example classes just for showing how CHRONO serialization
// system works, even with inheritance and versioning...
//
// The statements marked with //##### are mandatory if you want to
// take advantage of basic Chrono streaming and serialization mechanisms,
// that is if you want to save/load the object to&from a stream.
//
// The statements marked with //***** are needed only if you want to
// take advantage of Chrono advanced serialization mechanism, that is the
// abstract class creation (class factory) which can load an
// object from stream even if the object class is not known in advance.
// This more advanced feature requires a 'class factory' registration
// of your object type by means of the ChClassRegister<> statement (in
// its turn, it requires that your object has also the Chrono simulation
// of run-time-type information enabled, that is the CH_RTTI_.. macros).
//

class myEmployee {
    // Remember to enable the Chrono RTTI features with the CH_RTTI_.. macro
    // if you want to deserialize objects whose exact class is not known in advance!

    CH_RTTI_ROOT(myEmployee)  //***** for _advanced_ Chrono serialization

  public:
    int age;
    double wages;
    myEnum body;

    myEmployee(int m_age = 18, double m_wages = 1020.3, myEnum m_body = myEnum::ATHLETIC) : 
        age(m_age), 
        wages(m_wages),
        body(m_body){};

    // MEMBER FUNCTIONS FOR BINARY I/O
    // NOTE!!!In order to allow serialization with Chrono approach,
    // at least implement these two functions, with the exact names
    // ArchiveIN() and ArchiveOUT():

    virtual void ArchiveOUT(ChArchiveOut& marchive)  //##### for Chrono serialization
    {
        // suggested: use versioning
        marchive.VersionWrite(1);
        // stream out all member data
        marchive << CHNVP(age);
        marchive << CHNVP(wages);
        myEnum_mapper enum_map;
        marchive << CHNVP(enum_map(body), "body"); // note: CHNVP macro can override names used when streaming to ascii..
    }
    virtual void ArchiveIN(ChArchiveIn& marchive)  //##### for Chrono serialization
    {
        // suggested: use versioning
        int version = marchive.VersionRead();
        // stream in all member data
        marchive >> CHNVP(age);
        marchive >> CHNVP(wages);
        myEnum_mapper enum_map;
        marchive >> CHNVP(enum_map(body), "body");
    }

};


// Somewhere in your cpp code (not in .h headers!) you should put the
// 'class factory' registration of your class, assuming it has the CH_RTTI_..,
// if you want to use the AbstractReadCreate() feature, to deserialize
// objects whose exact class is not known in advance.

chrono::ChClassRegister<myEmployee> a_registration1;  //***** for _advanced_ Chrono serialization



// Ok, now let's do something even more difficult: an inherited class.
// Note the CH_RTTI macro. 

class myEmployeeBoss : public myEmployee {
    CH_RTTI(myEmployeeBoss, myEmployee)  //***** for _advanced_ Chrono serialization

  public:
    bool is_dumb;
    myEmployee slave;


    myEmployeeBoss(int m_age = 38, double m_wages = 9000.4, bool m_is_dumb = true)
        : myEmployee(m_age, m_wages), 
        is_dumb(m_is_dumb),
        slave(21, 300){};

    // MEMBER FUNCTIONS FOR BINARY I/O

    virtual void ArchiveOUT(ChArchiveOut& marchive)  //##### for Chrono serialization
    {
        // suggested: use versioning
        marchive.VersionWrite(2);
        // remember to serialize the parent class data too!!!
        myEmployee::ArchiveOUT(marchive);

        // stream out member data
        marchive << CHNVP(is_dumb);
        marchive << CHNVP(slave);  // this added only from version >1
    }
    virtual void ArchiveIN(ChArchiveIn& marchive)  //##### for Chrono serialization
    {
        // suggested: use versioning
        int version = marchive.VersionRead();
        // remember to deserialize the parent class data too!!!
        myEmployee::ArchiveIN(marchive);

        // stream in member data
        marchive >> CHNVP(is_dumb);
        if (version > 1){
            marchive >> CHNVP(slave);  // this added only from version >1
        }
    }

};

chrono::ChClassRegister<myEmployeeBoss> a_registration2;  //***** for _advanced_ Chrono serialization



//
// Example on how to serialize OUT some data:
//

void my_serialization_example(ChArchiveOut& marchive)
{
        // All basic primitives (strings, int,etc.), plus and objects that has
        // an ArchiveOUT() function defined can be serialized in archives.      

        // Write from transient data into persistent binary file
        double m_double = 0.123456;
        int m_int = -123;
        double m_array[] = {13.1, 15.3, 16.5};
        char m_text[] = "test string"; // better use std::string
        std::string m_string = "hey! stl string";
        std::vector< double > m_stlvector; 
        m_stlvector.push_back (2.3); 
        m_stlvector.push_back (45.3);
        m_stlvector.push_back (66.44);
        std::list< ChVector<> > m_stllist; 
        m_stllist.push_back ( ChVector<>(1,2,3) ); 
        m_stllist.push_back ( ChVector<>(3,4,5) );
        ChMatrixDynamic<double> m_matr(3, 5);
        m_matr.FillRandom(10, 0);
        ChVector<> m_vect(0.5, 0.6, 0.7);
        ChQuaternion<> m_quat(0.1, 0.2, 0.3, 0.4);  
   
        marchive << CHNVP(m_double,"custom double");  // store data n.1      
        marchive << CHNVP(m_int);     // store data n.2 
        marchive << CHNVP(m_array);   // store data n.3
        marchive << CHNVP(m_text);    // store data n....
        marchive << CHNVP(m_string);  
        marchive << CHNVP(m_stlvector);
        marchive << CHNVP(m_stllist);
        marchive << CHNVP(m_matr);    
        marchive << CHNVP(m_vect);
        marchive << CHNVP(m_quat, "m_quaternion", NVP_TRACK_OBJECT);  
        
        // Also store a c++ object 
        // In order to use this feature, the classes must implement 
        // ArchiveIN and ArchiveOUT functions.
        myEmployeeBoss m_boss(53, 12000.34, true);
        m_boss.body = FAT;
        marchive << CHNVP(m_boss);    

        // Also store a c++ objects referenced by pointer(s).
        // One could have multiple pointers to the same object: 
        // the serialization of pointers takes care of redundancy.
        // In order to use this feature, the classes must implement 
        // ArchiveIN and ArchiveOUT functions.
        ChVector<>* a_vect = new ChVector<>(1,2,3); 
        marchive << CHNVP(a_vect);
        delete a_vect;

        // Null pointers can be serialized. They will be deserialized as null.
        ChVector<>* a_null_ptr = 0; 
        marchive << CHNVP(a_null_ptr);

        // Also store c++ objects referenced by pointer, using the
        // class abstraction (class factory) mechanism, so that it
        // can be loaded later even if we do not know if it was an object of
        // class 'myEmployee' or specialized class 'myEmployeeBoss'...
        // In order to use this feature, the classes must implement 
        // CH_RTTI_ROOT or CH_RTTI functions and ArchiveIN and ArchiveOUT
        myEmployeeBoss* a_boss = new myEmployeeBoss(64, 22356, false);
        a_boss->slave.age = 24;
        marchive << CHNVP(a_boss);  //  object was referenced by pointer.

        // If another pointer shares the same object instance, you can serialize
        // it too without worrying, because the serialization system will save only
        // the first copy and following copies will just use references.

        myEmployeeBoss* a_boss2 = a_boss;
        marchive << CHNVP(a_boss2);  //  object was referenced by pointer.

        // Also store c++ objects referenced by shared pointers.
        // If pointed objects objects have CH_RTTI, the class abstraction
        // vill be automatically used.
        auto s_boss = std::make_shared<myEmployeeBoss>();
        marchive << CHNVP(s_boss);  //  object was referenced by shared pointer.

        // Serialize null shared pointer
        std::shared_ptr<myEmployeeBoss> null_boss;
        marchive << CHNVP(null_boss); 

        delete a_boss;
}


//
// Example on how to deserialize IN some data:
//

void my_deserialization_example(ChArchiveIn& marchive)
{  
        // Read from persistent binary file to transient data
        double m_double;
        int m_int;
        double m_array[3];
        char m_text[12]; // better use std::string
        std::string m_string;
        std::vector< double > m_stlvector;
        std::list< ChVector<> > m_stllist;
        ChMatrixDynamic<> m_matr;
        ChVector<> m_vect;
        ChQuaternion<> m_quat;
        myEmployeeBoss m_boss;
        ChVector<>* a_vect;
        ChVector<>* a_null_ptr;

        marchive >> CHNVP(m_double,"custom double");  // deserialize data n.1
        marchive >> CHNVP(m_int);     // deserialize data n.2
        marchive >> CHNVP(m_array);   // deserialize data n.3
        marchive >> CHNVP(m_text);    // deserialize data n....
        marchive >> CHNVP(m_string);  
        marchive >> CHNVP(m_stlvector);
        marchive >> CHNVP(m_stllist);
        marchive >> CHNVP(m_matr);
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
        // the base class 'myEmployee' or the specialized 'myEmployeeBoss' class..
        myEmployee* a_boss = 0;
        marchive >> CHNVP(a_boss);  // object will be created

        // Since the two pointers a_boss and a_boss2 were serialized when pointing to 
        // the same object instance, now the following will Not create another copy but will
        // automatically point to the same object of a_boss. 
        myEmployee* a_boss2 = 0;
        marchive >> CHNVP(a_boss2); 


        // Also store c++ objects referenced by shared pointers.
        // If pointed objects objects have CH_RTTI, the class abstraction
        // will be automatically used.
        std::shared_ptr<myEmployeeBoss> s_boss(0);
        marchive >> CHNVP(s_boss);

        // Deserialize a null shared pointer
        std::shared_ptr<myEmployeeBoss> null_boss(0);
        marchive >> CHNVP(null_boss);


        // Just for safety, log some of the restored data:

        GetLog() << "\n\nResult of deserialization I/O: \n " << m_text << " \n " << m_int << " \n " << m_double << "\n";
        GetLog() << m_matr;
        GetLog() << m_vect;
        GetLog() << m_quat;
        GetLog() << m_string.c_str() << "\n";
        GetLog() << m_stlvector;
        GetLog() << m_boss;
        GetLog() << a_vect;

        if (a_boss) {
            GetLog() << "\n\n We loaded an obj inherited from myEmployee class:\n";
            GetLog() << *a_boss;

        if (a_boss2) {
            GetLog() << "\n\n We loaded a 2nd obj inherited from myEmployee class (referencing the 1st):\n";
            GetLog() << *a_boss2;
        }
        if (s_boss) {
            GetLog() << "\n\n We loaded a 3nd obj inherited from myEmployee class:\n";
            GetLog() << *(s_boss);
        }
        if (!null_boss) {
            GetLog() << "\n\n We tried to load a 4th obj with shared pointer, but was null.\n";
        }

            // By the way, now show some feaures of Chrono run-time-type-identifier
            // methods, since class had CH_RTTI enabled.
            // Note that Chrono RTTI is _not_ the standard C++ RTTI! Also, it
            // does not need RTTI to be enabled in compilation. It is platform-
            // and compiler-independent.
            // The only drawback of Chrono RTTI is that it is a bit slower than
            // standard C++ RTTI, and it introduces a virtual method in the class (not a
            // big problem however, since also C++ RTTI eats some memory...)

            GetLog() << "\n";
            GetLog() << "loaded object is a myEmployee?     :" << ChIsExactlyClass(myEmployee, a_boss) << "\n";
            GetLog() << "loaded object is a myEmployeeBoss? :" << ChIsExactlyClass(myEmployeeBoss, a_boss) << "\n";
            GetLog() << "loaded object derives myEmployee?  :" << ChIsDerivedFromClass(myEmployee, a_boss) << "\n";
            GetLog() << "Obvious! we loaded an object of class: " << a_boss->GetRTTI()->GetName() << "\n";
            delete a_boss;
        }
}



int main(int argc, char* argv[]) {

    GetLog() << "\n"
             << "CHRONO foundation classes demo: archives (serialization)\n\n";

    
    //  Archives inherited from the base class ChArchiveOut can be
    // used to serialize objects, and streams inherited from ChArchiveIn
    // can be used to get them back. For example, file streams like
    // ChArchiveOutBinary and ChArchiveInBinary can be used for this
    // purpose.
    
    try {       
        
        {
            //
            // Example: SERIALIZE TO ASCII DUMP (useful for debugging etc.):
            //

        
            ChStreamOutAsciiFile mfileo("foo_archive.txt");

            // Create a binary archive, that uses the binary file as storage.
            ChArchiveAsciiDump marchiveout(mfileo);
        
            my_serialization_example(marchiveout);
        }


        {
            //
            // Example: SERIALIZE TO/FROM BINARY:
            //

            {
                ChStreamOutBinaryFile mfileo("foo_archive.dat");

                // Create a binary archive, that uses the binary file as storage.
                ChArchiveOutBinary marchiveout(mfileo);
        
                my_serialization_example(marchiveout);
            }

            {
                ChStreamInBinaryFile mfilei("foo_archive.dat");

                // Create a binary archive, that uses the binary file as storage.
                ChArchiveInBinary marchivein(mfilei);

                 my_deserialization_example(marchivein);
            }
        }


        {
            //
            // Example: SERIALIZE TO/FROM JSON:
            //

            {
                ChStreamOutAsciiFile mfileo("foo_archive.json");

                // Create a binary archive, that uses the binary file as storage.
                ChArchiveOutJSON marchiveout(mfileo);
        
                my_serialization_example(marchiveout);
            }

            
            {
                ChStreamInAsciiFile mfilei("foo_archive.json");

                // Create a binary archive, that uses the binary file as storage.
                ChArchiveInJSON marchivein(mfilei);

                my_deserialization_example(marchivein);
            }
            
        }
 

        GetLog() << "Serialization test ended with success.\n";

    } catch (ChException myex) {
        GetLog() << "ERROR: " << myex.what() << "\n\n";
    }


    return 0;
}
