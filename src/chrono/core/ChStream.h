// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================

#ifndef CHSTREAM_H
#define CHSTREAM_H

#include <cstdio>
#include <cstring>
#include <cassert>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <vector>
#include <ios>

#include "chrono/core/ChException.h"
#include "chrono/core/ChApiCE.h"

namespace chrono {
/// Ugly hack added by hammad to get code to compile on osx.
/// Compiler had trouble finding the create function,
/// adding #incldue to ChClassRegister made other things break in ChLog so I couldn't do that....
template <class T>
void create(std::string cls_name, T** ppObj);

///
/// This is a base class for input/output (streaming)
///

class ChApi ChStream {
  public:
    /// Modes for chrono files (the ch-modes) - obsolete -
    enum eChMode {
        CHFILE_NORMAL = 0,  ///< writes/reads as a normal file
        CHFILE_NOWRITE,     ///< force do not write even if in w or wb (write mode)
        CHFILE_SAFEWRITE,   ///< safe writing (obsolete)
        CHFILE_OPENLATER,   ///< creates the file only as soon as first writing is attempted
    };

    /// Errors for chrono files (the ch-modes)
    enum eChStreamError {
        CHSTREAM_OK = 0,  ///< write/read operation successfull
        CHSTREAM_EOF,     ///< end of file
        CHSTREAM_FAIL,    ///< operation failed
    };

  protected:
    //
    // DATA
    //
};

///
/// This is a base class for all OUTPUT streams.
/// Does nothing special - it must be implemented by child classes
///

class ChApi ChStreamOut : public ChStream {
  public:
    ChStreamOut();

    virtual ~ChStreamOut();

  protected:
    /// Outputs a raw chunk of data, from pointer 'data' up to 'n'
    /// chars (bytes).
    /// NOTE! Inherited class could override only this function to have
    /// everything working, for example to output into a message box,
    /// or to sockets, etc..
    virtual void Output(const char* data, size_t n) = 0;
};

///
/// This is a base class for all INPUT streams.
/// Does nothing special - it must be implemented by child classes
///

class ChApi ChStreamIn : public ChStream {
  public:
    ChStreamIn();

    virtual ~ChStreamIn();

    /// Returns true if end of stream reached.
    /// Child classes should implement it.
    virtual bool End_of_stream() { return true; };

  protected:
    /// Inputs a raw chunk of data, from pointer 'data' up to 'n'
    /// chars (bytes).
    /// NOTE! Inherited class could override only this function to have
    /// everything working, for example to output into a message box,
    /// or to sockets, etc..
    virtual void Input(char* data, size_t n) = 0;
};

///
/// This is a base class for all ASCII OUTPUT streams, in the
/// sense that number are formatted into readable strings, etc.
/// Defines some << operators from basic
/// types, converting all them into calls to the Output() function.
/// Also, defines the Format() function, which can be used a bit
/// like the printf formatting.
/// At least some inherited class should be used by the user in
/// order to get some practical effect (file saving, etc.)
///

class ChApi ChStreamOutAscii : public ChStreamOut {
  protected:
    char number_format[10];
    char comment_trailer[10];

  public:
    ChStreamOutAscii();

    virtual ~ChStreamOutAscii();

    //  Operators

    /// Generic << operator for all classes which implement serialization
    /// by means of a method named  'void StreamOUT(ChStreamOutAscii&)'

    ChStreamOutAscii& operator<<(bool bVal);
    ChStreamOutAscii& operator<<(char tch);
    ChStreamOutAscii& operator<<(const int nVal);
    ChStreamOutAscii& operator<<(const double dVal);
    ChStreamOutAscii& operator<<(const float dVal);
    ChStreamOutAscii& operator<<(unsigned int unVal);
    ChStreamOutAscii& operator<<(char* str);
    ChStreamOutAscii& operator<<(const char* str);
    ChStreamOutAscii& operator<<(std::string& str);
    ChStreamOutAscii& operator<<(unsigned long unVal);
    ChStreamOutAscii& operator<<(unsigned long long unVal);

    ///***OBSOLETE*** use ChArchive << operator and ArchiveOUT()
    template <class T>
    ChStreamOutAscii& operator<(T obj) {
        obj.StreamOUT(*this);
        return *this;
    }

    /// Output the log message in form of a formatted string, like in printf()
    virtual void Format(char* formatString, ...);

    /// Set the formatting string (ex "%f" or "%g" etc.) for float->text conversion
    void SetNumFormat(const char* mf) {
        if (strlen(mf) < 10)
            strcpy(number_format, mf);
    }
    char* GetNumFormat() { return number_format; }
    /// Set the trailer symbol before each comment (example: "#" , or "//" etc.)
    void SetCommentTrailer(char* mt) {
        if (strlen(mt) < 10)
            strcpy(comment_trailer, mt);
    }

    /// Outputs carriage return
    void CR() { *this << "\n"; };

    /// Output tab
    void TAB() { *this << "\t"; };

    /// Output comment with preceding trailer (ex #)
    void Comment(char m_string[]) {
        TAB();
        TAB();
        TAB();
        *this << comment_trailer << m_string;
        CR();
    }
    /// Output a separation bar in Ascii file
    void Bar() { *this << comment_trailer << "#------------------------------------------------------------ \n"; };
};

///
/// This is a base class for all ASCII INPUT streams, in the
/// sense that number are formatted into readable strings, etc.
/// Defines some << operators from basic
/// types, converting all them into calls to the Input() function.
/// At least some inherited class should be used by the user in
/// order to get some practical effect (file loading, etc.)
///

class ChApi ChStreamInAscii : public ChStreamIn {
  protected:
    char number_format[10];

  public:
    ChStreamInAscii();

    virtual ~ChStreamInAscii();

    //  Operators

    virtual ChStreamInAscii& operator>>(bool& bVal);
    virtual ChStreamInAscii& operator>>(char& tch);
    virtual ChStreamInAscii& operator>>(int& nVal);
    virtual ChStreamInAscii& operator>>(double& dVal);
    virtual ChStreamInAscii& operator>>(float& dVal);
    virtual ChStreamInAscii& operator>>(unsigned int& unVal);
    virtual ChStreamInAscii& operator>>(char* str);
    virtual ChStreamInAscii& operator>>(std::string& str);

    /// Set the formatting string (ex "%f" or "%g" etc.) for text->float conversion
    void SetNumFormat(const char* mf) {
        if (strlen(mf) < 10)
            strcpy(number_format, mf);
    }
};

/// Templated function for swapping bytes of objects of type 'T',
/// in general fo rwhatever T type. This is used for cross-platform
/// compatibility when sharing objects between big-endian and little-endian
/// memory models, depending on the microprocessor type.

template <class T>
inline void StreamSwapBytes(T* ptData) {
    char* acBytes = (char*)ptData;
    int iSize = (int)(sizeof(T));
    int iSizeM1 = iSize - 1;
    int iHSize = iSize / 2;
    for (int i0 = 0, i1 = iSizeM1; i0 < iHSize; i0++, i1--) {
        char cSave = acBytes[i0];
        acBytes[i0] = acBytes[i1];
        acBytes[i1] = cSave;
    }
}

///
/// Base class for streams (either in or out) based on
/// binary formats.
///   This class implements a basic functionality
/// about platform-independent treatment of data (the
/// big-endian/little-endian issue).
///   Also this class implements functionality to avoid
/// storing multiple times the same object (persistent
/// data archive).
///

class ChApi ChBinaryArchive {
  protected:
    bool big_endian_machine;

    /// vector of pointers to stored/retrieved objects,
    /// to avoid saving duplicates or deadlocks
    std::vector<void*> objects_pointers;

  public:
    ChBinaryArchive();

    virtual ~ChBinaryArchive();

    /// Returns true if the machine where the code runs
    /// has big endian byte ordering, returns false otherwise.
    bool IsBigEndianMachine();

    /// Reinitialize the vector of pointers to loaded/saved objects
    void Init() {
        big_endian_machine = IsBigEndianMachine();
        objects_pointers.clear();
        objects_pointers.push_back(NULL);
    }
    /// Put a pointer in pointer vector, but only if it
    /// was not previously interted. Returns position of pointer
    /// if already existing, otherwise -1.
    int PutPointer(void* object) {
        for (size_t i = 0; i < objects_pointers.size(); ++i) {
            if (objects_pointers[i] == object)
                return (int)i;
        }
        // wasn't in list.. add to it
        objects_pointers.push_back(object);

        return -1;
    }
};

///
/// This is a base class for all BINARY OUTPUT streams, in a way such
/// that the stream is platform indepent (see the 'little endian' stuff
/// in 'floating point to persistent data' topics..)
/// Defines some << operators from basic
/// types, converting all them into calls to the Output() function.
/// At least some inherited class should be used by the user in
/// order to get some practical effect (file saving, etc.)
///

class ChApi ChStreamOutBinary : public ChStreamOut, public ChBinaryArchive {
  private:
  public:
    ChStreamOutBinary();

    virtual ~ChStreamOutBinary();

    //  Operators

    /// Generic << operator for all classes which implement serialization
    /// by means of a method named  'void StreamOUT(ChStreamOutBinary&)'
    ///***OBSOLETE*** use ChArchive << operator and ArchiveOUT()
    template <class T>
    ChStreamOutBinary& operator<(T& obj) {
        obj.StreamOUT(*this);
        return *this;
    }

    /// Specialized operators for basic primitives (numbers like long,
    /// double, int. etc., and booleans)
    /// Handle byte ordering issues in little.endian/big.endian representations,
    /// for cross platform portability.
    ChStreamOutBinary& operator<<(char Val);
    ChStreamOutBinary& operator<<(bool Val);
    ChStreamOutBinary& operator<<(int Val);
    ChStreamOutBinary& operator<<(unsigned int Val);
    ChStreamOutBinary& operator<<(double Val);
    ChStreamOutBinary& operator<<(float Val);
    ChStreamOutBinary& operator<<(std::string& str);
    ChStreamOutBinary& operator<<(long Val);
    ChStreamOutBinary& operator<<(unsigned long Val);
    ChStreamOutBinary& operator<<(unsigned long long Val);

    /// Specialized operator for C strings.
    ChStreamOutBinary& operator<<(const char* str);
    ChStreamOutBinary& operator<<(char* str);

    /// Generic operator for binary streaming of generic objects.
    /// WARNING!!! raw byte streaming! If class 'T' contains double,
    /// int, long, etc, these may give problems when loading on another
    /// platform which has big-endian ordering, if generated on a little-edian platform...
    template <class T>
    void GenericBinaryOutput(T& ogg) {
        // from object to stream, all bytes of objects of 'T' type
        this->Output((char*)&ogg, sizeof(T));
    }

    /// Stores an object, given the pointer, into the archive.
    /// This function can be used to serialize objects from
    /// nontrivial class trees, where at load time one may wonder
    /// which was the class type of the saved object.
    /// Note: the object must be load with ChStreamInBinary::AbstractReadCreate()
    /// Also, the AbstractWrite()-AbstractReadCreate() mechanism avoids
    /// storing/creating multiple times the shared objects.
    /// Supports only objects with Chrono RTTI and serializer member StreamOUT().
    ///***OBSOLETE*** use ChArchive << operator
    template <class t>
    t* AbstractWrite(t* pObj) {
        int pos = PutPointer(pObj);

        if (pos == -1) {
            // New Object, we have to full serialize it
            std::string str = pObj->FactoryNameTag();
            *this << str;   // serialize class type
            *this < *pObj;  // serialize data
        } else {
            // Object already in list. Only store position
            std::string str = "NULL";
            *this << str;  // serialize 'this was already saved' info
            *this << pos;  // serialize position in pointers vector
        }

        return pObj;
    }

    /// Stores an object, given the pointer, into the archive.
    /// This function can be used to serialize objects from
    /// nontrivial class trees, where at load time one may wonder
    /// which was the class type of the saved object.
    /// Note: the object must be load with ChStreamInBinary::AbstractReadCreate()
    /// Also, the AbstractWrite()-AbstractReadCreate() mechanism avoids
    /// storing/creating multiple times the shared objects.
    /// Supports only objects with Chrono RTTI and serializer member StreamOUT().
    ///***OBSOLETE*** use ChArchive << operator
    template <class t>
    t* AbstractWriteAll(t* pObj) {
        int pos = PutPointer(pObj);

        if (pos == -1) {
            // New Object, we have to full serialize it
            std::string str = pObj->FactoryNameTag();
            *this << str;               // serialize class type
            pObj->StreamOUTall(*this);  // serialize data
        } else {
            // Object already in list. Only store position
            std::string str = "NULL";
            *this << str;  // serialize 'this was already saved' info
            *this << pos;  // serialize position in pointers vector
        }

        return pObj;
    }

    /// Some objects may write class version at the beginning
    /// of the streamed data, using this function.
    void VersionWrite(int mver);
};

///
/// This is a base class for all BINARY INPUT streams, in a way such
/// that the stream is platform indepent (see the 'little endian' stuff
/// in 'floating point to persistent data' topics..)
/// Defines some << operators from basic
/// types, converting all them into calls to the Output() function.
/// At least some inherited class should be used by the user in
/// order to get some practical effect (file saving, etc.)
///

class ChApi ChStreamInBinary : public ChStreamIn, public ChBinaryArchive {
  private:
  public:
    ChStreamInBinary();

    virtual ~ChStreamInBinary();

    //  Operators

    /// Generic >> operator for all classes which implement serialization
    /// by means of a method named  'void StreamIN(ChStreamInBinary&)'
    template <class T>
    ChStreamInBinary& operator>>(T& obj) {
        obj.StreamIN(*this);
        return *this;
    }

    /// Specialized operators for basic primitives (numbers like long,
    /// double, int. etc., and booleans)
    /// Handle byte ordering issues in little.endian/big.endian representations,
    /// for cross platform portability.
    ChStreamInBinary& operator>>(char& Val);
    ChStreamInBinary& operator>>(bool& Val);
    ChStreamInBinary& operator>>(int& Val);
    ChStreamInBinary& operator>>(unsigned int& Val);
    ChStreamInBinary& operator>>(double& Val);
    ChStreamInBinary& operator>>(float& Val);
    ChStreamInBinary& operator>>(long& Val);
    ChStreamInBinary& operator>>(unsigned long& Val);
    ChStreamInBinary& operator>>(unsigned long long& Val);
    ChStreamInBinary& operator>>(std::string& str);

    /// Specialized operator for C strings
    ChStreamInBinary& operator>>(char* str);

    /// Generic operator for raw binary streaming of generic objects
    /// WARNING!!! raw byte streaming! If class 'T' contains double,
    /// int, long, etc, these may give problems when loading on another
    /// platform which has big-endian ordering, if generated on a little-edian platform...
    template <class T>
    void GenericBinaryInput(T& ogg) {
        // from stream to object, all bytes of objects of 'T' type
        this->Input((char*)&ogg, sizeof(T));
    }

    /// Extract an object from the archive, and assignes the pointer to it.
    /// This function can be used to load objects whose class is not
    /// known in advance (anyway, assuming the class had been registered
    /// with Chrono class factory registration). It _creates_ the object
    /// of the proper downcasted class, and deserializes it.
    /// Note: the object must be saved with ChStreamOutBinary::AbstractWrite()
    /// Also, the AbstractWrite()-AbstractReadCreate() mechanism avoids
    /// storing/creating multiple times the shared objects.
    /// Supports only objects with Chrono RTTI and serializer member StreamIN().
    template <class t>
    t* AbstractReadCreate(t** mObj) {
        std::string cls_name;
        *mObj = NULL;

        // 1) Read name of class (chrono RTTI type id)
        *this >> cls_name;

        if (cls_name != "NULL") {
            // 2) Dynamically create using class factory
            chrono::create(cls_name, mObj);

            if ((*mObj) != NULL) {
                objects_pointers.push_back(*mObj);
                // 3) Deserialize
                *this >> **mObj;
            } else {
                throw(ChException("Stream cannot create object"));
            }
        } else {
            int pos = 0;
            // 2b) Was a shared object: just get the pointer to already-retrieved
            *this >> pos;

            *mObj = static_cast<t*>(objects_pointers[pos]);
        }

        return *mObj;
    }

    /// Extract an object from the archive, and assignes the pointer to it.
    /// This function can be used to load objects whose class is not
    /// known in advance (anyway, assuming the class had been registered
    /// with Chrono class factory registration). It _creates_ the object
    /// of the proper downcasted class, and deserializes it.
    /// Note: the object must be saved with ChStreamOutBinary::AbstractWrite()
    /// Also, the AbstractWrite()-AbstractReadCreate() mechanism avoids
    /// storing/creating multiple times the shared objects.
    /// Supports only objects with Chrono RTTI and serializer member StreamIN().
    template <class t>
    t* AbstractReadAllCreate(t** mObj) {
        std::string cls_name;
        *mObj = NULL;

        // 1) Read name of class (chrono RTTI type id)
        *this >> cls_name;

        if (cls_name != "NULL") {
            // 2) Dynamically create using class factory
            create(cls_name, mObj);

            if ((*mObj) != NULL) {
                objects_pointers.push_back(*mObj);
                // 3) Deserialize
                (*mObj)->StreamINall(*this);
                //*this >> **mObj;
            } else {
                throw(ChException("Stream cannot create object"));
            }
        } else {
            int pos = 0;
            // 2b) Was a shared object: just get the pointer to already-retrieved
            *this >> pos;

            *mObj = static_cast<t*>(objects_pointers[pos]);
        }

        return *mObj;
    }

    /// Some objects may write class version at the beginning
    /// of the streamed data, they can use this function to read class from stream.
    int VersionRead();
};

///
/// This is a base class for typical output on system's file,
/// on a disk, using the typical C++ 'fstream' handler.
///

class ChApi ChStreamFile {
  private:
    /// Handler to a C++ file
    std::fstream file;
    /// The file name
    char name[180];

  public:
    /// Creates a system file, like the C++ fstream, and opens it,
    /// given filename on disk, and the opening mode (ex: std::ios::out or
    /// std::ios::in)
    ChStreamFile(const char* filename, std::ios::openmode mmode);

    /// Destruction means that the file stream is also closed.
    virtual ~ChStreamFile();

    /// Synchronizes the associated stream buffer with its controlled output
    /// sequence
    virtual void Flush();

    /// Writes to file, up to n chars.
    /// If does not succeed, throws exception.
    virtual void Write(const char* data, size_t n);

    /// Reads from file, up to n chars.
    /// If does not succeed, throws exception.
    virtual void Read(char* data, size_t n);

    /// Returns true if end of stream reached.
    virtual bool End_of_stream() { return file.eof(); };

    /// Reference to fstream encapsulated here.
    std::fstream& GetFstream() { return file; }
};

///
/// This is a wrapper for already-opened std::ostream
/// output streams (like std::cout or similar)
///

class ChApi ChStreamOstreamWrapper {
  private:
    /// Handler to a C++ file
    std::ostream* afile;

  public:
    /// Creates a wrapper for an already existing, already opened,
    /// ostream, given the pointer to that ostream.
    ChStreamOstreamWrapper(std::ostream* mfile);

    /// Deleting this stream wrapper does not delete nor closes
    /// the wrapped ostream!
    virtual ~ChStreamOstreamWrapper();

    /// Writes raw data to file, up to n chars.
    /// If does not succeed, throws exception.
    virtual void Write(const char* data, size_t n);

    /// Returns true if end of stream reached.
    virtual bool End_of_stream() const { return afile->eof(); };

    /// Reference to ostream encapsulated here.
    std::ostream* GetOstream() const { return afile; }
};

///
/// This is a wrapper for already-opened std::istream
/// input streams
///

class ChApi ChStreamIstreamWrapper {
  private:
    /// Handler to a C++ stream
    std::istream* afile;

  public:
    /// Creates a wrapper for an already existing, already opened,
    /// istream, given the pointer to that istream.
    ChStreamIstreamWrapper(std::istream* mfile);

    /// Deleting this stream wrapper does not delete nor closes
    /// the wrapped istream!
    virtual ~ChStreamIstreamWrapper();

    /// Reads from stream, up to n chars.
    /// If does not succeed, throws exception.
    virtual void Read(char* data, size_t n);

    /// Returns true if end of stream reached.
    virtual bool End_of_stream() { return afile->eof(); };

    /// Reference to istream encapsulated here.
    std::istream* GetIstream() { return afile; }
};

///
/// This is a wrapper for a std::vector<char> (buffer of chars)
///

class ChApi ChStreamVectorWrapper {
  private:
    /// Handler to a C++ stream
    std::vector<char>* vbuffer;
    int pos;

  public:
    /// Creates a wrapper for an already existing std::vector<char>,
    /// given the pointer to that vector.
    ChStreamVectorWrapper(std::vector<char>* mchars);

    /// Deleting this wrapper does not delete nor closes
    /// the wrapped buffer!
    virtual ~ChStreamVectorWrapper();

    /// Reads from vector, up to n chars.
    /// If does not succeed, throws exception.
    virtual void Read(char* data, size_t n);

    /// Writes raw data to vector, up to n chars.
    /// If does not succeed, throws exception.
    virtual void Write(const char* data, size_t n);

    /// Returns true if end of stream (end of vector) reached.
    virtual bool End_of_stream();

    /// Direct reference to std::vector<char> encapsulated here.
    std::vector<char>* GetVector() { return vbuffer; }

    /// Rewind read position to char number (0= beginning).
    /// Write position is always at the end (append).
    void Seek(int position) { pos = position; }
};

///
/// This is a specialized class for BINARY output to wrapped std::ostream,
///

class ChApi ChStreamOutBinaryStream : public ChStreamOstreamWrapper, public ChStreamOutBinary {
  public:
    ChStreamOutBinaryStream(std::ostream* mfile) : ChStreamOstreamWrapper(mfile), ChStreamOutBinary(){};
    virtual ~ChStreamOutBinaryStream(){};

    virtual bool End_of_stream() { return ChStreamOstreamWrapper::End_of_stream(); }

  private:
    virtual void Output(const char* data, size_t n) { ChStreamOstreamWrapper::Write(data, n); }
};

///
/// This is a specialized class for BINARY input from wrapped std::istream,
///

class ChApi ChStreamInBinaryStream : public ChStreamIstreamWrapper, public ChStreamInBinary {
  public:
    ChStreamInBinaryStream(std::istream* mfile) : ChStreamIstreamWrapper(mfile), ChStreamInBinary(){};
    virtual ~ChStreamInBinaryStream(){};

    virtual bool End_of_stream() { return ChStreamIstreamWrapper::End_of_stream(); }

  private:
    virtual void Input(char* data, size_t n) { ChStreamIstreamWrapper::Read(data, n); }
};

///
/// This is a specialized class for BINARY output to wrapped std::vector<char>,
///

class ChApi ChStreamOutBinaryVector : public ChStreamVectorWrapper, public ChStreamOutBinary {
  public:
    ChStreamOutBinaryVector(std::vector<char>* mchars) : ChStreamVectorWrapper(mchars), ChStreamOutBinary(){};
    virtual ~ChStreamOutBinaryVector(){};

    virtual bool End_of_stream() { return ChStreamVectorWrapper::End_of_stream(); }

  private:
    virtual void Output(const char* data, size_t n) { ChStreamVectorWrapper::Write(data, n); }
};

///
/// This is a specialized class for BINARY input from wrapped std::vector<char>,
///

class ChApi ChStreamInBinaryVector : public ChStreamVectorWrapper, public ChStreamInBinary {
  public:
    ChStreamInBinaryVector(std::vector<char>* mchars) : ChStreamVectorWrapper(mchars), ChStreamInBinary(){};
    virtual ~ChStreamInBinaryVector(){};

    virtual bool End_of_stream() { return ChStreamVectorWrapper::End_of_stream(); }

  private:
    virtual void Input(char* data, size_t n) { ChStreamVectorWrapper::Read(data, n); }
};

///
/// This is a specialized class for ASCII output to wrapped std::vector<char>,
///

class ChApi ChStreamOutAsciiVector : public ChStreamVectorWrapper, public ChStreamOutAscii {
  public:
    ChStreamOutAsciiVector(std::vector<char>* mchars) : ChStreamVectorWrapper(mchars), ChStreamOutAscii(){};
    virtual ~ChStreamOutAsciiVector(){};

    virtual bool End_of_stream() { return ChStreamVectorWrapper::End_of_stream(); }

  private:
    virtual void Output(const char* data, size_t n) { ChStreamVectorWrapper::Write(data, n); }
};

///
/// This is a specialized class for ASCII input from wrapped std::vector<char>,
///

class ChApi ChStreamInAsciiVector : public ChStreamVectorWrapper, public ChStreamInAscii {
  public:
    ChStreamInAsciiVector(std::vector<char>* mchars) : ChStreamVectorWrapper(mchars), ChStreamInAscii(){};
    virtual ~ChStreamInAsciiVector(){};

    virtual bool End_of_stream() { return ChStreamVectorWrapper::End_of_stream(); }

  private:
    virtual void Input(char* data, size_t n) { ChStreamVectorWrapper::Read(data, n); }
};

///
/// This is a specialized class for BINARY output on system's file,
///

class ChApi ChStreamOutBinaryFile : public ChStreamFile, public ChStreamOutBinary {
  public:
    ChStreamOutBinaryFile(const char* filename, std::ios::openmode mmode = std::ios::trunc);  // trunc or app
    virtual ~ChStreamOutBinaryFile();

  private:
    virtual void Output(const char* data, size_t n) { ChStreamFile::Write(data, n); }
};

///
/// This is a specialized class for ASCII output on system's file,
///

class ChApi ChStreamOutAsciiFile : public ChStreamFile, public ChStreamOutAscii {
  public:
    ChStreamOutAsciiFile(const char* filename, std::ios::openmode mmode = std::ios::trunc);  // trunc or app);
    virtual ~ChStreamOutAsciiFile();

  private:
    virtual void Output(const char* data, size_t n) { ChStreamFile::Write(data, n); }
};

///
/// This is a specialized class for BINARY input on system's file,
///

class ChApi ChStreamInBinaryFile : public ChStreamFile, public ChStreamInBinary {
  public:
    ChStreamInBinaryFile(const char* filename);
    virtual ~ChStreamInBinaryFile();

    virtual bool End_of_stream() { return ChStreamFile::End_of_stream(); }

  private:
    virtual void Input(char* data, size_t n) { ChStreamFile::Read(data, n); }
};

///
/// This is a specialized class for ASCII input on system's file,
///

class ChApi ChStreamInAsciiFile : public ChStreamFile, public ChStreamInAscii {
  public:
    ChStreamInAsciiFile(const char* filename);
    virtual ~ChStreamInAsciiFile();

    virtual bool End_of_stream() { return ChStreamFile::End_of_stream(); }

  private:
    virtual void Input(char* data, size_t n) { ChStreamFile::Read(data, n); }
};

}  // end namespace chrono

#endif
