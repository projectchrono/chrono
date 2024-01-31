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

#include <cmath>
#include <cstdarg>
#include <cerrno>
#include <iterator>

#include "chrono/core/ChStream.h"
#include "chrono/core/ChException.h"
#include "chrono/core/ChLog.h"

namespace chrono {

// -----------------------------------------------------------------------------
// ChStreamOutAscii

ChStreamOutAscii& ChStreamOutAscii::operator<<(bool bVal) {
    if (bVal)
        Output("1", 1);
    else
        Output("0", 1);
    return *this;
}

ChStreamOutAscii& ChStreamOutAscii::operator<<(char* str) {
    Output(str, strlen(str));
    return *this;
}

ChStreamOutAscii& ChStreamOutAscii::operator<<(const std::string& str) {
    Output(str.c_str(), str.length());
    return *this;
}

ChStreamOutAscii& ChStreamOutAscii::operator<<(const char* str) {
    Output(str, strlen(str));
    return *this;
}

ChStreamOutAscii& ChStreamOutAscii::operator<<(int unVal) {
    auto str = std::to_string(unVal);
    return operator<<(str);
}

ChStreamOutAscii& ChStreamOutAscii::operator<<(unsigned int unVal) {
    auto str = std::to_string(unVal);
    return operator<<(str);
}

ChStreamOutAscii& ChStreamOutAscii::operator<<(unsigned long unVal) {
    auto str = std::to_string(unVal);
    return operator<<(str);
}

ChStreamOutAscii& ChStreamOutAscii::operator<<(unsigned long long unVal) {
    auto str = std::to_string(unVal);
    return operator<<(str);
}

/*
ChStreamOutAscii& ChStreamOutAscii::operator<<(long lVal) {
    return operator<<(std::to_string(lVal));
}
*/

ChStreamOutAscii& ChStreamOutAscii::operator<<(char tch) {
    char szCh[2];
    szCh[0] = tch;
    szCh[1] = '\0';
    Output(szCh, 1);
    return *this;
}

/*
ChStreamOutAscii& ChStreamOutAscii::operator<<(unsigned long ulVal) {
    return operator<<(std::to_string(ulVal));
}
*/

ChStreamOutAscii& ChStreamOutAscii::operator<<(double dVal) {
    char buffer[100];

    snprintf(buffer, sizeof(buffer), number_format, dVal);

    Output(buffer, strlen(buffer));

    return *this;
}

ChStreamOutAscii& ChStreamOutAscii::operator<<(float fVal) {
    return operator<<((double)fVal);
}

void ChStreamOutAscii::Format(char* formatString, ...) {
    if (formatString == NULL) {
        throw(ChException("Bad format string"));
    }

    va_list argList;

    // Set va_list to the beginning of optional arguments

    va_start(argList, formatString);
    char* ptr = formatString;

    while (*ptr != '\0') {
        char* str = NULL;
        int nInteger = 0;
        unsigned int unInt = 0;
        double dDoub = 0;

        if (*ptr == '%') {
            switch (*(ptr + 1)) {
                case 's':
                    str = va_arg(argList, char*);

                    if (NULL == str)
                        break;
                    *this << str;
                    ptr++;
                    break;

                case 'd':
                    nInteger = va_arg(argList, int);
                    *this << nInteger;
                    ptr++;
                    break;

                case 'u':
                    unInt = va_arg(argList, unsigned int);
                    *this << unInt;
                    ptr++;
                    break;

                case 'f':
                    dDoub = va_arg(argList, double);
                    *this << dDoub;
                    ptr++;
                    break;
                default:
                    *this << *ptr;
            }
        } else {
            *this << *ptr;
        }

        ptr++;
    }
}

// -----------------------------------------------------------------------------
// ChStreamInAscii

static void RemoveTrailingCommas(std::string mstring) {
    /*
    while(mstring.length() > 0)
    {
        if ((mstring.substr(mstring.length()-1,1) == ",") ||
            (mstring.substr(mstring.length()-1,1) == ";") )
                mstring.resize(mstring.length()-1);
    }
    */
}

ChStreamInAscii& ChStreamInAscii::operator>>(bool& bVal) {
    std::string textboolean;
    bool parsed = false;
    *this >> textboolean;
    RemoveTrailingCommas(textboolean);
    if (textboolean == "true") {
        bVal = true;
        parsed = true;
    }
    if (textboolean == "false") {
        bVal = false;
        parsed = true;
    }
    if (textboolean == "1") {
        bVal = true;
        parsed = true;
    }
    if (textboolean == "0") {
        bVal = false;
        parsed = true;
    }
    if (!parsed)
        throw(ChException("String " + textboolean + " is not a valid 'true'/'false' value"));
    return *this;
}

/*
ChStreamInAscii& ChStreamInAscii::operator >> (long	&lVal)
{
    std::string buffer;
    *this >> buffer;
    RemoveTrailingCommas(buffer);
    lVal = atoi(buffer.c_str());
    return *this;
}

*/
ChStreamInAscii& ChStreamInAscii::operator>>(char& tch) {
    char loa[1];
    Input(loa, 1);
    tch = loa[0];
    return *this;
}

ChStreamInAscii& ChStreamInAscii::operator>>(int& nVal) {
    std::string buffer;
    *this >> buffer;
    RemoveTrailingCommas(buffer);
    nVal = atoi(buffer.c_str());
    return *this;
}

/*
ChStreamInAscii& ChStreamInAscii::operator >>(unsigned	long	&ulVal)
{
    std::string buffer;
    *this >> buffer;
    RemoveTrailingCommas(buffer);
    ulVal = atoi(buffer.c_str());
    return *this;
}
*/

ChStreamInAscii& ChStreamInAscii::operator>>(double& dVal) {
    std::string buffer;
    *this >> buffer;
    RemoveTrailingCommas(buffer);
    dVal = atof(buffer.c_str());
    // if (errno)
    // throw (ChException( "String " +buffer+ " is not a valid number format"));
    return *this;
}

ChStreamInAscii& ChStreamInAscii::operator>>(float& dVal) {
    std::string buffer;
    *this >> buffer;
    RemoveTrailingCommas(buffer);
    dVal = (float)(atof(buffer.c_str()));
    // if (errno)
    // throw (ChException( "String " +buffer+ " is not a valid number format"));
    return *this;
}

ChStreamInAscii& ChStreamInAscii::operator>>(unsigned int& unVal) {
    std::string buffer;
    *this >> buffer;
    RemoveTrailingCommas(buffer);
    unVal = atoi(buffer.c_str());
    return *this;
}

ChStreamInAscii& ChStreamInAscii::operator>>(char* str) {
    /*
    bool stop = 0;
    int cnt = 0;
    char loa[2];
    loa[1]=0;

    while(true)
    {

        try {Input(loa, 1);}
        catch (const std::exception&)
        {
            if (!(this->End_of_stream() && cnt>0))
                throw (ChException("Cannot read from stream"));
        }

        if (this->End_of_stream() || loa[0]==*"\n" || loa[0]==*" " || loa[0]==*"\t" || loa[0]==0)
            break;

        str[cnt]=loa[0];
        cnt++;
    }
    str[cnt]=NULL;
    */

    std::string buffer;
    *this >> buffer;
    strcpy(str, buffer.c_str());

    return *this;
}

ChStreamInAscii& ChStreamInAscii::operator>>(std::string& str) {
    // bool stop = 0; //unused
    int cnt = 0;
    char loa[2];
    loa[1] = 0;
    str = "";
    bool started = false;

    while (true) {
        try {
            Input(loa, 1);
        } catch (const std::exception&) {
            if (!(this->End_of_stream() && cnt > 0))
                throw(ChException("Cannot read from stream"));
        }

        if (this->End_of_stream())
            break;

        if (loa[0] != *"\n" && loa[0] != *" " && loa[0] != *"\t")
            started = true;

        if (started) {
            if (loa[0] == *"\n" || loa[0] == *" " || loa[0] == *"\t" || loa[0] == 0)
                break;
            str.append(loa);
        }

        cnt++;
    }

    return *this;
}

// -----------------------------------------------------------------------------
// ChBinaryArchive

// Endian test
bool ChBinaryArchive::IsBigEndianMachine() {
    union {
        int word;
        unsigned char byte;
    } endian_test;

    endian_test.word = 1;
    if (endian_test.byte != 1)
        return true;

    return false;
}

// -----------------------------------------------------------------------------
// ChStreamOutBinary

ChStreamOutBinary& ChStreamOutBinary::operator<<(char Val) {
    this->Output((char*)&Val, sizeof(char));
    return (*this);
}

ChStreamOutBinary& ChStreamOutBinary::operator<<(bool Val) {
    // convert booleans to char, for cross-platform compatibility.
    char tmp;
    tmp = (char)Val;
    this->Output((char*)&tmp, sizeof(char));
    return (*this);
}

ChStreamOutBinary& ChStreamOutBinary::operator<<(long Val) {
    if (big_endian_machine) {
        long tmp = Val;
        StreamSwapBytes<long>(&tmp);
        this->Output((char*)&tmp, sizeof(long));
    }
    this->Output((char*)&Val, sizeof(long));
    return (*this);
}

ChStreamOutBinary& ChStreamOutBinary::operator<<(unsigned long Val) {
    if (big_endian_machine) {
        unsigned long tmp = Val;
        StreamSwapBytes<unsigned long>(&tmp);
        this->Output((char*)&tmp, sizeof(unsigned long));
    }
    this->Output((char*)&Val, sizeof(unsigned long));
    return (*this);
}

ChStreamOutBinary& ChStreamOutBinary::operator<<(unsigned long long Val) {
    if (big_endian_machine) {
        unsigned long long tmp = Val;
        StreamSwapBytes<unsigned long long>(&tmp);
        this->Output((char*)&tmp, sizeof(unsigned long long));
    }
    this->Output((char*)&Val, sizeof(unsigned long long));
    return (*this);
}

ChStreamOutBinary& ChStreamOutBinary::operator<<(const int Val) {
    if (big_endian_machine) {
        int tmp = Val;
        StreamSwapBytes<int>(&tmp);
        this->Output((char*)&tmp, sizeof(int));
    } else {
        this->Output((char*)&Val, sizeof(int));
    }
    return (*this);
}

ChStreamOutBinary& ChStreamOutBinary::operator<<(unsigned int Val) {
    if (big_endian_machine) {
        unsigned int tmp = Val;
        StreamSwapBytes<unsigned int>(&tmp);
        this->Output((char*)&tmp, sizeof(unsigned int));
    } else {
        this->Output((char*)&Val, sizeof(unsigned int));
    }
    return (*this);
}

ChStreamOutBinary& ChStreamOutBinary::operator<<(const double Val) {
    if (big_endian_machine) {
        double tmp = Val;
        StreamSwapBytes<double>(&tmp);
        this->Output((char*)&tmp, sizeof(double));
    } else {
        this->Output((char*)&Val, sizeof(double));
    }
    return (*this);
}

ChStreamOutBinary& ChStreamOutBinary::operator<<(const float Val) {
    if (big_endian_machine) {
        float tmp = Val;
        StreamSwapBytes<float>(&tmp);
        this->Output((char*)&tmp, sizeof(float));
    } else {
        this->Output((char*)&Val, sizeof(float));
    }
    return (*this);
}

ChStreamOutBinary& ChStreamOutBinary::operator<<(const char* str) {
    // save string length, including null termination
    int mlength = (int)strlen(str) + 1;
    *this << mlength;
    // save all bytes including null termination
    this->Output(str, mlength);
    return *this;
}

ChStreamOutBinary& ChStreamOutBinary::operator<<(char* str) {
    // save string length, including null termination
    int mlength = (int)strlen(str) + 1;
    *this << mlength;
    // save all bytes including null termination
    this->Output(str, mlength);
    return *this;
}

ChStreamOutBinary& ChStreamOutBinary::operator<<(const std::string& str) {
    // save string length, including null termination
    int mlength = (int)strlen(str.c_str());
    *this << mlength;
    // save all bytes including null termination
    this->Output(str.c_str(), strlen(str.c_str()));
    return *this;
}

void ChStreamOutBinary::VersionWrite(int mver) {
    *this << mver;
}

// -----------------------------------------------------------------------------
// ChStreamInBinary

ChStreamInBinary& ChStreamInBinary::operator>>(char& Val) {
    this->Input((char*)&Val, sizeof(char));
    return (*this);
}

ChStreamInBinary& ChStreamInBinary::operator>>(bool& Val) {
    // booleans as chars, for cross platform portability
    char tmp;
    this->Input((char*)&tmp, sizeof(char));
    if (tmp)
        Val = true;
    else
        Val = false;
    // Val = (bool)tmp;
    return (*this);
}

ChStreamInBinary& ChStreamInBinary::operator>>(long& Val) {
    if (big_endian_machine) {
        long tmp;
        this->Input((char*)&tmp, sizeof(long));
        StreamSwapBytes<long>(&tmp);
        Val = tmp;
    } else {
        this->Input((char*)&Val, sizeof(long));
    }
    return (*this);
}

ChStreamInBinary& ChStreamInBinary::operator>>(unsigned long& Val) {
    if (big_endian_machine) {
        unsigned long tmp;
        this->Input((char*)&tmp, sizeof(unsigned long));
        StreamSwapBytes<unsigned long>(&tmp);
        Val = tmp;
    } else {
        this->Input((char*)&Val, sizeof(unsigned long));
    }
    return (*this);
}

ChStreamInBinary& ChStreamInBinary::operator>>(unsigned long long& Val) {
    if (big_endian_machine) {
        unsigned long long tmp;
        this->Input((char*)&tmp, sizeof(unsigned long long));
        StreamSwapBytes<unsigned long long>(&tmp);
        Val = tmp;
    } else {
        this->Input((char*)&Val, sizeof(unsigned long long));
    }
    return (*this);
}

ChStreamInBinary& ChStreamInBinary::operator>>(int& Val) {
    if (big_endian_machine) {
        int tmp;
        this->Input((char*)&tmp, sizeof(int));
        StreamSwapBytes<int>(&tmp);
        Val = tmp;
    } else {
        this->Input((char*)&Val, sizeof(int));
    }
    return (*this);
}

ChStreamInBinary& ChStreamInBinary::operator>>(unsigned int& Val) {
    if (big_endian_machine) {
        unsigned int tmp;
        this->Input((char*)&tmp, sizeof(unsigned int));
        StreamSwapBytes<unsigned int>(&tmp);
        Val = tmp;
    } else {
        this->Input((char*)&Val, sizeof(unsigned int));
    }
    return (*this);
}

ChStreamInBinary& ChStreamInBinary::operator>>(double& Val) {
    if (big_endian_machine) {
        double tmp;
        this->Input((char*)&tmp, sizeof(double));
        StreamSwapBytes<double>(&tmp);
        Val = tmp;
    } else {
        this->Input((char*)&Val, sizeof(double));
    }
    return (*this);
}

ChStreamInBinary& ChStreamInBinary::operator>>(float& Val) {
    if (big_endian_machine) {
        float tmp;
        this->Input((char*)&tmp, sizeof(float));
        StreamSwapBytes<float>(&tmp);
        Val = tmp;
    } else {
        this->Input((char*)&Val, sizeof(float));
    }
    return (*this);
}

ChStreamInBinary& ChStreamInBinary::operator>>(char* str) {
    // Read string length , plus null-termination char
    int mlength;
    *this >> mlength;
    // Read all bytes of string.
    this->Input(str, mlength);
    return *this;
}

ChStreamInBinary& ChStreamInBinary::operator>>(std::string& str) {
    // Read string length , plus null-termination char
    str = "";
    char buf[2];
    buf[1] = '\0';
    int mlength;
    *this >> mlength;
    str.reserve(mlength);
    for (int i = 0; i < mlength; i++) {
        this->Input(buf, 1);
        str.append(buf);
    }
    // Read all bytes of string.

    return *this;
}

int ChStreamInBinary::VersionRead() {
    int mres;
    *this >> mres;
    return mres;
}

// -----------------------------------------------------------------------------
// ChStreamFile

ChStreamFile::ChStreamFile(const std::string& filename, std::ios::openmode mode) : name(filename) {
    try {
        // file.exceptions(std::ios::failbit | std::ios::badbit);
        file.exceptions(std::ios::failbit | std::ios::badbit | std::ios::eofbit);
        file.open(filename, mode);
    } catch (const std::exception&) {
        std::string msg = "Cannot open stream " + filename;
        throw ChException(msg);
    };
}

// Destruction means that the file stream is also closed.
ChStreamFile::~ChStreamFile() {
    file.flush();
    file.close();
}

void ChStreamFile::Flush() {
    file.flush();
}

void ChStreamFile::Write(const char* data, size_t n) {
    try {
        file.write(data, n);
    } catch (const std::exception&) {
        throw ChException("Cannot write to stream");
    };
}

void ChStreamFile::Read(char* data, size_t n) {
    try {
        file.read(data, n);
    } catch (const std::exception&) {
        throw ChException("Cannot read from stream");
    };
}

// -----------------------------------------------------------------------------

ChStreamOstreamWrapper::ChStreamOstreamWrapper(std::ostream* mfile) {
    assert(mfile);
    afile = mfile;
}

ChStreamOstreamWrapper::~ChStreamOstreamWrapper() {}

void ChStreamOstreamWrapper::Write(const char* data, size_t n) {
    try {
        afile->write(data, n);
    } catch (const std::exception&) {
        throw ChException("Cannot write to wrapped stream");
    };
}

// -----------------------------------------------------------------------------

ChStreamIstreamWrapper::ChStreamIstreamWrapper(std::istream* mfile) {
    assert(mfile);
    afile = mfile;
}

ChStreamIstreamWrapper::~ChStreamIstreamWrapper() {}

void ChStreamIstreamWrapper::Read(char* data, size_t n) {
    try {
        afile->read(data, n);
    } catch (const std::exception&) {
        throw ChException("Cannot read from wrapped stream");
    };
}

// -----------------------------------------------------------------------------

ChStreamVectorWrapper::ChStreamVectorWrapper(std::vector<char>* mchars) {
    assert(mchars);
    vbuffer = mchars;
    pos = 0;
}

ChStreamVectorWrapper::~ChStreamVectorWrapper() {}

void ChStreamVectorWrapper::Write(const char* data, size_t n) {
    std::copy(data, data + n, std::back_inserter(*vbuffer));
}

void ChStreamVectorWrapper::Read(char* data, size_t n) {
    if (pos + n > vbuffer->size())
        n = vbuffer->size() - pos;

    for (int i = 0; i < n; i++) {
        data[i] = (*vbuffer)[pos];
        pos++;
    }
}

bool ChStreamVectorWrapper::End_of_stream() const {
    if (pos >= vbuffer->size())
        return true;
    return false;
}

// -----------------------------------------------------------------------------
// These constructors / destructors, though concise, cannot stay in .h because
// the GNU GCC linker gives strange problems...

ChStreamOut::ChStreamOut() {}
ChStreamOut::~ChStreamOut() {}

ChStreamIn::ChStreamIn() {}
ChStreamIn::~ChStreamIn() {}

ChStreamOutAscii::ChStreamOutAscii() {
    strcpy(number_format, "%g");
    strcpy(comment_trailer, "#");
}
ChStreamOutAscii::~ChStreamOutAscii() {}

ChStreamInAscii::ChStreamInAscii() {
    strcpy(number_format, "%g");
}
ChStreamInAscii::~ChStreamInAscii() {}

ChStreamOutBinary::ChStreamOutBinary() {}
ChStreamOutBinary::~ChStreamOutBinary() {}

ChBinaryArchive::ChBinaryArchive() {
    Init();
}
ChBinaryArchive::~ChBinaryArchive() {}

ChStreamInBinary::ChStreamInBinary() {}
ChStreamInBinary::~ChStreamInBinary() {}

ChStreamOutBinaryFile::ChStreamOutBinaryFile(const std::string& filename, std::ios::openmode mode)
    : ChStreamFile(filename, mode | std::ios::out | std::ios::binary) {}
ChStreamOutBinaryFile::~ChStreamOutBinaryFile() {}

ChStreamOutAsciiFile::ChStreamOutAsciiFile(const std::string& filename, std::ios::openmode mode)
    : ChStreamFile(filename, mode | std::ios::out) {}
ChStreamOutAsciiFile::~ChStreamOutAsciiFile() {}

ChStreamInBinaryFile::ChStreamInBinaryFile(const std::string& filename)
    : ChStreamFile(filename, std::ios::in | std::ios::binary) {}
ChStreamInBinaryFile::~ChStreamInBinaryFile() {}

ChStreamInAsciiFile::ChStreamInAsciiFile(const std::string& filename) : ChStreamFile(filename, std::ios::in) {}
ChStreamInAsciiFile::~ChStreamInAsciiFile() {}

}  // end namespace chrono
