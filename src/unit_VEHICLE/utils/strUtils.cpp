#include "strUtils.h"


namespace utils {
// some auxiliary functions
std::vector<std::string>& StrUtils::splitStr(const std::string &s, char delim, std::vector<std::string> &elems) {
	std::stringstream ss(s);
	std::string item;
	while(getline(ss, item, delim)) {
		elems.push_back(item);
	}
	return elems;
}

std::vector<std::string> StrUtils::splitStr(const std::string &s, char delim) {
	std::vector<std::string> elems;
	return splitStr(s, delim, elems);
}

// convert a string to another type
template<class T> T StrUtils::fromString(const std::string& s) {
		std::istringstream stream (s);
		T t;
		stream >> t;
		return t;
}

// read an input line from tm.config, return the number that matters
template<class T> T StrUtils::fromTline(const std::string& tline) {
	T t = fromString<T>(splitStr(splitStr(tline,'/')[0],'=')[1]);
	return t;
}

}	// end namespace utils