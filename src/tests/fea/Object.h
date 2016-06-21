/*
* Object.h
*
*  Created on: May 18, 2015
*      Author: felipegb94
*/

#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>

class Object
{

public:

	Object()
	: GlobalKey("")
	{
		std::cout << "Create object with no key" << std::endl;
		json << "{";
	}

	void AddMember(std::string key, double value)
	{
		std::cout << "Adding Double" << std::endl;
		json << "\n    \"" << key << "\"" << ": " << value << ",";
	}

	void AddMember(std::string key, Object& value)
	{
		std::cout << "Adding Object" << std::endl;
		json << "\n    \"" << key << "\"" << ": " << value.GetObject() << ",";
	}

	void AddMember(std::string key, uint64_t value)
	{
		std::cout << "Adding Uint64" << std::endl;
		json << "\n    \"" << key << "\"" << ": " << value << ",";
	}

	void AddMember(std::string key, int value)
	{
		std::cout << "Adding int" << std::endl;
		json << "\n    \"" << key << "\"" << ": " << value << ",";
	}

	void AddMember(std::string key, std::string value)
	{
		std::cout << "Adding string" << std::endl;
		json << "\n    \"" << key << "\"" << ": \"" << value << "\",";
	}

    // Gets JSON object. Closes JSON string and removes last comma.
    std::string GetObject() const 
    {
        return json.str().substr(0, json.str().length()-1) + "\n}\n";    // Does some internal work
    }

	virtual ~Object() {}

private:
	std::string GlobalKey;
	std::stringstream json; // String Stream with all content of the file
};