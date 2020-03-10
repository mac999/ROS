// package: reschy
// module: robot_test_controller
// revision history
// date       | author      | description
// 2015.10.4  | T.W.Kang    | utility functions
//
//
#include <string>
#include <deque>
#include <vector>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"

void parsingIMU(std::string value, float& roll, float& pitch, float& yaw)
{
	std::stringstream data(value);

	std::vector<std::string> result;	
	while(data.good())
	{
		std::string sub;
		std::getline(data, sub, ',');
		result.push_back(sub);
	}

	if(result.size() == 3)
	{
		roll = atof(result[0].c_str());
		pitch = atof(result[1].c_str());
		yaw = atof(result[2].c_str());
		return;
	}
	
	std::vector<std::string> list;
	for(int i = 0; i < result.size(); i++)
	{
		std::string term = result[i];
		
		std::stringstream data(term);

		bool nameField = true;
		while(data.good())
		{
			std::string sub;
			std::getline(data, sub, ':');
			if(nameField == false)
				list.push_back(sub);
			nameField = nameField == true ? false : true;
		}
	}	

	if(list.size() == 3)
	{
		roll = atof(list[0].c_str());
		pitch = atof(list[1].c_str());
		yaw = atof(list[2].c_str());
		return;
	}
}


