/*
 * Logger.h
 *
 *  Created on: Nov 17, 2014
 *      Author: sina
 */


#ifndef LOGGER_CPP_
#define LOGGER_CPP_

#include <iostream>
#include <string>
#include <fstream>
#include <vector>
using namespace std;

class Logger{

public:

	ofstream myfile;
	char buffer [500];

	Logger(){
		myfile.open("log.log");
	}

	Logger(char* filename){
		myfile.open(filename);
	}

	~Logger(){
		myfile.close();
	}

	inline void log(){
		myfile << buffer << endl;
	}

	inline void log_array(char* initial_string, std::vector<int> vec){
		sprintf(buffer, "%s", initial_string);
		for (int j = 0; j < vec.size(); j++){
			sprintf(buffer, "%s%d  ", buffer, vec[j]);
		}
		myfile << buffer << endl;
	}

	inline void log_here(){
		sprintf(buffer, "HEREEE!!!");
		myfile << buffer << endl;
	}

};

#endif /* LOGGER_CPP_ */
