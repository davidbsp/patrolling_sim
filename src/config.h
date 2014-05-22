#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include <stdio.h>

// please avoid following line
using std::map;
using std::string;
using std::ifstream;

class ConfigFile
{
private:
	ifstream f;
	map<string,string> params;
	
public:
	ConfigFile(const char *filename);
	~ConfigFile();
	
	string getParam(string p);	
	string getParam(const char *p);
	int getIParam(const char *p, int def=0);	
	double getDParam(const char *p, double def=0.0);	
};

#endif
