#include "config.h"
#include <cstring>
#include <cstdlib>

#define STRLEN 200


ConfigFile::ConfigFile(const char *filename)
{
	f.open(filename);
	if (!f.good()) {
		printf("ERROR. Cannot open config file %s.\n",filename);
		return;
	}

	printf("Opened config file %s\n",filename);
	char buf[STRLEN]; char *p; char par[STRLEN], val[STRLEN], profile[STRLEN];
	int state=0;
	while (f.good()) {
		f.getline(buf,STRLEN);
		switch (state) {
			case 0: // find SECTION
			p = strstr(buf,"[SECTION Main]");
			if (p)
				state++;
			break;
			case 1: // find PROFILE
			p = strstr(buf,"PROFILE");
			if (p) {
				sscanf(buf,"%s %s",par,profile);
				printf("Config Profile %s\n",profile);
				state++;
			}
			break;
            
			case 2: // find correct profile
			p = strstr(buf,"PROFILE");
			if (p) {
				sscanf(p,"%s %s",par,val); int n = strlen(val);
				int k=0;
				while (val[k]!=']' && k<n) k++;
				val[k]='\0';
				printf("Read %s\n",val);
				if (strcmp(profile,val)==0) {
					printf("Right profile\n");
					state++;
				}
			}
			break;
			
            case 3: // reading params from profile
			p = strstr(buf,"[END]");
			if (p) {
				state++;
			}
			else {
				if (buf[0]!='#' && buf[0]!=';' && buf[0]!='/') {
					int r = sscanf(buf,"%s %s",par,val);
					if (r>0) {
						char *p = strstr(buf,"\"");
						if (p) {
							char *r = strstr(p+1,"\"");
							if (r) {
								*r='\0';
								params[string(par)] = string(p+1);
								printf("   %s = %s\n",par,p+1);
							}
						}
						else {
							params[string(par)] = string(val);
							printf("   %s = %s\n",par,val);
						}
					}
				}
			}
			break;

			case 4: // end
			break;
		}
		// cout << buf << endl;
	}

	f.close();
}

ConfigFile::~ConfigFile()
{

}

string ConfigFile::getParam(string p) {
	map<string,string>::const_iterator it = params.find(p);
	if (it!=params.end())
		return it->second;
	else
		return string("");
}

string ConfigFile::getParam(const char *p) {
	return getParam(string(p));
}


int ConfigFile::getIParam(const char *p, int def)
{
	map<string,string>::const_iterator it = params.find(string(p));
	if (it!=params.end())
		return atoi(it->second.c_str());
	else
		return def;
}

double ConfigFile::getDParam(const char *p, double def)
{
	map<string,string>::const_iterator it = params.find(string(p));
	if (it!=params.end())
		return atof(it->second.c_str());
	else
		return def;
}

