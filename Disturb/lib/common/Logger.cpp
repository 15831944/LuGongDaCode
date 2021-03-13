/*
* Copyright 2009, 2010 Free Software Foundation, Inc.
*
* This software is distributed under the terms of the GNU Affero Public License.
* See the COPYING file in the main directory for details.
*
* This use of this software may be subject to additional restrictions.
* See the LEGAL file in the main directory for details.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Affero General Public License for more details.

    You should have received a copy of the GNU Affero General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#include <string.h>
#include <cstdio>
#include <fstream>
#include <time.h>
#include <stdio.h>

#include <iostream>
#include <iomanip>
#include <sys/time.h>
#include <assert.h>
#include <common/Logger.h>
#include <common/Thread.h>
using namespace std;






/**@ The global alarms table. */
//@{
Mutex gLogLock;
list<string>    alarmsList;
void            addAlarm(const string&);
//@}






/** Names of the logging levels. */
const char* levelNames[] =
	{"SHOW","RECORD","FORCE", "ERROR", "ALARM", "WARN", "NOTICE", "INFO", "DEBUG", "DEEPDEBUG","ALERT"};
const unsigned numLevels = 11;

ostream& operator<<(ostream& os, Log::Level level)
{
	unsigned il = (unsigned)level;
	assert(il<numLevels);
	os << levelNames[il];
	return os;
}


/** Given a string, return the corresponding level name. */
Log::Level gLookupLevel(const char* name)
{
	for (unsigned i=0; i<numLevels; i++) {
		if (strcmp(levelNames[i],name)==0) return (Log::Level)i;
	}
	// This should never be called with a bogus name.
	LOG(ERROR) << "undefined logging level " << name << "defaulting to FORCE";
	return Log::LOG_FORCE;
}

/** Return the current logging level for a given source file. */
Log::Level gLoggingLevel(const char* filename)
{
	//const string keyName = string("Log.Level.") + string(filename);
	//if (gConfig.defines(keyName)) return gLookupLevel(gConfig.getStr(keyName));
	return gLookupLevel("DEEPDEBUG");
}





/** The current global log sink. */
static FILE *gLoggingFile = stdout;

void gSetLogFile(FILE *wFile)
{
	gLogLock.lock();
	gLoggingFile = wFile;
	gLogLock.unlock();
}


bool gSetLogFile(const char *name)
{
	assert(name);
	LOG(DEEPDEBUG) << "setting log path to " << name;
	bool retVal = true;
	gLogLock.lock();
	FILE* newLoggingFile = fopen(name,"a+");
	if (!newLoggingFile) {
		LOG(ERROR) << "cannot open \"" << name << "\" for logging.";
		retVal = false;
	} else {
		gLoggingFile = newLoggingFile;
	}
	gLogLock.unlock();
	LOG(FORCE) << "new log path " << name;
	return retVal;
}




Log::~Log()
{
	// XXX always handle alarms, even if the logging level is too low

	// Current logging level was already checked by the macro.
	// So just log.
	gLogLock.lock();
	mStream << std::endl;
	fprintf(gLoggingFile, "%s", mStream.str().c_str());
	fflush(gLoggingFile);
	gLogLock.unlock();
}


ostringstream& Log::get()
{
	timeval now;
	time_t lt;
	struct tm *ptm;
	ptm=localtime(&lt);
	mStream.precision(4);
	mStream <<ptm->tm_hour<<":"<<ptm->tm_min<<":"<<ptm->tm_sec<< ' ' << mReportLevel <<  ' ';
	return mStream;
}
ostringstream& Log::set()
{   
	time_t lt;
    lt=time(NULL);
    struct tm *ptm;
    ptm=localtime(&lt);
	mStream.precision(4);
	mStream <<ptm->tm_year+1900<<"-" 
			<<setw(2)<<setfill('0')<<ptm->tm_mon+1<<"-"
			<<setw(2)<<setfill('0')<<ptm->tm_mday<<" "
			<<setw(2)<<setfill('0')<<ptm->tm_hour<<":"
			<<setw(2)<<setfill('0')<<ptm->tm_min<<":" 
			<<setw(2)<<setfill('0')<<ptm->tm_sec;
	return mStream;
}




void gLogInit(const char* defaultLevel)
{
	// Define defaults in the global config
	/*if (!gConfig.defines("Log.Level")) {
		gConfig.set("Log.Level",defaultLevel);
		LOG(FORCE) << "Setting initial global logging level to " << defaultLevel;
	}
	if (!gConfig.defines("Log.Alarms.TargetPort")) {
		gConfig.set("Log.Alarms.TargetPort",DEFAULT_ALARM_PORT);
	}
	if (!gConfig.defines("Log.Alarms.Max")) {
		gConfig.set("Log.Alarms.Max",DEFAULT_MAX_ALARMS);
	}*/
}




// vim: ts=4 sw=4
