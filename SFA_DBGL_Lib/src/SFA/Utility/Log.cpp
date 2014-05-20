//////////////////////////////////////////////////////////////////////
/// Statistical Face Analysis
///
/// Copyright (c) 2014 by Jan Moeller
///
/// This software is provided "as-is" and does not claim to be
/// complete or free of bugs in any way. It should work, but
/// it might also begin to hurt your kittens.
//////////////////////////////////////////////////////////////////////

#include "SFA/Utility/Log.h"

namespace sfa
{
    Log::~Log()
    {
    }
    void Log::info(const char* msg, ...)
    {
	char buffer[128]; // Buffer for arguments
	va_list pArgList; // List of arguments

	// Make string from arguments
	va_start(pArgList, msg);
	vsprintf(buffer, msg, pArgList);
	va_end(pArgList);

	// Call DBGL log
	LOG.debug(buffer);
    }
    void Log::warning(const char* msg, ...)
    {
	char buffer[128]; // Buffer for arguments
	va_list pArgList; // List of arguments

	// Make string from arguments
	va_start(pArgList, msg);
	vsprintf(buffer, msg, pArgList);
	va_end(pArgList);

	// Call DBGL log
	LOG.warning(buffer);
    }
}

