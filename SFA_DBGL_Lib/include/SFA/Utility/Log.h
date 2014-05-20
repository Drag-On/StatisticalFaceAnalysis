//////////////////////////////////////////////////////////////////////
/// Statistical Shape Analysis
///
/// Copyright (c) 2014 by Jan Moeller
///
/// This software is provided "as-is" and does not claim to be
/// complete or free of bugs in any way. It should work, but
/// it might also begin to hurt your kittens.
//////////////////////////////////////////////////////////////////////

#ifndef SFA_LOG_H_
#define SFA_LOG_H_

#include <cstdarg>
#include <cstdio>
#include <DBGL/System/Log/Log.h>
#include "SFA/Utility/AbstractLog.h"

namespace sfa
{
    /**
     * @brief Wrapper for the DBGL log implementation
     */
    class Log : public AbstractLog
    {
	public:
	    virtual ~Log();
	    /**
	     * @brief Logs messages in case the logger is in info mode or lower
	     * @param msg Message to log
	     */
	    virtual void info(const char* msg, ...);

	    /**
	     * @brief Logs messages in case the logger is in warning mode or lower
	     * @param msg Message to log
	     */
	    virtual void warning(const char* msg, ...);
    };
}



#endif /* SFA_LOG_H_ */
