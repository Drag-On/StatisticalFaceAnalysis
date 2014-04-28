//////////////////////////////////////////////////////////////////////
/// Statistical Shape Analysis
///
/// Copyright (c) 2014 by Jan Moeller
///
/// This software is provided "as-is" and does not claim to be
/// complete or free of bugs in any way. It should work, but
/// it might also begin to hurt your kittens.
//////////////////////////////////////////////////////////////////////

#ifndef ABSTRACTLOG_H_
#define ABSTRACTLOG_H_

namespace sfa
{
    /**
     * @brief Pure virtual interface class for a log file
     */
    class AbstractLog
    {
	public:
	    virtual ~AbstractLog() = 0;
	    /**
	     * @brief Logs messages in case the logger is in info mode or lower
	     * @param msg Message to log
	     */
	    virtual void info(const char* msg, ...) = 0;

	    /**
	     * @brief Logs messages in case the logger is in warning mode or lower
	     * @param msg Message to log
	     */
	    virtual void warning(const char* msg, ...) = 0;
    };
}

#endif /* ABSTRACTLOG_H_ */
