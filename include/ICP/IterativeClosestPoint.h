//////////////////////////////////////////////////////////////////////
/// Statistical Shape Analysis
///
/// Copyright (c) 2014 by Jan Moeller
///
/// This software is provided "as-is" and does not claim to be
/// complete or free of bugs in any way. It should work, but
/// it might also begin to hurt your kittens.
//////////////////////////////////////////////////////////////////////

#ifndef ITERATIVECLOSESTPOINT_H_
#define ITERATIVECLOSESTPOINT_H_

#include "SFA/Model.h"

namespace sfa
{
    /**
     * @brief Base class for ICP implementations
     */
    class IterativeClosestPoint
    {
	public:
	    /**
	     * @brief Destructor
	     */
	    virtual ~IterativeClosestPoint() = 0;
	    /**
	     * @brief Calculates the next ICP step and applies it to source
	     * @param source Source model
	     * @param dest Destination model
	     */
	    virtual void calcNextStep(Model& source, Model const& dest) = 0;
	private:
    };
}

#endif /* ITERATIVECLOSESTPOINT_H_ */
