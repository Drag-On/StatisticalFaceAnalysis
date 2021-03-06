//////////////////////////////////////////////////////////////////////
/// Statistical Shape Analysis
///
/// Copyright (c) 2014 by Jan Moeller
///
/// This software is provided "as-is" and does not claim to be
/// complete or free of bugs in any way. It should work, but
/// it might also begin to hurt your kittens.
//////////////////////////////////////////////////////////////////////

#ifndef STATRUNNER_H_
#define STATRUNNER_H_

#include <DBGL/System/Properties/Properties.h>
#include "SFA/Utility/Model.h"
#include "SFA/NearestNeighbor/NearestNeighbor.h"
#include "SFA/ICP/ICP.h"

namespace sfa
{
    class StatRunner
    {
	public:
	    virtual ~StatRunner() {};
	    virtual void run(Model& src, Model& dest, NearestNeighbor& nn, ICP& icp, dbgl::Properties& props) = 0;
	    virtual void printResults(dbgl::Properties& props) = 0;
	    virtual void writeResults(dbgl::Properties& props) = 0;
	    template <typename Iterator> double calcMean(Iterator begin, Iterator end) const;
	    template <typename Iterator> double calcVariance(Iterator begin, Iterator end) const;
	    template <typename Iterator> double calcStandardDeviation(Iterator begin, Iterator end) const;
    };
}

#include "StatRunner.imp"

#endif /* STATRUNNER_H_ */
