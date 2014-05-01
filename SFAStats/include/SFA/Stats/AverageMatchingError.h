//////////////////////////////////////////////////////////////////////
/// Statistical Shape Analysis
///
/// Copyright (c) 2014 by Jan Moeller
///
/// This software is provided "as-is" and does not claim to be
/// complete or free of bugs in any way. It should work, but
/// it might also begin to hurt your kittens.
//////////////////////////////////////////////////////////////////////

#ifndef AVERAGEMATCHINGERROR_H_
#define AVERAGEMATCHINGERROR_H_

#include <vector>
#include <DBGL/System/Log/Log.h>
#include "StatRunner.h"
#include "SFA/Utility/Model.h"
#include "SFA/NearestNeighbor/NearestNeighbor.h"
#include "SFA/ICP/ICP.h"

namespace sfa
{
    class AverageMatchingError : public StatRunner
    {
	public:
	    unsigned int randCycles = 100;
	    unsigned int icpCycles = 30;
	    double maxRot = dbgl::pi_4();
	    double maxTrans = 0.3;
	    std::vector<unsigned int> correctPairs;
	    std::vector<double> averageAlgoResults;
	    std::vector<double> averageRealResults;
	    std::vector<double> averageAmountOfMatches;

	    AverageMatchingError();
	    virtual void run(Model& src, Model& dest, NearestNeighbor& nn, ICP& icp, dbgl::Properties& props);
	    void testWithModel(Model& src, Model& dest, NearestNeighbor& nn, ICP& icp);
	    void initCorrectPairs(Model& src, Model& dest, NearestNeighbor& nn, ICP& icp);
	    virtual void printResults();
    };
}

#endif /* AVERAGEMATCHINGERROR_H_ */
