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

#include <string>
#include <vector>
#include <DBGL/System/Log/Log.h>
#include "StatRunner.h"
#include "SFA/Utility/Model.h"
#include "SFA/NearestNeighbor/NearestNeighbor.h"
#include "SFA/ICP/ICP.h"
#include "SFA/ICP/PCA_ICP.h"

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
	    double averageAlgoErrorBeforePCA = 0;
	    double averageAlgoErrorAfterPCA = 0;
	    double averageRealErrorBeforePCA = 0;
	    double averageRealErrorAfterPCA = 0;
	    PCA_ICP pca_icp;
	    std::string pairSelection;

	    AverageMatchingError();
	    virtual void run(Model& src, Model& dest, NearestNeighbor& nn, ICP& icp, dbgl::Properties& props);
	    virtual void printResults();
	    void testWithModel(Model& src, Model& dest, NearestNeighbor& nn, ICP& icp, bool pcaFirst = false);
	    void initCorrectPairs(Model& src, Model& dest, NearestNeighbor& nn, ICP& icp);
	    std::string getPairSelectionFlags(dbgl::Bitmask<> flags);
    };
}

#endif /* AVERAGEMATCHINGERROR_H_ */
