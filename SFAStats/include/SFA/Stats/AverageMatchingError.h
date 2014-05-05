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

#include <iostream>
#include <fstream>
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
	    virtual void run(Model& src, Model& dest, NearestNeighbor& nn, ICP& icp, dbgl::Properties& props);
	    virtual void printResults();
	    virtual void writeResults();

	private:
	    void testWithModel(Model& src, Model& dest, NearestNeighbor& nn, ICP& icp);
	    void initCorrectPairs(Model& src, Model& dest, NearestNeighbor& nn, ICP& icp);
	    std::string getPairSelectionFlags(dbgl::Bitmask<> flags);

	    const std::string Prop_RandCycles = "AverageMatching_RandCycles";
	    const std::string Prop_ICPCycles = "AverageMatching_IcpCycles";
	    const std::string Prop_MaxRot = "AverageMatching_MaxRot";
	    const std::string Prop_MinRot = "AverageMatching_MinRot";
	    const std::string Prop_MaxTrans = "AverageMatching_MaxTrans";
	    const std::string Prop_MinTrans = "AverageMatching_MinTrans";
	    const std::string Prop_PairSelection = "AverageMatching_PairSelection";
	    const std::string Prop_NoiseLevel = "AverageMatching_NoiseLevel";
	    const std::string Prop_Holes = "AverageMatching_Holes";

	    unsigned int randCycles = 100;
	    unsigned int icpCycles = 30;
	    double maxRot = dbgl::pi_4();
	    double minRot = 0;
	    double maxTrans = 0.3;
	    double minTrans = 0;
	    unsigned int holes = 0;
	    unsigned int noiseLevel = 0;
	    std::vector<unsigned int> correctPairs;
	    std::vector<double> averageAlgoResults;
	    std::vector<double> averageRealResults;
	    std::vector<double> averageAmountOfMatches;
	    double averageAlgoErrorBegin = 0;
	    double averageRealErrorBegin = 0;
	    double averageRotation = 0;
	    double averageTranslation = 0;
	    double averageSelectedPoints = 0;
	    PCA_ICP pca_icp;
	    std::string pairSelection;
	    unsigned int srcVertices = 0;
	    unsigned int destVertices = 0;
	    dbgl::Properties* props = nullptr;
    };
}

#endif /* AVERAGEMATCHINGERROR_H_ */
