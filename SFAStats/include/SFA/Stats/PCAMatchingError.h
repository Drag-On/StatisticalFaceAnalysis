//////////////////////////////////////////////////////////////////////
/// Statistical Shape Analysis
///
/// Copyright (c) 2014 by Jan Moeller
///
/// This software is provided "as-is" and does not claim to be
/// complete or free of bugs in any way. It should work, but
/// it might also begin to hurt your kittens.
//////////////////////////////////////////////////////////////////////

#ifndef PCAMATCHINGERROR_H_
#define PCAMATCHINGERROR_H_

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
    class PCAMatchingError: public StatRunner
    {
	public:
	    virtual void run(Model& src, Model& dest, NearestNeighbor& nn, ICP& icp, dbgl::Properties& props);
	    virtual void printResults();
	    virtual void writeResults();

	private:
	    void testWithModel(Model& src, Model& dest, NearestNeighbor& nn);
	    void initCorrectPairs(Model& src, Model& dest, NearestNeighbor& nn, ICP& icp);

	    const std::string Prop_RandCycles = "PCAMatching_RandCycles";
	    const std::string Prop_ToRot = "PCAMatching_ToRot";
	    const std::string Prop_FromRot = "PCAMatching_FromRot";
	    const std::string Prop_RotSteps = "PCAMatching_RotSteps";
	    const std::string Prop_NoiseLevel = "PCAMatching_NoiseLevel";
	    const std::string Prop_Holes = "PCAMatching_Holes";

	    unsigned int randCycles = 100;
	    double toRot = dbgl::pi_4();
	    double fromRot = 0;
	    unsigned int rotSteps = 10;
	    unsigned int holes = 0;
	    unsigned int noiseLevel = 0;
	    std::vector<unsigned int> correctPairs;
	    std::vector<double> algoResults;
	    std::vector<double> realResults;
	    std::vector<int> amountOfMatches;
	    dbgl::Properties* props = nullptr;
	    PCA_ICP pca_icp;
	    unsigned int srcVertices = 0;
	    unsigned int destVertices = 0;
    };
}

#endif /* PCAMATCHINGERROR_H_ */
