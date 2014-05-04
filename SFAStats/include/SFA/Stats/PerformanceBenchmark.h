//////////////////////////////////////////////////////////////////////
/// Statistical Shape Analysis
///
/// Copyright (c) 2014 by Jan Moeller
///
/// This software is provided "as-is" and does not claim to be
/// complete or free of bugs in any way. It should work, but
/// it might also begin to hurt your kittens.
//////////////////////////////////////////////////////////////////////

#ifndef PERFORMANCEBENCHMARK_H_
#define PERFORMANCEBENCHMARK_H_

#include <chrono>
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

using std::chrono::duration_cast;
using std::chrono::microseconds;
using std::chrono::steady_clock;

namespace sfa
{
    class PerformanceBenchmark : public StatRunner
    {
	public:
	    virtual void run(Model& src, Model& dest, NearestNeighbor& nn, ICP& icp, dbgl::Properties& props);
	    virtual void printResults();
	    virtual void writeResults();
	private:
	    void testWithModel(Model& src, Model& dest, NearestNeighbor& nn, ICP& icp);
	    std::string getPairSelectionFlags(dbgl::Bitmask<> flags);

	    const std::string Prop_RandCycles = "PerformanceBenchmark_RandCycles";
	    const std::string Prop_ICPCycles = "PerformanceBenchmark_IcpCycles";
	    const std::string Prop_MaxRot = "PerformanceBenchmark_MaxRot";
	    const std::string Prop_MinRot = "PerformanceBenchmark_MinRot";
	    const std::string Prop_MaxTrans = "PerformanceBenchmark_MaxTrans";
	    const std::string Prop_MinTrans = "PerformanceBenchmark_MinTrans";
	    const std::string Prop_PairSelection = "PerformanceBenchmark_PairSelection";

	    unsigned int randCycles = 100;
	    unsigned int icpCycles = 30;
	    double maxRot = dbgl::pi_4();
	    double minRot = 0;
	    double maxTrans = 0.3;
	    double minTrans = 0;
	    double averageTime = 0;
	    double averageRotation = 0;
	    double averageTranslation = 0;
	    PCA_ICP pca_icp;
	    std::string pairSelection;
	    unsigned int srcVertices = 0;
	    unsigned int destVertices = 0;
	    dbgl::Properties* props = nullptr;
    };
}



#endif /* PERFORMANCEBENCHMARK_H_ */
