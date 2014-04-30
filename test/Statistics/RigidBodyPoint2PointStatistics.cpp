//////////////////////////////////////////////////////////////////////
/// Statistical Face Analysis
///
/// Copyright (c) 2014 by Jan Moeller
///
/// This software is provided "as-is" and does not claim to be
/// complete or free of bugs in any way. It should work, but
/// it might also begin to hurt your kittens.
//////////////////////////////////////////////////////////////////////

#include <stdexcept>
#include <assert.h>
#include <vector>
#include <DBGL/System/Log/Log.h>
#include <SFA/Utility/Model.h>
#include <SFA/ICP/RigidPointICP.h>
#include <SFA/NearestNeighbor/KdTreeNearestNeighbor.h>

using namespace sfa;

KdTreeNearestNeighbor nn;
RigidPointICP icp(nn);
unsigned int randCycles = 100;
unsigned int icpCycles = 30;
double maxRot = dbgl::pi_4();
double maxTrans = 0.3;
std::vector<unsigned int> correctPairs;
std::vector<double> averageAlgoResults(icpCycles, 0);
std::vector<double> averageRealResults(icpCycles, 0);
std::vector<double> averageAmountOfMatches(icpCycles, 0);

void testWithModel(Model& src, Model& dest)
{
    // Store original model
    Model original(src);
    // Iterate %randCycles% times
    for (unsigned int i = 0; i < randCycles; i++)
    {
	// Reset original vertex positions
	src = original;
	// Displace src
	if (maxRot > 0)
	    src.rotateRandom(maxRot);
	if (maxTrans > 0)
	    src.translateRandom(maxTrans);
	for (unsigned int j = 0; j < icpCycles; j++)
	{
	    std::vector<bool> matches;
	    averageAlgoResults[j] += nn.computeError(src, dest);
	    averageRealResults[j] += nn.computeError(src, dest, correctPairs, &matches);
	    // Calc amount of matches
	    unsigned int amountMatches = 0;
	    for(unsigned int k = 0; k < matches.size(); k++)
		if(matches[k])
		    amountMatches++;
	    averageAmountOfMatches[j] += amountMatches;
	    // Calculate next icp step
	    icp.calcNextStep(src, dest);
	}
    }
    // Average results
    unsigned int amount = randCycles * icpCycles;
    for (unsigned int i = 0; i < averageAlgoResults.size(); i++)
    {
	averageAlgoResults[i] /= amount;
	averageRealResults[i] /= amount;
	averageAmountOfMatches[i] /= amount;
    }
}

void initCorrectPairs(Model& src, Model& dest)
{
    // Store original vertex positions
    Model original(src);
    icp.setSelectionMethod(ICP::NO_EDGES);
    // Calculate a lot if icp steps to make sure we have the correct pairs
    for (unsigned int i = 0; i < icpCycles; i++)
    {
	icp.calcNextStep(src, dest);
    }
    // Store pairs
    correctPairs.clear();
    correctPairs.resize(src.getAmountOfVertices());
    for(unsigned int i = 0; i < src.getAmountOfVertices(); i++)
    {
	correctPairs[i] = nn.getNearest(i, src, dest);
    }
    // Revert back to original vertex positions
    icp.setSelectionMethod(0);
    src = std::move(original);
}

void printResults()
{
    LOG->info("Average matching error the algorithm sees after %d cycles for the first %d icp steps (max rotation of %f, max translation of %f):", randCycles, icpCycles, maxRot, maxTrans);
    for(unsigned int i = 0; i < averageAlgoResults.size(); i++)
    {
	LOG->info("Step %d: %.10f.", i, averageAlgoResults[i]);
    }
    LOG->info("Average real matching error after %d cycles for the first %d icp steps (max rotation of %f, max translation of %f):", randCycles, icpCycles, maxRot, maxTrans);
    for(unsigned int i = 0; i < averageRealResults.size(); i++)
    {
	LOG->info("Step %d: %.10f.", i, averageRealResults[i]);
    }
    LOG->info("Average amount of matching pairs after %d cycles for the first %d icp steps (max rotation of %f, max translation of %f):", randCycles, icpCycles, maxRot, maxTrans);
    for(unsigned int i = 0; i < averageAmountOfMatches.size(); i++)
    {
	LOG->info("Step %d: %f / %d.", i, averageAmountOfMatches[i], correctPairs.size());
    }
}

void testRigidPointICPStatistics()
{
    LOG->info("Starting rigid body point-to-point ICP statistics test suite...");
    // Load models
    Model src("Resources/Generic_Face_2_Lowpoly.obj");
    Model dest("Resources/Generic_Face_Lowpoly.obj");
    initCorrectPairs(src, dest);
    testWithModel(src, dest);
    printResults();
}
