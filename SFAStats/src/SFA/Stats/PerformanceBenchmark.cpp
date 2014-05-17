//////////////////////////////////////////////////////////////////////
/// Statistical Face Analysis
///
/// Copyright (c) 2014 by Jan Moeller
///
/// This software is provided "as-is" and does not claim to be
/// complete or free of bugs in any way. It should work, but
/// it might also begin to hurt your kittens.
//////////////////////////////////////////////////////////////////////

#include "SFA/Stats/PerformanceBenchmark.h"

namespace sfa
{
    void PerformanceBenchmark::run(Model& src, Model& dest, NearestNeighbor& nn, ICP& icp, dbgl::Properties& props)
    {
    	LOG->info("Starting PerformanceBenchmark test suite...");

    	this->props = &props;

    	// Initialize variables from properties
    	if(props.getStringValue(Prop_RandCycles) != "")
    	    randCycles = props.getIntValue(Prop_RandCycles);
    	if(props.getStringValue(Prop_ICPCycles) != "")
    	    icpCycles = props.getIntValue(Prop_ICPCycles);
    	if(props.getStringValue(Prop_MaxRot) != "")
    	    maxRot = props.getFloatValue(Prop_MaxRot);
    	if(props.getStringValue(Prop_MaxTrans) != "")
    	    maxTrans = props.getFloatValue(Prop_MaxTrans);
    	if(props.getStringValue(Prop_MinRot) != "")
    	    minRot = props.getFloatValue(Prop_MinRot);
    	if(props.getStringValue(Prop_MinTrans) != "")
    	    minTrans = props.getFloatValue(Prop_MinTrans);

    	int pointSelection = 0;
    	if(props.getStringValue(Prop_PairSelection) != "")
    	    pointSelection = props.getIntValue(Prop_PairSelection);
	if(props.getStringValue(Prop_PairSelectionPercent) != "")
	    pairSelectionPercent = props.getFloatValue(Prop_PairSelectionPercent);
    	icp.setSelectionMethod(pointSelection);
    	pairSelection = getPairSelectionFlags(icp.getSelectionMethod());
    	icp.setSelectionPercentage(pairSelectionPercent);

    	srcVertices = src.getAmountOfVertices();
    	destVertices = dest.getAmountOfVertices();

    	times.resize(randCycles * icpCycles);

    	testWithModel(src, dest, nn, icp);
    }

    void PerformanceBenchmark::testWithModel(Model& src, Model& dest, NearestNeighbor& nn, ICP& icp)
    {
	// Store original model
	Model original(src);
	// Iterate %randCycles% times
	for (unsigned int i = 0; i < randCycles; i++)
	{
	    // Log
	    if(i % 10 == 0)
		LOG->info("%...", i);
	    // Reset original vertex positions
	    src = original;
	    // Displace src
	    if (maxRot > 0)
		averageRotation += src.rotateRandom(maxRot, minRot);
	    if (maxTrans > 0)
		averageTranslation += src.translateRandom(maxTrans, minTrans);
	    for (unsigned int j = 0; j < icpCycles; j++)
	    {
		if(typeid(icp) == typeid(PCA_ICP))
		    dynamic_cast<PCA_ICP*>(&icp)->reset();
		// Start time
		steady_clock::time_point start = steady_clock::now();
		// Calculate next icp step
		icp.calcNextStep(src, dest);
		// End time
		steady_clock::time_point end = steady_clock::now();
		// Store computation time
		double time = duration_cast<microseconds>(end - start).count();
		times[(i * icpCycles) + j] = time;
	    }
	}
	// Average results
	averageTime = calcMean(times.begin(), times.end());
	variance = calcVariance(times.begin(), times.end());
	standardDeviation = calcStandardDeviation(times.begin(), times.end());
	averageRotation /= randCycles;
	averageTranslation /= randCycles;
    }

    void PerformanceBenchmark::printResults(dbgl::Properties& props)
    {
	LOG->info("RESULTS (rotation in the range of [%, %], average rotation: %, translation in the range of [%, %], average translation: %, pair selection filter: %, % source vertices, % destination vertices):", maxRot, minRot, averageRotation, maxTrans, minTrans, averageTranslation, pairSelection.c_str(), srcVertices, destVertices);
	LOG->info("Source: %", props.getStringValue("src").c_str());
	LOG->info("Destination: %", props.getStringValue("dest").c_str());
	LOG->info("Average ICP time: % microseconds", averageTime);
	LOG->info("Variance: % microseconds", variance);
	LOG->info("Standard deviation: % microseconds", standardDeviation);
    }

    void PerformanceBenchmark::writeResults(dbgl::Properties& props)
    {
	// Generate file name
	std::string fileName = "Results_Avrg_Performance_";
	fileName += pairSelection;

	// Write average nearest neighbor error
	std::string fileNameTime(fileName);
	fileNameTime += ".txt";
	std::ofstream file;
	file.open(fileNameTime, std::ios::out | std::ios::app);
	if (file.is_open())
	{
	    file << "# " << fileNameTime << "\n";
	    file << "# Source: " << props.getStringValue("src") << "\n";
	    file << "# Destination: " << props.getStringValue("dest") << "\n";
	    file << "# Rotation in the range of [" << maxRot << ", " << minRot << "], average: "
		    << averageRotation << ".\n";
	    file << "# Translation in the range of [" << maxTrans << ", " << minTrans << "], average: "
		    << averageTranslation << ".\n";
	    file << "# Pair selection filter: " << pairSelection.c_str() << ".\n";
	    file << "# " << srcVertices << " source vertices, " << destVertices << " destination vertices\n";
	    file << "# Average (micro seconds): " << averageTime << "\n";
	    file << "# Variance: " << variance << "\n";
	    file << "# Standard deviation: " << standardDeviation << "\n";
	    file << "Iteration\t micro seconds\n";
	    for(unsigned int i = 0; i < times.size(); i++)
		file << i << "\t" << times[i] << "\n";
	    file.close();
	}
	else
	    LOG->warning("Unable to write %.", fileNameTime.c_str());
    }

    std::string PerformanceBenchmark::getPairSelectionFlags(dbgl::Bitmask<> flags)
    {
	std::string flagString;
	if(flags.isSet(ICP::NO_EDGES))
	    flagString += "NO_EDGES___";
	if(flags.isSet(ICP::RANDOM))
	    flagString += "RANDOM_";
	if(flags.isSet(ICP::EVERY_SECOND))
	    flagString += "EVERY_SECOND___";
	if(flags.isSet(ICP::EVERY_THIRD))
	    flagString += "EVERY_THIRD___";
	if(flags.isSet(ICP::EVERY_FOURTH))
	    flagString += "EVERY_FOURTH___";
	if(flags.isSet(ICP::EVERY_FIFTH))
	    flagString += "EVERY_FIFTH___";
	std::ostringstream s;
	s << pairSelectionPercent * 100;
	flagString += s.str();
	flagString += "_Percent";
	return flagString;
    }
}
