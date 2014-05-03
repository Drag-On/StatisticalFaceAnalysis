//////////////////////////////////////////////////////////////////////
/// Statistical Face Analysis
///
/// Copyright (c) 2014 by Jan Moeller
///
/// This software is provided "as-is" and does not claim to be
/// complete or free of bugs in any way. It should work, but
/// it might also begin to hurt your kittens.
//////////////////////////////////////////////////////////////////////

#include "SFA/Stats/AverageMatchingError.h"

namespace sfa
{
    void AverageMatchingError::run(Model& src, Model& dest, NearestNeighbor& nn, ICP& icp, dbgl::Properties& props)
    {
	LOG->info("Starting rigid body point-to-point ICP statistics test suite...");

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
	bool pcaFirst = false;
	if(props.getStringValue(Prop_PCAFirst) != "")
	    pcaFirst = props.getBoolValue(Prop_PCAFirst);

	int pointSelection = 0;
	if(props.getStringValue(Prop_PairSelection) != "")
	    pointSelection = props.getIntValue(Prop_PairSelection);
	icp.setSelectionMethod(pointSelection);
	pairSelection = getPairSelectionFlags(icp.getSelectionMethod());

	// Allocate enough space
	averageAlgoResults.resize(icpCycles, 0);
	averageRealResults.resize(icpCycles, 0);
	averageAmountOfMatches.resize(icpCycles, 0);

	srcVertices = src.getAmountOfVertices();
	destVertices = dest.getAmountOfVertices();

	// Add noise?
	noiseLevel = 0;
	if(props.getStringValue(Prop_NoiseLevel) != "")
	    noiseLevel = props.getIntValue(Prop_NoiseLevel);
	for(unsigned int i = 0; i < noiseLevel; i++)
	    src.addNoise();

	// Add holes?
	holes = 0;
	if(props.getStringValue(Prop_Holes) != "")
	    holes = props.getIntValue(Prop_Holes);
	for(unsigned int i = 0; i < holes; i++)
	    src.addHole();

	initCorrectPairs(src, dest, nn, icp);
	testWithModel(src, dest, nn, icp, pcaFirst);
    }

    void AverageMatchingError::testWithModel(Model& src, Model& dest, NearestNeighbor& nn, ICP& icp, bool pcaFirst)
    {
	// Store original model
	Model original(src);
	// Iterate %randCycles% times
	for (unsigned int i = 0; i < randCycles; i++)
	{
	    // Log
	    if(i % 10 == 0)
		LOG->info("%d...", i);
	    // Reset original vertex positions
	    src = original;
	    // Displace src
	    if (maxRot > 0)
		averageRotation += src.rotateRandom(maxRot, minRot);
	    if (maxTrans > 0)
		averageTranslation += src.translateRandom(maxTrans, minTrans);
	    // Do pca alignment first?
	    averageAlgoErrorBeforePCA += nn.computeError(src, dest);
	    averageRealErrorBeforePCA += nn.computeError(src, dest, correctPairs);
	    if(pcaFirst)
	    {
		pca_icp.calcNextStep(src, dest); // First dimension
		pca_icp.calcNextStep(src, dest); // Second dimension
		averageAlgoErrorAfterPCA += nn.computeError(src, dest);
		averageRealErrorAfterPCA += nn.computeError(src, dest, correctPairs);
	    }
	    for (unsigned int j = 0; j < icpCycles; j++)
	    {
		// Calculate next icp step
		icp.calcNextStep(src, dest);
		// Check matching error
		unsigned int matches = 0;
		averageAlgoResults[j] += nn.computeError(src, dest);
		averageRealResults[j] += nn.computeError(src, dest, correctPairs, &matches);
		averageAmountOfMatches[j] += matches;
	    }
	}
	// Average results
	for (unsigned int i = 0; i < averageAlgoResults.size(); i++)
	{
	    averageAlgoResults[i] /= randCycles;
	    averageRealResults[i] /= randCycles;
	    averageAmountOfMatches[i] /= randCycles;
	}
	averageAlgoErrorBeforePCA /= randCycles;
	averageAlgoErrorAfterPCA /= randCycles;
	averageRealErrorBeforePCA /= randCycles;
	averageRealErrorAfterPCA /= randCycles;
	averageRotation /= randCycles;
	averageTranslation /= randCycles;
    }

    void AverageMatchingError::initCorrectPairs(Model& src, Model& dest, NearestNeighbor& nn, ICP& icp)
    {
	// Store original vertex positions
	Model original(src);
	unsigned int selectionMethod = icp.getSelectionMethod();
	icp.setSelectionMethod(ICP::NO_EDGES);
	// Calculate a lot if icp steps to make sure we have the correct pairs
	for (unsigned int i = 0; i < icpCycles; i++)
	{
	    icp.calcNextStep(src, dest);
	}
	// Store pairs
	correctPairs.clear();
	correctPairs.resize(src.getAmountOfVertices());
	for (unsigned int i = 0; i < src.getAmountOfVertices(); i++)
	{
	    correctPairs[i] = nn.getNearest(i, src, dest);
	}
	// Revert back to original vertex positions
	icp.setSelectionMethod(selectionMethod);
	src = std::move(original);
	LOG->info("Initialization done.");
    }

    void AverageMatchingError::printResults()
    {
	LOG->info("RESULTS (rotation in the range of [%f, %f], average rotation: %f, translation in the range of [%f, %f], average translation: %f, pair selection filter: %s, noise level: %d, holes: %d, %d source vertices, %d destination vertices):", maxRot, minRot, averageRotation, maxTrans, minTrans, averageTranslation, pairSelection.c_str(), noiseLevel, holes, srcVertices, destVertices);
	LOG->info("Average matching error before any ICP steps:");
	LOG->info("Nearest neighbor matching error: %.10f.", averageAlgoErrorBeforePCA);
	LOG->info("Real matching error: %.10f.", averageRealErrorBeforePCA);
	LOG->info("Average matching error after PCA alignment:");
	LOG->info("Nearest neighbor matching error: %.10f.", averageAlgoErrorAfterPCA);
	LOG->info("Real matching error: %.10f.", averageRealErrorAfterPCA);
	LOG->info("Average nearest neighbor matching error after %d cycles for the first %d ICP steps:", randCycles, icpCycles);
	for(unsigned int i = 0; i < averageAlgoResults.size(); i++)
	    LOG->info("Step %d: %.10f.", i+1, averageAlgoResults[i]);
	LOG->info("Average real matching error after %d cycles for the first %d ICP steps:", randCycles, icpCycles);
	for(unsigned int i = 0; i < averageRealResults.size(); i++)
	    LOG->info("Step %d: %.10f.", i+1, averageRealResults[i]);
	LOG->info("Average amount of matching pairs after %d cycles for the first %d icp steps:", randCycles, icpCycles);
	for(unsigned int i = 0; i < averageAmountOfMatches.size(); i++)
	    LOG->info("Step %d: %f / %d.", i+1, averageAmountOfMatches[i], correctPairs.size());
    }

    void AverageMatchingError::writeResults()
    {
	// Generate file name
	std::string fileName = "Results_Avrg_Err_";
	if(props->getBoolValue(Prop_PCAFirst))
	    fileName += "PCA_";
	fileName += pairSelection;

	// Write average nearest neighbor error
	std::string fileNameNNError(fileName);
	fileNameNNError += "_NN";
	fileNameNNError += ".txt";
	std::ofstream file;
	file.open(fileNameNNError, std::ios::out | std::ios::app);
	if (file.is_open())
	{
	    file << "# " << fileNameNNError << "\n";
	    file << "# Rotation in the range of [" << maxRot << ", " << minRot << "], average: "
		    << averageRotation << ".\n";
	    file << "# Translation in the range of [" << maxTrans << ", " << minTrans << "], average: "
		    << averageTranslation << ".\n";
	    file << "# Pair selection filter: " << pairSelection.c_str() << ".\n";
	    file << "# Noise level: " << noiseLevel << ", holes: " << holes << ".\n";
	    file << "# " << srcVertices << " source vertices, " << destVertices << " destination vertices\n";
	    file << "0" << "\t" << averageAlgoErrorBeforePCA << "\n";
	    unsigned int offset = 0;
	    if(averageAlgoErrorAfterPCA > 0)
	    {
		file << "1" << "\t" << averageAlgoErrorAfterPCA << "\n";
		offset = 1;
	    }
	    for (unsigned int i = 0; i < averageAlgoResults.size(); i++)
		file << i+1+offset << "\t" << averageAlgoResults[i] << "\n";
	    file.close();
	}
	else
	    LOG->warning("Unable to write %s.", fileNameNNError.c_str());
	// Write average real error
	std::string fileNameRealError(fileName);
	fileNameRealError += "_Real";
	fileNameRealError += ".txt";
	file.open(fileNameRealError, std::ios::out | std::ios::app);
	if (file.is_open())
	{
	    file << "# " << fileNameRealError << "\n";
	    file << "# Rotation in the range of [" << maxRot << ", " << minRot << "], average: "
		    << averageRotation << ".\n";
	    file << "# Translation in the range of [" << maxTrans << ", " << minTrans << "], average: "
		    << averageTranslation << ".\n";
	    file << "# Pair selection filter: " << pairSelection.c_str() << ".\n";
	    file << "# Noise level: " << noiseLevel << ", holes: " << holes << ".\n";
	    file << "# " << srcVertices << " source vertices, " << destVertices << " destination vertices\n";
	    file << "0" << "\t" << averageRealErrorBeforePCA << "\n";
	    unsigned int offset = 0;
	    if(averageRealErrorAfterPCA > 0)
	    {
		file << "1" << "\t" << averageRealErrorAfterPCA << "\n";
		offset = 1;
	    }
	    for (unsigned int i = 0; i < averageRealResults.size(); i++)
		file << i+1+offset << "\t" << averageRealResults[i] << "\n";
	    file.close();
	}
	else
	    LOG->warning("Unable to write %s.", fileNameRealError.c_str());
	// Write average amount of matching pairs
	std::string fileNamePairs(fileName);
	fileNamePairs += "_Pairs";
	fileNamePairs += ".txt";
	file.open(fileNamePairs, std::ios::out | std::ios::app);
	if (file.is_open())
	{
	    file << "# " << fileNamePairs << "\n";
	    file << "# Rotation in the range of [" << maxRot << ", " << minRot << "], average: "
		    << averageRotation << ".\n";
	    file << "# Translation in the range of [" << maxTrans << ", " << minTrans << "], average: "
		    << averageTranslation << ".\n";
	    file << "# Pair selection filter: " << pairSelection.c_str() << ".\n";
	    file << "# Noise level: " << noiseLevel << ", holes: " << holes << ".\n";
	    file << "# " << srcVertices << " source vertices, " << destVertices << " destination vertices\n";
	    for (unsigned int i = 0; i < averageAmountOfMatches.size(); i++)
		file << i+1 << "\t" << averageAmountOfMatches[i] << "\n";
	    file.close();
	}
	else
	    LOG->warning("Unable to write %s.", fileNamePairs.c_str());
    }

    std::string AverageMatchingError::getPairSelectionFlags(dbgl::Bitmask<> flags)
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
	if(flagString.size() > 3)
	    flagString.erase(flagString.end() - 3, flagString.end());
	return flagString;
    }
}
