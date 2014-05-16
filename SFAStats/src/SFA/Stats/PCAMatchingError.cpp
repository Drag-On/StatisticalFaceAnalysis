//////////////////////////////////////////////////////////////////////
/// Statistical Face Analysis
///
/// Copyright (c) 2014 by Jan Moeller
///
/// This software is provided "as-is" and does not claim to be
/// complete or free of bugs in any way. It should work, but
/// it might also begin to hurt your kittens.
//////////////////////////////////////////////////////////////////////

#include "SFA/Stats/PCAMatchingError.h"

namespace sfa
{
    void PCAMatchingError::run(Model& src, Model& dest, NearestNeighbor& nn, ICP& icp, dbgl::Properties& props)
    {
	LOG->info("Starting PCAMatchingError test suite...");

	this->props = &props;

	// Initialize variables from properties
	if(props.getStringValue(Prop_RandCycles) != "")
	    randCycles = props.getIntValue(Prop_RandCycles);
	if(props.getStringValue(Prop_ToRot) != "")
	    toRot = props.getFloatValue(Prop_ToRot);
	if(props.getStringValue(Prop_FromRot) != "")
	    fromRot = props.getFloatValue(Prop_FromRot);
	if(props.getStringValue(Prop_RotSteps) != "")
	    rotSteps = props.getFloatValue(Prop_RotSteps);

	// Allocate enough space
	avrgAlgoResults.resize(rotSteps, 0);
	avrgRealResults.resize(rotSteps, 0);
	avrgAmountOfMatches.resize(rotSteps, 0);
	algoResults.resize(rotSteps);
	realResults.resize(rotSteps);
	amountOfMatches.resize(rotSteps);
	algoStdDeviation.resize(rotSteps, 0);
	realStdDeviation.resize(rotSteps, 0);
	pairsStdDeviation.resize(rotSteps, 0);

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
	testWithModel(src, dest, nn);
    }

    void PCAMatchingError::testWithModel(Model& src, Model& dest, NearestNeighbor& nn)
    {
	// Store original model
	Model original(src);
	// Rotate as often as wanted
	for(unsigned int rotCycle = 0; rotCycle < rotSteps; rotCycle++)
	{
	    double curRotation = rotCycle * ((toRot - fromRot) / rotSteps + fromRot);
	    // Log
	    LOG->info("%d...", rotCycle);
	    double curNNMatching = 0;
	    double curRealMatching = 0;
	    unsigned int curAmountOfMatches = 0;
	    for(unsigned int i = 0; i < randCycles; i++)
	    {
		// Reset original vertex positions
		src = original;
		// Displace src
		if (curRotation > 0)
		    src.rotateRandom(curRotation, curRotation);
		// Store matching error before PCA
		double nnError = nn.computeError(src, dest);
		double realError = nn.computeError(src, dest, correctPairs, &curAmountOfMatches);
		curNNMatching = nnError;
		curRealMatching = realError;
		// Do PCA alignment
		pca_icp.reset();
		pca_icp.calcNextStep(src, dest); // First dimension
		pca_icp.calcNextStep(src, dest); // Second dimension
		// Calculate error again
		unsigned int matches = 0;
		double nnErrorChange = curNNMatching - nn.computeError(src, dest);
		double realErrorChange = curRealMatching - nn.computeError(src, dest, correctPairs, &matches);
		double pairChange = (int)matches - (int)curAmountOfMatches;
		avrgAlgoResults[rotCycle] += nnErrorChange;
		avrgRealResults[rotCycle] += realErrorChange;
		avrgAmountOfMatches[rotCycle] += pairChange;
		algoResults[rotCycle].push_back(nnErrorChange);
		realResults[rotCycle].push_back(realErrorChange);
		amountOfMatches[rotCycle].push_back(pairChange);
	    }
	    // Average results
	    avrgAlgoResults[rotCycle] /= randCycles;
	    avrgRealResults[rotCycle] /= randCycles;
	    avrgAmountOfMatches[rotCycle] /= randCycles;
	    algoStdDeviation[rotCycle] = calcStandardDeviation(algoResults[rotCycle].begin(), algoResults[rotCycle].end());
	    realStdDeviation[rotCycle] = calcStandardDeviation(realResults[rotCycle].begin(), realResults[rotCycle].end());
	    pairsStdDeviation[rotCycle] = calcStandardDeviation(amountOfMatches[rotCycle].begin(), amountOfMatches[rotCycle].end());
	}
    }

    void PCAMatchingError::initCorrectPairs(Model& src, Model& dest, NearestNeighbor& nn, ICP& icp)
    {
	// Store original vertex positions
	Model original(src);
	unsigned int selectionMethod = icp.getSelectionMethod();
	icp.setSelectionMethod(ICP::NO_EDGES);
	// Calculate a lot if icp steps to make sure we have the correct pairs
	for (unsigned int i = 0; i < 30; i++)
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

    void PCAMatchingError::printResults(dbgl::Properties& props)
    {
	LOG->info("RESULTS (rotation in %d steps from %f to %f, noise level: %d, holes: %d, %d source vertices, %d destination vertices):", rotSteps, fromRot, toRot, noiseLevel, holes, srcVertices, destVertices);
	LOG->info("Source: %s", props.getStringValue("src").c_str());
	LOG->info("Destination: %s", props.getStringValue("dest").c_str());
	for(unsigned int rotCycle = 0; rotCycle < rotSteps; rotCycle++)
	{
	    double curRotation = rotCycle * ((toRot - fromRot) / rotSteps + fromRot);
	    LOG->info("Rotation step %d (%f rad): %f NN error. Std deviation: %f.", rotCycle, curRotation, avrgAlgoResults[rotCycle], algoStdDeviation[rotCycle]);
	    LOG->info("Rotation step %d (%f rad): %f real error. Std deviation: %f.", rotCycle, curRotation, avrgRealResults[rotCycle], realStdDeviation[rotCycle]);
	    LOG->info("Rotation step %d (%f rad): %f matches. Std deviation: %f.", rotCycle, curRotation, avrgAmountOfMatches[rotCycle], pairsStdDeviation[rotCycle]);
	}
    }

    void PCAMatchingError::writeResults(dbgl::Properties& props)
    {
	// Generate file name
	std::string fileName = "Results_PCA_Matching_";
	fileName += rotSteps + "_Steps";

	// Write nearest neighbor error
	std::string fileNameNNError(fileName);
	fileNameNNError += "_NN";
	fileNameNNError += ".txt";
	std::ofstream file;
	file.open(fileNameNNError, std::ios::out | std::ios::app);
	if (file.is_open())
	{
	    file << "# " << fileNameNNError << "\n";
	    file << "# Source: " << props.getStringValue("src") << "\n";
	    file << "# Destination: " << props.getStringValue("dest") << "\n";
	    file << "# Rotation in " << rotSteps << " steps from " << fromRot << " to " << toRot << ".\n";
	    file << "# Noise level: " << noiseLevel << ", holes: " << holes << ".\n";
	    file << "# " << srcVertices << " source vertices, " << destVertices << " destination vertices\n";
	    file << "rotation\tnn error\tstd deviation\n";
	    for(unsigned int rotCycle = 0; rotCycle < rotSteps; rotCycle++)
	    {
		double curRotation = rotCycle * ((toRot - fromRot) / rotSteps + fromRot);
		file << curRotation << "\t" << avrgAlgoResults[rotCycle] << "\t" << algoStdDeviation[rotCycle] << "\n";
	    }
	    file.close();
	}
	else
	    LOG->warning("Unable to write %s.", fileNameNNError.c_str());
	// Write real error
	std::string fileNameRealError(fileName);
	fileNameRealError += "_Real";
	fileNameRealError += ".txt";
	file.open(fileNameRealError, std::ios::out | std::ios::app);
	if (file.is_open())
	{
	    file << "# " << fileNameRealError << "\n";
	    file << "# Source: " << props.getStringValue("src") << "\n";
	    file << "# Destination: " << props.getStringValue("dest") << "\n";
	    file << "# Rotation in " << rotSteps << " from " << fromRot << " to " << toRot << ".\n";
	    file << "# Noise level: " << noiseLevel << ", holes: " << holes << ".\n";
	    file << "# " << srcVertices << " source vertices, " << destVertices << " destination vertices\n";
	    file << "rotation\treal error\tstd deviation\n";
	    for(unsigned int rotCycle = 0; rotCycle < rotSteps; rotCycle++)
	    {
		double curRotation = rotCycle * ((toRot - fromRot) / rotSteps + fromRot);
		file << curRotation << "\t" << avrgRealResults[rotCycle] << "\t" << realStdDeviation[rotCycle] << "\n";
	    }
	    file.close();
	}
	else
	    LOG->warning("Unable to write %s.", fileNameRealError.c_str());
	// Write amount of matching pairs
	std::string fileNamePairs(fileName);
	fileNamePairs += "_Pairs";
	fileNamePairs += ".txt";
	file.open(fileNamePairs, std::ios::out | std::ios::app);
	if (file.is_open())
	{
	    file << "# " << fileNamePairs << "\n";
	    file << "# Source: " << props.getStringValue("src") << "\n";
	    file << "# Destination: " << props.getStringValue("dest") << "\n";
	    file << "# Rotation in " << rotSteps << " from " << fromRot << " to " << toRot << ".\n";
	    file << "# Noise level: " << noiseLevel << ", holes: " << holes << ".\n";
	    file << "# " << srcVertices << " source vertices, " << destVertices << " destination vertices\n";
	    file << "rotation\tpair error\tstd deviation\n";
	    for(unsigned int rotCycle = 0; rotCycle < rotSteps; rotCycle++)
	    {
		double curRotation = rotCycle * ((toRot - fromRot) / rotSteps + fromRot);
		file << curRotation << "\t" << avrgAmountOfMatches[rotCycle] << "\t" << pairsStdDeviation[rotCycle] << "\n";
	    }
	    file.close();
	}
	else
	    LOG->warning("Unable to write %s.", fileNamePairs.c_str());
    }
}
