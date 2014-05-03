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
	LOG->info("Starting rigid body point-to-point ICP statistics test suite...");

	this->props = &props;

	// Initialize variables from properties
	if(props.getStringValue(Prop_ToRot) != "")
	    toRot = props.getFloatValue(Prop_ToRot);
	if(props.getStringValue(Prop_FromRot) != "")
	    fromRot = props.getFloatValue(Prop_FromRot);
	if(props.getStringValue(Prop_RotSteps) != "")
	    rotSteps = props.getFloatValue(Prop_RotSteps);

	// Allocate enough space
	algoResults.resize(rotSteps, 0);
	realResults.resize(rotSteps, 0);
	amountOfMatches.resize(rotSteps, 0);

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
	    // Reset original vertex positions
	    src = original;
	    // Displace src
	    if (curRotation > 0)
		src.rotateRandom(curRotation, curRotation);
	    // Store matching error before PCA
	    unsigned int matches = 0;
	    algoResults[rotCycle] = nn.computeError(src, dest);
	    realResults[rotCycle] = nn.computeError(src, dest, correctPairs, &matches);
	    amountOfMatches[rotCycle] = matches;
	    // Do PCA alignment
	    pca_icp.reset();
	    pca_icp.calcNextStep(src, dest); // First dimension
	    pca_icp.calcNextStep(src, dest); // Second dimension
	    // Calculate error again
	    algoResults[rotCycle] = algoResults[rotCycle] - nn.computeError(src, dest);
	    realResults[rotCycle] = realResults[rotCycle] - nn.computeError(src, dest, correctPairs, &matches);
	    amountOfMatches[rotCycle] = matches - amountOfMatches[rotCycle];
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

    void PCAMatchingError::printResults()
    {
	LOG->info("RESULTS (rotation in %d steps from %f to %f, noise level: %d, holes: %d, %d source vertices, %d destination vertices):", rotSteps, fromRot, toRot, noiseLevel, holes, srcVertices, destVertices);
	for(unsigned int rotCycle = 0; rotCycle < rotSteps; rotCycle++)
	{
	    double curRotation = rotCycle * ((toRot - fromRot) / rotSteps + fromRot);
	    LOG->info("Rotation step %d (%f rad): %f NN error.", rotCycle, curRotation, algoResults[rotCycle]);
	    LOG->info("Rotation step %d (%f rad): %f real error.", rotCycle, curRotation, realResults[rotCycle]);
	    LOG->info("Rotation step %d (%f rad): %f matches.", rotCycle, curRotation, amountOfMatches[rotCycle]);
	}
    }

    void PCAMatchingError::writeResults()
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
	    file << "# Rotation in " << rotSteps << " steps from " << fromRot << " to " << toRot << ".\n";
	    file << "# Noise level: " << noiseLevel << ", holes: " << holes << ".\n";
	    file << "# " << srcVertices << " source vertices, " << destVertices << " destination vertices\n";
	    for(unsigned int rotCycle = 0; rotCycle < rotSteps; rotCycle++)
	    {
		double curRotation = rotCycle * ((toRot - fromRot) / rotSteps + fromRot);
		file << curRotation << "\t" << algoResults[rotCycle] << "\n";
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
	    file << "# Rotation in " << rotSteps << " from " << fromRot << " to " << toRot << ".\n";
	    file << "# Noise level: " << noiseLevel << ", holes: " << holes << ".\n";
	    file << "# " << srcVertices << " source vertices, " << destVertices << " destination vertices\n";
	    for(unsigned int rotCycle = 0; rotCycle < rotSteps; rotCycle++)
	    {
		double curRotation = rotCycle * ((toRot - fromRot) / rotSteps + fromRot);
		file << curRotation << "\t" << realResults[rotCycle] << "\n";
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
	    file << "# Rotation in " << rotSteps << " from " << fromRot << " to " << toRot << ".\n";
	    file << "# Noise level: " << noiseLevel << ", holes: " << holes << ".\n";
	    file << "# " << srcVertices << " source vertices, " << destVertices << " destination vertices\n";
	    for(unsigned int rotCycle = 0; rotCycle < rotSteps; rotCycle++)
	    {
		double curRotation = rotCycle * ((toRot - fromRot) / rotSteps + fromRot);
		file << curRotation << "\t" << amountOfMatches[rotCycle] << "\n";
	    }
	    file.close();
	}
	else
	    LOG->warning("Unable to write %s.", fileNamePairs.c_str());
    }
}
