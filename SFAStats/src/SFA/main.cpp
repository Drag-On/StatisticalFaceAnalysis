//////////////////////////////////////////////////////////////////////
/// Statistical Face Analysis
///
/// Copyright (c) 2014 by Jan Moeller
///
/// This software is provided "as-is" and does not claim to be
/// complete or free of bugs in any way. It should work, but
/// it might also begin to hurt your kittens.
//////////////////////////////////////////////////////////////////////

#include <DBGL/System/Log/Log.h>
#include <DBGL/System/Properties/Properties.h>
#include <DBGL/Window/WindowManager.h>
#include "SFA/NearestNeighbor/NearestNeighbor.h"
#include "SFA/NearestNeighbor/SimpleNearestNeighbor.h"
#include "SFA/NearestNeighbor/KdTreeNearestNeighbor.h"
#include "SFA/ICP/ICP.h"
#include "SFA/ICP/RigidPointICP.h"
#include "SFA/ICP/RigidPlaneICP.h"
#include "SFA/ICP/PCA_ICP.h"
#include "SFA/Stats/StatRunner.h"
#include "SFA/Stats/AverageMatchingError.h"
#include "SFA/Stats/PCAMatchingError.h"
#include "SFA/Stats/PerformanceBenchmark.h"
#include "SFA/Utility/PoissonDiskPointSelector.h"

using namespace dbgl;
using namespace sfa;

Properties properties;
PoissonDiskPointSelector pointSelector;

bool checkProperties()
{
    return properties.getStringValue("src") != "" && properties.getStringValue("dest") != "";
}

NearestNeighbor* selectNN()
{
    if (properties.getStringValue("NearestNeighbor") == "KdTree")
    {
	LOG.info("Using K-d tree for nearest neighbor search.");
	return new KdTreeNearestNeighbor;
    }
    else if(properties.getStringValue("NearestNeighbor") == "Simple")
    {
	LOG.info("Using simple nearest neighbor search.");
	return new SimpleNearestNeighbor;
    }
    else
    {
	LOG.info("No nearest neighbor search specified. Falling back to K-d tree nearest neighbor search.");
	return new KdTreeNearestNeighbor;
    }
}

ICP* selectICP(NearestNeighbor& nn)
{
    if (properties.getStringValue("ICP") == "RigidPoint2Point")
    {
	LOG.info("Using rigid body point-to-point ICP.");
	return new RigidPointICP(nn, &pointSelector);
    }
    else if(properties.getStringValue("ICP") == "RigidPoint2Plane")
    {
	LOG.info("Using rigid body point-to-plane ICP.");
	return new RigidPlaneICP(nn, &pointSelector);
    }
    else if(properties.getStringValue("ICP") == "PCA")
    {
	LOG.info("Using PCA ICP.");
	return new PCA_ICP;
    }
    else
    {
	LOG.info("No ICP specified. Falling back to rigid body point-to-point ICP.");
	return new RigidPointICP(nn, &pointSelector);
    }
}

StatRunner* selectStatRunner()
{
    if (properties.getStringValue("StatRunner") == "AverageMatchingError")
    {
	LOG.info("Using AverageMatchingError StatRunner.");
	return new AverageMatchingError;
    }
    else if(properties.getStringValue("StatRunner") == "PCAMatchingError")
    {
	LOG.info("Using PCAMatchingError StatRunner.");
	return new PCAMatchingError;
    }
    else if(properties.getStringValue("StatRunner") == "PerformanceBenchmark")
    {
	LOG.info("Using PerformanceBenchmark StatRunner.");
	return new PerformanceBenchmark;
    }
    else
    {
	LOG.info("No StatRunner specified. Falling back to AverageMatchingError.");
	return new AverageMatchingError;
    }
}

int main(int argc, char** argv)
{
    LOG.setLogLevel(dbgl::Log::Level::DBG);
    LOG.info("Starting...");

    // Load properties file from disk
    properties.read("Properties.txt");
    // Interpret arguments
    // Skip first argument as it's the executable's path
    properties.interpret(argc-1, argv+1);

    if(!checkProperties())
    {
	LOG.info("Usage: -src Path/To/Source/Mesh");
	LOG.info("       -dest Path/To/Destination/Mesh");
	return -1;
    }

    // Select appropriate algorithms
    NearestNeighbor* pnn = selectNN();
    ICP* picp = selectICP(*pnn);
    StatRunner* pStatRunner = selectStatRunner();

    // Load meshes
    Model* pSourceModel = new Model(properties.getStringValue("src"));
    Model* pDestModel = new Model(properties.getStringValue("dest"));

    pStatRunner->run(*pSourceModel, *pDestModel, *pnn, *picp, properties);
    pStatRunner->printResults(properties);
    pStatRunner->writeResults(properties);

    delete pnn;
    delete picp;
    delete pStatRunner;
    delete pSourceModel;
    delete pDestModel;

    LOG.info("That's it!");

    // Free remaining internal resources
    WindowManager::get()->terminate();
    return 0;
}
