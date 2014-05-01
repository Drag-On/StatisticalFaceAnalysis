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
#include "SFA/Stats/StatRunner.h"
#include "SFA/Stats/AverageMatchingError.h"

using namespace dbgl;
using namespace sfa;

Properties properties;

bool checkProperties()
{
    return properties.getStringValue("src") != "" && properties.getStringValue("dest") != "";
}

NearestNeighbor* selectNN()
{
    if (properties.getStringValue("NearestNeighbor") == "K-d tree")
    {
	LOG->info("Using K-d tree for nearest neighbor search.");
	return new KdTreeNearestNeighbor;
    }
    else if(properties.getStringValue("NearestNeighbor") == "Simple")
    {
	LOG->info("Using simple nearest neighbor search.");
	return new SimpleNearestNeighbor;
    }
    else
    {
	LOG->info("No nearest neighbor search specified. Falling back to K-d tree nearest neighbor search.");
	return new KdTreeNearestNeighbor;
    }
}

ICP* selectICP(NearestNeighbor& nn)
{
    if (properties.getStringValue("ICP") == "Rigid-Body Point-to-point")
    {
	LOG->info("Using rigid body point-to-point ICP.");
	return new RigidPointICP(nn);
    }
    else
    {
	LOG->info("No ICP specified. Falling back to rigid body point-to-point ICP.");
	return new RigidPointICP(nn);
    }
}

StatRunner* selectStatRunner()
{
    if (properties.getStringValue("StatRunner") == "AverageMatchingError")
    {
	LOG->info("Using AverageMatchingError StatRunner.");
	return new AverageMatchingError;
    }
    else
    {
	LOG->info("No StatRunner specified. Falling back to AverageMatchingError.");
	return new AverageMatchingError;
    }
}

int main(int argc, char** argv)
{
    LOG->setLogLevel(dbgl::DBG);
    LOG->info("Starting...");

    // Load properties file from disk
    properties.load("Properties.txt");
    // Interpret arguments
    // Skip first argument as it's the executable's path
    properties.interpret(argc-1, argv+1);

    if(!checkProperties())
    {
	LOG->info("Usage: -src Path/To/Source/Mesh");
	LOG->info("       -dest Path/To/Destination/Mesh");
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
    pStatRunner->printResults();

    delete pnn;
    delete picp;
    delete pStatRunner;
    delete pSourceModel;
    delete pDestModel;

    LOG->info("That's it!");

    // Free remaining internal resources
    WindowManager::get()->terminate();
    return 0;
}
