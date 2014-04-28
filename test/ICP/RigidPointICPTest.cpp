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
#include <DBGL/System/Log/Log.h>
#include <SFA/Utility/Model.h>
#include <SFA/NearestNeighbor/SimpleNearestNeighbor.h>
#include <SFA/ICP/RigidPointICP.h>

using namespace sfa;

void testRigidPointICP()
{
    LOG->info("Starting RigidPointICP test suite...");

    // Load models
//    Model src("Resources/Cube_Transformed.obj");
//    Model dest("Resources/Cube.obj");
//    Model src("Resources/Generic_Face_Lowpoly_Transformed.obj");
//    Model dest("Resources/Generic_Face_Lowpoly.obj");
    Model src("Resources/Plane_Transformed.obj");
    Model dest("Resources/Plane.obj");

    // Check error
    SimpleNearestNeighbor nn;
    auto startError = nn.computeError(src, dest);
    auto error = startError;
    LOG->info("Matching error: %.20f", startError);

    // Do ICP
    RigidPointICP icp(nn);
    for(unsigned int i = 0; i < 3; i++)
    {
	// Calculate next step
	icp.calcNextStep(src, dest);

	// Check error
	error = nn.computeError(src, dest);
	LOG->info("Matching error: %.20f", error);
    }
    assert(error < startError);
}

