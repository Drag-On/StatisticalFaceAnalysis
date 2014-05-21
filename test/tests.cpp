//////////////////////////////////////////////////////////////////////
/// Statistical Face Analysis
///
/// Copyright (c) 2014 by Jan Moeller
///
/// This software is provided "as-is" and does not claim to be
/// complete or free of bugs in any way. It should work, but
/// it might also begin to hurt your kittens.
//////////////////////////////////////////////////////////////////////

#include <DBGL/Window/WindowManager.h>
#include <DBGL/System/Log/Log.h>

// Prototypes
void testModel();
void testNearestNeighbor();
void testPoissonDiskSampler();
void testRigidPointICP();
void testRigidPlaneICP();

int main()
{
    LOG.setLogLevel(dbgl::Log::Level::DBG);
    LOG.info("Starting tests...");

    testModel();
    testNearestNeighbor();
    testPoissonDiskSampler();
    testRigidPointICP();
    testRigidPlaneICP();

    LOG.info("Done!");
    dbgl::WindowManager::get()->terminate();

    return 0;
}

