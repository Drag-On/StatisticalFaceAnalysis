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

// Prototypes
void testSimpleNearestNeighbor();
void testRigidPointICP();

int main()
{
    LOG->setLogLevel(dbgl::DBG);
    LOG->info("Starting tests...");

    testSimpleNearestNeighbor();
    testRigidPointICP();

    LOG->info("Done!");
    return 0;
}

