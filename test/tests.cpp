//////////////////////////////////////////////////////////////////////
/// Statistical Face Analysis
///
/// Copyright (c) 2014 by Jan Moeller
///
/// This software is provided "as-is" and does not claim to be
/// complete or free of bugs in any way. It should work, but
/// it might also begin to hurt your kittens.
//////////////////////////////////////////////////////////////////////

#include <System/Log/Log.h>

// Prototypes
void testSimpleNearestNeighbor();

int main()
{
    LOG->setLogLevel(dbgl::DBG);
    LOG->info("Starting tests...");

    testSimpleNearestNeighbor();

    LOG->info("Done!");
    return 0;
}

