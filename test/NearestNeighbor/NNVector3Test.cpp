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
#include <System/Log/Log.h>
#include "NearestNeighbor/NNVector3.h"

using namespace sfa;

void testNNVector3()
{
    LOG->info("Starting NNVector3 test suite...");

    NNVector3 vec_1(1, 0, 0);
    NNVector3 vec_2(1, 0, 0);
    NNVector3 vec_3(0, 0, 0);
    NNVector3 vec_4(-1, 0, 0);
    NNVector3 vec_5(-1.2, 2, -3);
    NNVector3 vec_6(-1, 0, -1);

    assert(vec_1.getSquaredDistance(vec_2) == 0);
    assert(vec_1.getSquaredDistance(vec_3) == 1);
    assert(vec_1.getSquaredDistance(vec_4) == 4);
    assert(vec_1.getSquaredDistance(vec_5) == 2.2 * 2.2 + 2 * 2 + 3 * 3);
    assert(vec_6.getSquaredDistance(vec_6) == 0);
}
