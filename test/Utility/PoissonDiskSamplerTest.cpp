//////////////////////////////////////////////////////////////////////
/// Statistical Face Analysis
///
/// Copyright (c) 2014 by Jan Moeller
///
/// This software is provided "as-is" and does not claim to be
/// complete or free of bugs in any way. It should work, but
/// it might also begin to hurt your kittens.
//////////////////////////////////////////////////////////////////////

//#include <stdexcept>
//#include <assert.h>
#include <vector>
#include <DBGL/System/Log/Log.h>
#include <DBGL/System/Tree/KdTree.h>
#include <SFA/Utility/PoissonDiskSampler.h>

using namespace sfa;

void checkResult(std::vector<typename dbgl::KdTree<int, dbgl::Vec2d>::Container> result, std::vector<int> needed)
{
    for(auto item : result)
    {
	auto it = std::find(needed.begin(), needed.end(), item.data);
	if(it == needed.end())
	{
	    LOG.error("Item % was returned wrongly.", item.data);
	    assert(false);
	}
    }
    assert(result.size() == needed.size());
}

void testPoissonDiskSampler()
{
    LOG.info("Starting PoissonDiskSampler test suite...");

    // Create K-d tree with some sample data
    std::vector<dbgl::KdTree<int, dbgl::Vec2d>::Container> nodes = {
	    {dbgl::Vec2d(0, 0), 0},
	    {dbgl::Vec2d(1, 0), 1},
	    {dbgl::Vec2d(0, 1), 2},
	    {dbgl::Vec2d(1, 1), 3},
	    {dbgl::Vec2d(2, 0), 4},
	    {dbgl::Vec2d(2, 1), 5},
	    {dbgl::Vec2d(2, 2), 6},
	    {dbgl::Vec2d(1, 2), 7},
	    {dbgl::Vec2d(0, 2), 8},
    };
    dbgl::KdTree<int, dbgl::Vec2d> tree(nodes.begin(), nodes.end());

    PoissonDiskSampler sampler;

    // Test
    auto result = sampler.sample(nodes.begin(), nodes.end(), 1.1);
    assert(result.size() == 4 || result.size() == 5);
    if(result.size() == 4)
	checkResult(result, {2, 7, 1, 5});
    if(result.size() == 5)
	checkResult(result, {8, 6, 3, 0, 4});

    // Test with different values
    result = sampler.sample(nodes.begin(), nodes.end(), 1);
    assert(result.size() == 9);
    checkResult(result, {0, 1, 2, 3, 4, 5, 6, 7, 8});

    // Another test
    result = sampler.sample(nodes.begin(), nodes.end(), 1.5);
    assert(result.size() == 1 || result.size() == 2 || result.size() == 3 || result.size() == 4);
    if(result.size() == 1)
	checkResult(result, {3});
    if(result.size() == 4)
	checkResult(result, {0, 4, 6, 8});
}
