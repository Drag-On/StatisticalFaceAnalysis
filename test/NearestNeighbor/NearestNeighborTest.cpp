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
#include <SFA/NearestNeighbor/KdTreeNearestNeighbor.h>

using namespace sfa;

/**
 * @brief Starts the test suite for the passed nearest neighbor instance
 * @param nn Nearest neighbor implementation to test
 */
void testNN(NearestNeighbor& nn)
{
    // Clear nn cache
    nn.clearCache();

    // Create two default meshes
    Model source;
    Model destination;

    // Should have same amount of vertices
    assert(source.getAmountOfVertices() == destination.getAmountOfVertices());

    // Check some other methods
    for (unsigned int i = 0; i < source.getAmountOfVertices(); i++)
    {
	assert(source.getVertex(i).id == i);
	assert(destination.getVertex(i).id == i);
	assert(source.getVertex(i).coords.x() == destination.getVertex(i).coords.x());
	assert(source.getVertex(i).coords.y() == destination.getVertex(i).coords.y());
	assert(source.getVertex(i).coords.z() == destination.getVertex(i).coords.z());
	assert(source.getVertex(i).normal.x() == destination.getVertex(i).normal.x());
	assert(source.getVertex(i).normal.y() == destination.getVertex(i).normal.y());
	assert(source.getVertex(i).normal.z() == destination.getVertex(i).normal.z());
    }

    // Check for nearest neighbors
    // Both of them are identical, therefore the nearest neighbor should
    // be at the same index
    for (unsigned int i = 0; i < source.getAmountOfVertices(); i++)
    {
	auto nearest = nn.getNearest(i, source, destination);
	assert(nearest == i);
    }

    // Check alignment error
    assert(nn.computeError(source, destination) == 0);

    // Check if it throws correctly
    try
    {
	nn.getNearest(source.getAmountOfVertices(), source, destination);
    }
    catch (std::invalid_argument& e)
    {
	LOG.info(e.what());
    }
    catch (std::out_of_range& e)
    {
	LOG.info(e.what());
    }

    // Clear cache again
    nn.clearCache();

    // Now use meshes from HD
    Model source2("Resources/Cube_Transformed.obj");
    Model destination2("Resources/Cube.obj");

    // Check alignment error
    assert(nn.computeError(source2, destination2) > 0);

    // Check for nearest neighbors again, this time of the meshes from HD
    for (unsigned int i = 0; i < source2.getAmountOfVertices(); i++)
    {
	auto nearest = nn.getNearest(i, source2, destination2);
	assert(nearest == i);
    }

    // Clear cache again
    nn.clearCache();

    // Check for nearest neighbors of the cube and the plane.
    // In this case the nearest neighbors are not necessarily the ones with the same index
    bool alwaysSame = true;
    for (unsigned int i = 0; i < source2.getAmountOfVertices(); i++)
    {
	auto nearest = nn.getNearest(i, source2, destination);
	if (nearest != i)
	    alwaysSame = false;
    }
    assert(alwaysSame == false);
}

void testNearestNeighbor()
{
    LOG.info("Starting SimpleNearestNeighbor test suite...");
    SimpleNearestNeighbor snn;
    testNN(snn);

    LOG.info("Starting KdTreeNearestNeighbor test suite...");
    KdTreeNearestNeighbor kdnn;
    testNN(kdnn);

}

