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
#include "SFA/Model.h"
#include "NearestNeighbor/SimpleNearestNeighbor.h"

using namespace sfa;

void testSimpleNearestNeighbor()
{
    LOG->info("Starting SimpleNearestNeighbor test suite...");

    // Create two default meshes
    Model source;
    Model destination;

    // Should have same amount of vertices
    assert(source.getAmountOfVertices() == destination.getAmountOfVertices());

    // Check some other methods
    for(unsigned int i = 0; i < source.getAmountOfVertices(); i++)
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
    SimpleNearestNeighbor nn;
    for (unsigned int i = 0; i < source.getAmountOfVertices(); i++)
    {
	auto nearest = nn.getNearest(i, source, destination);
	assert(nearest == i);
    }

    // Check if it throws correctly
    try
    {
	nn.getNearest(source.getAmountOfVertices(), source, destination);
    }
    catch(std::invalid_argument& e)
    {
	LOG->info(e.what());
    }
    catch(std::out_of_range& e)
    {
	LOG->info(e.what());
    }

    // Now use meshes from HD
    Model source2("Resources/Generic_Face_Translated.obj");
    Model destination2("Resources/Generic_Face.obj");

    // Check for nearest neighbors again
    // In this case the nearest neighbors are not necessarily the ones with the same index
    bool alwaysSame = true;
    for (unsigned int i = 0; i < source2.getAmountOfVertices(); i++)
    {
	auto nearest = nn.getNearest(i, source2, destination2);
	if(nearest != i)
	    alwaysSame = false;
    }
    assert(alwaysSame == false);
}

