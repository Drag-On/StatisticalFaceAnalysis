//////////////////////////////////////////////////////////////////////
/// Statistical Face Analysis
///
/// Copyright (c) 2014 by Jan Moeller
///
/// This software is provided "as-is" and does not claim to be
/// complete or free of bugs in any way. It should work, but
/// it might also begin to hurt your kittens.
//////////////////////////////////////////////////////////////////////

#include "SFA/NearestNeighbor/NearestNeighbor.h"

namespace sfa
{
    NearestNeighbor::~NearestNeighbor()
    {
    }

    std::vector<Vertex> NearestNeighbor::getAllNearest(std::vector<Vertex> points, AbstractMesh const& source,
	    AbstractMesh const& dest)
    {
	std::vector<Vertex> vertices;
	for(auto point : points)
	{
	    vertices.push_back(dest.getVertex(getNearest(point.id, source, dest)));
	}
	return vertices;
    }

    double NearestNeighbor::computeError(AbstractMesh const& source, AbstractMesh const& dest)
    {
	// Check if arguments are valid
	if(source.getAmountOfVertices() <= 0 || dest.getAmountOfVertices() <= 0)
	    throw std::invalid_argument("Source and/or destination mesh don't have any vertices!");

	double error = 0;
	unsigned int amount = source.getAmountOfVertices();
	for(unsigned int i = 0; i < amount; i++)
	{
	    unsigned int nearestNum = getNearest(i, source, dest);
	    auto sourceVert = source.getVertex(i);
	    auto destVert = dest.getVertex(nearestNum);
	    auto s = sourceVert.coords;
	    auto d = destVert.coords;
	    auto con = s - d;
	    error += con.squaredNorm();
	}
	error /= amount;
	return error;
    }

    double NearestNeighbor::computeError(AbstractMesh const& source, AbstractMesh const& dest,
	    std::vector<unsigned int> const& pairs, std::vector<bool>* matches)
    {
	// Check if arguments are valid
	if(source.getAmountOfVertices() <= 0 || dest.getAmountOfVertices() <= 0)
	    throw std::invalid_argument("Source and/or destination mesh don't have any vertices!");

	double error = 0;
	unsigned int amount = source.getAmountOfVertices();
	for(unsigned int i = 0; i < amount; i++)
	{
	    unsigned int targetIndex = pairs[i];

	    if(matches != nullptr)
	    {
		// Get nearest neighbor and check if it matches with the one defined by pairs
		unsigned int nearestNum = getNearest(i, source, dest);
		matches->push_back(nearestNum == targetIndex);
	    }
	    auto sourceVert = source.getVertex(i);
	    auto destVert = dest.getVertex(targetIndex);
	    auto s = sourceVert.coords;
	    auto d = destVert.coords;
	    auto con = s - d;
	    error += con.squaredNorm();
	}
	error /= amount;
	return error;
    }
}

