//////////////////////////////////////////////////////////////////////
/// Statistical Face Analysis
///
/// Copyright (c) 2014 by Jan Moeller
///
/// This software is provided "as-is" and does not claim to be
/// complete or free of bugs in any way. It should work, but
/// it might also begin to hurt your kittens.
//////////////////////////////////////////////////////////////////////

#include "NearestNeighbor/SimpleNearestNeighbor.h"

namespace sfa
{
    unsigned int SimpleNearestNeighbor::getNearest(unsigned int n, NNMesh const& source,
	    NNMesh const& dest) const
    {
	// Check if arguments are valid
	if(source.getAmountOfVertices() <= 0 || dest.getAmountOfVertices() <= 0)
	    throw std::invalid_argument("Source and/or destination mesh don't have any vertices!");

	// Start out with "infinite" distance
	double minSqDist = std::numeric_limits<double>::max();
	unsigned int curIndex = 0;
	NNVertex sourceVertex = source.getVertex(n);
	// Iterate over all destination vertices
	for(unsigned int i = 0; i < dest.getAmountOfVertices(); i++)
	{
	    NNVertex destVertex = dest.getVertex(i);
	    double sqDist = sourceVertex.coords.getSquaredDistance(destVertex.coords);
	    if(sqDist < minSqDist)
	    {
		minSqDist = sqDist;
		curIndex = i;
	    }
	}
	return curIndex;
    }
}
