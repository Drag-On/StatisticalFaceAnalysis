//////////////////////////////////////////////////////////////////////
/// Statistical Face Analysis
///
/// Copyright (c) 2014 by Jan Moeller
///
/// This software is provided "as-is" and does not claim to be
/// complete or free of bugs in any way. It should work, but
/// it might also begin to hurt your kittens.
//////////////////////////////////////////////////////////////////////

#include "SFA/NearestNeighbor/SimpleNearestNeighbor.h"

namespace sfa
{
    unsigned int SimpleNearestNeighbor::getNearest(unsigned int n, AbstractMesh const& source,
	    AbstractMesh const& dest)
    {
	// Check if arguments are valid
	if(source.getAmountOfVertices() <= 0 || dest.getAmountOfVertices() <= 0)
	    throw std::invalid_argument("Source and/or destination mesh don't have any vertices!");

	unsigned int curIndex = 0;
	auto cached = m_cache.find(n);
	if(cached != m_cache.end())
	    curIndex = cached->second;
	else
	{
	    // Start out with "infinite" distance
	    double minSqDist = std::numeric_limits<double>::max();
	    Vertex sourceVertex = source.getVertex(n);
	    // Iterate over all destination vertices
	    for (unsigned int i = 0; i < dest.getAmountOfVertices(); i++)
	    {
		Vertex destVertex = dest.getVertex(i);
		double sqDist = (sourceVertex.coords - destVertex.coords).squaredNorm();
		if (sqDist < minSqDist)
		{
		    minSqDist = sqDist;
		    curIndex = i;
		}
	    }
	    m_cache.insert({n, curIndex});
	}
	return curIndex;
    }

    void SimpleNearestNeighbor::clearCache()
    {
	m_cache.clear();
    }
}
