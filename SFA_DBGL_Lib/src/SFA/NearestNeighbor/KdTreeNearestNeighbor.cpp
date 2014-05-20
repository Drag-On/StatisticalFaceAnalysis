//////////////////////////////////////////////////////////////////////
/// Statistical Face Analysis
///
/// Copyright (c) 2014 by Jan Moeller
///
/// This software is provided "as-is" and does not claim to be
/// complete or free of bugs in any way. It should work, but
/// it might also begin to hurt your kittens.
//////////////////////////////////////////////////////////////////////

#include "SFA/NearestNeighbor/KdTreeNearestNeighbor.h"

namespace sfa
{
    unsigned int KdTreeNearestNeighbor::getNearest(unsigned int n, AbstractMesh const& source, AbstractMesh const& dest)
    {
	// Check if arguments are valid
	if (source.getAmountOfVertices() <= 0 || dest.getAmountOfVertices() <= 0)
	    throw std::invalid_argument("Source and/or destination mesh don't have any vertices!");

	// Get nearest neighbor
	auto realDest = dynamic_cast<const Model*>(&dest);
	dbgl::Vec3d nearest;
	unsigned int data;
	dbgl::Vec3d coords(source.getVertex(n).coords[0], source.getVertex(n).coords[1], source.getVertex(n).coords[2]);
	realDest->getVertexTree().findNearestNeighbor(coords, nearest, data);

	return data;
    }

    void KdTreeNearestNeighbor::clearCache()
    {
    }
}
