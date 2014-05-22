//////////////////////////////////////////////////////////////////////
/// Statistical Face Analysis
///
/// Copyright (c) 2014 by Jan Moeller
///
/// This software is provided "as-is" and does not claim to be
/// complete or free of bugs in any way. It should work, but
/// it might also begin to hurt your kittens.
//////////////////////////////////////////////////////////////////////

#include "SFA/Utility/PoissonDiskPointSelector.h"

namespace sfa
{
    std::vector<Vertex> PoissonDiskPointSelector::select(AbstractMesh& source, float percentage, int flags)
    {
	// In this case we know that source will be of type Model
	Model* pModel = dynamic_cast<Model*>(&source);

	// All points
	auto pointList = pModel->getVertexTree().getAll();

	// Sort out edges if needed
	if(flags & AbstractPointSelector::SelectionFlags::NO_EDGES)
	{
	    for(auto it = pointList.begin(); it != pointList.end();)
	    {
		if(source.getVertex((*it).data).isEdge)
		    pointList.erase(it);
		else
		    ++it;
	    }
	}

	// If all points are supposed to be used just return all of them
	if(percentage >= 1)
	{
	    std::vector<Vertex> all;
	    all.reserve(pointList.size());
	    for (auto it = pointList.begin(); it != pointList.end(); ++it)
		all.push_back(source.getVertex((*it).data));
	    LOG.info("Selected points: %, targeted %.", all.size(), pointList.size());
	    return all;
	}

	// Compute amount of points to target
	double targetPoints = percentage * pointList.size();

	// Compute average distance between two points
	double avrgDist = 0;
	for(auto it = pointList.begin(); it != pointList.end(); ++it)
	{
	    decltype(pointList) found;
	    pModel->getVertexTree().findKNearestNeighbors((*it).point, 2, found);
	    dbgl::KdTree<unsigned int, dbgl::Vector3<double>>::Container nn;
	    if(found.size() > 0 && found[0].data != (*it).data)
		nn = found[0];
	    else if(found.size() > 1)
		nn = found[1];
	    avrgDist += (it->point-nn.point).getLength();
	}
	avrgDist /= pointList.size();

	// Guess a distance to acquire the right amount of points
	double r = (3.0 / 4.0) * avrgDist + (avrgDist * ((1.0 / percentage) - 1.0));

	// Sample
	auto selected = m_sampler.sample(pointList.begin(), pointList.end(), r, 30);

	// Log
	LOG.info("Selected points: %, targeted %.", selected.size(), targetPoints);

	// Write results back
	std::vector<Vertex> result;
	result.reserve(selected.size());
	for(auto it = selected.begin(); it != selected.end(); ++it)
	    result.push_back(source.getVertex((*it).data));

	return result;
    }
}
