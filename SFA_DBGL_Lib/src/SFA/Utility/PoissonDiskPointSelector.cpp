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
    std::vector<Vertex> PoissonDiskPointSelector::select(AbstractMesh& source)
    {
	// In this case we know that source will be of type Model
	Model* pModel = dynamic_cast<Model*>(&source);

	// All points
	auto pointList = pModel->getVertexTree().getAll();
	auto selected = m_sampler.sample(pointList.begin(), pointList.end(), 0.1);

	LOG.debug("Selected points: %", selected.size());

	std::vector<Vertex> result(selected.size());
	for(auto it = selected.begin(); it != selected.end(); ++it)
	    result.push_back(source.getVertex((*it).data));

	return result;
    }
}
