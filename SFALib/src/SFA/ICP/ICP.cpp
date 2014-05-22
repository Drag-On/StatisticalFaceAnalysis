//////////////////////////////////////////////////////////////////////
/// Statistical Face Analysis
///
/// Copyright (c) 2014 by Jan Moeller
///
/// This software is provided "as-is" and does not claim to be
/// complete or free of bugs in any way. It should work, but
/// it might also begin to hurt your kittens.
//////////////////////////////////////////////////////////////////////

#include "SFA/ICP/ICP.h"

namespace sfa
{
    ICP::ICP(AbstractPointSelector* pointSelector, AbstractLog* pLog)
    {
	std::random_device rd;
	m_random.seed(rd());
	m_pLog = pLog;
	m_pPointSelector = pointSelector;
    }

    ICP::~ICP()
    {
    }

    std::vector<Vertex> ICP::selectPoints(AbstractMesh& source)
    {
	// In case we've got a proper point selector use it
	if(m_pPointSelector != nullptr)
	    return m_pPointSelector->select(source, m_pointSelectionPercentage, m_pointSelectionFlags);

	// Otherwise just pick all points
	std::vector<Vertex> vertices;
	vertices.reserve(source.getAmountOfVertices());
	for(unsigned int i = 0; i < source.getAmountOfVertices(); i++)
	    vertices.push_back(source.getVertex(i));
	return vertices;
    }

    Eigen::Vector3d ICP::getAverage(std::vector<Vertex> const& points) const
    {
	Eigen::Vector3d average(0, 0, 0);
	unsigned int amount = points.size();
	for (unsigned int i = 0; i < amount; i++)
	{
	    average += points[i].coords;
	}
	average /= amount;
	return average;
    }

    void ICP::setSelectionFlags(int flags)
    {
	m_pointSelectionFlags = flags;
    }

    int ICP::getSelectionFlags() const
    {
	return m_pointSelectionFlags;
    }

    void ICP::setSelectionPercentage(float percentage)
    {
	m_pointSelectionPercentage = percentage;
    }

    float ICP::getSelectionPercentage() const
    {
	return m_pointSelectionPercentage;
    }
}
