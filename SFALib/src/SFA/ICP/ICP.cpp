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
	    return m_pPointSelector->select(source);

	// Otherwise we'll just pick some points randomly. Note that the chosen points won't be
	// distributed evenly across the mesh, making data acquired with that method less reliable.

	// Copy all vertices
	std::vector<Vertex> vertices;
	// Initialize random number generator
	std::uniform_int_distribution<uint32_t> rand_uint_0_1(0,1);
	// Remove all the ones we don't need
	for(unsigned int i = 0; i < source.getAmountOfVertices(); i++)
	{
	    auto vertex = source.getVertex(i);
	    bool insert = true;
	    if (insert && (m_selectionMethod & PointSelection::NO_EDGES))
		insert = !vertex.isEdge;
	    if(insert && (m_selectionMethod & PointSelection::RANDOM))
		insert = rand_uint_0_1(m_random);
	    if(insert && (m_selectionMethod & PointSelection::EVERY_SECOND))
		insert = (i % 2 != 0);
	    if(insert && (m_selectionMethod & PointSelection::EVERY_THIRD))
		insert = (i % 3 != 0);
	    if(insert && (m_selectionMethod & PointSelection::EVERY_FOURTH))
		insert = (i % 4 != 0);
	    if(insert && (m_selectionMethod & PointSelection::EVERY_FIFTH))
		insert = (i % 5 != 0);
	    if(insert)
		vertices.push_back(vertex);
	}
	// Pick n% of those already selected
	if(m_selectionPercentage < 1)
	{
	    double elementsToPick = vertices.size() * m_selectionPercentage;
	    for(auto it = vertices.begin(); it != vertices.end(); ++it)
	    {
		double needed = elementsToPick;
		double left = std::distance(it, vertices.end());
		double probability = needed / left;
		std::uniform_real_distribution<double> rand_double(0, 1);
		if(rand_double(m_random) <= probability)
		    elementsToPick--;
		else
		{
		    vertices.erase(it);
		    --it;
		}
	    }
	}
	if (m_pLog != nullptr)
	    m_pLog->info("Selected %d points on source mesh.", vertices.size());
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

    unsigned int& ICP::selectionMethod()
    {
	return m_selectionMethod;
    }

    unsigned int const& ICP::getSelectionMethod() const
    {
	return m_selectionMethod;
    }

    void ICP::setSelectionMethod(unsigned int flags)
    {
	m_selectionMethod = flags;
    }

    double const& ICP::getSelectionPercentage() const
    {
	return m_selectionPercentage;
    }

    void ICP::setSelectionPercentage(double percentage)
    {
	m_selectionPercentage = percentage;
    }
}
