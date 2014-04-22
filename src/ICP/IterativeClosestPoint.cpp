//////////////////////////////////////////////////////////////////////
/// Statistical Face Analysis
///
/// Copyright (c) 2014 by Jan Moeller
///
/// This software is provided "as-is" and does not claim to be
/// complete or free of bugs in any way. It should work, but
/// it might also begin to hurt your kittens.
//////////////////////////////////////////////////////////////////////

#include "ICP/IterativeClosestPoint.h"

namespace sfa
{
    IterativeClosestPoint::IterativeClosestPoint()
    {
	std::random_device rd;
	m_random.seed(rd());
    }

    IterativeClosestPoint::~IterativeClosestPoint()
    {
    }

    std::vector<Vertex> IterativeClosestPoint::selectPoints(Model& source)
    {
	// Copy all vertices
	std::vector<Vertex> vertices;
	// Initialize random number generator
	std::uniform_int_distribution<uint32_t> rand_uint_0_1(0,1);
	// Remove all the ones we don't need
	for(unsigned int i = 0; i < source.getVertices().size(); i++)
	{
	    auto vertex = source.getVertex(i);
	    bool insert = true;
	    if (insert && m_selectionMethod.isSet(PointSelection::NO_EDGES))
		insert = !vertex.isEdge;
	    if(insert && m_selectionMethod.isSet(PointSelection::RANDOM))
		insert = rand_uint_0_1(m_random);
	    if(insert && m_selectionMethod.isSet(PointSelection::EVERY_SECOND))
		insert = (i % 2 != 0);
	    if(insert && m_selectionMethod.isSet(PointSelection::EVERY_THIRD))
		insert = (i % 3 != 0);
	    if(insert && m_selectionMethod.isSet(PointSelection::EVERY_FOURTH))
		insert = (i % 4 != 0);
	    if(insert && m_selectionMethod.isSet(PointSelection::EVERY_FIFTH))
		insert = (i % 5 != 0);
	    if(insert)
		vertices.push_back(vertex);
	}
	LOG->info("Selected %d points on source mesh.", vertices.size());
	return vertices;
    }

    Eigen::Vector3d IterativeClosestPoint::getAverage(std::vector<Vertex> const& points) const
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

    dbgl::Bitmask<>& IterativeClosestPoint::selectionMethod()
    {
	return m_selectionMethod;
    }

    dbgl::Bitmask<> const& IterativeClosestPoint::getSelectionMethod() const
    {
	return m_selectionMethod;
    }

    void IterativeClosestPoint::setSelectionMethod(dbgl::Bitmask<> flags)
    {
	m_selectionMethod = flags;
    }
}
