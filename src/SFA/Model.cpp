//////////////////////////////////////////////////////////////////////
/// Statistical Face Analysis
///
/// Copyright (c) 2014 by Jan Moeller
///
/// This software is provided "as-is" and does not claim to be
/// complete or free of bugs in any way. It should work, but
/// it might also begin to hurt your kittens.
//////////////////////////////////////////////////////////////////////

#include "SFA/Model.h"

namespace sfa
{
    Model::Model()
    {
	// If no path is specified use a plane. This is just for testing
	// purposes where we don't want to load a file from HD.
	m_pMesh = dbgl::Mesh::makePlane(0);
	analyzeMesh();
    }

    Model::Model(std::string path)
    {
	m_pMesh = dbgl::Mesh::load(dbgl::Mesh::OBJ, path, dbgl::Mesh::Optimize);
	analyzeMesh();
    }

    Model::~Model()
    {
	delete m_pMesh;
    }

    Vertex Model::getVertex(unsigned int n) const
    {
	if (n >= getAmountOfVertices())
	{
	    std::stringstream msg;
	    msg << "Vertex number out of bounds: " << n;
	    throw std::out_of_range(msg.str());
	}
	return m_vertices[n];
    }

    void Model::setVertex(unsigned int n, Eigen::Vector3d coords, Eigen::Vector3d normal)
    {
	if (n >= getAmountOfVertices())
	{
	    std::stringstream msg;
	    msg << "Vertex number out of bounds: " << n;
	    throw std::out_of_range(msg.str());
	}

	// Store in own data structure
	auto vertex = getVertex(n);
	vertex.coords = coords;
	vertex.normal = normal;
	m_vertices[n] = vertex;

	// Pass to mesh class
	m_pMesh->vertices()[n].x() = coords.x();
	m_pMesh->vertices()[n].y() = coords.y();
	m_pMesh->vertices()[n].z() = coords.z();
	m_pMesh->normals()[n].x() = normal.x();
	m_pMesh->normals()[n].y() = normal.y();
	m_pMesh->normals()[n].z() = normal.z();
    }

    unsigned int Model::getAmountOfVertices() const
    {
	return m_vertices.size();
    }

    Eigen::Vector3d Model::getAverage() const
    {
	Eigen::Vector3d average(0, 0, 0);
	unsigned int amount = getAmountOfVertices();
	for (unsigned int i = 0; i < amount; i++)
	{
	    average += getVertex(i).coords;
	}
	average /= amount;
	return average;
    }

    dbgl::Mesh* Model::getBasePointer()
    {
	return m_pMesh;
    }

    void Model::analyzeMesh()
    {
	// Add all vertices
	for (unsigned int i = 0; i < m_pMesh->getVertices().size(); i++)
	{
	    auto vertex = m_pMesh->getVertices()[i];
	    auto normal = m_pMesh->getNormals()[i];

	    Vertex vert;
	    vert.id = i;
	    vert.coords = Eigen::Vector3d(vertex.x(), vertex.y(), vertex.z());
	    vert.normal = Eigen::Vector3d(normal.x(), normal.y(), normal.z());
	    m_vertices.push_back(vert);
	}

	// Compute neighbors
	for (unsigned int i = 0; i < m_pMesh->getIndices().size(); i += 3)
	{
	    m_vertices[m_pMesh->getIndices()[i + 0]].neighbors.insert(m_pMesh->getIndices()[i + 1]);
	    m_vertices[m_pMesh->getIndices()[i + 0]].neighbors.insert(m_pMesh->getIndices()[i + 2]);
	    m_vertices[m_pMesh->getIndices()[i + 1]].neighbors.insert(m_pMesh->getIndices()[i + 0]);
	    m_vertices[m_pMesh->getIndices()[i + 1]].neighbors.insert(m_pMesh->getIndices()[i + 2]);
	    m_vertices[m_pMesh->getIndices()[i + 2]].neighbors.insert(m_pMesh->getIndices()[i + 0]);
	    m_vertices[m_pMesh->getIndices()[i + 2]].neighbors.insert(m_pMesh->getIndices()[i + 1]);
	}

	// Compute edge vertices
	for (unsigned int i = 0; i < m_vertices.size(); i++)
	{
	    if (m_vertices[i].neighbors.empty())
		continue;
	    bool isEdge = false;
	    for(auto it = m_vertices[i].neighbors.begin(); it != m_vertices[i].neighbors.end(); ++it)
	    {
		// Usually this should finish on first iteration. Only due to bad luck it might need more.
		auto start = m_vertices[*it];
		isEdge = checkEdge(m_vertices[i], start, start, start);
		if(!isEdge)
		    break;
	    }

	    m_vertices[i].isEdge = isEdge;
	}
    }

    bool Model::checkEdge(Vertex base, Vertex start, Vertex last, Vertex begin, double angle)
    {
	// Get a vertex that is both a neighbor of base and start and different than last
	for (auto it = start.neighbors.begin(); it != start.neighbors.end(); ++it)
	{
	    if (*it == last.id)
		continue;
	    for (auto it2 = base.neighbors.begin(); it2 != base.neighbors.end(); ++it2)
	    {
		if (*it == *it2)
		{
		    // Found a match, now calculate angle and add it to the overall angle
		    auto vec1 = start.coords - base.coords;
		    auto vec2 = m_vertices[*it].coords - base.coords;
		    auto cosTheta = vec1.dot(vec2);
		    if (cosTheta > 1)
			cosTheta = 1.0f;
		    if (cosTheta < -1)
			cosTheta = -1.0f;
		    auto newAngle = std::acos(cosTheta);
		    auto totalAngle = newAngle + angle;
		    // Check if we're back at the beginning
		    if(*it == begin.id)
		    {
			// If we're back at the beginning and the overall angle is more than 360°
			// this is surely no edge vertex
			if (totalAngle >= 2 * dbgl::pi())
			    return false;
			else
			    // Otherwise it might be
			    return true;
		    }
		    else
		    {
			// Break if angle gets implausible. The algorithm might get trapped in a loop.
			if(totalAngle >= 4 * dbgl::pi())
			    return true;
			// Check the next vertex
			bool isEdge = checkEdge(base, m_vertices[*it], start, begin, totalAngle);
			// If checkEdge returns true there might still be a different "path" to prove
			// it's no edge, thus we continue iterating
			if (!isEdge)
			    return false;
		    }
		}
	    }
	}

	return true;
    }
}
