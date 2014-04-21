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
	dbgl::OBJMeshLoader loader;
	loader.setNormalCompatibilityAngle(std::numeric_limits<float>::max());
	m_pMesh = dbgl::Mesh::load(loader, path, dbgl::Mesh::Optimize);
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

    std::vector<Vertex> const& Model::getVertices() const
    {
	return m_vertices;
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
		isEdge = checkEdge(m_vertices[i], start);
		if(!isEdge)
		    break;
	    }

	    m_vertices[i].isEdge = isEdge;
	}
    }

    bool Model::checkEdge(Vertex base, Vertex start)
    {
	std::map<unsigned int, bool> checked;
	for(auto neighbor : base.neighbors)
	    checked[neighbor] = false;
	checked[start.id] = true;
	return checkEdge(base, start, start, start, checked);
    }

    bool Model::checkEdge(Vertex base, Vertex start, Vertex begin, Vertex last,
	    std::map<unsigned int, bool> checked)
    {
	// Get a vertex that is both a neighbor of base and start and different than last
	for (auto it = start.neighbors.begin(); it != start.neighbors.end(); ++it)
	{
	    // If the neighbors neighbor is the beginning but not the one we just came from
	    // then there is a way of circling base through their neighbors
	    if(*it == begin.id && *it != last.id)
		return false;
	    // If the neighbor has already been used don't use it again
	    if(checked[*it])
		continue;
	    for (auto it2 = base.neighbors.begin(); it2 != base.neighbors.end(); ++it2)
	    {
		if (*it == *it2)
		{
		    // Found a match
		    checked[*it] = true;
		    // Check the next vertex
		    bool isEdge = checkEdge(base, m_vertices[*it], begin, start, checked);
		    // If checkEdge returns true there might still be a different "path" to prove
		    // that it's no edge, thus we continue iterating
		    if (!isEdge)
			return false;
		}
	    }
	}
	// If we get here the algorithm has not found a proper way
	return true;
    }
}
