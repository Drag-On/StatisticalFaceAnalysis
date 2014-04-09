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
	m_pMesh = dbgl::Mesh::makePlane(false, false);
    }

    Model::Model(std::string path)
    {
	m_pMesh = dbgl::Mesh::load(path, dbgl::Mesh::OBJ, false, false, false);
    }

    Model::~Model()
    {
	delete m_pMesh;
    }

    Vertex Model::getVertex(unsigned int n) const
    {
	if(n >= getAmountOfVertices())
	{
	    std::stringstream msg;
	    msg << "Vertex number out of bounds: " << n;
	    throw std::out_of_range(msg.str());
	}

	auto vertex = m_pMesh->getVertices()[n];
	auto normal = m_pMesh->getNormals()[n];

	Vertex vert {n,
	    Eigen::Vector3d(vertex.x(), vertex.y(), vertex.z()),
	    Eigen::Vector3d(normal.x(), normal.y(), normal.z())};

	return vert;
    }

    void Model::setVertex(unsigned int n, Vertex v) const
    {
	m_pMesh->vertices()[n].x() = v.coords.x();
	m_pMesh->vertices()[n].y() = v.coords.y();
	m_pMesh->vertices()[n].z() = v.coords.z();
	m_pMesh->normals()[n].x() = v.normal.x();
	m_pMesh->normals()[n].y() = v.normal.y();
	m_pMesh->normals()[n].z() = v.normal.z();
    }

    unsigned int Model::getAmountOfVertices() const
    {
	return m_pMesh->getVertices().size();
    }

    Eigen::Vector3d Model::getAverage() const
    {
	Eigen::Vector3d average(0, 0, 0);
	unsigned int amount = getAmountOfVertices();
	for(unsigned int i = 0; i < amount; i++)
	{
	    average += getVertex(i).coords;
	}
	average /= amount;
	return average;
    }
}
