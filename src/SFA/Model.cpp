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
	m_pMesh = dbgl::Mesh::load(path, dbgl::Mesh::OBJ, false, false);
    }

    Model::~Model()
    {
	delete m_pMesh;
    }

    NNVertex Model::getVertex(unsigned int n) const
    {
	if(n >= getAmountOfVertices())
	{
	    std::stringstream msg;
	    msg << "Vertex number out of bounds: " << n;
	    throw std::out_of_range(msg.str());
	}

	auto vertex = m_pMesh->getVertices()[n];
	auto normal = m_pMesh->getNormals()[n];

	NNVertex vert {n,
	    Eigen::Vector3d(vertex.x(), vertex.y(), vertex.z()),
	    Eigen::Vector3d(normal.x(), normal.y(), normal.z())};

	return vert;
    }

    unsigned int Model::getAmountOfVertices() const
    {
	return m_pMesh->getVertices().size();
    }
}
