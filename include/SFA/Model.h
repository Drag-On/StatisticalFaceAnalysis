//////////////////////////////////////////////////////////////////////
/// Statistical Shape Analysis
///
/// Copyright (c) 2014 by Jan Moeller
///
/// This software is provided "as-is" and does not claim to be
/// complete or free of bugs in any way. It should work, but
/// it might also begin to hurt your kittens.
//////////////////////////////////////////////////////////////////////

#ifndef MODEL_H_
#define MODEL_H_

#include <string>
#include <stdexcept>
#include <sstream>
#include <iostream>
#include <Eigen/Core>
#include <Rendering/Mesh.h>
#include "NearestNeighbor/NNMesh.h"
#include "SFA/Vertex.h"

namespace sfa
{
    /**
     * @brief This is the mesh implementation used for calculations.
     * 	      Internally it uses the dbgl mesh implementation.
     */
    class Model: public NNMesh
    {
	public:
	    Model();
	    Model(std::string path);
	    virtual ~Model();
	    virtual Vertex getVertex(unsigned int n) const;
	    void setVertex(unsigned int n, Vertex v) const;
	    virtual unsigned int getAmountOfVertices() const;
	    Eigen::Vector3d getAverage() const;
	    dbgl::Mesh* getBasePointer();
	private:
	    dbgl::Mesh* m_pMesh;
    };
}

#endif /* MODEL_H_ */
