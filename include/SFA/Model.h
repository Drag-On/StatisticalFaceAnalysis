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
#include <vector>
#include <Eigen/Core>
#include <Rendering/Mesh/Mesh.h>
#include <Math/Utility.h>
#include "NearestNeighbor/NNMesh.h"

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
	    void setVertex(unsigned int n, Eigen::Vector3d coords, Eigen::Vector3d normal);
	    virtual unsigned int getAmountOfVertices() const;
	    Eigen::Vector3d getAverage() const;
	    dbgl::Mesh* getBasePointer();
	private:
	    /**
	     * @brief Analyzes the underlying mesh and generates some additional data
	     * @details Additional data includes neighboring vertices and if the vertex is part
	     * 		of the edge of the mesh. Vertices are considered neighbors if they share
	     * 		a face. They are considered edge vertices if there is no possible way
	     * 		of moving from one neighbor to the other and ending up at the beginning
	     * 		with a cumulative angle of 360� degree or more.
	     */
	    void analyzeMesh();
	    /**
	     * @brief Called internally by analyzeMesh(). Checks if a vertex is situated on the
	     * 	      edge of a mesh.
	     * @param base Vertex to check
	     * @param start Neighbor of base to start algorithm from
	     * @param last Last neighbor of base that has been checked
	     * @param begin Vertex the algorithm has to come back to (should equal start on
	     * 		    first call)
	     * @param angle Cumulative angle so far. Used for recursive calls. Should be 0 for
	     * 		    manual calls.
	     * @return True in case no way has been found to circle base through its neighbors
	     * 	       starting from start, and ending up at begin with a total angle of 360�
	     * 	       or more. False otherwise.
	     * @note If this algorithm returns false base definitely isn't an edge vertex. However,
	     * 	     if it returns true, it might still be no edge vertex as there might be a path
	     * 	     from a different neighbor to start from. To make sure that base is an edge
	     * 	     vertex this method has to be called for each neighboring vertex (passed as
	     * 	     start). If it still didn't return false, it is an edge vertex.
	     */
	    bool checkEdge(Vertex base, Vertex start, Vertex last, Vertex begin, double angle = 0);

	    dbgl::Mesh* m_pMesh;
	    std::vector<Vertex> m_vertices;
    };
}

#endif /* MODEL_H_ */
