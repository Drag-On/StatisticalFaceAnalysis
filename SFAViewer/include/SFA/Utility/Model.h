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
#include <map>
#include <limits>
#include <random>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <DBGL/Rendering/Mesh/Mesh.h>
#include <DBGL/Rendering/Mesh/OBJMeshLoader.h>
#include <DBGL/Math/Utility.h>
#include <DBGL/Math/Vector3.h>
#include <DBGL/System/Tree/KdTree.h>
#include "SFA/Utility/AbstractMesh.h"

namespace sfa
{
    /**
     * @brief This is the mesh implementation used for calculations.
     * 	      Internally it uses the dbgl mesh implementation.
     */
    class Model: public AbstractMesh
    {
	public:
	    Model();
	    Model(std::string path);
	    Model(Model const& other);
	    Model(Model&& other);
	    virtual ~Model();
	    virtual unsigned int getID() const;
	    virtual Vertex getVertex(unsigned int n) const;
	    std::vector<Vertex> const& getVertices() const;
	    dbgl::KdTree<unsigned int, dbgl::Vec3d> const& getVertexTree() const;
	    virtual void setVertex(unsigned int n, Eigen::Vector3d const& coords, Eigen::Vector3d const& normal);
	    virtual unsigned int getAmountOfVertices() const;
	    Eigen::Vector3d getAverage() const;
	    void addNoise();
	    void addHole();
	    double rotateRandom(double maxAngle, double minAngle = 0);
	    double translateRandom(double maxTranslation, double minTranslation = 0);
	    dbgl::Mesh* getBasePointer();
	    Model& operator=(Model const& other);
	    Model& operator=(Model&& other);
	private:
	    /**
	     * @brief Analyzes the underlying mesh and generates some additional data
	     * @details Additional data includes neighboring vertices and if the vertex is part
	     * 		of the edge of the mesh. Vertices are considered neighbors if they share
	     * 		a face. They are considered edge vertices if there is no possible way
	     * 		of moving from one neighbor to the other and ending up at the beginning
	     * 		with a cumulative angle of 360° degree or more.
	     */
	    void analyzeMesh();
	    /**
	     * @brief Checks if a vertex is situated on the edge of a mesh.
	     * @param base Vertex to check
	     * @param start Neighbor of base to start algorithm from
	     * @return True in case no way has been found to circle base through its neighbors
	     * 	       starting from start, and ending up at start with a total angle of 360°
	     * 	       or more. False otherwise.
	     * @note If this algorithm returns false base definitely isn't an edge vertex. However,
	     * 	     if it returns true, it might still be no edge vertex as there might be a path
	     * 	     from a different neighbor to start from. To make sure that base is an edge
	     * 	     vertex this method has to be called for each neighboring vertex (passed as
	     * 	     start). If it still didn't return false, it is an edge vertex.
	     */
	    bool checkEdge(Vertex base, Vertex start);
	    /**
	     * @brief Called internally by analyzeMesh(). Checks if a vertex is situated on the
	     * 	      edge of a mesh.
	     * @param base Vertex to check
	     * @param start Neighbor of base to start algorithm from
	     * @param begin Vertex the algorithm has to come back to (should equal start on
	     * 		    first call)
	     * @param last Last checked vertex
	     * @param checked Map containing flags for every neighbor of base if it has already
	     * 		      been checked or not
	     * @return True in case no way has been found to circle base through its neighbors
	     * 	       starting from start, and ending up at begin with a total angle of 360°
	     * 	       or more. False otherwise.
	     * @note If this algorithm returns false base definitely isn't an edge vertex. However,
	     * 	     if it returns true, it might still be no edge vertex as there might be a path
	     * 	     from a different neighbor to start from. To make sure that base is an edge
	     * 	     vertex this method has to be called for each neighboring vertex (passed as
	     * 	     start). If it still didn't return false, it is an edge vertex.
	     */
	    bool checkEdge(Vertex base, Vertex start, Vertex begin, Vertex last, std::map<unsigned int, bool> checked);

	    dbgl::Mesh* m_pMesh;
	    std::vector<Vertex> m_vertices;
	    std::vector<unsigned int> m_baseIndex2ModelIndex;
	    dbgl::KdTree<unsigned int, dbgl::Vec3d> m_vertexTree;
	    std::mt19937 m_random;
    };
}

#endif /* MODEL_H_ */
