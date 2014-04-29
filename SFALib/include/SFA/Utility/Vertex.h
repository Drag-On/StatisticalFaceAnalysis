//////////////////////////////////////////////////////////////////////
/// Statistical Shape Analysis
///
/// Copyright (c) 2014 by Jan Moeller
///
/// This software is provided "as-is" and does not claim to be
/// complete or free of bugs in any way. It should work, but
/// it might also begin to hurt your kittens.
//////////////////////////////////////////////////////////////////////

#ifndef VERTEX_H_
#define VERTEX_H_

#include <Eigen/Core>
#include <set>

namespace sfa
{
    /**
     * @brief Contains all the data of a vertex needed for nearest neighbor search and icp
     */
    struct Vertex
    {
	public:
	    /**
	     * @brief Index of the vertex
	     */
	    unsigned int id;
	    /**
	     * @brief Coordinates of the vertex
	     */
	    Eigen::Vector3d coords;
	    /**
	     * @brief Normal of the vertex
	     */
	    Eigen::Vector3d normal;
	    /**
	     * @brief Indices of all neighboring vertices
	     */
	    std::set<unsigned int> neighbors;
	    /**
	     * @brief Indices of all vertices that represent this vertex internally
	     * @details For visualization vertices might need to be copied. E.g. in OpenGL
	     * 		hard edges can only be displayed by copying the edge vertices. Thus
	     * 		one "high-level" vertex (that is one used point) might have multiple
	     * 		copies internally. This set is intended to abstract away those copies.
	     */
	    std::set<unsigned int> baseVertices;
	    /**
	     * @brief Indicates if the vertex is located on the edge of the mesh
	     */
	    bool isEdge = false;
    };
}



#endif /* VERTEX_H_ */
