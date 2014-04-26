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
     * @brief Contains all the data of one vertex
     */
    struct Vertex
    {
	public:
	    unsigned int id;			// Index
	    Eigen::Vector3d coords;		// Coordinates
	    Eigen::Vector3d normal;		// Normal
	    std::set<unsigned int> neighbors;	// IDs of all neighboring vertices
	    std::set<unsigned int> baseVertices;// IDs of the vertices of the internal representation that belong to this vertex
	    bool isEdge = false;		// Indicates if the vertex is located on the edge of the mesh
    };
}



#endif /* VERTEX_H_ */
