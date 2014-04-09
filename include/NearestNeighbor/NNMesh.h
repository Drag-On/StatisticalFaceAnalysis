//////////////////////////////////////////////////////////////////////
/// Statistical Shape Analysis
///
/// Copyright (c) 2014 by Jan Moeller
///
/// This software is provided "as-is" and does not claim to be
/// complete or free of bugs in any way. It should work, but
/// it might also begin to hurt your kittens.
//////////////////////////////////////////////////////////////////////

#ifndef NNMESH_H_
#define NNMESH_H_

#include "SFA/Vertex.h"

namespace sfa
{
    /**
     * @brief Marker interface that mesh classes need to implement so
     * 	      they can be used with nearest neighbor search
     */
    class NNMesh
    {
	public:
	    virtual ~NNMesh() = 0;
	    /**
	     * @brief Provides all data associated with vertex n
	     * @param n Number of the vertex
	     * @return All data of vertex n
	     * @exception Throws an exception in case n is out of bounds
	     * 		  (i.e. no vertex with this number exists)
	     */
	    virtual Vertex getVertex(unsigned int n) const = 0;
	    /**
	     * @return Amount of vertices of this mesh
	     */
	    virtual unsigned int getAmountOfVertices() const = 0;
    };
}



#endif /* NNMESH_H_ */
