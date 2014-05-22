//////////////////////////////////////////////////////////////////////
/// Statistical Shape Analysis
///
/// Copyright (c) 2014 by Jan Moeller
///
/// This software is provided "as-is" and does not claim to be
/// complete or free of bugs in any way. It should work, but
/// it might also begin to hurt your kittens.
//////////////////////////////////////////////////////////////////////

#ifndef ABSTRACTMESH_H_
#define ABSTRACTMESH_H_

#include <Eigen/Core>
#include "SFA/Utility/Vertex.h"

namespace sfa
{
    /**
     * @brief Marker interface that mesh classes need to implement so
     * 	      they can be used with nearest neighbor search
     */
    class AbstractMesh
    {
	public:
	    /**
	     * @brief Destructor
	     */
	    virtual ~AbstractMesh() = 0;
	    /**
	     * @brief Provides a unique ID
	     * @return The unique ID of this mesh
	     */
	    virtual unsigned int getID() const = 0;
	    /**
	     * @brief Provides all data associated with vertex n
	     * @param n Number of the vertex
	     * @return All data of vertex n
	     * @exception Throws an exception in case n is out of bounds
	     * 		  (i.e. no vertex with this number exists)
	     */
	    virtual Vertex getVertex(unsigned int n) const = 0;
	    /**
	     * @brief Alters a vertex's position and normal
	     * @param n ID of the vertex to modify
	     * @param coords New coordinates
	     * @param normal New normal
	     */
	    virtual void setVertex(unsigned int n, Eigen::Vector3d const& coords, Eigen::Vector3d const& normal) = 0;
	    /**
	     * @return Amount of vertices of this mesh
	     */
	    virtual unsigned int getAmountOfVertices() const = 0;
	    /**
	     * @brief Computes the average of the mesh
	     * @return Average vector of the mesh
	     */
	    virtual Eigen::Vector3d getAverage() const = 0;
	    /**
	     * @brief Updates the mesh after some of its vertices have been modified
	     */
	    virtual void refresh() = 0;
    };
}



#endif /* ABSTRACTMESH_H_ */
