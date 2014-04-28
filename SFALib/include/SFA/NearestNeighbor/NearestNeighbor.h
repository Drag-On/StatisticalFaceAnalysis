//////////////////////////////////////////////////////////////////////
/// Statistical Shape Analysis
///
/// Copyright (c) 2014 by Jan Moeller
///
/// This software is provided "as-is" and does not claim to be
/// complete or free of bugs in any way. It should work, but
/// it might also begin to hurt your kittens.
//////////////////////////////////////////////////////////////////////

#ifndef NEARESTNEIGHBOR_H_
#define NEARESTNEIGHBOR_H_

#include <vector>
#include <stdexcept>
#include <Eigen/Core>
#include "SFA/Utility/AbstractMesh.h"

namespace sfa
{
    /**
     * @brief Base class for nearest neighbor calculations
     */
    class NearestNeighbor
    {
	public:
	    virtual ~NearestNeighbor() = 0;
	    /**
	     * @brief Computes the nearest neighbor of source's n-th vertex on dest
	     * @param n Number of vertex on source mesh
	     * @param source Source mesh
	     * @param dest Destination mesh
	     * @return Number of the destination vertex that's closest to source
	     */
	    virtual unsigned int getNearest(unsigned int n, AbstractMesh const& source, AbstractMesh const& dest) = 0;
	    /**
	     * @brief Calculates all nearest neighbors for points on source on dest
	     * @param points Points on source to calculate nearest neighbors for
	     * @param source Source mesh
	     * @param dest Destination mesh
	     * @return List of all nearest neighbors for the passed points
	     */
	    std::vector<Vertex> getAllNearest(std::vector<Vertex> points, AbstractMesh const& source, AbstractMesh const& dest);
	    /**
	     * @brief Computes the error between two meshes
	     * @details Error is measured by the mean squared distance between points
	     * 		and their nearest neighbors. Note that switching source and
	     * 		dest can give a different result due to recalculation of
	     * 		nearest neighbors.
	     * @param source Source mesh
	     * @param dest Destination mesh
	     * @return Error value
	     */
	    double computeError(AbstractMesh const& source, AbstractMesh const& dest);
	    /**
	     * @brief Clears previously cached correspondences
	     */
	    virtual void clearCache() = 0;
	private:
    };
}



#endif /* NEARESTNEIGHBOR_H_ */
