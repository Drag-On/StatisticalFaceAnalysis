//////////////////////////////////////////////////////////////////////
/// Statistical Shape Analysis
///
/// Copyright (c) 2014 by Jan Moeller
///
/// This software is provided "as-is" and does not claim to be
/// complete or free of bugs in any way. It should work, but
/// it might also begin to hurt your kittens.
//////////////////////////////////////////////////////////////////////

#ifndef ITERATIVECLOSESTPOINT_H_
#define ITERATIVECLOSESTPOINT_H_

#include <vector>
#include <random>
#include <Eigen/Core>
#include <DBGL/System/Bitmask/Bitmask.h>
#include <DBGL/System/Log/Log.h>
#include "SFA/Model.h"
#include "NearestNeighbor/Vertex.h"

namespace sfa
{
    /**
     * @brief Base class for ICP implementations
     */
    class IterativeClosestPoint
    {
	public:
	    enum PointSelection
	    {
		NO_EDGES = 1 << 0,
		RANDOM = 1 << 1,
		EVERY_SECOND = 1 << 2,
		EVERY_THIRD = 1 << 3,
		EVERY_FOURTH = 1 << 4,
		EVERY_FIFTH = 1 << 5,
	    };

	    /**
	     * @brief Constructor
	     */
	    IterativeClosestPoint();
	    /**
	     * @brief Destructor
	     */
	    virtual ~IterativeClosestPoint() = 0;
	    /**
	     * @brief Calculates the next ICP step and applies it to source
	     * @param source Source model
	     * @param dest Destination model
	     */
	    virtual void calcNextStep(Model& source, Model const& dest) = 0;
	    /**
	     * @brief Selects a certain amount of points on the source mesh
	     * @param source Source model
	     * @return List with all points to use for ICP
	     */
	    std::vector<Vertex> selectPoints(Model& source);
	    /**
	     * @brief Calculates the average of the passed points
	     * @param points Points to calculate average from
	     * @return The Average of the passed points
	     */
	    Eigen::Vector3d getAverage(std::vector<Vertex> const& points) const;
	    /**
	     * @return Selection method flags
	     */
	    dbgl::Bitmask& selectionMethod();
	    /**
	     * @return Selection method flags
	     */
	    dbgl::Bitmask const& getSelectionMethod() const;
	    /**
	     * @brief Modifies the selection method
	     * @param flags Determines which points are filtered out
	     */
	    void setSelectionMethod(dbgl::Bitmask flags);
	protected:
	    dbgl::Bitmask m_selectionMethod = 0;
	    std::mt19937 m_random;
    };
}

#endif /* ITERATIVECLOSESTPOINT_H_ */
