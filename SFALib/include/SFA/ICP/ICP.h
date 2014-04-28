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
#include "SFA/Utility/AbstractMesh.h"
#include "SFA/Utility/Vertex.h"
#include "SFA/Utility/AbstractLog.h"

namespace sfa
{
    /**
     * @brief Base class for iterative closest point implementations
     */
    class ICP
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
	     * @param log Pointer to a log object in case logging is wanted
	     */
	    ICP(AbstractLog* pLog = nullptr);
	    /**
	     * @brief Destructor
	     */
	    virtual ~ICP() = 0;
	    /**
	     * @brief Calculates the next ICP step and applies it to source
	     * @param source Source model
	     * @param dest Destination model
	     */
	    virtual void calcNextStep(AbstractMesh& source, AbstractMesh const& dest) = 0;
	    /**
	     * @brief Selects a certain amount of points on the source mesh
	     * @param source Source model
	     * @return List with all points to use for ICP
	     */
	    std::vector<Vertex> selectPoints(AbstractMesh& source);
	    /**
	     * @brief Calculates the average of the passed points
	     * @param points Points to calculate average from
	     * @return The Average of the passed points
	     */
	    Eigen::Vector3d getAverage(std::vector<Vertex> const& points) const;
	    /**
	     * @return Selection method flags
	     */
	    unsigned int& selectionMethod();
	    /**
	     * @return Selection method flags
	     */
	    unsigned int const& getSelectionMethod() const;
	    /**
	     * @brief Modifies the selection method
	     * @param flags Determines which points are filtered out
	     */
	    void setSelectionMethod(unsigned int flags);
	protected:
	    unsigned int m_selectionMethod = 0;
	    std::mt19937 m_random;
	    AbstractLog* m_pLog = nullptr;
    };
}

#endif /* ITERATIVECLOSESTPOINT_H_ */
