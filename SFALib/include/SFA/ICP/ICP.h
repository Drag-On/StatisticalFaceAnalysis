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
	    /**
	     * @brief Parameters to consider for point selection algorithm
	     */
	    enum PointSelection
	    {
		NO_EDGES = 1 << 0,    //!< NO_EDGES
		RANDOM = 1 << 1,      //!< RANDOM
		EVERY_SECOND = 1 << 2,//!< EVERY_SECOND
		EVERY_THIRD = 1 << 3, //!< EVERY_THIRD
		EVERY_FOURTH = 1 << 4,//!< EVERY_FOURTH
		EVERY_FIFTH = 1 << 5, //!< EVERY_FIFTH
	    };

	    /**
	     * @brief Constructor
	     * @param pLog Pointer to a log object in case logging is wanted
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
	     * @return The amount of points used for the calculation
	     */
	    virtual unsigned int calcNextStep(AbstractMesh& source, AbstractMesh const& dest) = 0;
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
	    /**
	     * @return Percentage (range of [0,1])
	     */
	    double const& getSelectionPercentage() const;
	    /**
	     * @brief Modifies the selection percentage
	     * @param percentage Percentage in the range of [0,1]
	     */
	    void setSelectionPercentage(double percentage);
	protected:
	    /**
	     * @brief Bitwise OR-ed parameters from PointSelection
	     */
	    unsigned int m_selectionMethod = 0;
	    double m_selectionPercentage = 1;
	    /**
	     * @brief Random number generator
	     */
	    std::mt19937 m_random;
	    /**
	     * @brief Plug-in possibility for library users to have some logfile output
	     */
	    AbstractLog* m_pLog = nullptr;
    };
}

#endif /* ITERATIVECLOSESTPOINT_H_ */
