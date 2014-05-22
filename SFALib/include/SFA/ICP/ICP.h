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
#include "SFA/Utility/AbstractPointSelector.h"

namespace sfa
{
    /**
     * @brief Base class for iterative closest point implementations
     */
    class ICP
    {
	public:
	    /**
	     * @brief Constructor
	     * @param pLog Pointer to a log object in case logging is wanted
	     */
	    ICP(AbstractPointSelector* pointSelector, AbstractLog* pLog = nullptr);
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
	     * @brief Calculates the average of the passed points
	     * @param points Points to calculate average from
	     * @return The Average of the passed points
	     */
	    Eigen::Vector3d getAverage(std::vector<Vertex> const& points) const;
	    /**
	     * @param flags New flags
	     */
	    void setSelectionFlags(int flags);
	    /**
	     * @return Current flags
	     */
	    int getSelectionFlags() const;
	    /**
	     * @param percentage New percentage
	     */
	    void setSelectionPercentage(float percentage);
	    /**
	     * @return Current percentage
	     */
	    float getSelectionPercentage() const;
	protected:
	    /**
	     * @brief Selects a certain amount of points on the source mesh
	     * @param source Source model
	     * @return List with all points to use for ICP
	     */
	    std::vector<Vertex> selectPoints(AbstractMesh& source);

	    /**
	     * @brief Random number generator
	     */
	    std::mt19937 m_random;
	    /**
	     * @brief Plug-in possibility for library users to have some logfile output
	     */
	    AbstractLog* m_pLog = nullptr;
	    /**
	     * @brief Point selection algorithm
	     */
	    AbstractPointSelector* m_pPointSelector;
	    /**
	     * @brief Flags to request for point selection
	     */
	    int m_pointSelectionFlags = 0;
	    /**
	     * @brief Percentage of points to select from mesh
	     */
	    float m_pointSelectionPercentage = 1.0;
    };
}

#endif /* ITERATIVECLOSESTPOINT_H_ */
