//////////////////////////////////////////////////////////////////////
/// Statistical Shape Analysis
///
/// Copyright (c) 2014 by Jan Moeller
///
/// This software is provided "as-is" and does not claim to be
/// complete or free of bugs in any way. It should work, but
/// it might also begin to hurt your kittens.
//////////////////////////////////////////////////////////////////////

#ifndef RIGIDPOINTICP_H_
#define RIGIDPOINTICP_H_

#include <Eigen/SVD>
#include "ICP.h"
#include "SFA/NearestNeighbor/NearestNeighbor.h"

namespace sfa
{
    /**
     * @brief Rigid body point-to-point ICP
     */
    class RigidPointICP: public ICP
    {
	public:
	    /**
	     * @brief Constructs the icp object using nn to get the nearest neighbors
	     * @param nn Nearest neighbor implementation to use
	     * @param pLog Log to use or nullptr to disable logging
	     */
	    RigidPointICP(NearestNeighbor& nn, AbstractLog* pLog = nullptr);
	    virtual ~RigidPointICP();
	    virtual unsigned int calcNextStep(AbstractMesh& source, AbstractMesh const& dest);
	private:
	    NearestNeighbor& m_nearestNeighbor;
    };
}

#endif /* RIGIDPOINTICP_H_ */
