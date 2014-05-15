//////////////////////////////////////////////////////////////////////
/// Statistical Shape Analysis
///
/// Copyright (c) 2014 by Jan Moeller
///
/// This software is provided "as-is" and does not claim to be
/// complete or free of bugs in any way. It should work, but
/// it might also begin to hurt your kittens.
//////////////////////////////////////////////////////////////////////

#ifndef RIGIDPLANEICP_H_
#define RIGIDPLANEICP_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include "ICP.h"
#include "SFA/NearestNeighbor/NearestNeighbor.h"

namespace sfa
{
    class RigidPlaneICP : public ICP
    {
	public:
	    /**
	     * @brief Constructs the icp object using \p nn to get the nearest neighbors
	     * @param nn Nearest neighbor implementation to use
	     * @param pLog Log to use or nullptr to disable logging
	     */
	    RigidPlaneICP(NearestNeighbor& nn, AbstractLog* pLog = nullptr);
	    virtual ~RigidPlaneICP();
	    virtual unsigned int calcNextStep(AbstractMesh& source, AbstractMesh const& dest);
	private:
	    NearestNeighbor& m_nearestNeighbor;
    };
}

#endif /* RIGIDPLANEICP_H_ */
