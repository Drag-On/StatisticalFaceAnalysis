//////////////////////////////////////////////////////////////////////
/// Statistical Shape Analysis
///
/// Copyright (c) 2014 by Jan Moeller
///
/// This software is provided "as-is" and does not claim to be
/// complete or free of bugs in any way. It should work, but
/// it might also begin to hurt your kittens.
//////////////////////////////////////////////////////////////////////

#ifndef PCA_ICP_H_
#define PCA_ICP_H_

#include <vector>
#include <algorithm>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <System/Log/Log.h>
#include "IterativeClosestPoint.h"
#include "IterativeClosestPoint.h"
#include "NearestNeighbor/NearestNeighbor.h"

namespace sfa
{
    /**
     * @brief "Fake" ICP algorithm based on principal components of the models
     * @details This "PCA-ICP" simply aligns the principal components of source and destination,
     * 		one after another. It isn't really an ICP implementation as it doesn't rely on
     * 		nearest neighbors. In fact, it does only work under certain assumptions.
     * 		Alignment will be improved if both source and destination have obvious principal
     * 		components (this should usually be the case for face models). If this is not the
     * 		case, the algorithm is likely to fail miserably.
     */
    class PCA_ICP: public IterativeClosestPoint
    {
	public:
	    virtual ~PCA_ICP();
	    virtual void calcNextStep(Model& source, Model const& dest);
	    void reset();
	private:
	    unsigned int m_index = 0;
    };
}

#endif /* PCA_ICP_H_ */
