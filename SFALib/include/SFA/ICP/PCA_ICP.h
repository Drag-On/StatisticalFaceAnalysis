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
#include "ICP.h"
#include "SFA/NearestNeighbor/NearestNeighbor.h"
#include "SFA/Utility/AbstractMesh.h"

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
    class PCA_ICP: public ICP
    {
	public:
	    PCA_ICP();
	    virtual ~PCA_ICP();
	    /**
	     * @brief Aligns one principal component after the other
	     * @param source Source mesh
	     * @param dest Destination mesh
	     */
	    virtual unsigned int calcNextStep(AbstractMesh& source, AbstractMesh const& dest);
	    /**
	     * @brief Resets the the amount of already aligned dimensions to zero
	     */
	    void reset();
	private:
	    /**
	     * @brief Amount of dimensions already aligned
	     */
	    unsigned int m_index = 0;
    };
}

#endif /* PCA_ICP_H_ */
