//////////////////////////////////////////////////////////////////////
/// Statistical Shape Analysis
///
/// Copyright (c) 2014 by Jan Moeller
///
/// This software is provided "as-is" and does not claim to be
/// complete or free of bugs in any way. It should work, but
/// it might also begin to hurt your kittens.
//////////////////////////////////////////////////////////////////////

#ifndef KDTREENEARESTNEIGHBOR_H_
#define KDTREENEARESTNEIGHBOR_H_

#include "SFA/NearestNeighbor/NearestNeighbor.h"
#include "SFA/Utility/Model.h"

namespace sfa
{
    /**
     * @brief This nearest neighbor search uses a k-d tree for acceleration
     */
    class KdTreeNearestNeighbor : public NearestNeighbor
    {
	public:
	    virtual unsigned int getNearest(unsigned int n, AbstractMesh const& source, AbstractMesh const& dest);
	    virtual void clearCache();
	private:
    };
}



#endif /* KDTREENEARESTNEIGHBOR_H_ */
