//////////////////////////////////////////////////////////////////////
/// Statistical Shape Analysis
///
/// Copyright (c) 2014 by Jan Moeller
///
/// This software is provided "as-is" and does not claim to be
/// complete or free of bugs in any way. It should work, but
/// it might also begin to hurt your kittens.
//////////////////////////////////////////////////////////////////////

#ifndef SIMPLENEARESTNEIGHBOR_H_
#define SIMPLENEARESTNEIGHBOR_H_

#include <limits>
#include "NearestNeighbor.h"

namespace sfa
{
    /**
     * @brief This is a very simple nearest neighbor search. It just
     * 	      iterates over all vertices to find the closest one.
     */
    class SimpleNearestNeighbor: NearestNeighbor
    {
	public:
	    virtual unsigned int getNearest(unsigned int n, NNMesh const& source,
		    NNMesh const& dest) const;
    };
}

#endif /* SIMPLENEARESTNEIGHBOR_H_ */
