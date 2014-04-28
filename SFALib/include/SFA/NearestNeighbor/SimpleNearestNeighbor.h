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

#include <map>
#include <limits>
#include "NearestNeighbor.h"

namespace sfa
{
    /**
     * @brief This is a very simple nearest neighbor search. It just
     * 	      iterates over all vertices to find the closest one.
     */
    class SimpleNearestNeighbor: public NearestNeighbor
    {
	public:
	    virtual unsigned int getNearest(unsigned int n, AbstractMesh const& source,
		    AbstractMesh const& dest);
	    virtual void clearCache();
	private:
	    std::map<unsigned int, unsigned int> m_cache;
    };
}

#endif /* SIMPLENEARESTNEIGHBOR_H_ */
