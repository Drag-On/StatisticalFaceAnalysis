//////////////////////////////////////////////////////////////////////
/// Statistical Shape Analysis
///
/// Copyright (c) 2014 by Jan Moeller
///
/// This software is provided "as-is" and does not claim to be
/// complete or free of bugs in any way. It should work, but
/// it might also begin to hurt your kittens.
//////////////////////////////////////////////////////////////////////

#ifndef ABSTRACTPOINTSELECTOR_H_
#define ABSTRACTPOINTSELECTOR_H_

#include <vector>
#include "Vertex.h"
#include "AbstractMesh.h"

namespace sfa
{
    /**
     * @brief Abstract interface class for point selection algorithms
     */
    class AbstractPointSelector
    {
	public:
	    /**
	     * @brief Parameters to consider for point selection algorithm
	     */
	    enum SelectionFlags
	    {
		NO_EDGES = 1 << 0,    //!< NO_EDGES
	    };

	    /**
	     * @brief Destructor
	     */
	    virtual ~AbstractPointSelector();
	    /**
	     * @brief Selects \p percentage % of points from \p source considering \p flags
	     * @param source Source model to select from
	     * @param percentage Percentage of points to select in range [0, 1]
	     * @param flags Flags to consider
	     * @return List of selected vertices
	     */
	    virtual std::vector<Vertex> select(AbstractMesh& source, float percentage = 1.0f, int flags = 0) = 0;
    };
}

#endif /* ABSTRACTPOINTSELECTOR_H_ */
