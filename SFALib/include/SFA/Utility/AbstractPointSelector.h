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
    class AbstractPointSelector
    {
	public:
	    virtual ~AbstractPointSelector();
	    virtual std::vector<Vertex> select(AbstractMesh& source) = 0;
    };
}

#endif /* ABSTRACTPOINTSELECTOR_H_ */
