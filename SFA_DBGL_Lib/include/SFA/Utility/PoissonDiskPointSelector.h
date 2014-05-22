//////////////////////////////////////////////////////////////////////
/// Statistical Shape Analysis
///
/// Copyright (c) 2014 by Jan Moeller
///
/// This software is provided "as-is" and does not claim to be
/// complete or free of bugs in any way. It should work, but
/// it might also begin to hurt your kittens.
//////////////////////////////////////////////////////////////////////

#ifndef POISSONDISKPOINTSELECTOR_H_
#define POISSONDISKPOINTSELECTOR_H_

#include "Model.h"
#include "PoissonDiskSampler.h"
#include "SFA/Utility/AbstractPointSelector.h"

namespace sfa
{
    class PoissonDiskPointSelector: public AbstractPointSelector
    {
	public:
	    virtual std::vector<Vertex> select(AbstractMesh& source);
	private:
	    PoissonDiskSampler m_sampler;
    };
}

#endif /* POISSONDISKPOINTSELECTOR_H_ */
