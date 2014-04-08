//////////////////////////////////////////////////////////////////////
/// Statistical Shape Analysis
///
/// Copyright (c) 2014 by Jan Moeller
///
/// This software is provided "as-is" and does not claim to be
/// complete or free of bugs in any way. It should work, but
/// it might also begin to hurt your kittens.
//////////////////////////////////////////////////////////////////////

#ifndef NNVERTEX_H_
#define NNVERTEX_H_

#include"NNVector3.h"

namespace sfa
{
    /**
     * @brief Contains all the data of one vertex
     */
    struct NNVertex
    {
	public:
	    unsigned int id;		// Index
	    NNVector3 coords;		// Coordinates
	    NNVector3 normal;		// Normal
    };
}



#endif /* NNVERTEX_H_ */
