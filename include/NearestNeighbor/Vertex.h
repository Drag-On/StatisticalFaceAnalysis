//////////////////////////////////////////////////////////////////////
/// Statistical Shape Analysis
///
/// Copyright (c) 2014 by Jan Moeller
///
/// This software is provided "as-is" and does not claim to be
/// complete or free of bugs in any way. It should work, but
/// it might also begin to hurt your kittens.
//////////////////////////////////////////////////////////////////////

#ifndef VERTEX_H_
#define VERTEX_H_

#include <Eigen/Core>

namespace sfa
{
    /**
     * @brief Contains all the data of one vertex
     */
    struct Vertex
    {
	public:
	    unsigned int id;		// Index
	    Eigen::Vector3d coords;	// Coordinates
	    Eigen::Vector3d normal;	// Normal
    };
}



#endif /* VERTEX_H_ */
