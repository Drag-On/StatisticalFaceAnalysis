//////////////////////////////////////////////////////////////////////
/// Statistical Shape Analysis
///
/// Copyright (c) 2014 by Jan Moeller
///
/// This software is provided "as-is" and does not claim to be
/// complete or free of bugs in any way. It should work, but
/// it might also begin to hurt your kittens.
//////////////////////////////////////////////////////////////////////

#ifndef NNVECTOR3_H_
#define NNVECTOR3_H_

namespace sfa
{
    /**
     * @brief Marker interface for vectors. This is not a vector implementation
     * 	      but merely defines methods needed by the nearest neighbor search.
     * 	      They have to be implemented in a derived class.
     */
    class NNVector3
    {
	public:
	    virtual ~NNVector3();
	    virtual double& x() = 0;
	    virtual double& y() = 0;
	    virtual double& z() = 0;
	    virtual double const& x() const = 0;
	    virtual double const& y() const = 0;
	    virtual double const& z() const = 0;
	    /**
	     * @brief Computes the square distance between two vectors interpreting
	     * 	      both as coordinate vectors
	     * @param other Other vector
	     * @return The square distance between this and other
	     */
	    virtual double getSquaredDistance(NNVector3 const& other) const = 0;
    };
}

#endif /* NNVECTOR3_H_ */
