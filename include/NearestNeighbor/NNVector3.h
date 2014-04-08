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
	    NNVector3(double x, double y, double z);
	    double& x();
	    double& y();
	    double& z();
	    double const& x() const;
	    double const& y() const;
	    double const& z() const;
	    /**
	     * @brief Computes the square distance between two vectors interpreting
	     * 	      both as coordinate vectors
	     * @param other Other vector
	     * @return The square distance between this and other
	     */
	    double getSquaredDistance(NNVector3 const& other) const;

	private:
	    double m_x, m_y, m_z;
    };
}

#endif /* NNVECTOR3_H_ */
