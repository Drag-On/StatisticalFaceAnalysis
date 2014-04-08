//////////////////////////////////////////////////////////////////////
/// Statistical Shape Analysis
///
/// Copyright (c) 2014 by Jan Moeller
///
/// This software is provided "as-is" and does not claim to be
/// complete or free of bugs in any way. It should work, but
/// it might also begin to hurt your kittens.
//////////////////////////////////////////////////////////////////////

#ifndef VEC3_H_
#define VEC3_H_

#include <Eigen/Core>
#include "NearestNeighbor/NNVector3.h"

namespace sfa
{
    /**
     * @brief This is the vector implementation used for calculations.
     * 	      Internally it uses the vector implementation of "Eigen"
     */
    class Vec3: public Eigen::Vector3d, public NNVector3
    {
	public:
	    Vec3(double x, double y, double z);
	    virtual ~Vec3();
	    virtual double& x();
	    virtual double& y();
	    virtual double& z();
	    virtual double const& x() const;
	    virtual double const& y() const;
	    virtual double const& z() const;
	    virtual double getSquaredDistance(NNVector3 const& other) const;
	private:
    };
}

#endif /* VEC3_H_ */
