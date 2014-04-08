//////////////////////////////////////////////////////////////////////
/// Statistical Face Analysis
///
/// Copyright (c) 2014 by Jan Moeller
///
/// This software is provided "as-is" and does not claim to be
/// complete or free of bugs in any way. It should work, but
/// it might also begin to hurt your kittens.
//////////////////////////////////////////////////////////////////////

#include "SFA/Vec3.h"

namespace sfa
{
    Vec3::Vec3(double x, double y, double z) : Eigen::Vector3d(x, y, z)
    {
    }

    Vec3::~Vec3()
    {
    }

    double& Vec3::x()
    {
	return (*this)[0];
    }

    double& Vec3::y()
    {
	return (*this)[1];
    }

    double& Vec3::z()
    {
	return (*this)[2];
    }

    double const& Vec3::x() const
    {
	return (*this)[0];
    }

    double const& Vec3::y() const
    {
	return (*this)[1];
    }

    double const& Vec3::z() const
    {
	return (*this)[2];
    }

    double Vec3::getSquaredDistance(NNVector3 const& other) const
    {
	auto distVec = (*this) - Vec3(other.x(), other.y(), other.z());
	return distVec.squaredNorm();
    }
}

