//////////////////////////////////////////////////////////////////////
/// Statistical Face Analysis
///
/// Copyright (c) 2014 by Jan Moeller
///
/// This software is provided "as-is" and does not claim to be
/// complete or free of bugs in any way. It should work, but
/// it might also begin to hurt your kittens.
//////////////////////////////////////////////////////////////////////

#include "NearestNeighbor/NNVector3.h"

namespace sfa
{
    NNVector3::NNVector3(double x, double y, double z) : m_x(x), m_y(y), m_z(z)
    {
    }

    double& NNVector3::x()
    {
	return m_x;
    }

    double& NNVector3::y()
    {
	return m_y;
    }

    double& NNVector3::z()
    {
	return m_z;
    }

    double const& NNVector3::x() const
    {
	return m_x;
    }

    double const& NNVector3::y() const
    {
	return m_y;
    }

    double const& NNVector3::z() const
    {
	return m_z;
    }

    double NNVector3::getSquaredDistance(NNVector3 const& other) const
    {
	double a = this->x() - other.x();
	double b = this->y() - other.y();
	double c = this->z() - other.z();
	return a * a + b * b + c * c;
    }
}
