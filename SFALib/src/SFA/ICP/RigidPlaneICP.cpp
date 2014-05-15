//////////////////////////////////////////////////////////////////////
/// Statistical Face Analysis
///
/// Copyright (c) 2014 by Jan Moeller
///
/// This software is provided "as-is" and does not claim to be
/// complete or free of bugs in any way. It should work, but
/// it might also begin to hurt your kittens.
//////////////////////////////////////////////////////////////////////

#include "SFA/ICP/RigidPlaneICP.h"

namespace sfa
{
    RigidPlaneICP::RigidPlaneICP(NearestNeighbor& nn, AbstractLog* pLog) : ICP(pLog), m_nearestNeighbor(nn)
    {
    }

    RigidPlaneICP::~RigidPlaneICP()
    {
    }

    unsigned int RigidPlaneICP::calcNextStep(AbstractMesh& source, AbstractMesh const& dest)
    {
	// Select points
	auto sourcePoints = selectPoints(source);
	auto nearestPoints = m_nearestNeighbor.getAllNearest(sourcePoints, source, dest);
	// Sort out edge points on dest
	if((m_selectionMethod & NO_EDGES))
	{
	    for (unsigned int i = 0; i < nearestPoints.size(); i++)
	    {
		if (nearestPoints[i].isEdge)
		{
		    nearestPoints.erase(nearestPoints.begin() + i);
		    sourcePoints.erase(sourcePoints.begin() + i);
		}
	    }
	}
	// Setup values
	auto amountOfPoints = sourcePoints.size();
	Eigen::MatrixXd A(6, amountOfPoints);
	Eigen::VectorXd b(amountOfPoints);
	for(unsigned int i = 0; i < amountOfPoints; i++)
	{
	    Eigen::Vector3d c = sourcePoints[i].coords.cross(sourcePoints[i].normal);
	    Eigen::Vector3d n = sourcePoints[i].normal;
	    A(0, i) = c[0];
	    A(1, i) = c[1];
	    A(2, i) = c[2];
	    A(3, i) = n[0];
	    A(4, i) = n[1];
	    A(5, i) = n[2];
	    b(i) = n.dot(nearestPoints[i].coords) - n.dot(sourcePoints[i].coords);
	}
	// Calculate values
	Eigen::Vector3d x = A.inverse() * b; // x = (alpha, beta, gamma, tx, ty, tz)
	// Rotation matrix
	Eigen::Matrix3d R;
	R = Eigen::AngleAxis<double>(x[0], Eigen::Vector3d::UnitX())
		* Eigen::AngleAxis<double>(x[1], Eigen::Vector3d::UnitY())
		* Eigen::AngleAxis<double>(x[2], Eigen::Vector3d::UnitZ());
	// Translation vector
	Eigen::Vector3d t(x[3], x[4], x[5]);
	// Apply values to all vertices of source
	for (unsigned int i = 0; i < source.getAmountOfVertices(); i++)
	{
	    auto vertex = source.getVertex(i);
	    auto coords = R * vertex.coords + t;
	    // Should be okay to use the same R for normal since the inverse of a rotation matrix
	    // is its transpose. Thus the correct matrix is transpose(transpose(R)) = R.
	    auto normal = R * vertex.normal;
	    source.setVertex(i, coords, normal);
	}
	// Clear nearest neighbor cache
	m_nearestNeighbor.clearCache();

	return sourcePoints.size();
    }
}
