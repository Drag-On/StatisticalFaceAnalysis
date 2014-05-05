//////////////////////////////////////////////////////////////////////
/// Statistical Face Analysis
///
/// Copyright (c) 2014 by Jan Moeller
///
/// This software is provided "as-is" and does not claim to be
/// complete or free of bugs in any way. It should work, but
/// it might also begin to hurt your kittens.
//////////////////////////////////////////////////////////////////////

#include "SFA/ICP/RigidPointICP.h"

namespace sfa
{
    RigidPointICP::RigidPointICP(NearestNeighbor& nn, AbstractLog* pLog) : ICP(pLog), m_nearestNeighbor(nn)
    {
    }

    RigidPointICP::~RigidPointICP()
    {
    }

    unsigned int RigidPointICP::calcNextStep(AbstractMesh& source, AbstractMesh const& dest)
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
	// Calc averages
	auto srcAvg = getAverage(sourcePoints);
	auto destAvg = getAverage(nearestPoints);
	auto amountOfPoints = sourcePoints.size();
	Eigen::MatrixXd X(3, amountOfPoints);
	Eigen::MatrixXd Y(3, amountOfPoints);
	// Fill X and Y
	for(unsigned int i = 0; i < amountOfPoints; i++)
	{
	    auto srcVertex = sourcePoints[i].coords;
	    auto corSrcVertex = srcVertex - srcAvg;
	    X(0,i) = corSrcVertex.x();
	    X(1,i) = corSrcVertex.y();
	    X(2,i) = corSrcVertex.z();
	    auto destVertex = nearestPoints[i].coords;
	    auto corDestVertex = destVertex - destAvg;
	    Y(0, i) = corDestVertex.x();
	    Y(1, i) = corDestVertex.y();
	    Y(2, i) = corDestVertex.z();
	}
	// Calculate optimal rotation
	auto XYT = X * Y.transpose();
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(XYT, Eigen::ComputeThinU | Eigen::ComputeThinV);
	auto R = svd.matrixV() * svd.matrixU().transpose();
	// Translation
	auto t = destAvg - R * srcAvg;
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

