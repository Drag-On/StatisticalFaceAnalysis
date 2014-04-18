//////////////////////////////////////////////////////////////////////
/// Statistical Face Analysis
///
/// Copyright (c) 2014 by Jan Moeller
///
/// This software is provided "as-is" and does not claim to be
/// complete or free of bugs in any way. It should work, but
/// it might also begin to hurt your kittens.
//////////////////////////////////////////////////////////////////////

#include "ICP/RigidPointICP.h"

namespace sfa
{
    RigidPointICP::RigidPointICP(NearestNeighbor& nn) : m_nearestNeighbor(nn)
    {
    }

    RigidPointICP::~RigidPointICP()
    {
    }

    void RigidPointICP::calcNextStep(Model& source, Model const& dest)
    {
	auto srcAvg = source.getAverage();
	auto destAvg = dest.getAverage();
	auto amountOfPoints = source.getAmountOfVertices();
	Eigen::MatrixXd X(3, amountOfPoints);
	Eigen::MatrixXd Y(3, amountOfPoints);
	// Fill X and Y
	for(unsigned int i = 0; i < amountOfPoints; i++)
	{
	    auto srcVertex = source.getVertex(i).coords;
	    auto corSrcVertex = srcVertex - srcAvg;
	    X(0,i) = corSrcVertex.x();
	    X(1,i) = corSrcVertex.y();
	    X(2,i) = corSrcVertex.z();
	    unsigned int nearestIndex = m_nearestNeighbor.getNearest(i, source, dest);
	    auto destVertex = dest.getVertex(nearestIndex).coords;
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
    }
}

