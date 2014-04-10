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
    RigidPointICP::~RigidPointICP()
    {
    }

    void RigidPointICP::calcNextStep(Model& source, Model const& dest)
    {
	auto srcAvg = source.getAverage();
	auto destAvg = dest.getAverage();
	assert(source.getAmountOfVertices() == dest.getAmountOfVertices());
	Eigen::MatrixXd X(3, source.getAmountOfVertices());
	Eigen::MatrixXd Y(3, dest.getAmountOfVertices());
	// Fill X and Y
	for(unsigned int i = 0; i < source.getAmountOfVertices(); i++)
	{
	    auto vertex = source.getVertex(i).coords;
	    auto corVertex = vertex - srcAvg;
	    X(0,i) = corVertex.x();
	    X(1,i) = corVertex.y();
	    X(2,i) = corVertex.z();
	}
	for(unsigned int i = 0; i < dest.getAmountOfVertices(); i++)
	{
	    auto vertex = dest.getVertex(i).coords;
	    auto corVertex = vertex - destAvg;
	    Y(0,i) = corVertex.x();
	    Y(1,i) = corVertex.y();
	    Y(2,i) = corVertex.z();
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
	    vertex.coords = R * vertex.coords + t;
	    // Should be okay to use the same R for normal since the inverse of a rotation matrix
	    // is its transpose. Thus the correct matrix is transpose(transpose(R)) = R.
	    vertex.normal = R * vertex.normal;
	    source.setVertex(i, vertex);
	}
    }
}

