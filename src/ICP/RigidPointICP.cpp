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
//	std::cout << "srcAvg:" << std::endl << srcAvg << std::endl;
	auto destAvg = dest.getAverage();
//	std::cout << "destAvg:" << std::endl << destAvg << std::endl;
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
//	std::cout << "X:" << std::endl << X << std::endl;
//	std::cout << "Y:" << std::endl << Y << std::endl;
	// Calculate optimal rotation
//	LOG->debug("Compute XYT");
	auto XYT = X * Y.transpose();
//	std::cout << "XYT:" << std::endl << XYT << std::endl;
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(XYT, Eigen::ComputeThinU | Eigen::ComputeThinV);
//	LOG->debug("Compute R");
	auto R = svd.matrixV() * svd.matrixU().transpose();
//	std::cout << "R:" << std::endl << R << std::endl;
	// Translation
//	LOG->debug("Compute t");
	auto t = destAvg - R * srcAvg;
//	std::cout << "t:" << std::endl << t << std::endl;
	// Apply values to all vertices of source
	for (unsigned int i = 0; i < source.getAmountOfVertices(); i++)
	{
	    auto vertex = source.getVertex(i);
	    vertex.coords = R * vertex.coords + t;
	    source.setVertex(i, vertex);
	}
    }
}

