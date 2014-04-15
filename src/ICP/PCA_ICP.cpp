//////////////////////////////////////////////////////////////////////
/// Statistical Face Analysis
///
/// Copyright (c) 2014 by Jan Moeller
///
/// This software is provided "as-is" and does not claim to be
/// complete or free of bugs in any way. It should work, but
/// it might also begin to hurt your kittens.
//////////////////////////////////////////////////////////////////////

#include "ICP/PCA_ICP.h"

namespace sfa
{
    PCA_ICP::~PCA_ICP()
    {
    }

    void PCA_ICP::calcNextStep(Model& source, Model const& dest)
    {
	// Only 3 principal components... but after matching 2 of them the third matches automatically
	if(m_index >= 2)
	    return;

	auto srcAvg = source.getAverage();
	auto destAvg = dest.getAverage();
	auto amountOfPoints = source.getAmountOfVertices();

	// Align next principal component
	// Calculate X & Y
	Eigen::MatrixXd X(3, amountOfPoints);
	Eigen::MatrixXd Y(3, amountOfPoints);
	for (unsigned int i = 0; i < amountOfPoints; i++)
	{
	    auto srcVertex = source.getVertex(i).coords;
	    auto corSrcVertex = srcVertex - srcAvg;
	    X(0, i) = corSrcVertex.x();
	    X(1, i) = corSrcVertex.y();
	    X(2, i) = corSrcVertex.z();
	    auto destVertex = dest.getVertex(i).coords;
	    auto corDestVertex = destVertex - destAvg;
	    Y(0, i) = corDestVertex.x();
	    Y(1, i) = corDestVertex.y();
	    Y(2, i) = corDestVertex.z();
	}
	// Compute eigenvectors and -values
	Eigen::EigenSolver<Eigen::MatrixXd> eigenX(X * X.transpose(), true);
	Eigen::EigenSolver<Eigen::MatrixXd> eigenY(Y * Y.transpose(), true);

	// Sort by eigenvalue
	struct EigenValVec
	{
	    public:
		Eigen::Vector3d vector;
		double value;
	};
	std::vector<EigenValVec> srcVectors, destVectors;
	for (unsigned int i = 0; i < 3; i++)
	{
	    Eigen::Vector3d curSrcVec = eigenX.pseudoEigenvectors().col(i);
	    double curSrcVal = eigenX.pseudoEigenvalueMatrix()(i, i);
	    srcVectors.push_back( {
	    curSrcVec, curSrcVal
	    });
	    Eigen::Vector3d curDestVec = eigenY.pseudoEigenvectors().col(i);
	    double curDestVal = eigenY.pseudoEigenvalueMatrix()(i, i);
	    destVectors.push_back( {
	    curDestVec, curDestVal
	    });
	}
	struct SortByValue
	{
		bool operator()(EigenValVec i, EigenValVec j)
		{
		    return (i.value > j.value);
		}
	} comparator;
	std::sort(srcVectors.begin(), srcVectors.end(), comparator);
	std::sort(destVectors.begin(), destVectors.end(), comparator);

	// Rotate source such that its chosen principal component matches the one of destination.
	// Since we can't be sure that both coordinate systems have the same handedness we have to
	// choose the minimal rotation, either the one onto the destination vector or the one onto
	// the negative destination vector
	Eigen::Quaterniond R1 = Eigen::Quaterniond::FromTwoVectors(srcVectors[m_index].vector,
		destVectors[m_index].vector);
	Eigen::Quaterniond R2 = Eigen::Quaterniond::FromTwoVectors(-srcVectors[m_index].vector,
		destVectors[m_index].vector);
	auto R = R1.norm() < R2.norm() ? R1 : R2;
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

	// Increment index
	m_index++;
    }

}
