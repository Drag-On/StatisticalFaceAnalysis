//////////////////////////////////////////////////////////////////////
/// Statistical Face Analysis
///
/// Copyright (c) 2014 by Jan Moeller
///
/// This software is provided "as-is" and does not claim to be
/// complete or free of bugs in any way. It should work, but
/// it might also begin to hurt your kittens.
//////////////////////////////////////////////////////////////////////

#include "SFA/ICP/PCA_ICP.h"

namespace sfa
{
    PCA_ICP::~PCA_ICP()
    {
    }

    void PCA_ICP::calcNextStep(AbstractMesh& source, AbstractMesh const& dest)
    {
	// Only 3 principal components...
	if(m_index >= 3)
	    return;

	auto srcAvg = source.getAverage();
	auto destAvg = dest.getAverage();
	auto amountOfPointsSource = source.getAmountOfVertices();
	auto amountOfPointsDest = dest.getAmountOfVertices();

	// Align next principal component
	// Calculate X & Y
	Eigen::MatrixXd X(3, amountOfPointsSource);
	Eigen::MatrixXd Y(3, amountOfPointsDest);
	for (unsigned int i = 0; i < amountOfPointsSource; i++)
	{
	    auto srcVertex = source.getVertex(i).coords;
	    auto corSrcVertex = srcVertex - srcAvg;
	    X(0, i) = corSrcVertex.x();
	    X(1, i) = corSrcVertex.y();
	    X(2, i) = corSrcVertex.z();
	}
	for (unsigned int i = 0; i < amountOfPointsDest; i++)
	{
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
	auto srcEigenVec = srcVectors[m_index].vector.normalized();
	auto destEigenVec = destVectors[m_index].vector.normalized();
	Eigen::Vector3d vec;
	auto theta1 = srcEigenVec.dot(destEigenVec);
	if (theta1 < -1.0)
	    theta1 = -1.0f;
	else if (theta1 > 1.0)
	    theta1 = 1.0f;
	auto theta2 = (-srcEigenVec).dot(destEigenVec);
	if (theta2 < -1.0)
	    theta2 = -1.0f;
	else if (theta2 > 1.0)
	    theta2 = 1.0f;
	auto angle1 = std::acos(theta1);
	auto angle2 = std::acos(theta2);
	if(angle1 < angle2)
	    vec = srcEigenVec;
	else
	    vec = -srcEigenVec;
	Eigen::Quaterniond R = Eigen::Quaterniond::FromTwoVectors(vec, destEigenVec);
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

	// Increment index
	m_index++;
    }

    void PCA_ICP::reset()
    {
	m_index = 0;
    }

}
