//////////////////////////////////////////////////////////////////////
/// Statistical Face Analysis
///
/// Copyright (c) 2014 by Jan Moeller
///
/// This software is provided "as-is" and does not claim to be
/// complete or free of bugs in any way. It should work, but
/// it might also begin to hurt your kittens.
//////////////////////////////////////////////////////////////////////

#include <iostream>
#include <System/Log/Log.h>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

int main(int argc, char** argv)
{
    LOG->setLogLevel(dbgl::DBG);
    LOG->info("Starting...");

    MatrixXf m = MatrixXf::Random(3,2);
    cout << "Here is the matrix m:" << endl << m << endl;
    JacobiSVD<MatrixXf> svd(m, ComputeThinU | ComputeThinV);
    cout << "Its singular values are:" << endl << svd.singularValues() << endl;
    cout << "Its left singular vectors are the columns of the thin U matrix:" << endl << svd.matrixU() << endl;
    cout << "Its right singular vectors are the columns of the thin V matrix:" << endl << svd.matrixV() << endl;
    Vector3f rhs(1, 0, 0);
    cout << "Now consider this rhs vector:" << endl << rhs << endl;
    cout << "A least-squares solution of m*x = rhs is:" << endl << svd.solve(rhs) << endl;

    LOG->info("That's it!");
    return 0;
}

