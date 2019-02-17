//
// Created by yangt on 18-12-21.
//

#ifndef PATH_SMOOTHING_EIGEN_TYPEDEF_HPP
#define PATH_SMOOTHING_EIGEN_TYPEDEF_HPP

#include <Eigen/Core>
#include <Eigen/Dense>

namespace ncopt {

typedef Eigen::Matrix<double, Eigen::Dynamic, 1> Vector;
typedef Eigen::Matrix<double,
        Eigen::Dynamic,
        Eigen::Dynamic,
        Eigen::RowMajor> Matrix;
typedef Eigen::Map<Vector> VectorRef;
typedef Eigen::Map<Matrix> MatrixRef;
typedef Eigen::Map<const Vector> ConstVectorRef;
typedef Eigen::Map<const Matrix> ConstMatrixRef;

// Column major matrices for DenseSparseMatrix/DenseQRSolver
typedef Eigen::Matrix<double,
        Eigen::Dynamic,
        Eigen::Dynamic,
        Eigen::ColMajor> ColMajorMatrix;

typedef Eigen::Map<ColMajorMatrix, 0,
        Eigen::Stride<Eigen::Dynamic, 1>> ColMajorMatrixRef;

typedef Eigen::Map<const ColMajorMatrix,
        0,
        Eigen::Stride<Eigen::Dynamic, 1>> ConstColMajorMatrixRef;

}
#endif //PATH_SMOOTHING_EIGEN_TYPEDEF_HPP
