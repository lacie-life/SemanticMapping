//
// Created by lacie-life on 09/10/2021.
//

#ifndef SEMATICSLAM_UTILITIES_H
#define SEMATICSLAM_UTILITIES_H

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Pose3.h>

#include <vector>

namespace semanticSLAM {
    namespace utils {
        /**
         * Returns the real roots of the polynomial
         * If disc > 0: 2 solutions
         * If disc == 0: 1 real solution
         * If disc < 0: 2 imaginary solutions
         */
        gtsam::Vector2 solvePolynomial(const double &a, const double &b,
                                       const double &c);

        gtsam::Vector2 getConicPointsAtX(
                const Eigen::Matrix<long double, 3, 3> &pointConic, const double &x);

        gtsam::Vector2 getConicPointsAtY(
                const Eigen::Matrix<long double, 3, 3> &pointConic, const double &y);

        /** Interpolate poses */
        gtsam::Pose3 interpolate(const gtsam::Pose3 &p1, const gtsam::Pose3 &p2,
                                 const double &percent);

        /**
         * Converts Pose3 to Matrix and provides optional jacobians
         * https://atmos.washington.edu/~dennis/MatrixCalculus.pdf
         * https://en.wikipedia.org/wiki/Kronecker_product
         * https://www.math.uwaterloo.ca/~hwolkowi/matrixcookbook.pdf
         * https://people.maths.ox.ac.uk/gilesm/files/NA-08-01.pdf
         * Some Theorems on Matrix Differentiation with Special Reference to Kronecker
         * Matrix Products (H. Neudecker, 1969) A tutorial on SE(3) transformation
         * parameterizations and on-manifold optimization Jose-Luis Blanco (p.35)
         * -
         * https://jinyongjeong.github.io/Download/SE3/jlblanco2010geometry3d_techrep.pdf
         * http://www.ee.ic.ac.uk/hp/staff/dmb/matrix/special.html#VecTranspose
         */
        gtsam::Matrix44 matrix(const gtsam::Pose3 &pose,
                               gtsam::OptionalJacobian<16, 6> H = boost::none);

        /**
         * Performs the kronecker product
         * See: https://en.wikipedia.org/wiki/Kronecker_product
         */
        gtsam::Matrix kron(const gtsam::Matrix m1, const gtsam::Matrix m2);

        /**
         * Builds the orthogonal transpose vectorization matrix of an m by n matrix
         * See: http://www.ee.ic.ac.uk/hp/staff/dmb/matrix/special.html#VecTranspose
         */
        gtsam::Matrix TVEC(const int m, const int n);
    }
}

#endif //SEMATICSLAM_UTILITIES_H
