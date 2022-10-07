//
// Created by lacie-life on 10/10/2021.
//

#include <gtsam/base/numericalDerivative.h>
#include <boost/function.hpp>
#include "sematicSLAM/core/QuadricAngleFactor.h"

#define NUMERICAL_DERIVATIVE false

using namespace std;

namespace semanticSLAM {

    /* ************************************************************************* */
    gtsam::Vector QuadricAngleFactor::evaluateError(
            const ConstrainedDualQuadric& quadric,
            boost::optional<gtsam::Matrix&> H) const {
        // evaluate error
        gtsam::Rot3 QRot = quadric.pose().rotation();
        gtsam::Vector3 error = measured_.localCoordinates(QRot);
        // Rot3::LocalCoordinates(quadric.pose().rotation());

        boost::function<gtsam::Vector(const ConstrainedDualQuadric&)> funPtr(
        boost::bind(&QuadricAngleFactor::evaluateError, this, _1, boost::none));
        if (H) {
            Eigen::Matrix<double, 3, 9> de_dr =
                    gtsam::numericalDerivative11(funPtr, quadric, 1e-6);
            *H = de_dr;
        }
        return error;
    }

    /* ************************************************************************* */
    void QuadricAngleFactor::print(const std::string& s,
                                   const gtsam::KeyFormatter& keyFormatter) const {
        cout << s << "QuadricAngleFactor(" << keyFormatter(key()) << ")" << endl;
        cout << "    NoiseModel: ";
        noiseModel()->print();
        cout << endl;
    }

    /* ************************************************************************* */
    bool QuadricAngleFactor::equals(const QuadricAngleFactor& other,
                                    double tol) const {
        bool equal =
                noiseModel()->equals(*other.noiseModel(), tol) && key() == other.key();
        return equal;
    }
}
