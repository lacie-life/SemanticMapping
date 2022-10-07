//
// Created by lacie-life on 10/10/2021.
//

#ifndef SEMATICSLAM_QUADRICANGLEFACTOR_H
#define SEMATICSLAM_QUADRICANGLEFACTOR_H

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include "core/AlignedBox2.h"
#include "core/ConstrainedDualQuadric.h"

namespace semanticSLAM{
    /**
     * @class QuadricAngleFactor
     * AlignedBox3 factor between Pose3 and ConstrainedDualQuadric
     * Projects the quadric at the current pose estimates,
     * Calculates the bounds of the dual conic,
     * and compares this to the measured bounding box.
     */
    class QuadricAngleFactor
            : public gtsam::NoiseModelFactor1<ConstrainedDualQuadric> {
    protected:
        typedef NoiseModelFactor1<ConstrainedDualQuadric>
                Base;  ///< base class has keys and noisemodel as private members
        gtsam::Rot3 measured_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /// @name Constructors and named constructors
        /// @{

        /** Constructor from measured box, calbration, dimensions and posekey,
         * quadrickey, noisemodel */
        QuadricAngleFactor(const gtsam::Key& quadricKey, const gtsam::Rot3 measured,
                           const gtsam::SharedNoiseModel& model)
                : Base(model, quadricKey), measured_(measured){};

        /// @}
        /// @name Class methods
        /// @{

        /**
         * Evaluate the error between a quadric and 3D pose
         * @param pose the 6DOF camera position
         * @param quadric the constrained dual quadric
         * @param H1 the derivative of the error wrt camera pose (4x6)
         * @param H2 the derivative of the error wrt quadric (4x9)
         */
        gtsam::Vector evaluateError(
                const ConstrainedDualQuadric& quadric,
                boost::optional<gtsam::Matrix&> H = boost::none) const;

        /// @}
        /// @name Testable group traits
        /// @{

        /** Prints the boundingbox factor with optional string */
        void print(const std::string& s = "",
                   const gtsam::KeyFormatter& keyFormatter =
                   gtsam::DefaultKeyFormatter) const override;

        /** Returns true if equal keys, measurement, noisemodel and calibration */
        bool equals(const QuadricAngleFactor& other, double tol = 1e-9) const;
    };
}

/** \cond PRIVATE */
// Add to testable group
template <>
struct gtsam::traits<semanticSLAM::QuadricAngleFactor>
        : public gtsam::Testable<semanticSLAM::QuadricAngleFactor> {};
/** \endcond */

#endif //SEMATICSLAM_QUADRICANGLEFACTOR_H
