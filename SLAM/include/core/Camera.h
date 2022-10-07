//
// Created by lacie-life on 09/10/2021.
//

#ifndef SEMATICSLAM_CAMERA_H
#define SEMATICSLAM_CAMERA_H

#include <gtsam/base/ThreadsafeException.h>
#include <gtsam/base/types.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholePose.h>
#include "sematicSLAM/core/ConstrainedDualQuadric.h"
#include "sematicSLAM/core/DualConic.h"

namespace semanticSLAM{
    /**
     * @class QuadricCamera
     * A camera that projects quadrics
     */
    class QuadricCamera {
    public:
        /** Static projection matrix */
        static gtsam::Matrix34 transformToImage(
                const gtsam::Pose3& pose,
                const boost::shared_ptr<gtsam::Cal3_S2>& calibration);

        /**
         * Project a quadric at the stored 3D pose and calibration
         * @param quadric the 3D quadric surface to be projected
         * @return the projected dual conic
         */
        static DualConic project(const ConstrainedDualQuadric& quadric,
                                 const gtsam::Pose3& pose,
                                 const boost::shared_ptr<gtsam::Cal3_S2>& calibration,
                                 gtsam::OptionalJacobian<9, 9> dC_dq = boost::none,
                                 gtsam::OptionalJacobian<9, 6> dC_dx = boost::none);

        /** Project box to planes */
        static std::vector<gtsam::Vector4> project(
                const AlignedBox2& box, const gtsam::Pose3& pose,
                const boost::shared_ptr<gtsam::Cal3_S2>& calibration);
    };
}
#endif //SEMATICSLAM_CAMERA_H
