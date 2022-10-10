//
// Created by lacie-life on 09/10/2021.
//

#ifndef SEMATICSLAM_QUADRICPROJECTIONEXCEPTION_H
#define SEMATICSLAM_QUADRICPROJECTIONEXCEPTION_H

#include <gtsam/base/ThreadsafeException.h>
#include <gtsam/inference/Key.h>

#include <string>

namespace semanticSLAM{

    /**
    * @class QuadricProjectionException
    * Exception thrown when attemption to calculate quadric bounding box fails
    */
    class QuadricProjectionException
        : public gtsam::ThreadsafeException<QuadricProjectionException> {
            public:
            QuadricProjectionException()
            : QuadricProjectionException(std::numeric_limits<gtsam::Key>::max()) {}

            QuadricProjectionException(gtsam::Key j)
            : ThreadsafeException<QuadricProjectionException>(
                    "QuadricProjectionException"),
                    j_(j) {}

            QuadricProjectionException(const std::string& description)
            : ThreadsafeException<QuadricProjectionException>(description) {}

            gtsam::Key nearbyVariable() const { return j_; }

            private:
            gtsam::Key j_;
    };
}
#endif //SEMATICSLAM_QUADRICPROJECTIONEXCEPTION_H
