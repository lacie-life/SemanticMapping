//
// Created by lacie-life on 09/10/2021.
//

#ifndef SEMATICSLAM_ALIGNEDBOX3_H
#define SEMATICSLAM_ALIGNEDBOX3_H

#include <gtsam/geometry/Pose3.h>

#include <vector>

namespace semanticSLAM {
    /**
     * @class AlignedBox3
     * An axis aligned 3D bounding box
     * (xmin, xmax, ymin, ymax, zmin, zmax)
     */
    class AlignedBox3 {
    protected:
        gtsam::Vector6 xxyyzz_;  ///< bounds vector

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /// @name Constructors and named constructors
        /// @{

        /** Default constructor */
        AlignedBox3() { xxyyzz_.setZero(); };

        /** Constructor from doubles */
        AlignedBox3(const double &xmin, const double &xmax, const double &ymin,
                    const double &ymax, const double &zmin, const double &zmax);

        /** Constructor from vector */
        AlignedBox3(const gtsam::Vector6 &xxyyzz) : xxyyzz_(xxyyzz) {};

        /// @}
        /// @name Class accessors
        /// @{

        /** Get xmin */
        double xmin() const { return xxyyzz_[0]; }

        /** Get xmax */
        double xmax() const { return xxyyzz_[1]; }

        /** Get ymin */
        double ymin() const { return xxyyzz_[2]; }

        /** Get ymax */
        double ymax() const { return xxyyzz_[3]; }

        /** Get zmin */
        double zmin() const { return xxyyzz_[4]; }

        /** Get zmax */
        double zmax() const { return xxyyzz_[5]; }

        /** Returns box in xxyyzz vector */
        gtsam::Vector6 vector() const { return xxyyzz_; };

        /// @}
        /// @name Class methods
        /// @{

        /** Returns x,y,z lengths as a vector */
        gtsam::Vector3 dimensions() const;

        /** Returns box centroid as x,y,z vector */
        gtsam::Vector3 centroid() const;

        /** calculates volume, assuming ordered correctly */
        double volume() const {
            return (xmax() - xmin()) * (ymax() - ymin()) * (zmax() - zmin());
        }

        /**
         * Calculates the standard intersection over union
         * between two axis aligned bounding boxes.
         */
        double iou(const AlignedBox3 &other) const;

        /// @}
        /// @name Testable group traits
        /// @{

        /** Prints the box vector with optional string */
        void print(const std::string &s = "") const;

        /** Compares two boxes */
        bool equals(const AlignedBox3 &other, double tol = 1e-9) const;

        /// @}
    };
}

/** \cond PRIVATE */
// Add to testable group
template <>
struct gtsam::traits<semanticSLAM::AlignedBox3>
        : public gtsam::Testable<semanticSLAM::AlignedBox3> {};
/** \endcond */

#endif //SEMATICSLAM_ALIGNEDBOX3_H
