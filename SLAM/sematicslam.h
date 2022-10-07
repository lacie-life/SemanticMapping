//
// Created by lacie-life on 12/10/2021.
//

#ifndef SEMATICSLAM_SEMATICSLAM_H
#define SEMATICSLAM_SEMATICSLAM_H


virtual class NonlinearFactor;
virtual class NoiseModelFactor : NonlinearFactor;
class NonlinearFactorGraph;
class PriorFactor;
class Cal3_S2;
class Point2;
class Point3;
class Values;
class Pose3;

class ConstrainedDualQuadric;
class AlignedBox2;
class AlignedBox3;
class DualConic;


namespace gtsam {

#include "sematicSLAM/sematicBase/Utilities.h"
namespace utils {
    Pose3 interpolate(const Pose3& p1, const Pose3& p2, const double& percent);
}

#include "sematicSLAM/core/ConstrainedDualQuadric.h"
class ConstrainedDualQuadric {
    ConstrainedDualQuadric();
    ConstrainedDualQuadric(const Matrix& dQ);
    ConstrainedDualQuadric(const Pose3& pose, const Vector& radii);
    ConstrainedDualQuadric(const Rot3& R, const Point3& t, const Vector& r);
    Pose3 pose() const;
    Vector radii() const;
    Point3 centroid() const;
    Matrix matrix() const;
    static ConstrainedDualQuadric constrain(const Matrix& dual_quadric);
    gtsam::AlignedBox3 bounds() const;
    bool isBehind(const Pose3& cameraPose) const;
    bool contains(const Pose3& cameraPose) const;
    static ConstrainedDualQuadric Retract(const Vector& v);
    static Vector LocalCoordinates(const ConstrainedDualQuadric& q);
    gtsam::ConstrainedDualQuadric retract(const Vector& v) const;
    Vector localCoordinates(const ConstrainedDualQuadric& other) const;
    void addToValues(Values &v, const size_t& k);
    static ConstrainedDualQuadric getFromValues(const Values &v, const size_t& k);
    void print(const string& s) const;
    void print() const;
    bool equals(const ConstrainedDualQuadric& other, double tol) const;
    bool equals(const ConstrainedDualQuadric& other) const;
};

#include "sematicSLAM/core/BoundingBoxFactor.h"
virtual class BoundingBoxFactor : NoiseModelFactor {
    BoundingBoxFactor();
    BoundingBoxFactor(const AlignedBox2& measured, const Cal3_S2* calibration,
                      const size_t& poseKey, const size_t& quadricKey, const gtsam::noiseModel::Base* model);
    BoundingBoxFactor(const AlignedBox2& measured, const Cal3_S2* calibration,
                      const size_t& poseKey, const size_t& quadricKey, const gtsam::noiseModel::Base* model, const string& errorString);
    gtsam::AlignedBox2 measurement() const;
    size_t poseKey() const;
    size_t objectKey() const;
    Vector evaluateError(const Pose3& pose, const ConstrainedDualQuadric& quadric) const;
    Matrix evaluateH1(const Pose3& pose, const ConstrainedDualQuadric& quadric) const;
    Matrix evaluateH2(const Pose3& pose, const ConstrainedDualQuadric& quadric) const;
    Matrix evaluateH1(const Values& x) const;
    Matrix evaluateH2(const Values& x) const;
};

#include "sematicSLAM/core/QuadricAngleFactor.h"
virtual class QuadricAngleFactor : NoiseModelFactor {
    QuadricAngleFactor(const size_t& quadricKey, const Rot3& measured, const gtsam::noiseModel::Base* model);
    Vector evaluateError(const ConstrainedDualQuadric& quadric) const;
};

#include <gtsam/nonlinear/PriorFactor.h>
template<T = {gtsam::ConstrainedDualQuadric}>
virtual class PriorFactor : NoiseModelFactor {
    PriorFactor(size_t key, const T& prior, const gtsam::noiseModel::Base* noiseModel);
    T prior() const;

    // enabling serialization functionality
    void serialize() const;
};

#include "sematicSLAM/core/AlignedBox2.h"
class Vector3Vector
{
    Vector3Vector();
    size_t size() const;
    bool empty() const;
    Vector at(size_t n) const;
    void push_back(const Vector& pose);
};

class AlignedBox2 {
    AlignedBox2();
    AlignedBox2(const double& xmin, const double& ymin, const double& xmax, const double& ymax);
    AlignedBox2(const Vector& tlbr);
    AlignedBox2(const AlignedBox2& other);
    double xmin() const;
    double ymin() const;
    double xmax() const;
    double ymax() const;
    Vector vector() const;
    Point2 center() const;
    double width() const;
    double height() const;
    bool contains(const Point2& point) const;
    bool contains(const AlignedBox2& other) const;
    bool intersects(const AlignedBox2& other) const;
    double iou(const AlignedBox2& other) const;
    gtsam::Vector3Vector lines() const;
    void print(const string& s) const;
    void print() const;
    bool equals(const AlignedBox2& other, double tol) const;
    bool equals(const AlignedBox2& other) const;
};

#include "sematicSLAM/core/AlignedBox3.h"
class AlignedBox3 {
    AlignedBox3();
    AlignedBox3(const double& xmin, const double& xmax, const double& ymin, const double& ymax, const double& zmin, const double& zmax);
    AlignedBox3(const Vector& xxyyzz);
    AlignedBox3(const AlignedBox3& other);
    double xmin() const;
    double xmax() const;
    double ymin() const;
    double ymax() const;
    double zmin() const;
    double zmax() const;
    Vector vector() const;
    Vector dimensions() const;
    Vector centroid() const;
    double volume() const;
    double iou(const AlignedBox3& other) const;
    void print(const string& s) const;
    void print() const;
    bool equals(const AlignedBox3& other, double tol) const;
    bool equals(const AlignedBox3& other) const;
};

#include "sematicSLAM/core/DualConic.h"
class DualConic {
    DualConic();
    DualConic(const Matrix& dC);
    DualConic(const Pose2& pose, const Vector& radii);
    Matrix matrix() const;
    gtsam::AlignedBox2 bounds() const;
    bool isDegenerate() const;
    bool isEllipse() const;
};

#include "sematicSLAM/core/Camera.h"
    class QuadricCamera {
        static Matrix transformToImage(const Pose3& pose, const Cal3_S2* calibration);
        static gtsam::DualConic project(const ConstrainedDualQuadric& quadric, const Pose3& pose, const Cal3_S2* calibration);
    };

}

#endif //SEMATICSLAM_SEMATICSLAM_H
