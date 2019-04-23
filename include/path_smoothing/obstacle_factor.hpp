//
// Created by yangt on 19-2-24.
//

#ifndef PATH_SMOOTHING_OBSTACLE_FACTOR_HPP
#define PATH_SMOOTHING_OBSTACLE_FACTOR_HPP

/**
 *  @file  ObstacleFactor.h
 *  @brief Obstacle avoidance cost factor, using 3D signed distance field
 *  @author Jing Dong
 *  @date  May 11, 2016
 **/


#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>

#include <iostream>
#include <vector>

#include "path_smoothing/distance_function.hpp"

namespace gpmp2 {

/**
 * unary factor for obstacle avoidance
 * template robot model version
 */
template<class ROBOT>
class ObstacleFactor
        : public gtsam::NoiseModelFactor1<typename ROBOT::Pose> {

 public:
  // typedefs
  typedef ROBOT Robot;
  typedef typename Robot::Pose Pose;

 private:
  // typedefs
  typedef ObstacleFactor This;
  typedef gtsam::NoiseModelFactor1<Pose> Base;

  // arm: planar one, all alpha = 0
  const Robot robot_;

  // signed distance field from matlab
  const DistanceFunction2D &function_;

 public:

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;

  /* Default constructor do nothing */
  ObstacleFactor()
          : robot_(Robot()),
            function_(DistanceFunction2D()) {}

  /**
   * Constructor
   * @param cost_model cost function covariance, should to identity model
   * @param field      signed distance field
   * @param nn_index   nearest neighbour index of signed distance field
   */
  ObstacleFactor(gtsam::Key poseKey,
                 const Robot &robot,
                 const DistanceFunction2D &function,
                 double cost_sigma) :
          Base(gtsam::noiseModel::Isotropic::Sigma(robot.nr_body_spheres(),
                                                   cost_sigma), poseKey),
          robot_(robot), function_(function) {}

  virtual ~ObstacleFactor() {}

  /// error function
  /// numerical jacobians / analytic jacobians from cost function
  gtsam::Vector evaluateError(const typename Robot::Pose &conf,
                              boost::optional<gtsam::Matrix &> H1 = boost::none) const {
      if (H1) {
          *H1 = gtsam::Matrix::Zero(robot_.nr_body_spheres(), robot_.dof());
      }
      std::vector<gtsam::Point3> sph_centers;
      std::vector<gtsam::Matrix> J_px_jp;
      if (H1) {
          robot_.sphereCenters(conf, sph_centers, J_px_jp);
      } else {
          robot_.sphereCenters(conf, sph_centers);
      }
      // allocate cost vector
      gtsam::Vector3 g(0.0, 0.0, 0.0);
      gtsam::Vector err(robot_.nr_body_spheres());

      for (size_t sph_idx = 0; sph_idx < robot_.nr_body_spheres(); sph_idx++) {
          err(sph_idx) = function_.cost(sph_centers[sph_idx].x(),
                                        sph_centers[sph_idx].y());
          if (H1) {
              function_.gradient(sph_centers[sph_idx].x(),
                                 sph_centers[sph_idx].y(),
                                 g.data());
              H1->row(sph_idx) = g;
          }
      }
      return err;
  }

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
              gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /** print contents */
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter = gtsam::DefaultKeyFormatter) const {
      std::cout << s << "ObstacleFactor :" << std::endl;
      Base::print("", keyFormatter);
  }

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {
      ar & boost::serialization::make_nvp("NoiseModelFactor1",
                                          boost::serialization::base_object<Base>(
                                                  *this));
  }
};

}

#endif //PATH_SMOOTHING_OBSTACLE_FACTOR_HPP
