#pragma once

#include <SpaceVecAlg/SpaceVecAlg>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

namespace lipm_walking
{

/** Template functions used in the Ceres optimization */
namespace
{

/** We redefined sva::Rot. functions to make them work with non-scalar types */

template<typename T>
inline Eigen::Matrix3<T> RotX(T theta)
{
  T s = sin(theta);
  T c = cos(theta);
  return (Eigen::Matrix3<T>() << T(1), T(0), T(0), T(0), c, s, T(0), -s, c).finished();
}

template<typename T>
inline Eigen::Matrix3<T> RotY(T theta)
{
  T s = sin(theta);
  T c = cos(theta);
  return (Eigen::Matrix3<T>() << c, T(0), -s, T(0), T(1), T(0), s, T(0), c).finished();
}

template<typename T>
inline Eigen::Matrix3<T> RotZ(T theta)
{
  T s = sin(theta);
  T c = cos(theta);
  return (Eigen::Matrix3<T>() << c, s, T(0), -s, c, T(0), T(0), T(0), T(1)).finished();
}

template<typename T>
Eigen::Matrix3<T> rpyToMat(const T & r, const T & p, const T & y)
{
  return RotX<T>(r) * RotY<T>(p) * RotZ<T>(y);
}

struct PitchZCostFunctor
{
  PitchZCostFunctor(const sva::PTransformd & X_s_p, const sva::PTransformd & X_0_b, const sva::PTransformd & X_b_s)
  : X_s_p_(X_s_p), X_0_b_(X_0_b), X_b_s_(X_b_s)
  {
  }

  template<typename T>
  bool operator()(const T * const pitch, const T * const t_z, T * residual) const
  {
    sva::PTransform<T> X_0_b = X_0_b_;
    sva::PTransform<T> X_b_b(RotY(pitch[0]), Eigen::Vector3<T>(T(0), T(0), t_z[0]));
    sva::PTransform<T> X_b_s = X_b_s_;
    sva::PTransform<T> X_s_p = X_s_p_;

    sva::PTransform<T> X_0_p = (X_s_p * X_b_s * X_b_b * X_0_b);

    residual[0] = X_0_p.translation().z();

    return true;
  }

  // Factory to hide the construction of the CostFunction object from the client code.
  static ceres::CostFunction * Create(const sva::PTransformd & X_s_p,
                                      const sva::PTransformd & X_0_b,
                                      const sva::PTransformd & X_b_s)
  {
    return (new ceres::AutoDiffCostFunction<PitchZCostFunctor, 1, 1, 1>(new PitchZCostFunctor(X_s_p, X_0_b, X_b_s)));
  }

private:
  const sva::PTransformd X_s_p_;
  const sva::PTransformd X_0_b_;
  const sva::PTransformd X_b_s_;
};

struct Minimize
{
  template<typename T>
  bool operator()(const T * const x, T * residual) const
  {
    residual[0] = x[0];
    return true;
  }
};

} // namespace

} // namespace lipm_walking
