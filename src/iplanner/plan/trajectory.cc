#include "iplanner/plan/trajectory.h"

namespace iplanner
{
Trajectory::Trajectory(int num_joints, int num_timepoints, double time)
  : time_(time)
{
  trajectory_.resize(num_joints, num_timepoints);
  trajectory_.setZero();
}

Trajectory::~Trajectory() = default;

VectorXd Trajectory::AtTime(double t) const
{
  const int num_joints = Rows();
  const int num_timepoints = Cols();

  if (t <= 0.)
    return trajectory_.col(0);

  else if (t >= time_)
    return trajectory_.col(trajectory_.cols() - 1);

  else
  {
    double normalized_t = t / time_ * num_timepoints;
    auto index = static_cast<int>(normalized_t);
    normalized_t -= index;

    if (normalized_t == 0.0)
    {
      if (index == num_timepoints)
        return trajectory_.col(index - 1);

      return trajectory_.col(index);
    }

    // Cubic hermite spline
    auto& x = normalized_t;
    double x2 = x * x;
    double x3 = x2 * x;

    VectorXd p0 = trajectory_.col(index);
    VectorXd p1 = trajectory_.col(index + 1);

    //VectorXd pb = index == 0 ? static_cast<VectorXd>(p0 - (p1 - p0)) : trajectory_.col(index - 1);
    //VectorXd p2 = index + 1 == num_timepoints - 1 ? static_cast<VectorXd>(p1 - (p0 - p1)) : trajectory_.col(index + 2);

    VectorXd pb = index == 0 ? p0 : trajectory_.col(index - 1);
    VectorXd p2 = index + 1 == num_timepoints - 1 ? p1 : trajectory_.col(index + 2);

    VectorXd v0 = (p1 - pb) * .5;
    VectorXd v1 = (p2 - p0) * .5;

    return (2. * x3 - 3. * x2 + 1.) * p0
      + (x3 - 2. * x2 + x) * v0
      + (-2. * x3 + 3. * x2) * p1
      + (x3 - x2) * v1;

    // TODO: precompute basis and get as matrix multiplication
  }
}
}
