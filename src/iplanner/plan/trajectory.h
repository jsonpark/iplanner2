#ifndef IPLANNER_PLAN_TRAJECTORY_H_
#define IPLANNER_PLAN_TRAJECTORY_H_

#include "iplanner/types.h"

namespace iplanner
{
class Trajectory
{
public:
  Trajectory() = delete;

  Trajectory(int num_joints, int num_timepoints, double time);

  ~Trajectory();

  auto& operator () (int i, int j)
  {
    return trajectory_(i, j);
  }

  const auto& operator () (int i, int j) const
  {
    return trajectory_(i, j);
  }

  int Rows() const
  {
    return static_cast<int>(trajectory_.rows());
  }

  int Cols() const
  {
    return static_cast<int>(trajectory_.cols());
  }

  auto Time() const
  {
    return time_;
  }

  VectorXd AtTime(double t) const;

private:
  // Each column is joint angles
  MatrixXd trajectory_;
  double time_;
};
}

#endif // IPLANNER_PLAN_TRAJECTORY_H_
